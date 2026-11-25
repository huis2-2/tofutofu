// Jetson Orin Nano 엔코더 노드 (encoder.py 를 C/C++ 로 옮긴 버전)
//
// 배선 (Jetson 물리 핀 번호)
//   ENCODER_A : 물리 11번 = PR.04 = gpiochip0 line 112
//   ENCODER_B : 물리 13번 = PY.00 = gpiochip0 line 122
//
// A, B 두 채널의 상승/하강 에지를 모두 이벤트로 받아 4X 쿼드러처 디코딩한다.
// 커널이 에지를 발생 순서대로 큐에 넣으므로, 처리가 늦어져도 방향을 잘못 읽지 않는다.
// (예전 방식은 A 상승 에지 이벤트를 처리하는 시점에 B 를 읽어서, 속도가 빠르거나
//  A/B 위상차가 90도가 아니면 한쪽 방향에서 FORWARD/REVERSE 가 섞여 나왔다.)
//   A 상승 에지에서 B == LOW -> count + 1 (FORWARD, reverse=false 기준)
//   한 주기에 4 카운트 (예전 1X 방식의 4배)
//
// GPIO 는 Linux GPIO character device v2 API (linux/gpio.h, 순수 C ioctl) 로 직접 접근한다.
//
// 퍼블리시 토픽
//   /encoder/count     std_msgs/Int64   누적 카운트 (방향 반영, 4X)
//   /encoder/pulses    std_msgs/Int64   전체 에지 수 (4X)
//   /encoder/direction std_msgs/String  FORWARD / REVERSE / STOP
//   /encoder/speed     std_msgs/Float32 차량 속도 m/s (전진 +, 저역통과 필터)
//   /encoder/distance  std_msgs/Float64 누적 주행 거리 m (방향 반영, 후진하면 줄어듦)
//
// 속도 계산: meters_per_count = wheel_circumference_m / counts_per_wheel_rev
//   counts_per_wheel_rev 는 4X 기준 카운트. 차를 손으로 정확히 1m 밀고
//   /encoder/count 변화량 N 을 재면 meters_per_count = 1/N 으로 바로 넣어도 된다
//   (meters_per_count > 0 이면 그 값을 우선 사용).

#include <fcntl.h>
#include <linux/gpio.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

// A, B 두 라인을 양쪽 에지 이벤트로 한 번에 요청하고 request fd 를 반환한다.
// 한 요청으로 묶어야 두 라인의 이벤트가 하나의 큐에 발생 순서대로 들어온다.
int request_lines(int chip_fd, unsigned int line_a, unsigned int line_b)
{
  struct gpio_v2_line_request req;
  std::memset(&req, 0, sizeof(req));
  req.offsets[0] = line_a;
  req.offsets[1] = line_b;
  req.num_lines = 2;
  req.config.flags = GPIO_V2_LINE_FLAG_INPUT |
    GPIO_V2_LINE_FLAG_EDGE_RISING | GPIO_V2_LINE_FLAG_EDGE_FALLING;
  req.event_buffer_size = 256;
  std::strncpy(req.consumer, "encoder", sizeof(req.consumer) - 1);

  if (ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
    throw std::runtime_error(
            std::string("GPIO line ") + std::to_string(line_a) + ", " +
            std::to_string(line_b) + " 요청 실패: " + std::strerror(errno));
  }
  return req.fd;
}

// 두 라인 값을 읽어 (A << 1) | B 로 반환한다.
int read_state(int request_fd)
{
  struct gpio_v2_line_values values;
  values.mask = 0b11;
  values.bits = 0;
  if (ioctl(request_fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &values) < 0) {
    return -1;
  }
  return static_cast<int>(((values.bits & 1) << 1) | ((values.bits >> 1) & 1));
}

// 이전 상태 * 4 + 새 상태 -> 카운트 변화 (상태 = (A << 1) | B)
// 두 비트가 동시에 바뀐 경우(에지 누락)는 방향을 알 수 없어 0 으로 둔다.
constexpr int kQuadratureTable[16] = {
  0, -1, +1, 0,
  +1, 0, 0, -1,
  -1, 0, 0, +1,
  0, +1, -1, 0,
};

}  // namespace

class EncoderNode : public rclcpp::Node
{
public:
  EncoderNode()
  : Node("encoder_node")
  {
    const auto chip = declare_parameter<std::string>("gpio_chip", "/dev/gpiochip0");
    line_a_ = declare_parameter<int>("line_a", 112);  // 물리 11번 PR.04
    line_b_ = declare_parameter<int>("line_b", 122);  // 물리 13번 PY.00
    const auto rate_hz = declare_parameter<double>("publish_rate", 50.0);
    // TODO(실측): 바퀴 둘레와 바퀴 1회전당 4X 카운트
    const auto wheel_circ = declare_parameter<double>("wheel_circumference_m", 0.2042);
    const auto counts_per_rev = declare_parameter<double>("counts_per_wheel_rev", 1440.0);
    const auto mpc = declare_parameter<double>("meters_per_count", 0.0);
    meters_per_count_ = mpc > 0.0 ? mpc : wheel_circ / counts_per_rev;
    // 속도 저역통과 필터 시정수 (s). 0 이면 필터 없음
    speed_tau_ = declare_parameter<double>("speed_filter_tau", 0.05);
    stop_timeout_ = declare_parameter<double>("stop_timeout", 0.5);
    const auto print_period = declare_parameter<double>("print_period", 0.5);
    // 실차에서 전진 시 카운트가 줄어 기본값을 true 로 둔다
    reverse_ = declare_parameter<bool>("reverse", true);

    chip_fd_ = open(chip.c_str(), O_RDONLY | O_CLOEXEC);
    if (chip_fd_ < 0) {
      throw std::runtime_error(chip + " 열기 실패: " + std::strerror(errno));
    }
    request_fd_ = request_lines(
      chip_fd_, static_cast<unsigned int>(line_a_), static_cast<unsigned int>(line_b_));
    state_ = read_state(request_fd_);
    if (state_ < 0) {
      throw std::runtime_error(std::string("GPIO 값 읽기 실패: ") + std::strerror(errno));
    }

    count_pub_ = create_publisher<std_msgs::msg::Int64>("encoder/count", 10);
    pulses_pub_ = create_publisher<std_msgs::msg::Int64>("encoder/pulses", 10);
    dir_pub_ = create_publisher<std_msgs::msg::String>("encoder/direction", 10);
    speed_pub_ = create_publisher<std_msgs::msg::Float32>("encoder/speed", 10);
    dist_pub_ = create_publisher<std_msgs::msg::Float64>("encoder/distance", 10);

    last_move_ns_ = now_ns();
    last_speed_ns_ = last_move_ns_;
    running_ = true;
    event_thread_ = std::thread(&EncoderNode::event_loop, this);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz),
      std::bind(&EncoderNode::publish, this));

    // encoder.py 처럼 터미널에 주기적으로 출력 (0 이하이면 출력 안 함)
    if (print_period > 0.0) {
      print_timer_ = create_wall_timer(
        std::chrono::duration<double>(print_period),
        std::bind(&EncoderNode::print_status, this));
    }

    RCLCPP_INFO(
      get_logger(), "Encoder start: %s A=line %ld (pin 11), B=line %ld (pin 13), "
      "4X quadrature, reverse=%s, %.6f m/count",
      chip.c_str(), line_a_, line_b_, reverse_ ? "true" : "false", meters_per_count_);
  }

  ~EncoderNode() override
  {
    running_ = false;
    if (event_thread_.joinable()) {
      event_thread_.join();
    }
    if (request_fd_ >= 0) {close(request_fd_);}
    if (chip_fd_ >= 0) {close(chip_fd_);}
  }

private:
  static int64_t now_ns()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  // A, B 에지 이벤트를 발생 순서대로 처리하는 스레드
  void event_loop()
  {
    struct pollfd pfd;
    pfd.fd = request_fd_;
    pfd.events = POLLIN;
    struct gpio_v2_line_event events[64];
    const int sign = reverse_ ? -1 : 1;

    while (running_) {
      const int ret = poll(&pfd, 1, 100);  // 100ms 마다 종료 여부 확인
      if (ret <= 0 || !(pfd.revents & POLLIN)) {
        continue;
      }
      const ssize_t n = read(request_fd_, events, sizeof(events));
      if (n <= 0) {
        continue;
      }
      const int num = static_cast<int>(n / sizeof(events[0]));
      for (int i = 0; i < num; ++i) {
        const int bit = events[i].offset == static_cast<unsigned int>(line_a_) ? 0b10 : 0b01;
        const int next = events[i].id == GPIO_V2_LINE_EVENT_RISING_EDGE ?
          (state_ | bit) : (state_ & ~bit);
        const int delta = kQuadratureTable[state_ * 4 + next];
        state_ = next;
        if (delta == 0) {
          continue;
        }
        encoder_count_ += sign * delta;
        pulse_count_++;
      }
    }
  }

  void publish()
  {
    // 방향은 에지 하나가 아니라 퍼블리시 주기 동안의 카운트 변화로 정한다.
    // 일정 시간 카운트 변화가 없으면 STOP
    const int64_t count_now = encoder_count_.load();
    const int64_t t = now_ns();

    // 속도 (m/s): 퍼블리시 주기 동안의 카운트 변화량 / 시간, 1차 저역통과
    const double dt = static_cast<double>(t - last_speed_ns_) * 1e-9;
    if (dt > 0.0) {
      const double raw = static_cast<double>(count_now - last_speed_count_) *
        meters_per_count_ / dt;
      const double alpha = speed_tau_ > 0.0 ? dt / (speed_tau_ + dt) : 1.0;
      speed_ += alpha * (raw - speed_);
      last_speed_count_ = count_now;
      last_speed_ns_ = t;
    }
    std_msgs::msg::Float32 speed;
    speed.data = static_cast<float>(speed_);
    speed_pub_->publish(speed);
    std_msgs::msg::Float64 dist;
    dist.data = static_cast<double>(count_now) * meters_per_count_;
    dist_pub_->publish(dist);
    if (count_now != last_count_) {
      direction_ = count_now > last_count_ ? 1 : -1;
      last_count_ = count_now;
      last_move_ns_ = t;
    } else if (static_cast<double>(t - last_move_ns_) * 1e-9 > stop_timeout_) {
      direction_ = 0;
    }

    std_msgs::msg::Int64 count;
    count.data = count_now;
    count_pub_->publish(count);

    std_msgs::msg::Int64 pulses;
    pulses.data = pulse_count_.load();
    pulses_pub_->publish(pulses);

    std_msgs::msg::String dir;
    dir.data = direction_name();
    dir_pub_->publish(dir);
  }

  const char * direction_name() const
  {
    const int d = direction_.load();
    return d > 0 ? "FORWARD" : (d < 0 ? "REVERSE" : "STOP");
  }

  void print_status()
  {
    RCLCPP_INFO(
      get_logger(), "Direction: %s | Count: %ld | Total Pulses: %ld | %.3f m/s | %.3f m",
      direction_name(), encoder_count_.load(), pulse_count_.load(), speed_,
      static_cast<double>(encoder_count_.load()) * meters_per_count_);
  }

  int chip_fd_{-1};
  int request_fd_{-1};
  int64_t line_a_{112};
  int64_t line_b_{122};
  int state_{0};  // 이벤트 스레드 전용: (A << 1) | B
  double stop_timeout_{0.5};
  bool reverse_{true};
  double meters_per_count_{0.0};
  double speed_tau_{0.05};
  double speed_{0.0};          // 타이머 콜백 전용
  int64_t last_speed_count_{0};
  int64_t last_speed_ns_{0};

  std::atomic<int64_t> encoder_count_{0};
  std::atomic<int64_t> pulse_count_{0};
  std::atomic<int> direction_{0};
  int64_t last_count_{0};  // 타이머 콜백 전용
  int64_t last_move_ns_{0};
  std::atomic<bool> running_{false};
  std::thread event_thread_;

  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr count_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pulses_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dir_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dist_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr print_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<EncoderNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("encoder_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
