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
//   /encoder/direction std_msgs/String  FORWARD / REVERSE / STOP (ESC 명령 기준, 아래 참고)
//   /encoder/speed     std_msgs/Float32 차량 속도 m/s (전진 +, 후진 -, 저역통과 필터)
//   /encoder/distance  std_msgs/Float64 누적 주행 거리 m (전진 +, 후진 -)
//
// 거리·속도: Total Pulses(/encoder/pulses) 변화량 x meters_per_pulse (실측 0.00412 m)
// 방향: 엔코더 A/B 가 아니라 ESC 명령(/drive/throttle_us)으로 판단한다.
//   throttle_us > neutral_us(1680) → 전진, < neutral_us → 후진, == → 직전 방향 유지
//   단, 방향 전환은 '바퀴가 멈춰 있을 때'만 반영한다. 전진 중 1680 아래 펄스는
//   ESC 브레이크라서 그 순간 후진으로 세면 거리가 거꾸로 줄어들기 때문.
//   /drive/throttle_us 는 drive_node 와 drive_record.py 가 발행한다.
//   명령을 한 번도 못 받았으면 전진으로 간주.
//
//   /encoder/frequency std_msgs/Float64 A 채널 주파수 [Hz], 방향 부호 포함
//                                       (커널 이벤트 타임스탬프로 A 상승 에지 간격을 측정)
//   /encoder/period_ms std_msgs/Float64 A 채널 한 주기 [ms] (정지 시 0)

#include <fcntl.h>
#include <linux/gpio.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
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
#include "std_msgs/msg/int32.hpp"
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
    // 실측: Total Pulses 1개당 0.00412 m
    meters_per_pulse_ = declare_parameter<double>("meters_per_pulse", 0.00412);
    neutral_us_ = declare_parameter<int>("neutral_us", 1680);
    // 이 시간 동안 펄스가 없으면 '멈춤' → 이때만 ESC 명령으로 방향 전환
    switch_stop_time_ = declare_parameter<double>("switch_stop_time", 0.1);
    // 속도 저역통과 필터 시정수 (s). 0 이면 필터 없음
    speed_tau_ = declare_parameter<double>("speed_filter_tau", 0.05);
    stop_timeout_ = declare_parameter<double>("stop_timeout", 0.5);
    const auto print_period = declare_parameter<double>("print_period", 0.5);
    // 실차에서 전진 시 카운트가 줄어 기본값을 true 로 둔다
    reverse_ = declare_parameter<bool>("reverse", true);
    // Hz/ms 를 최근 몇 주기의 평균으로 낼지 (1 이면 한 주기마다 그대로)
    freq_window_ = std::clamp<int64_t>(
      declare_parameter<int>("freq_window", 8), 1, kMaxRiseWindow - 1);

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
    throttle_sub_ = create_subscription<std_msgs::msg::Int32>(
      "drive/throttle_us", 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        if (msg->data > neutral_us_) {
          cmd_sign_ = 1;
        } else if (msg->data < neutral_us_) {
          cmd_sign_ = -1;
        }  // == neutral: 직전 명령 방향 유지
      });
    freq_pub_ = create_publisher<std_msgs::msg::Float64>("encoder/frequency", 10);
    period_pub_ = create_publisher<std_msgs::msg::Float64>("encoder/period_ms", 10);

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
      "%.5f m/pulse, direction from /drive/throttle_us (neutral %ld us)",
      chip.c_str(), line_a_, line_b_, meters_per_pulse_, neutral_us_);
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
          // 이미 HIGH 인데 또 상승 에지처럼, 상태가 안 바뀌는 이벤트 = 그 라인의 에지를 놓쳤다
          missed_edges_++;
          continue;
        }
        encoder_count_ += sign * delta;
        pulse_count_++;
        last_edge_ns_ = static_cast<int64_t>(events[i].timestamp_ns);

        // A 상승 에지 사이 간격 = A 한 주기. 타임스탬프는 커널이 에지 발생 시점에 찍은 값이라
        // 이 스레드가 늦게 처리해도 정확하다. 최근 freq_window 주기의 평균으로 계산한다.
        // 방향이 바뀌었거나 오래 멈췄다가 다시 움직인 경우에는 간격이 의미 없으므로
        // 기준점부터 새로 모은다.
        if (bit == 0b10 && events[i].id == GPIO_V2_LINE_EVENT_RISING_EDGE) {
          const int64_t ts = static_cast<int64_t>(events[i].timestamp_ns);
          const int step = sign * delta;
          const int64_t period = ts - last_a_rise_ns_;
          if (last_a_rise_ns_ == 0 || step != last_a_step_ || period <= 0 ||
            static_cast<double>(period) * 1e-9 > stop_timeout_)
          {
            rise_count_ = 0;
          }
          rise_ts_[rise_count_ % kMaxRiseWindow] = ts;
          rise_count_++;
          const int64_t n = std::min<int64_t>(rise_count_, freq_window_ + 1);
          if (n >= 2) {
            const int64_t span = ts - rise_ts_[(rise_count_ - n) % kMaxRiseWindow];
            frequency_hz_ = step * (n - 1) * 1e9 / static_cast<double>(span);
            period_ms_ = static_cast<double>(span) * 1e-6 / static_cast<double>(n - 1);
          }
          last_a_rise_ns_ = ts;
          last_a_step_ = step;
        }
      }
    }
  }

  void publish()
  {
    const int64_t count_now = encoder_count_.load();
    const int64_t pulses_now = pulse_count_.load();
    const int64_t t = now_ns();
    const int64_t dp = pulses_now - last_pulses_;
    last_pulses_ = pulses_now;

    // 방향: 멈춰 있을 때만 ESC 명령 방향으로 바꾼다 (주행 중 1680 아래 = 브레이크)
    const double idle = static_cast<double>(t - last_move_ns_) * 1e-9;
    if (idle > switch_stop_time_ && cmd_sign_ != 0) {
      motion_sign_ = cmd_sign_;
    }
    if (dp > 0) {
      last_move_ns_ = t;
    }

    const double ds = static_cast<double>(motion_sign_ * dp) * meters_per_pulse_;
    distance_ += ds;

    // 속도 (m/s): 퍼블리시 주기 동안의 거리 / 시간, 1차 저역통과
    const double dt = static_cast<double>(t - last_speed_ns_) * 1e-9;
    if (dt > 0.0) {
      const double alpha = speed_tau_ > 0.0 ? dt / (speed_tau_ + dt) : 1.0;
      speed_ += alpha * (ds / dt - speed_);
      last_speed_ns_ = t;
    }
    const bool stopped = static_cast<double>(t - last_move_ns_) * 1e-9 > stop_timeout_;
    if (stopped) {
      speed_ = 0.0;
    }
    direction_ = stopped ? 0 : motion_sign_;

    std_msgs::msg::Float32 speed;
    speed.data = static_cast<float>(speed_);
    speed_pub_->publish(speed);

    std_msgs::msg::Float64 dist;
    dist.data = distance_;
    dist_pub_->publish(dist);

    // 이벤트 타임스탬프와 steady_clock 은 둘 다 CLOCK_MONOTONIC 이다
    if (static_cast<double>(t - last_edge_ns_.load()) * 1e-9 > stop_timeout_) {
      frequency_hz_ = 0.0;
      period_ms_ = 0.0;
    }

    std_msgs::msg::Int64 count;
    count.data = count_now;
    count_pub_->publish(count);

    std_msgs::msg::Int64 pulses;
    pulses.data = pulses_now;
    pulses_pub_->publish(pulses);

    std_msgs::msg::String dir;
    dir.data = direction_name();
    dir_pub_->publish(dir);

    std_msgs::msg::Float64 freq;
    freq.data = frequency_hz_.load();
    freq_pub_->publish(freq);

    std_msgs::msg::Float64 period;
    period.data = period_ms_.load();
    period_pub_->publish(period);
  }

  const char * direction_name() const
  {
    const int d = direction_.load();
    return d > 0 ? "FORWARD" : (d < 0 ? "REVERSE" : "STOP");
  }

  void print_status()
  {
    RCLCPP_INFO(
      get_logger(), "Direction: %s | Count: %ld | Total Pulses: %ld | %.3f m/s | %.3f m | "
      "Freq: %.1f Hz | Period: %.3f ms | Missed: %ld",
      direction_name(), encoder_count_.load(), pulse_count_.load(), speed_, distance_,
      frequency_hz_.load(), period_ms_.load(), missed_edges_.load());
  }

  int chip_fd_{-1};
  int request_fd_{-1};
  int64_t line_a_{112};
  int64_t line_b_{122};
  int state_{0};  // 이벤트 스레드 전용: (A << 1) | B
  int64_t last_a_rise_ns_{0};  // 이벤트 스레드 전용
  int last_a_step_{0};  // 이벤트 스레드 전용
  static constexpr int64_t kMaxRiseWindow = 64;
  int64_t rise_ts_[kMaxRiseWindow]{};  // 이벤트 스레드 전용: 최근 A 상승 에지 시각
  int64_t rise_count_{0};  // 이벤트 스레드 전용
  int64_t freq_window_{8};
  double stop_timeout_{0.5};
  bool reverse_{true};
  double meters_per_pulse_{0.00412};
  int64_t neutral_us_{1680};
  double switch_stop_time_{0.1};
  double speed_tau_{0.05};
  // 아래는 executor 스레드(타이머/구독 콜백) 전용
  double speed_{0.0};
  double distance_{0.0};
  int64_t last_pulses_{0};
  int64_t last_speed_ns_{0};
  int cmd_sign_{0};       // 마지막 ESC 명령 방향 (0 = 아직 없음)
  int motion_sign_{1};    // 실제로 적용 중인 방향 (기본 전진)

  std::atomic<int64_t> encoder_count_{0};
  std::atomic<int64_t> pulse_count_{0};
  std::atomic<int64_t> missed_edges_{0};
  std::atomic<int> direction_{0};
  std::atomic<double> frequency_hz_{0.0};
  std::atomic<double> period_ms_{0.0};
  std::atomic<int64_t> last_edge_ns_{0};
  int64_t last_move_ns_{0};
  std::atomic<bool> running_{false};
  std::thread event_thread_;

  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr count_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pulses_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dir_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dist_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr throttle_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr freq_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr period_pub_;
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
