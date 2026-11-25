# drive_ws — RC카 구동 (PCA9685 서보/ESC + 엔코더 속도 PID)

`/drive/cmd` (m/s, rad) → 조향: 좌/우 따로 선형 매핑 (`max_left_rad` ↔ `left_us`) / 속도: 피드포워드(`deadband_us + kff·v`) + PID (`/encoder/speed`).
배터리 전압이 떨어져도 PID 가 속도를 유지합니다. 정지 명령인데 아직 달리고 있으면 `brake_us` 로 브레이크.
하드웨어 값은 `drive_record.py` 에서 확인된 것: I2C 7번, 0x40, 300 Hz, CH8 조향, CH9 ESC, MUX SEL BOARD 15번.
`drive_record.py` 와 같은 `/drive/steering_us`, `/drive/throttle_us`, `/drive/steering` 을 발행해서 자율주행 rosbag 도 모방학습 데이터로 쓸 수 있습니다.

튜닝 순서: `open_loop: true` 로 `deadband_us`(바퀴가 막 도는 값), `kff`(0.5/1.0 m/s 에 필요한 µs) 를 맞춘 뒤 PID 켜기.
안전: 명령 0.3초 끊기면 중립, 종료 시 중립 + 조향 중앙. `max_forward_us` 상한.

## 빌드

```bash
cd ~/tofutofu/drive_ws && source env.sh
colcon build --symlink-install && source env.sh
```
