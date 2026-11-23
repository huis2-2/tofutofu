# 주행 데이터 (2026-11-23)

`drive_record.py`로 녹화한 트랙 주행 데이터입니다.

| 세션 | 길이 | 라벨링용 사진 | 위치 |
|---|---|---|---|
| session_20261123_142231 | 27초 | 55장 | `extracted/images`, `extracted/bev` |
| session_20261123_142305 | 128초 | 254장 | `extracted/images`, `extracted/bev` |
| session_20261123_142735 | 166초 | 329장 | `images`, `bev` |

- `images/`: 카메라 원본(640x480), `bev/`: 같은 프레임의 BEV(번호가 같음)
- `labels.csv`: filename, bev_filename, stamp, steering_us, throttle_us, steering(-1 오른쪽 ~ +1 왼쪽), source
- `stamp`는 rosbag 안의 같은 프레임 `header.stamp`와 일치합니다.
- 앞의 두 세션은 정지 프레임을 건너뛰던 버전으로 찍어서 JPG가 없었고,
  `bag_to_images.py`로 rosbag에서 0.5초마다 뽑은 사진이 `extracted/`에 있습니다.
  이 두 세션의 조향/속도 값은 1660/1500(중앙/정지)으로 고정이라 실제 움직임은 기록되지 않았습니다.

## rosbag, mp4 영상

용량이 커서(파일당 최대 약 440MB, GitHub 파일 제한 100MB) 저장소가 아니라
[Releases `data-20261123`](https://github.com/huis2-2/tofutofu/releases/tag/data-20261123)에 있습니다.

- `session_*_rosbag.zip`: 압축을 풀어 `session_*/rosbag/`에 두면 됩니다.
  토픽: `/camera/image_raw/compressed`, `/bev/image_raw/compressed`,
  `/drive/steering_us`, `/drive/throttle_us`, `/drive/steering`, `/drive/source`
- `session_*_camera.mp4`, `session_*_bev.mp4`: 같은 영상을 mp4(H.264)로 변환한 것

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag play session_20261123_142305/rosbag
python3 bag_to_images.py datasets/session_20261123_142305 --fps 5   # 사진 다시 뽑기
python3 bag_to_video.py  datasets/session_20261123_142305            # mp4 다시 만들기
```
