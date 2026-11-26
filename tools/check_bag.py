#!/usr/bin/env python3
"""주행 rosbag 이 모방학습에 쓸 만한지 검사하고 보고서(report.md, report.json)를 bag 폴더에 쓴다.

  source /opt/ros/jazzy/setup.bash
  python3 tools/check_bag.py datasets/bags/run_20261126_143000
  python3 tools/check_bag.py datasets/session_20261123_142735/rosbag     # drive_record.py 녹화도 가능

판정 (모방학습 = 카메라 이미지 → 조향/속도 라벨)
  OK      : 바로 학습에 사용
  CAUTION : 쓸 수는 있지만 문제 있음 (프레임 끊김, 라벨 일부 없음, 조향 다양성 부족 등)
  BAD     : 학습에 쓰지 말 것 (카메라/라벨 없음, 너무 짧음, 거의 안 움직임)
종료 코드: OK 0 / CAUTION 1 / BAD 2
"""

import argparse
import json
import math
import os
import sys

CAMERA_TOPICS = ('/camera/image_raw/compressed', '/camera/image_raw')
STEER_US = '/drive/steering_us'
THROTTLE_US = '/drive/throttle_us'
SPEED = '/encoder/speed'
DISTANCE = '/encoder/distance'
LANE_TARGET = '/lane/target'
MUX_SOURCE = '/mux/active_source'
DRIVE_SOURCE = '/drive/source'

# 값까지 읽을 토픽 (이미지는 시간만 센다 → 빠름)
VALUE_TOPICS = (STEER_US, THROTTLE_US, SPEED, DISTANCE, LANE_TARGET, MUX_SOURCE, DRIVE_SOURCE)

STEER_CENTER, STEER_LEFT, STEER_RIGHT = 1660, 2000, 1400
NEUTRAL_US = 1680


# ---------------- 분석 (ROS 없이 테스트 가능) ----------------

def rate_stats(ts):
    if len(ts) < 2:
        return {'count': len(ts), 'hz': 0.0, 'max_gap': None}
    gaps = [b - a for a, b in zip(ts, ts[1:])]
    dur = ts[-1] - ts[0]
    return {'count': len(ts), 'hz': (len(ts) - 1) / dur if dur > 0 else 0.0,
            'max_gap': max(gaps), 'gaps_over_0p3s': sum(g > 0.3 for g in gaps)}


def coverage(frame_ts, label_ts, tol=0.1):
    """각 카메라 프레임 ±tol 초 안에 라벨 메시지가 있는 비율."""
    if not frame_ts:
        return 0.0
    if not label_ts:
        return 0.0
    j, hit = 0, 0
    for t in frame_ts:
        while j + 1 < len(label_ts) and label_ts[j + 1] <= t:
            j += 1
        near = min(abs(label_ts[j] - t), abs(label_ts[min(j + 1, len(label_ts) - 1)] - t))
        hit += near <= tol
    return hit / len(frame_ts)


def steer_ratio(us):
    if us >= STEER_CENTER:
        return (us - STEER_CENTER) / (STEER_LEFT - STEER_CENTER)
    return -(STEER_CENTER - us) / (STEER_CENTER - STEER_RIGHT)


def analyze(times, values, start, end):
    """times: {topic: [t...]}, values: {topic: [(t, v)...]} (t = 초) → 보고서 dict."""
    rep = {'duration_sec': round(end - start, 1), 'topics': {}, 'problems': [], 'warnings': []}
    for topic, ts in sorted(times.items()):
        st = rate_stats(ts)
        rep['topics'][topic] = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in st.items()}

    cam = next((t for t in CAMERA_TOPICS if times.get(t)), None)
    rep['camera_topic'] = cam
    cam_ts = times.get(cam, []) if cam else []
    cam_st = rate_stats(cam_ts)
    rep['camera_fps'] = round(cam_st['hz'], 1)
    rep['camera_max_gap'] = None if cam_st['max_gap'] is None else round(cam_st['max_gap'], 3)

    steer = values.get(STEER_US, [])
    thr = values.get(THROTTLE_US, [])
    rep['label_coverage'] = round(coverage(cam_ts, [t for t, _ in steer]), 3)

    ratios = [steer_ratio(v) for _, v in steer]
    if ratios:
        mean = sum(ratios) / len(ratios)
        rep['steer_std'] = round(math.sqrt(sum((r - mean) ** 2 for r in ratios) / len(ratios)), 3)
        rep['steer_left_pct'] = round(100 * sum(r > 0.15 for r in ratios) / len(ratios), 1)
        rep['steer_right_pct'] = round(100 * sum(r < -0.15 for r in ratios) / len(ratios), 1)
    if thr:
        rep['throttle_forward_pct'] = round(100 * sum(v > NEUTRAL_US for _, v in thr) / len(thr), 1)
        rep['throttle_max_us'] = max(v for _, v in thr)

    spd = values.get(SPEED, [])
    if spd:
        rep['moving_pct'] = round(100 * sum(abs(v) > 0.05 for _, v in spd) / len(spd), 1)
        rep['speed_max'] = round(max(v for _, v in spd), 2)
        mov = [v for _, v in spd if v > 0.05]
        rep['speed_mean_moving'] = round(sum(mov) / len(mov), 2) if mov else 0.0
    dist = values.get(DISTANCE, [])
    if len(dist) >= 2:
        rep['distance_m'] = round(dist[-1][1] - dist[0][1], 2)

    lane = values.get(LANE_TARGET, [])
    if lane:
        rep['lane_valid_pct'] = round(100 * sum(v > 0.5 for _, v in lane) / len(lane), 1)
    src = values.get(MUX_SOURCE, []) or values.get(DRIVE_SOURCE, [])
    if src:
        rep['sources'] = sorted({v for _, v in src})

    # ---- 판정 ----
    P, W = rep['problems'], rep['warnings']
    if not cam:
        P.append('카메라 토픽 없음')
    if not steer:
        P.append(f'조향 라벨({STEER_US}) 없음')
    if rep['duration_sec'] < 10:
        P.append('10초 미만')
    if cam and rep['camera_fps'] < 15:
        P.append(f'카메라 {rep["camera_fps"]} fps (15 미만)')
    moving = rep.get('moving_pct')
    if moving is None and 'throttle_forward_pct' in rep:
        moving = rep['throttle_forward_pct']
    if moving is not None and moving < 20:
        P.append(f'움직인 시간 {moving}% (20% 미만)')
    if 'distance_m' in rep and rep['distance_m'] < 2.0:
        P.append(f'주행 거리 {rep["distance_m"]} m')
    if cam and 15 <= rep['camera_fps'] < 25:
        W.append(f'카메라 {rep["camera_fps"]} fps (25 미만, 처리 부하 확인)')
    if rep['camera_max_gap'] is not None and rep['camera_max_gap'] > 0.5:
        W.append(f'카메라 최대 끊김 {rep["camera_max_gap"]} s')
    if steer and rep['label_coverage'] < 0.9:
        W.append(f'라벨 있는 프레임 {rep["label_coverage"] * 100:.0f}% (90% 미만)')
    if ratios and rep['steer_std'] < 0.05:
        W.append('조향이 거의 직진뿐 (커브 데이터 부족)')
    if 'lane_valid_pct' in rep and rep['lane_valid_pct'] < 80:
        W.append(f'차선 인식 유효 {rep["lane_valid_pct"]}% (자율주행 기록이면 인식 확인 필요)')
    if steer and thr and all(v == 1500 for _, v in thr):
        W.append('throttle 이 전부 1500 (옛 drive_record 중립값 → 실제 움직임 라벨 아님)')
    rep['verdict'] = 'BAD' if P else ('CAUTION' if W else 'OK')
    return rep


def to_markdown(name, rep):
    v = rep['verdict']
    icon = {'OK': '✅', 'CAUTION': '⚠️', 'BAD': '❌'}[v]
    lines = [f'# {name} — {icon} {v}', '']
    if rep['problems']:
        lines += ['**쓰지 말아야 할 이유**', ''] + [f'- {p}' for p in rep['problems']] + ['']
    if rep['warnings']:
        lines += ['**주의**', ''] + [f'- {w}' for w in rep['warnings']] + ['']
    keys = [('duration_sec', '길이 (s)'), ('distance_m', '주행 거리 (m)'), ('camera_fps', '카메라 fps'),
            ('camera_max_gap', '카메라 최대 끊김 (s)'), ('label_coverage', '라벨 있는 프레임 비율'),
            ('moving_pct', '움직인 시간 %'), ('speed_mean_moving', '평균 속도 (m/s)'),
            ('speed_max', '최고 속도 (m/s)'), ('steer_std', '조향 표준편차'),
            ('steer_left_pct', '왼쪽 조향 %'), ('steer_right_pct', '오른쪽 조향 %'),
            ('throttle_max_us', '최대 throttle (us)'), ('lane_valid_pct', '차선 인식 유효 %'),
            ('sources', '제어 주체')]
    lines += ['| 항목 | 값 |', '|---|---|']
    for k, label in keys:
        if k in rep:
            lines.append(f'| {label} | {rep[k]} |')
    lines += ['', '| 토픽 | 개수 | Hz | 최대 간격 (s) |', '|---|---|---|---|']
    for t, s in rep['topics'].items():
        lines.append(f'| `{t}` | {s["count"]} | {s["hz"]} | {s["max_gap"]} |')
    return '\n'.join(lines) + '\n'


# ---------------- rosbag 읽기 (Jetson, ROS 2 Jazzy) ----------------

def find_bag_dir(path):
    """폴더 안 metadata.yaml 위치 (run_x/ 또는 run_x/rosbag/)."""
    for cand in (path, os.path.join(path, 'rosbag')):
        if os.path.isfile(os.path.join(cand, 'metadata.yaml')):
            return cand
    raise SystemExit(f'{path} 에 metadata.yaml 이 없습니다 (rosbag 폴더를 지정하세요)')


def read_bag(bag_dir):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    storage = 'mcap' if any(f.endswith('.mcap') for f in os.listdir(bag_dir)) else 'sqlite3'
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_cls = {}
    times, values = {}, {}
    start, end = None, None
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        t = t_ns * 1e-9
        start = t if start is None else min(start, t)
        end = t if end is None else max(end, t)
        times.setdefault(topic, []).append(t)
        if topic not in VALUE_TOPICS:
            continue
        if topic not in msg_cls:
            msg_cls[topic] = get_message(types[topic])
        msg = deserialize_message(data, msg_cls[topic])
        if topic == LANE_TARGET:
            v = msg.data[0] if len(msg.data) else 0.0
        else:
            v = msg.data
        values.setdefault(topic, []).append((t, v))
    for ts in times.values():
        ts.sort()
    return times, values, start or 0.0, end or 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('path', help='bag 폴더 (metadata.yaml 이 있는 곳 또는 그 상위)')
    args = ap.parse_args()
    bag_dir = find_bag_dir(os.path.abspath(args.path))
    times, values, start, end = read_bag(bag_dir)
    rep = analyze(times, values, start, end)
    run_dir = os.path.dirname(bag_dir) if os.path.basename(bag_dir) == 'rosbag' else bag_dir
    name = os.path.basename(run_dir.rstrip('/'))
    rep['name'] = name
    rep['size_mb'] = round(sum(os.path.getsize(os.path.join(bag_dir, f))
                               for f in os.listdir(bag_dir)) / 1e6, 1)
    md = to_markdown(name, rep)
    with open(os.path.join(run_dir, 'report.md'), 'w') as f:
        f.write(md)
    with open(os.path.join(run_dir, 'report.json'), 'w') as f:
        json.dump(rep, f, ensure_ascii=False, indent=1)
    print(md)
    sys.exit({'OK': 0, 'CAUTION': 1, 'BAD': 2}[rep['verdict']])


if __name__ == '__main__':
    main()
