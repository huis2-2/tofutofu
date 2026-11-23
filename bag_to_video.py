#!/usr/bin/env python3
"""Write the camera (and BEV) stream of a drive_record.py rosbag as mp4.

  source /opt/ros/jazzy/setup.bash
  python3 bag_to_video.py ~/datasets/session_YYYYmmdd_HHMMSS [more sessions]

Creates <session>/<session>_camera.mp4 and <session>_bev.mp4 (H.264).
The frame rate is the average rate the frames were recorded at.
"""

import argparse
from pathlib import Path

import cv2
import numpy as np

TOPICS = {'/camera/image_raw/compressed': 'camera',
          '/bev/image_raw/compressed': 'bev'}


def open_reader(bag, topic):
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    return reader


def write_video(session, topic, name):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage

    bag = session / 'rosbag'
    out = session / f'{session.name}_{name}.mp4'
    if out.exists():
        print(f'건너뜀(이미 있음): {out}')
        return

    # First pass: frame count and duration give the average frame rate.
    reader = open_reader(bag, topic)
    stamps = []
    while reader.has_next():
        _, _, t = reader.read_next()
        stamps.append(t)
    if len(stamps) < 2:
        print(f'건너뜀({topic} 프레임 없음): {session}')
        return
    fps = (len(stamps) - 1) / ((stamps[-1] - stamps[0]) / 1e9)

    reader = open_reader(bag, topic)
    writer = None
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, CompressedImage)
        image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if writer is None:
            h, w = image.shape[:2]
            writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*'avc1'), fps, (w, h))
            if not writer.isOpened():
                raise SystemExit(f'mp4를 만들 수 없습니다: {out}')
        writer.write(image)
    writer.release()
    print(f'{out.name}: {len(stamps)}프레임, {fps:.1f} fps, '
          f'{out.stat().st_size / 1e6:.0f} MB')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('sessions', nargs='+', type=Path,
                        help='drive_record.py 세션 폴더(session_*)')
    parser.add_argument('--no-bev', action='store_true', help='BEV 영상은 만들지 않음')
    args = parser.parse_args()
    try:
        import rosbag2_py  # noqa: F401
    except ImportError:
        raise SystemExit('"source /opt/ros/jazzy/setup.bash" 후 실행하세요.')
    for session in args.sessions:
        session = session.expanduser()
        if not (session / 'rosbag').is_dir():
            print(f'건너뜀(rosbag 없음): {session}')
            continue
        for topic, name in TOPICS.items():
            if name == 'bev' and args.no_bev:
                continue
            write_video(session, topic, name)


if __name__ == '__main__':
    main()
