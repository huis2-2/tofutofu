#!/usr/bin/env python3
"""Extract labeling images from a drive_record.py rosbag.

  source /opt/ros/jazzy/setup.bash
  python3 bag_to_images.py ~/datasets/session_YYYYmmdd_HHMMSS
  python3 bag_to_images.py ~/datasets/session_*          (several sessions)

For each session the camera (and BEV) frames are written at --fps per second
into <session>/extracted/images and extracted/bev, with extracted/labels.csv
(same columns as drive_record.py: filename, bev_filename, stamp, steering_us,
throttle_us, steering, source). The drive command is the latest one recorded
at or before the frame.
"""

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np

CAMERA_TOPIC = '/camera/image_raw/compressed'
BEV_TOPIC = '/bev/image_raw/compressed'
COMMAND_TOPICS = ('/drive/steering_us', '/drive/throttle_us', '/drive/steering',
                  '/drive/source')


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def extract(session, fps, out_name):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import Float32, Int32, String

    types = {CAMERA_TOPIC: CompressedImage, BEV_TOPIC: CompressedImage,
             '/drive/steering_us': Int32, '/drive/throttle_us': Int32,
             '/drive/steering': Float32, '/drive/source': String}
    bag = session / 'rosbag'
    if not bag.is_dir():
        print(f'건너뜀(rosbag 없음): {session}')
        return
    out = session / out_name
    if out.exists():
        print(f'건너뜀(이미 있음, 지우고 다시 실행): {out}')
        return

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))

    (out / 'images').mkdir(parents=True)
    with open(out / 'labels.csv', 'w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['filename', 'bev_filename', 'stamp', 'steering_us',
                         'throttle_us', 'steering', 'source'])
        command = {'/drive/steering_us': '', '/drive/throttle_us': '',
                   '/drive/steering': '', '/drive/source': ''}
        interval_ns = int(1e9 / fps)
        last_saved = None
        pending_bev = {}  # stamp -> name, BEV arrives right after its camera frame
        count = 0
        while reader.has_next():
            topic, data, _ = reader.read_next()
            msg = deserialize_message(data, types[topic])
            if topic in command:
                command[topic] = msg.data
            elif topic == CAMERA_TOPIC:
                stamp = stamp_ns(msg)
                if last_saved is not None and stamp - last_saved < interval_ns:
                    continue
                image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                name = f'{count:06d}.jpg'
                cv2.imwrite(str(out / 'images' / name), image)
                pending_bev[stamp] = name
                steering = command['/drive/steering']
                writer.writerow([f'images/{name}', f'bev/{name}',
                                 f'{stamp // 1_000_000_000}.{stamp % 1_000_000_000:09d}',
                                 command['/drive/steering_us'],
                                 command['/drive/throttle_us'],
                                 f'{steering:.3f}' if steering != '' else '',
                                 command['/drive/source']])
                last_saved = stamp
                count += 1
            elif topic == BEV_TOPIC:
                name = pending_bev.pop(stamp_ns(msg), None)
                if name is not None:
                    (out / 'bev').mkdir(exist_ok=True)
                    image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                    cv2.imwrite(str(out / 'bev' / name), image)
    print(f'{session.name}: {count}장 -> {out}')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('sessions', nargs='+', type=Path,
                        help='drive_record.py 세션 폴더(session_*)')
    parser.add_argument('--fps', type=float, default=2.0, help='초당 추출 장수(기본 2)')
    parser.add_argument('--out-name', default='extracted',
                        help='세션 안에 만들 폴더 이름(기본 extracted)')
    args = parser.parse_args()
    try:
        import rosbag2_py  # noqa: F401
    except ImportError:
        raise SystemExit('"source /opt/ros/jazzy/setup.bash" 후 실행하세요.')
    for session in args.sessions:
        extract(session.expanduser(), args.fps, args.out_name)


if __name__ == '__main__':
    main()
