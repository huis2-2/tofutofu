"""전체 실행: 센서 → YOLO → 차선/제어 → 미션 → mux → drive (+ rosbag 기록).

  ros2 launch autorace_bringup autorace.launch.py                 # 대회 주행
  ros2 launch autorace_bringup autorace.launch.py record:=true    # 주행하면서 rosbag (모방학습용)
  ros2 launch autorace_bringup autorace.launch.py drive:=false    # 바퀴 출력 없이 인식만 확인
  ros2 launch autorace_bringup autorace.launch.py missions:=false # 차선 주행만 (속도 튜닝)

각 패키지 설정은 <패키지>/config/*.yaml. 이 launch 는 그 파일들을 그대로 쓴다.
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def cfg(pkg, name):
    return os.path.join(get_package_share_directory(pkg), 'config', name)


RECORD_TOPICS = [
    '/camera/image_raw/compressed',
    '/drive/cmd', '/drive/steering_us', '/drive/throttle_us', '/drive/steering',
    '/encoder/speed', '/encoder/distance', '/encoder/count',
    '/scan',
    '/yolo/detections', '/yolo/signal', '/yolo/LR_sign',
    '/lane/target', '/lane/path', '/lane/zigzag',
    '/mission/lane', '/mux/active_source',
    '/crosswalk/state', '/signal/state', '/parking/state',
    '/slow/active', '/objavoid/active', '/cone/active',
]


def generate_launch_description():
    record = LaunchConfiguration('record')
    drive = LaunchConfiguration('drive')
    missions = LaunchConfiguration('missions')
    lidar = LaunchConfiguration('lidar')
    debug = LaunchConfiguration('debug')
    bag_dir = LaunchConfiguration('bag_dir')
    sensors = cfg('autorace_bringup', 'sensors.yaml')

    def node(pkg, exe, config, condition=None, extra=None):
        params = [config] + ([extra] if extra else [])
        return Node(package=pkg, executable=exe, name=exe, output='screen',
                    parameters=params, condition=condition)

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    return LaunchDescription([
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('drive', default_value='true'),
        DeclareLaunchArgument('missions', default_value='true'),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('debug', default_value='false',
                              description='YOLO/lane 디버그 영상 발행'),
        DeclareLaunchArgument('bag_dir', default_value=os.path.expanduser(
            f'~/tofutofu/datasets/bags/run_{stamp}')),

        # sensor_ws
        node('camera_node', 'camera_node', sensors,
             extra={'publish_compressed': ParameterValue(record, value_type=bool)}),
        node('encoder_node', 'encoder_node', sensors),
        node('sllidar_ros2', 'sllidar_node', sensors, IfCondition(lidar)),

        # yolo_ws, lane_ws
        node('autorace_yolo', 'yolo_node', cfg('autorace_yolo', 'yolo.yaml'),
             extra={'publish_debug': ParameterValue(debug, value_type=bool)}),
        node('autorace_lane', 'lane_node', cfg('autorace_lane', 'lane.yaml'),
             extra={'publish_debug': ParameterValue(debug, value_type=bool)}),
        node('autorace_lane', 'controller_node', cfg('autorace_lane', 'lane.yaml')),

        # 미션 ws
        node('autorace_crosswalk', 'crosswalk_node', cfg('autorace_crosswalk', 'crosswalk.yaml'),
             IfCondition(missions)),
        node('autorace_signal', 'signal_node', cfg('autorace_signal', 'signal.yaml'),
             IfCondition(missions)),
        node('autorace_parking', 'parking_node', cfg('autorace_parking', 'parking.yaml'),
             IfCondition(missions)),
        node('autorace_slow', 'slow_node', cfg('autorace_slow', 'slow.yaml'),
             IfCondition(missions)),
        node('autorace_objavoid', 'objavoid_node', cfg('autorace_objavoid', 'objavoid.yaml'),
             IfCondition(PythonExpression(["'", missions, "' == 'true' and '", lidar, "' == 'true'"]))),
        node('autorace_cone', 'cone_node', cfg('autorace_cone', 'cone.yaml'),
             IfCondition(PythonExpression(["'", missions, "' == 'true' and '", lidar, "' == 'true'"]))),

        # mux_ws, drive_ws
        node('autorace_mux', 'mux_node', cfg('autorace_mux', 'mux.yaml')),
        node('autorace_drive', 'drive_node', cfg('autorace_drive', 'drive.yaml'), IfCondition(drive)),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', bag_dir] + RECORD_TOPICS,
            output='screen', condition=IfCondition(record)),
    ])
