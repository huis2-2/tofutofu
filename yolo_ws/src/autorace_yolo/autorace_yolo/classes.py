"""Roboflow 클래스 이름 → 내부 이름 / 차선 마스크 값.

Roboflow 이름의 대소문자·하이픈·공백 차이는 normalize() 로 흡수한다.
모델 클래스가 바뀌어도 여기 표만 고치면 된다.
"""

# /yolo/lane_mask (mono8) 픽셀 값. 0 = 배경
LANE_IDS = {
    'yellow_line': 1,      # 노란 실선: 넘으면 안 됨
    'dashed_line': 2,      # 점선: 차선 변경 가능
    'horizontal_line': 3,  # 미션 시작선 / 정지선
    'zigzag_line': 4,      # 노란선인데 지그재그 (서행 유도선)
    'cross_walk': 5,       # 횡단보도
    'parking_line': 6,     # 주차 구획선
    'diamond': 7,          # 서행 마름모
}

# 겹칠 때 나중에 그린 것이 남는다 → 중요한 경계선(노란선)을 마지막에 그림
LANE_DRAW_ORDER = ['parking_line', 'cross_walk', 'diamond', 'horizontal_line',
                   'dashed_line', 'zigzag_line', 'yellow_line']

# 박스로 쓰는 클래스 (표지판·신호등·장애물)
SIGN_NAMES = ['slow', 'go_a', 'go_b', 'parking', 'red_light', 'yellow_light',
              'green_light', 'cone', 'fixed_obstacle', 'no_speed_limit']

ALIASES = {
    'crosswalk': 'cross_walk',
    'slow_sign': 'slow',
    'a': 'go_a',
    'b': 'go_b',
    'parking_sign': 'parking',
    'red': 'red_light',
    'yellow': 'yellow_light',
    'green': 'green_light',
    'obstacle': 'fixed_obstacle',
    'zigzag': 'zigzag_line',
    'dashed': 'dashed_line',
}


def normalize(name):
    key = str(name).strip().lower().replace('-', '_').replace(' ', '_')
    return ALIASES.get(key, key)
