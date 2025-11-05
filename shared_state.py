"""
shared_state.py
---------------

lane_tracer와 object_detector 간의 공유 상태
Lock을 이용해 thread-safe하게 접근 가능
"""

from threading import Lock

lock = Lock()

# 현재 인식된 객체

object_detected = None

# 바운딩박스 면적 (거리 판단용)

object_distance = None

# 최근 이벤트 트리거 (근접 후 수행)

last_trigger = None

# ===============================
# 신호등 관련 상태
# ===============================

traffic_light = None             # 'red' or 'green' or None
traffic_light_visible = False    # 초록불이 실제로 프레임에 보이는지
traffic_light_area = 0           # bbox 면적 (거리 추정)
traffic_light_last_ts = 0.0      # 마지막 업데이트 시각


# lane_tracer → object_detector 프레임 공유용
latest_frame = None