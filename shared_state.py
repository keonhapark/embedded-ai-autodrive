"""
shared_state.py
---------------
lane_tracer(주행 스레드)와 object_detector(추론 스레드)가 공유하는 상태입니다.
모든 읽기·쓰기는 lock 안에서 합니다.

모든 필드를 이 파일에 선언합니다. 런타임에 필드를 새로 만들면 이 파일만 봐서는
그 상태가 있는지 알 수 없습니다.
"""

from threading import Lock

import config

lock = Lock()

# ============================================================
# 객체 감지 상태 (object_detector 가 쓰고 lane_tracer·main 이 읽습니다)
# ============================================================

# 인식 가능한 객체 목록
KNOWN_OBJECTS = config.KNOWN_OBJECTS

object_state = {name: False for name in KNOWN_OBJECTS}   # 현재 감지 여부
object_area = {name: 0 for name in KNOWN_OBJECTS}        # 최근 감지 면적
object_last_seen = {name: 0.0 for name in KNOWN_OBJECTS} # 마지막 감지 시각
confidence = {name: 0.0 for name in KNOWN_OBJECTS}       # 신뢰도 (0.0 ~ 1.0)

# main.py 의 모니터 출력용
object_detected = None        # 가장 최근 감지된 객체 이름
object_distance = 0           # 해당 객체의 감지 면적 (근사 거리)

# ============================================================
# 동작 중복 실행 방지용
# ============================================================

action_last_time = {}         # 각 객체별 마지막 동작 시간
ACTION_COOLDOWN = config.ACTION_COOLDOWN

# ============================================================
# 감속 모드
# ============================================================
# "감속 중인가"는 slow_mode_until 에서 파생되는 사실이므로 별도 플래그를 두지
# 않습니다. 두 필드로 나누면 한쪽만 갱신됐을 때 둘이 어긋날 수 있습니다.

slow_mode_until = None        # 감속 종료 시각. None 이면 감속 중이 아닙니다


# ============================================================
# 프레임 공유용 (lane_tracer 가 쓰고 object_detector 가 읽습니다)
# ============================================================

latest_frame = None
