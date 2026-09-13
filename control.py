"""
control.py
----------
주행 판단 로직입니다. 순수 함수만 둡니다.

하드웨어(GPIO·카메라)와 시간(time.time())에 의존하지 않습니다. 현재 시각은
호출자가 인자로 넘기므로 라즈베리파이 없이 테스트할 수 있습니다.

입력은 픽셀 수 같은 숫자이고 출력은 "무엇을 할지"를 나타내는 값입니다.
모터 호출은 호출자가 합니다.

원본 lane_follow_loop() 의 판단 부분을 동작을 바꾸지 않고 옮겼습니다.
"""

from dataclasses import dataclass
from enum import Enum, auto

import config


# 동작. 조향 결정과 개루프 시퀀스가 함께 씁니다.
FORWARD = "FORWARD"
LEFT = "LEFT"
RIGHT = "RIGHT"
STOP = "STOP"


# ============================================================
# 속도 프로파일
# ============================================================

@dataclass(frozen=True)
class SpeedProfile:
    """직진·회전 속도 한 벌입니다 (PWM duty 0.0~1.0).

    "감속 표지판을 봤으니 느리게 간다"는 판단이지 구동이 아니므로
    hardware.Motor 가 아니라 여기에 둡니다. Motor 는 주어진 속도로 핀을 구동하고,
    어떤 속도를 줄지는 판단 계층이 정합니다.
    """
    forward: float
    turn: float

    def slowed_forward(self, value):
        """직진 속도만 바꾼 사본입니다. 회전 속도는 그대로 둡니다.

        STOP 표지판 재출발이 이 형태입니다. 감속 모드(SLOW)와 달리 회전은
        건드리지 않는 원본의 비대칭을 보존합니다.
        """
        return SpeedProfile(forward=value, turn=self.turn)


NORMAL_SPEED = SpeedProfile(config.SPEED_FORWARD_DEFAULT, config.SPEED_TURN_DEFAULT)
SLOW_SPEED = SpeedProfile(config.SPEED_SLOW_FORWARD, config.SPEED_SLOW_TURN)


def is_slow_mode(slow_until, now):
    """감속 창 안에 있는지 판단합니다.

    slow_until 이 None 이면 감속 중이 아닙니다. 0.0 같은 falsy 타임스탬프를
    "없음"으로 오해하지 않도록 is None 으로 봅니다.
    """
    return slow_until is not None and now <= slow_until


def profile_for(slow_until, now):
    """감속 창에서 현재 프로파일을 파생합니다.

    속도를 따로 저장하지 않고 감속 창 하나에서 파생하므로, "감속 창은 남았는데
    전속으로 달리는" 어긋난 상태가 생길 수 없습니다.

    이 모듈은 시간을 직접 읽지 않으므로 now 를 인자로 받습니다.
    """
    return SLOW_SPEED if is_slow_mode(slow_until, now) else NORMAL_SPEED


# ============================================================
# 좌우 균형
# ============================================================

def balance_ratios(left_pixels, right_pixels):
    """좌우 픽셀 수에서 비율과 편차를 구합니다.

    [원위치] lane_tracer.py 665-674

    반환: (left_ratio, right_ratio, diff)
    total 이 0 이면 비율을 0.0 으로 둡니다(0 나눗셈 방지).
    """
    total = left_pixels + right_pixels
    if total > 0:
        left_ratio = left_pixels / total
        right_ratio = right_pixels / total
    else:
        left_ratio = 0.0
        right_ratio = 0.0
    return left_ratio, right_ratio, abs(left_ratio - right_ratio)


def balance_threshold(speed_forward):
    """현재 속도에 따른 균형 임계값입니다.

    [원위치] lane_tracer.py 888-889

    주의: 현재 설정에서 speed_forward 는 0.225 또는 0.20 뿐이라
    HIGH_SPEED_THRESHOLD(0.6)를 넘지 않습니다. 즉 고속 분기에는 도달할 수
    없습니다. 동작 보존을 위해 로직은 남겨 두고, test_control.py 에 이 사실을
    고정하는 테스트가 있습니다.
    """
    if speed_forward > config.HIGH_SPEED_THRESHOLD:
        return config.HIGH_SPEED_BALANCE_THRESHOLD
    return config.BASE_BALANCE_THRESHOLD


# ============================================================
# 조향 결정
# ============================================================

@dataclass(frozen=True)
class OneSideState:
    """한쪽 차선이 사라진 상태를 추적합니다.

    차선이 사라진 직후 바로 크게 돌면 코스를 이탈하므로, 일정 시간
    (STRAIGHT_DURATION)은 직진을 유지한 뒤 강하게 돕니다.
    """
    missing_since: float | None = None
    direction: str | None = None


@dataclass(frozen=True)
class Steering:
    action: str            # FORWARD | LEFT | RIGHT
    intensity: float       # 회전 강도 0.0~1.0. FORWARD 일 때는 1.0
    one_side: OneSideState # 갱신된 상태. 호출자가 다음 프레임에 넘깁니다


def decide_steering(left_pixels, right_pixels, speed_forward, one_side, now):
    """좌우 픽셀 수로 조향을 결정합니다.

    [원위치] lane_tracer.py 887-942

    판정 순서
      1. 좌우 편차가 임계값 미만 → 직진
      2. 왼쪽이 많음 → 우회전. 오른쪽 차선이 거의 없으면 잠깐 직진 후 강하게
      3. 그 외(오른쪽이 많거나 같음) → 좌회전. 대칭

    원본은 left_pixels > right_pixels 가 아니면 모두 좌회전 분기로 갔습니다.
    좌우가 같은 경우도 좌회전이며, 이 동작을 유지합니다.
    """
    _, _, diff = balance_ratios(left_pixels, right_pixels)

    if diff < balance_threshold(speed_forward):
        return Steering(FORWARD, 1.0, OneSideState())

    intensity = min(1.0, diff / config.TURN_INTENSITY_NORMALIZER)

    if left_pixels > right_pixels:
        turn, missing_side, opposite_pixels = RIGHT, "RIGHT", right_pixels
    else:
        turn, missing_side, opposite_pixels = LEFT, "LEFT", left_pixels

    if opposite_pixels >= config.ONE_SIDE_MISSING_PIXELS:
        # 양쪽 다 보입니다 → 편차에 비례해 회전
        return Steering(turn, intensity, OneSideState())

    # 한쪽 차선이 사라졌습니다
    if one_side.missing_since is None or one_side.direction != missing_side:
        one_side = OneSideState(missing_since=now, direction=missing_side)

    if now - one_side.missing_since < config.STRAIGHT_DURATION:
        return Steering(FORWARD, 1.0, one_side)

    strong = min(1.0, intensity * config.STRONG_TURN_MULTIPLIER)
    return Steering(turn, strong, one_side)


# ============================================================
# 교차로 감지
# ============================================================

def is_intersection(center_pixels, total_pixels):
    """전방 중앙 박스가 차 있고 좌우가 비었으면 교차로로 봅니다.

    [원위치] lane_tracer.py 842

    두 조건을 함께 보는 이유는 오탐 방지입니다. 중앙만 보면 굵은 직선 구간에서도
    걸리고, 좌우만 보면 차선 이탈과 구분되지 않습니다.
    """
    return (center_pixels > config.CENTER_THRESHOLD
            and total_pixels < config.PIXEL_THRESHOLD * 2)


# ============================================================
# 후진 복구
# ============================================================

@dataclass(frozen=True)
class ReverseState:
    reverse_mode: bool = False
    high_since: float | None = None


def check_reverse(left_pixels, right_pixels, state, now):
    """차선 픽셀이 비정상적으로 많으면 후진으로 복구합니다.

    [원위치] lane_tracer.py 678-694

    벽이나 차선 뭉치에 붙으면 픽셀이 급증합니다. 순간적인 값 튐과 구분하려고
    HIGH_PIXEL_DURATION 이상 지속될 때만 후진에 들어갑니다.
    """
    over = (left_pixels > config.HIGH_PIXEL_THRESHOLD
            or right_pixels > config.HIGH_PIXEL_THRESHOLD)

    if over:
        if state.high_since is None:
            return ReverseState(state.reverse_mode, now)
        if now - state.high_since >= config.HIGH_PIXEL_DURATION:
            return ReverseState(True, state.high_since)
        return state

    # 정상 픽셀로 복귀
    if state.reverse_mode:
        return ReverseState(False, None)
    if state.high_since is not None:
        return ReverseState(state.reverse_mode, None)
    return state


# ============================================================
# 정지선 (주행 종료 판정)
# ============================================================

@dataclass(frozen=True)
class StoplineState:
    stage: int = 0                    # 0: 아직 못 봄, 1: 첫 정지선 통과
    first_seen_at: float | None = None


@dataclass(frozen=True)
class StoplineDecision:
    state: StoplineState
    should_stop: bool = False         # True 면 주행 종료
    passed_first: bool = False        # 첫 정지선을 방금 통과함


def check_stopline(blue_pixels, state, now):
    """파란색 정지선을 2단계로 판정합니다.

    [원위치] lane_tracer.py 818-840

    트랙에 정지선이 두 개 있습니다. 첫 번째는 통과하고 두 번째에서 멈춥니다.
    같은 선을 연속 프레임에서 두 번 잡지 않도록 두 감지 사이에
    STOPLINE_MIN_GAP_SEC 이상의 간격을 요구합니다.
    """
    if blue_pixels < config.BLUE_PIXEL_THRESHOLD:
        return StoplineDecision(state)

    if state.stage == 0:
        return StoplineDecision(
            StoplineState(stage=1, first_seen_at=now), passed_first=True)

    if now - state.first_seen_at < config.STOPLINE_MIN_GAP_SEC:
        # 첫 정지선과 너무 가깝습니다 → 같은 선으로 보고 무시
        return StoplineDecision(state)

    return StoplineDecision(state, should_stop=True)


# ============================================================
# 교차로 상태 기계
# ============================================================

class Intersection(Enum):
    """교차로와 관련된 차량의 상태입니다.

    원본은 이 상태를 변수 네 개(intersection_mode, intersection_wait_start,
    intersection_exit_time, vehicle_stopped + stop_reason)로 표현했습니다. 같은
    사실을 여러 곳에 두고 stop_reason == "교차로 대기" 처럼 문자열로 되묻는
    구조라, 같은 사실을 담은 변수 중 하나만 갱신되면 서로 어긋날 수 있었습니다.

    상태가 셋뿐이고 전이가 단순해서 Enum 하나로 충분합니다. 상태를 클래스로
    나누는 State 패턴까지 쓸 규모는 아닙니다.
    """
    NONE = auto()      # 교차로와 무관. 정상 주행
    WAITING = auto()   # 교차로에서 멈춰 지시(표지판 또는 키보드)를 기다림
    COOLDOWN = auto()  # 방금 통과함. 잠시 교차로·정지선 재감지를 억제


@dataclass(frozen=True)
class IntersectionState:
    """교차로 상태와 그 상태로 들어간 시각입니다.

    entered_at 의 의미는 하나입니다 — 현재 phase 로 들어간 시각. 그 시각을
    무엇과 비교하는지는 상태마다 다릅니다.
      WAITING  → INTERSECTION_TIMEOUT 과 비교 (지시를 너무 오래 기다렸나)
      COOLDOWN → INTERSECTION_EXIT_DURATION 과 비교 (억제가 끝났나)
      NONE     → 읽지 않음

    waiting_since / cooldown_since 로 나누지 않은 이유: 둘이 동시에 의미를 가질
    수 없어 항상 하나는 None 이 되고, 상태와 필드의 정합성을 따로 지켜야 하기
    때문입니다.
    """
    phase: Intersection = Intersection.NONE
    entered_at: float | None = None

    def __post_init__(self):
        """phase 와 entered_at 은 짝이 맞아야 합니다.

        NONE 이 아닌데 entered_at 이 없으면 elapsed() 가 0.0 을 돌려주고
        타임아웃 판정이 영원히 거짓이 됩니다. COOLDOWN 이면 억제가 풀리지 않아
        정지선·교차로를 감지하지 못하고, WAITING 이면 교차로에서 멈춘 채
        움직이지 않습니다. 원본에서 intersection_exit_time = None 은 "억제
        아님"이었으므로, 그대로 옮기면 의미가 정반대가 됩니다.
        """
        if (self.phase is Intersection.NONE) != (self.entered_at is None):
            raise ValueError(
                f"phase={self.phase.name} 과 entered_at={self.entered_at} 는 "
                f"짝이 맞지 않는다")

    @property
    def waiting(self):
        return self.phase is Intersection.WAITING

    @property
    def detecting(self):
        """교차로·정지선을 감지해도 되는 상태인지 판단합니다.

        COOLDOWN 중에는 방금 지나온 교차로를 다시 잡지 않도록 억제합니다.

        억제되는 것은 교차로와 정지선 감지뿐이고 라인 이탈 복구는 COOLDOWN
        중에도 동작합니다. 원본과 같은 동작이지만 "재감지 억제"라는 이름만
        보면 예상하기 어려워 적어 둡니다.

        [원위치] lane_tracer.py 의 `not intersection_exit_time` 조건
        """
        return self.phase is Intersection.NONE

    def elapsed(self, now):
        """현재 상태로 들어간 뒤 지난 시간입니다. NONE 이면 0.0 입니다."""
        return 0.0 if self.entered_at is None else now - self.entered_at


def enter_intersection(now):
    return IntersectionState(Intersection.WAITING, now)


def exit_intersection(now):
    """교차로를 통과해 재감지 억제 구간으로 들어갑니다."""
    return IntersectionState(Intersection.COOLDOWN, now)


def tick_intersection(state, now):
    """억제 구간이 끝났으면 정상 주행 상태로 돌립니다.

    [원위치] lane_tracer.py
        if intersection_exit_time:
            elapsed = time.time() - intersection_exit_time
            if elapsed >= INTERSECTION_EXIT_DURATION:
                intersection_exit_time = None
    """
    if state.phase is Intersection.COOLDOWN:
        if state.elapsed(now) >= config.INTERSECTION_EXIT_DURATION:
            return IntersectionState()
    return state


def intersection_timed_out(state, now):
    """교차로에서 지시를 기다린 시간이 한계를 넘었는지 판단합니다."""
    return state.waiting and state.elapsed(now) >= config.INTERSECTION_TIMEOUT


# ============================================================
# 프레임 분류 — 이 프레임에서 무엇에 반응할지
# ============================================================

class Branch(Enum):
    """한 프레임에서 선택되는 동작 갈래입니다.

    원본은 주행 루프 안의 if/elif 체인이었고, 순서가 곧 우선순위입니다 —
    정지선(종료)이 교차로보다, 교차로가 차선 추종보다 앞섭니다.

    체인에서는 앞 가지의 조건을 넓히면 뒤 가지가 조용히 도달 불가가 됩니다.
    함수로 분리하면 각 갈래에 실제로 도달하는지를 순수 함수 테스트로 직접
    확인할 수 있습니다. 각 갈래에 도달하는지는 tests/test_control.py 의
    test_네_갈래_모두_도달_가능하다 가 검사합니다.
    """
    STOPLINE = auto()      # 파란 정지선. 2단계 판정으로 주행을 끝냄
    INTERSECTION = auto()  # 교차로. 멈춰서 지시를 기다림
    LINE_LOST = auto()     # 차선을 놓침. 멈추고 수동 복구를 기다림
    TRACE = auto()         # 정상 차선 추종


def select_branch(measured, inter):
    """이 프레임에서 어느 갈래로 갈지 정합니다.

    [원위치] salvage/pi-final-2025-12-13:lane_tracer.py 819-881 의 if/elif 체인

    inter.detecting 은 교차로 통과 직후의 억제 구간이 아니라는 뜻입니다.
    억제되는 것은 정지선과 교차로뿐이고 차선 이탈 복구는 억제되지 않습니다.
    원본과 같은 동작입니다.
    """
    if inter.detecting and measured.blue_pixels >= config.BLUE_PIXEL_THRESHOLD:
        return Branch.STOPLINE
    if inter.detecting and is_intersection(measured.center_pixels,
                                          measured.total_pixels):
        return Branch.INTERSECTION
    if measured.total_pixels < config.PIXEL_THRESHOLD:
        return Branch.LINE_LOST
    return Branch.TRACE


# ============================================================
# 개루프 동작 시퀀스
# ============================================================

class Maneuver(Enum):
    """교차로나 차선 이탈에서 수행하는 정해진 동작입니다."""
    STRAIGHT = auto()
    LEFT = auto()
    RIGHT = auto()
    TRAFFIC_RIGHT = auto()   # 신호등: 잠깐 멈춘 뒤 우회전
    HALT = auto()
    IGNORE = auto()          # 아무것도 하지 않고 교차로를 빠져나감


# 표지판 인식 결과와 키보드 입력을 같은 Maneuver 로 모읍니다.
# 원본은 표지판을 키 문자로 바꾼 뒤 키 처리 분기를 타는 구조였습니다.
SIGN_TO_MANEUVER = {
    'go_straight': Maneuver.STRAIGHT,
    'turn_left': Maneuver.LEFT,
    'turn_right': Maneuver.RIGHT,
    'traffic': Maneuver.TRAFFIC_RIGHT,
}

KEY_TO_MANEUVER = {
    'w': Maneuver.STRAIGHT,
    'a': Maneuver.LEFT,
    'd': Maneuver.RIGHT,
    's': Maneuver.HALT,
}

# 원본은 `if user_input:` 로 truthy 판정만 하고, 어느 키에도 맞지 않으면 모터
# 명령 없이 교차로를 빠져나갔습니다. 알 수 없는 문자 하나로도 회전을 건너뛰고
# 탈출합니다.
#
# 실제로 일어날 수 있습니다. main.py 가 터미널을 raw 모드로 두지 않아 줄 단위
# 버퍼링이므로, 조작자가 'd' + Enter 를
# 치면 stdin 에 'd' 와 개행 두 문자가 들어가고, get_user_input() 은 프레임당 한
# 글자만 읽으므로 개행이 남습니다. 다음 교차로에서 그 개행을 읽어 회전 없이
# 탈출합니다.
#
# 동작 보존 원칙에 따라 고치지 않았습니다. 라즈베리파이에서 확인한 뒤 판단할
# 항목입니다.
UNKNOWN_KEY_MANEUVER = Maneuver.IGNORE

MANEUVER_NAMES = {
    Maneuver.STRAIGHT: '직진',
    Maneuver.LEFT: '좌회전',
    Maneuver.RIGHT: '우회전',
    Maneuver.TRAFFIC_RIGHT: f'신호등 ({config.TRAFFIC_STOP_SEC}초 대기 후 우회전)',
    Maneuver.HALT: '정지',
    Maneuver.IGNORE: '동작 없음 (미인식 입력)',
}


# (동작, 유지 시간)의 열입니다. 센서를 보지 않고 시간만으로 도는 개루프 방식이며,
# 값은 트랙에서 초 단위로 맞춘 것입니다.
#
# [원위치] salvage:lane_tracer.py 755-788 (교차로), 860-878 (차선 이탈)
#
# 교차로와 차선 이탈의 진입 시간이 다릅니다. 원본 주석에는 "교차로와 동일"이라고
# 적혀 있지만 값은 갈라져 있습니다.
#
#            진입        회전        복귀
#   교차로 좌  4.0        5.0        0.8
#   이탈  좌  5.0        5.0        0.8
#   교차로 우  1.0        5.0        0.8
#   이탈  우  4.0        5.0        0.8
#   신호등 우  4.0        5.0        0.8   (앞에 3초 정지)

INTERSECTION_STEPS = {
    Maneuver.STRAIGHT: (
        (FORWARD, config.TURN_STRAIGHT_FORWARD_SEC),
    ),
    Maneuver.LEFT: (
        (FORWARD, config.TURN_LEFT_ENTER_SEC),
        (LEFT, config.TURN_LEFT_ROTATE_SEC),
        (FORWARD, config.TURN_LEFT_EXIT_SEC),
    ),
    Maneuver.RIGHT: (
        (FORWARD, config.TURN_RIGHT_ENTER_SEC),
        (RIGHT, config.TURN_RIGHT_ROTATE_SEC),
        (FORWARD, config.TURN_RIGHT_EXIT_SEC),
    ),
    # 원본은 신호등을 별개 분기로 두고 우회전 코드를 복사했습니다.
    # 앞에 정지를 붙인 한 항목으로 표현하면 특수 분기가 필요 없습니다.
    Maneuver.TRAFFIC_RIGHT: (
        (STOP, config.TRAFFIC_STOP_SEC),
        (FORWARD, config.TRAFFIC_RIGHT_ENTER_SEC),
        (RIGHT, config.TRAFFIC_RIGHT_ROTATE_SEC),
        (FORWARD, config.TRAFFIC_RIGHT_EXIT_SEC),
    ),
    # 정지는 시퀀스가 아니지만, 호출부에 특수 분기를 만들지 않으려고 0초짜리
    # 한 단계로 둡니다.
    Maneuver.HALT: (
        (STOP, 0),
    ),
    # 빈 열. 모터를 건드리지 않고 교차로만 빠져나갑니다.
    Maneuver.IGNORE: (),
}

LINELOST_STEPS = {
    Maneuver.STRAIGHT: (
        (FORWARD, config.LINELOST_STRAIGHT_FORWARD_SEC),
    ),
    Maneuver.LEFT: (
        (FORWARD, config.LINELOST_LEFT_ENTER_SEC),
        (LEFT, config.LINELOST_LEFT_ROTATE_SEC),
        (FORWARD, config.LINELOST_LEFT_EXIT_SEC),
    ),
    Maneuver.RIGHT: (
        (FORWARD, config.LINELOST_RIGHT_ENTER_SEC),
        (RIGHT, config.LINELOST_RIGHT_ROTATE_SEC),
        (FORWARD, config.LINELOST_RIGHT_EXIT_SEC),
    ),
    Maneuver.HALT: (
        (STOP, 0),
    ),
    Maneuver.IGNORE: (),
}

# 교차로에서 지시가 오지 않아 타임아웃된 경우입니다. 잠깐 직진해서 빠져나가며,
# 교차로 STRAIGHT(6초)보다 훨씬 짧습니다.
TIMEOUT_STEPS = (
    (FORWARD, config.INTERSECTION_TIMEOUT_FORWARD_SEC),
)


class Source(Enum):
    """지시가 어디서 왔는지 나타냅니다. 로그 문구와 쓸 시퀀스 표가 달라집니다.

    문자열 상수로 두면 source == "timeuot" 같은 오타가 조용히 False 가 되고,
    steps_for 가 타임아웃용 0.5초 대신 교차로 직진 6초를 돌려줍니다.
    """
    SIGN = auto()
    KEY = auto()
    TIMEOUT = auto()


FROM_SIGN = Source.SIGN
FROM_KEY = Source.KEY
FROM_TIMEOUT = Source.TIMEOUT


def plan_intersection(state, sign_type, key, now):
    """교차로에서 무엇을 할지 정합니다. 부작용이 없습니다.

    [원위치] salvage/pi-final-2025-12-13:lane_tracer.py 705-807

    반환: (Maneuver 또는 None, 출처 또는 None)

    우선순위는 표지판 > 타임아웃 > 키보드입니다. 원본이

        if not user_input and timed_out(...):   # user_input 은 표지판에서만 옴
            ... 자동 직진 ...
            continue
        if not user_input:
            user_input = get_user_input()       # 타임아웃이면 여기 오지 않음

    구조였기 때문입니다. 타임아웃이 성립하면 키보드를 읽지 않습니다. 사람의 키
    입력보다 타임아웃이 앞서는 것은 어색하지만 동작을 보존합니다.

    표지판 큐에서 꺼내는 일(popleft)은 호출자가 합니다. 여기서 하면 순수 함수가
    아니게 되므로, 이 함수는 꺼낸 결과인 sign_type 문자열만 받습니다.
    """
    if not state.waiting:
        return None, None
    if sign_type in SIGN_TO_MANEUVER:
        return SIGN_TO_MANEUVER[sign_type], FROM_SIGN
    if intersection_timed_out(state, now):
        return Maneuver.STRAIGHT, FROM_TIMEOUT
    if key:
        # truthy 판정만 합니다. 원본의 `if user_input:` 과 같습니다.
        # 매핑에 없는 문자는 IGNORE — 동작 없이 탈출합니다.
        return KEY_TO_MANEUVER.get(key, UNKNOWN_KEY_MANEUVER), FROM_KEY
    return None, None


def steps_for(maneuver, source):
    """지시에 해당하는 (동작, 초) 열을 돌려줍니다.

    타임아웃 직진은 교차로 직진(6초)이 아니라 짧은 탈출(0.5초)입니다.
    원본에서 두 경로의 시간이 달랐습니다.
    """
    if source is FROM_TIMEOUT:
        # 타임아웃은 항상 직진입니다. 다른 지시와 조합되면 maneuver 가 조용히
        # 무시되므로 여기서 막습니다.
        assert maneuver is Maneuver.STRAIGHT, f"타임아웃에 {maneuver} 가 왔다"
        return TIMEOUT_STEPS
    return INTERSECTION_STEPS[maneuver]
