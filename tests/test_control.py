"""
판단 계층(control.py)의 현재 동작을 고정하는 테스트입니다.

목적은 "이렇게 동작해야 한다"가 아니라 **"지금 이렇게 동작한다"** 를 못 박는
것입니다. 구조를 바꾸는 동안 동작이 변하지 않았는지 이 테스트로 확인합니다.
라즈베리파이가 없어 실행 검증을 할 수 없으므로 주된 안전망입니다.

이상해 보이는 동작(도달할 수 없는 분기, 좌우 동수일 때 좌회전 등)도 현재
동작이므로 그대로 고정합니다. 바꿀지는 하드웨어에서 확인한 뒤 판단합니다.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest

import config
import control
from control import FORWARD, LEFT, RIGHT, OneSideState, ReverseState, StoplineState


# ============================================================
# balance_ratios
# ============================================================

def test_좌우가_같으면_편차가_0():
    left, right, diff = control.balance_ratios(1000, 1000)
    assert (left, right) == (0.5, 0.5)
    assert diff == 0.0


def test_한쪽만_있으면_편차가_1():
    _, _, diff = control.balance_ratios(1000, 0)
    assert diff == 1.0


def test_픽셀이_없으면_0나눗셈_없이_0을_반환():
    assert control.balance_ratios(0, 0) == (0.0, 0.0, 0.0)


def test_편차는_부호와_무관하다():
    _, _, a = control.balance_ratios(700, 300)
    _, _, b = control.balance_ratios(300, 700)
    assert a == b
    assert a == pytest.approx(0.4)


# ============================================================
# balance_threshold — 도달할 수 없는 분기를 기록합니다
# ============================================================

def test_설정된_속도에서는_항상_기본_임계값이_쓰인다():
    """고속 분기는 현재 설정에서 도달할 수 없습니다.

    직진 속도는 SPEED_FORWARD_DEFAULT(0.225) 와 SPEED_SLOW_FORWARD(0.20) 뿐인데
    둘 다 HIGH_SPEED_THRESHOLD(0.6) 미만입니다.
    """
    for speed in (config.SPEED_FORWARD_DEFAULT, config.SPEED_SLOW_FORWARD):
        assert speed <= config.HIGH_SPEED_THRESHOLD
        assert control.balance_threshold(speed) == config.BASE_BALANCE_THRESHOLD


def test_기준을_넘기면_고속_임계값이_쓰인다():
    """분기 자체는 살아 있고, 현재 설정에서 도달하지 못할 뿐입니다."""
    over = config.HIGH_SPEED_THRESHOLD + 0.01
    assert control.balance_threshold(over) == config.HIGH_SPEED_BALANCE_THRESHOLD


# ============================================================
# decide_steering
# ============================================================

def test_균형이_잡히면_직진하고_한쪽없음_상태를_지운다():
    dirty = OneSideState(missing_since=100.0, direction="RIGHT")
    s = control.decide_steering(1000, 1000, config.SPEED_FORWARD_DEFAULT, dirty, now=200.0)
    assert s.action == FORWARD
    assert s.intensity == 1.0
    assert s.one_side == OneSideState()


def test_왼쪽이_많으면_우회전한다():
    # 편차 0.4 > 0.35 이고 양쪽 다 50 이상
    s = control.decide_steering(700, 300, config.SPEED_FORWARD_DEFAULT, OneSideState(), now=0.0)
    assert s.action == RIGHT


def test_오른쪽이_많으면_좌회전한다():
    s = control.decide_steering(300, 700, config.SPEED_FORWARD_DEFAULT, OneSideState(), now=0.0)
    assert s.action == LEFT


def test_회전_강도는_편차에_비례한다():
    # diff 0.4 / 0.5 = 0.8
    s = control.decide_steering(700, 300, config.SPEED_FORWARD_DEFAULT, OneSideState(), now=0.0)
    assert s.intensity == pytest.approx(0.8)


def test_회전_강도는_1을_넘지_않는다():
    # diff 1.0 이면 반대쪽이 0 이라 한쪽없음 경로로 갑니다.
    # 양쪽이 보이면서 편차가 큰 경우: 950 vs 50 → diff 0.9 → 1.8 → 1.0 로 제한
    s = control.decide_steering(950, 50, config.SPEED_FORWARD_DEFAULT, OneSideState(), now=0.0)
    assert s.intensity == 1.0


def test_좌우가_같지만_편차가_임계값_이상인_경우는_생기지_않는다():
    """좌우 동수면 편차가 0 이라 항상 직진입니다. 좌회전 분기로 갈 수 없습니다."""
    s = control.decide_steering(500, 500, config.SPEED_FORWARD_DEFAULT, OneSideState(), now=0.0)
    assert s.action == FORWARD


# --- 한쪽 차선이 사라진 경우 ---

def test_오른쪽차선이_사라지면_처음에는_직진을_유지한다():
    s = control.decide_steering(1000, 10, config.SPEED_FORWARD_DEFAULT, OneSideState(), now=50.0)
    assert s.action == FORWARD
    assert s.one_side == OneSideState(missing_since=50.0, direction="RIGHT")


def test_직진유지_시간이_지나면_강하게_회전한다():
    """강한 회전 배율(STRONG_TURN_MULTIPLIER)이 실제로 곱해지는지 봅니다.

    편차가 큰 입력에서는 기본 강도가 이미 1.0 으로 포화돼, 배율을 곱해도 다시
    1.0 으로 잘려 배율이 드러나지 않습니다. 포화되지 않는 (84, 40) 을 씁니다.
    편차 0.3548 → 기본 강도 0.7097 → 배율 적용 후 0.8871 입니다.
    """
    st = OneSideState(missing_since=50.0, direction="RIGHT")
    now = 50.0 + config.STRAIGHT_DURATION + 0.01
    s = control.decide_steering(84, 40, config.SPEED_FORWARD_DEFAULT, st, now=now)
    assert s.action == RIGHT

    base = min(1.0, (84 - 40) / (84 + 40) / config.TURN_INTENSITY_NORMALIZER)
    assert s.intensity == pytest.approx(base * config.STRONG_TURN_MULTIPLIER)
    assert s.intensity > base, "강한 회전 배율이 적용되지 않았다"


def test_직진유지_경계에서는_이미_회전이다():
    """경과 == STRAIGHT_DURATION 이면 회전입니다.

    경계를 비켜 간 시각만 쓰면 `<` 가 `<=` 로 바뀌어도 잡히지 않습니다.
    """
    st = OneSideState(missing_since=50.0, direction="RIGHT")
    s = control.decide_steering(84, 40, config.SPEED_FORWARD_DEFAULT, st,
                                now=50.0 + config.STRAIGHT_DURATION)
    assert s.action == RIGHT, "경계에서 아직 직진 유지로 판정했다"


def test_직진유지_중에는_타이머를_다시_시작하지_않는다():
    st = OneSideState(missing_since=50.0, direction="RIGHT")
    s = control.decide_steering(1000, 10, config.SPEED_FORWARD_DEFAULT, st, now=50.2)
    assert s.action == FORWARD
    assert s.one_side.missing_since == 50.0   # 유지


def test_사라진_방향이_바뀌면_타이머를_다시_시작한다():
    st = OneSideState(missing_since=50.0, direction="RIGHT")
    s = control.decide_steering(10, 1000, config.SPEED_FORWARD_DEFAULT, st, now=99.0)
    assert s.one_side == OneSideState(missing_since=99.0, direction="LEFT")
    assert s.action == FORWARD


def test_양쪽이_다시_보이면_한쪽없음_상태를_지운다():
    st = OneSideState(missing_since=50.0, direction="RIGHT")
    s = control.decide_steering(700, 300, config.SPEED_FORWARD_DEFAULT, st, now=99.0)
    assert s.action == RIGHT
    assert s.one_side == OneSideState()


def test_한쪽없음_판정_경계값():
    """ONE_SIDE_MISSING_PIXELS 미만일 때만 한쪽없음으로 봅니다."""
    n = config.ONE_SIDE_MISSING_PIXELS
    at = control.decide_steering(1000, n, config.SPEED_FORWARD_DEFAULT, OneSideState(), now=0.0)
    assert at.one_side == OneSideState()          # 경계값은 보이는 것으로 취급
    below = control.decide_steering(1000, n - 1, config.SPEED_FORWARD_DEFAULT, OneSideState(), now=0.0)
    assert below.one_side.direction == "RIGHT"


# ============================================================
# is_intersection
# ============================================================

def test_중앙이_차고_좌우가_비면_교차로():
    assert control.is_intersection(
        center_pixels=config.CENTER_THRESHOLD + 1,
        total_pixels=config.PIXEL_THRESHOLD * 2 - 1)


def test_중앙만_차도_좌우가_많으면_교차로가_아니다():
    assert not control.is_intersection(
        center_pixels=config.CENTER_THRESHOLD + 1,
        total_pixels=config.PIXEL_THRESHOLD * 2)


def test_중앙이_비면_교차로가_아니다():
    assert not control.is_intersection(
        center_pixels=config.CENTER_THRESHOLD, total_pixels=0)


# ============================================================
# check_reverse
# ============================================================

def test_픽셀이_급증하면_타이머를_시작한다():
    st = control.check_reverse(config.HIGH_PIXEL_THRESHOLD + 1, 0, ReverseState(), now=10.0)
    assert st.high_since == 10.0
    assert st.reverse_mode is False      # 아직 후진하지 않습니다


def test_지속시간을_넘기면_후진에_들어간다():
    st = ReverseState(reverse_mode=False, high_since=10.0)
    now = 10.0 + config.HIGH_PIXEL_DURATION
    out = control.check_reverse(config.HIGH_PIXEL_THRESHOLD + 1, 0, st, now=now)
    assert out.reverse_mode is True


def test_지속시간_전에는_후진하지_않는다():
    st = ReverseState(reverse_mode=False, high_since=10.0)
    out = control.check_reverse(config.HIGH_PIXEL_THRESHOLD + 1, 0, st, now=10.1)
    assert out.reverse_mode is False
    assert out.high_since == 10.0


def test_정상복귀하면_후진을_해제한다():
    st = ReverseState(reverse_mode=True, high_since=10.0)
    out = control.check_reverse(0, 0, st, now=20.0)
    assert out == ReverseState(False, None)


def test_지속시간_전에_정상복귀하면_타이머만_지운다():
    st = ReverseState(reverse_mode=False, high_since=10.0)
    out = control.check_reverse(0, 0, st, now=10.2)
    assert out == ReverseState(False, None)


def test_한쪽만_급증해도_감지한다():
    a = control.check_reverse(config.HIGH_PIXEL_THRESHOLD + 1, 0, ReverseState(), now=1.0)
    b = control.check_reverse(0, config.HIGH_PIXEL_THRESHOLD + 1, ReverseState(), now=1.0)
    assert a.high_since == b.high_since == 1.0


def test_경계값은_급증으로_보지_않는다():
    out = control.check_reverse(config.HIGH_PIXEL_THRESHOLD, 0, ReverseState(), now=1.0)
    assert out.high_since is None


# ============================================================
# check_stopline
# ============================================================

def test_임계값_미달이면_아무_일도_없다():
    d = control.check_stopline(config.BLUE_PIXEL_THRESHOLD - 1, StoplineState(), now=5.0)
    assert d.should_stop is False
    assert d.state == StoplineState()


def test_첫_정지선은_통과한다():
    d = control.check_stopline(config.BLUE_PIXEL_THRESHOLD, StoplineState(), now=5.0)
    assert d.should_stop is False
    assert d.passed_first is True
    assert d.state == StoplineState(stage=1, first_seen_at=5.0)


def test_두번째_정지선에서_멈춘다():
    st = StoplineState(stage=1, first_seen_at=5.0)
    # 5.0 기준으로 경계를 만들면 5.3 - 5.0 = 0.2999... 가 되어 판정이 뒤집힙니다.
    # 여기서는 확실히 지난 시점을 쓰고, 경계는
    # test_정지선_간격_경계는_두_번째_정지선으로_본다 가 0.0 기준으로 검사합니다.
    now = 5.0 + config.STOPLINE_MIN_GAP_SEC * 2
    d = control.check_stopline(config.BLUE_PIXEL_THRESHOLD, st, now=now)
    assert d.should_stop is True


def test_첫_정지선과_너무_가까우면_같은_선으로_보고_무시한다():
    st = StoplineState(stage=1, first_seen_at=5.0)
    d = control.check_stopline(config.BLUE_PIXEL_THRESHOLD, st, now=5.1)
    assert d.should_stop is False
    assert d.state == st


def test_정지선_시나리오_전체():
    """한 번의 주행에서 일어나는 순서를 그대로 따라갑니다."""
    st = StoplineState()

    # 정지선 없는 구간
    d = control.check_stopline(0, st, now=0.0)
    assert not d.should_stop
    st = d.state

    # 첫 정지선
    d = control.check_stopline(30000, st, now=10.0)
    assert not d.should_stop and d.passed_first
    st = d.state

    # 같은 선을 다음 프레임에서 또 봄 → 무시
    d = control.check_stopline(30000, st, now=10.05)
    assert not d.should_stop
    st = d.state

    # 한참 뒤 두 번째 정지선 → 종료
    d = control.check_stopline(30000, st, now=25.0)
    assert d.should_stop


# ============================================================
# 교차로 상태 기계
# ============================================================

from control import Intersection, IntersectionState


def test_처음에는_정상_주행_상태다():
    st = IntersectionState()
    assert st.phase is Intersection.NONE
    assert st.detecting is True
    assert st.waiting is False
    assert st.elapsed(100.0) == 0.0


def test_교차로에_진입하면_대기_상태가_된다():
    st = control.enter_intersection(now=10.0)
    assert st.phase is Intersection.WAITING
    assert st.waiting is True
    assert st.detecting is False, "대기 중에는 교차로를 다시 감지하지 않는다"
    assert st.elapsed(12.5) == 2.5


def test_통과하면_억제_상태가_된다():
    st = control.exit_intersection(now=20.0)
    assert st.phase is Intersection.COOLDOWN
    assert st.waiting is False
    assert st.detecting is False, "방금 지나온 교차로를 다시 잡으면 안 된다"


def test_억제_시간이_지나면_정상_주행으로_돌아간다():
    st = control.exit_intersection(now=20.0)
    later = 20.0 + config.INTERSECTION_EXIT_DURATION
    assert control.tick_intersection(st, later) == IntersectionState()


def test_억제_시간_전에는_그대로다():
    st = control.exit_intersection(now=20.0)
    assert control.tick_intersection(st, 20.1) == st


def test_tick_은_대기_상태를_건드리지_않는다():
    """대기는 시간이 아니라 지시(표지판·키보드)로 끝납니다."""
    st = control.enter_intersection(now=10.0)
    assert control.tick_intersection(st, 10.0 + 999) == st


def test_대기가_너무_길면_타임아웃이다():
    st = control.enter_intersection(now=10.0)
    assert not control.intersection_timed_out(st, 10.0)
    assert not control.intersection_timed_out(st, 10.0 + config.INTERSECTION_TIMEOUT - 0.1)
    assert control.intersection_timed_out(st, 10.0 + config.INTERSECTION_TIMEOUT)


def test_대기_상태가_아니면_타임아웃이_아니다():
    assert not control.intersection_timed_out(IntersectionState(), 999.0)
    assert not control.intersection_timed_out(control.exit_intersection(0.0), 999.0)


def test_한_바퀴_전이():
    """감지 → 대기 → 통과 → 억제 → 감지."""
    st = IntersectionState()
    assert st.detecting

    st = control.enter_intersection(now=0.0)
    assert st.waiting and not st.detecting

    st = control.exit_intersection(now=5.0)
    assert not st.waiting and not st.detecting

    st = control.tick_intersection(st, 5.0 + config.INTERSECTION_EXIT_DURATION)
    assert st.detecting, "한 바퀴 돌아 다시 감지 가능해야 한다"


def test_세_상태는_서로_배타적이다():
    """detecting 과 waiting 이 동시에 참인 상태는 없습니다.

    원본은 intersection_mode 와 intersection_exit_time 이 따로 있어서 둘 다 참인
    조합을 표현할 수 있었습니다.
    """
    for st in (IntersectionState(),
               control.enter_intersection(0.0),
               control.exit_intersection(0.0)):
        assert not (st.detecting and st.waiting), st


def test_phase_와_entered_at_은_짝이_맞아야_한다():
    """허용되지 않는 상태를 타입이 거부합니다.

    COOLDOWN + entered_at=None 이면 elapsed() 가 0.0 이라 억제가 영원히 풀리지
    않습니다. 원본에서 intersection_exit_time = None 은 "억제 아님"이었으므로,
    그대로 옮기면 의미가 정반대가 됩니다.
    """
    for phase in (Intersection.WAITING, Intersection.COOLDOWN):
        with pytest.raises(ValueError):
            IntersectionState(phase)                  # entered_at 없음
    with pytest.raises(ValueError):
        IntersectionState(Intersection.NONE, 5.0)     # NONE 인데 시각이 있음


def test_생성_경로는_모두_합법_상태를_만든다():
    for st in (IntersectionState(),
               control.enter_intersection(0.0),
               control.exit_intersection(0.0),
               control.tick_intersection(control.exit_intersection(0.0),
                                         config.INTERSECTION_EXIT_DURATION)):
        assert (st.phase is Intersection.NONE) == (st.entered_at is None), st


def test_정상_주행_상태에_tick_을_해도_그대로다():
    """호출부가 매 프레임 부르는 가장 흔한 형태입니다."""
    assert control.tick_intersection(IntersectionState(), 999.0) == IntersectionState()


# ============================================================
# 원본 상수값 고정
# ============================================================

def test_원본_상수값을_그대로_옮겼다():
    """동등성 테스트로는 config 값 변경을 잡을 수 없습니다.

    test_vision.py 의 legacy_measure() 가 measure() 와 같은 config 를 읽어 둘이
    함께 변하기 때문입니다. 원본 하드코딩 값을 여기서 직접 못 박습니다.

    기준: salvage/pi-final-2025-12-13 브랜치의 lane_tracer.py
    """
    assert config.LANE_HSV_LOWER == (65, 20, 20)
    assert config.LANE_HSV_UPPER == (115, 255, 255)
    assert config.STOPLINE_HSV_LOWER == (100, 100, 100)
    assert config.STOPLINE_HSV_UPPER == (130, 255, 255)
    assert (config.ROI_BOX_WIDTH_RATIO, config.ROI_BOX_HEIGHT_RATIO) == (0.25, 0.25)
    assert (config.CENTER_BOX_WIDTH_RATIO, config.CENTER_BOX_HEIGHT_RATIO,
            config.CENTER_BOX_TOP_RATIO) == (0.6, 0.15, 0.3)
    assert config.STOPLINE_ROI_TOP_RATIO == 0.75
    assert (config.MORPH_KERNEL_SIZE, config.MORPH_ERODE_ITERATIONS,
            config.MORPH_DILATE_ITERATIONS) == (3, 2, 3)


def test_정지선_간격_경계는_두_번째_정지선으로_본다():
    """경과 == STOPLINE_MIN_GAP_SEC 이면 멈춥니다.

    `<` 가 `<=` 로 바뀌면 이 경계를 "같은 정지선을 다시 본 것"으로 판정해 두 번째
    정지선을 놓치고, 주행이 끝나지 않습니다.
    """
    st = control.StoplineState(stage=1, first_seen_at=0.0)
    d = control.check_stopline(config.BLUE_PIXEL_THRESHOLD, st,
                              now=config.STOPLINE_MIN_GAP_SEC)
    assert d.should_stop is True, "경계를 같은 정지선으로 판정했다"


def test_원본_판정_임계값을_그대로_옮겼다():
    """이 파일의 다른 테스트는 대부분 config.X ± 1 로 단정하므로 값 자체는
    무엇이든 통과합니다. 여기서 값을 못 박습니다."""
    assert config.PIXEL_THRESHOLD == 800
    assert config.CENTER_THRESHOLD == 5000
    assert config.BLUE_PIXEL_THRESHOLD == 25000
    assert config.HIGH_PIXEL_THRESHOLD == 12000
    assert config.BASE_BALANCE_THRESHOLD == 0.35
    assert config.ONE_SIDE_MISSING_PIXELS == 50
    assert config.STRAIGHT_DURATION == 0.5
    assert config.STOPLINE_MIN_GAP_SEC == 0.3
    assert config.INTERSECTION_EXIT_DURATION == 2.0
    assert config.INTERSECTION_TIMEOUT == 3.0
    assert config.STRONG_TURN_MULTIPLIER == 1.25
    assert config.TURN_INTENSITY_NORMALIZER == 0.5


def test_시간_상수를_원본_값으로_못_박는다():
    """시간 단정은 대부분 좌우변이 같은 상수를 읽어 값이 무엇이든 통과합니다.

    예를 들어 "정지 2초 + 재출발 0.5초만큼 시계가 흘렀다"를
    `>= STOP_SIGN_HOLD_SEC + STOP_SIGN_RESTART_SLOW_SEC` 로 단정하면, 두 값이
    0 이 돼도 `>= 0.0` 이 되어 통과합니다. 값은 여기서 못 박습니다.
    """
    assert config.STOP_SIGN_HOLD_SEC == 2.0
    assert config.STOP_SIGN_RESTART_SLOW_SEC == 0.5
    assert config.STOPLINE_FINAL_STOP_SEC == 1.5
    assert config.SLOW_MODE_DURATION == 3.0
    assert config.ACTION_COOLDOWN == 4.0
    assert config.SIGN_COOLDOWN == 3.0
    assert config.LANE_LOOP_DELAY_SEC == 0.02
    assert config.REVERSE_LOOP_DELAY_SEC == 0.1
    assert config.FRAME_SHARE_INTERVAL == 2


def test_종료_타임아웃은_협조적이지_않은_구간보다_길다():
    """값뿐 아니라 왜 그 값인지를 관계로 고정합니다.

    주행 루프에는 아직 stop_event 를 보지 않는 time.sleep 구간이 있습니다
    (STOP 정지 2.0 + 저속 재출발 0.5 + 정지선 최종 정지 1.5). main 의 join
    타임아웃이 그보다 짧으면 스레드가 깨어나 이미 닫힌 핀을 만질 수 있습니다.
    """
    import main

    uncooperative = (config.STOP_SIGN_HOLD_SEC
                     + config.STOP_SIGN_RESTART_SLOW_SEC
                     + config.STOPLINE_FINAL_STOP_SEC)
    assert main.SHUTDOWN_TIMEOUT_SEC == 6.0
    assert main.SHUTDOWN_TIMEOUT_SEC > uncooperative, (
        f"종료 타임아웃 {main.SHUTDOWN_TIMEOUT_SEC} 가 비협조 구간 "
        f"{uncooperative} 보다 짧다")


def test_GPIO_핀_번호를_못_박는다():
    """배선과 어긋나는 편집을 막습니다. 하드웨어가 없어 동작 검증은 못 하지만
    값이 바뀌는 것은 잡을 수 있습니다."""
    assert config.MOTOR_PWMA == 18
    assert config.MOTOR_AIN1 == 22
    assert config.MOTOR_AIN2 == 27
    assert config.MOTOR_PWMB == 23
    assert config.MOTOR_BIN1 == 25
    assert config.MOTOR_BIN2 == 24
    assert config.BUZZER_PIN == 12
    assert config.BUZZER_FREQUENCY == 2000


# ============================================================
# 프레임 분류 (select_branch)
# ============================================================

from control import Branch, Maneuver


def m(left=0, right=0, center=0, blue=0):
    import vision
    return vision.Measurements(left_pixels=left, right_pixels=right,
                               center_pixels=center, blue_pixels=blue)


DETECTING = IntersectionState()
COOLDOWN = control.exit_intersection(0.0)


def test_정지선이_가장_앞선다():
    """정지선은 주행 종료 조건이라 교차로보다 우선합니다.

    두 조건이 실제로 동시에 성립하는 장면이어야 우선순위를 검증할 수 있습니다.
    교차로 조건은 total < PIXEL_THRESHOLD * 2 (1600)도 요구하므로 좌우 합을
    1500 으로 둡니다.
    """
    total = 800 + 700          # 1500 < 1600 — 교차로 조건도 만족합니다
    both = m(left=800, right=700,
             center=config.CENTER_THRESHOLD + 1,
             blue=config.BLUE_PIXEL_THRESHOLD)
    assert total < config.PIXEL_THRESHOLD * 2, "장면이 교차로 조건을 만족해야 한다"
    assert control.is_intersection(both.center_pixels, both.total_pixels), \
        "교차로 조건이 성립해야 경합이 된다"
    assert control.select_branch(both, DETECTING) is Branch.STOPLINE


def test_교차로가_차선_추종보다_앞선다():
    measured = m(center=config.CENTER_THRESHOLD + 1, left=0, right=0)
    assert control.select_branch(measured, DETECTING) is Branch.INTERSECTION


def test_차선이_적으면_이탈():
    measured = m(left=100, right=100)     # 합 200 < 800
    assert control.select_branch(measured, DETECTING) is Branch.LINE_LOST


def test_차선이_충분하면_추종():
    measured = m(left=5000, right=5000)
    assert control.select_branch(measured, DETECTING) is Branch.TRACE


def test_네_갈래_모두_도달_가능하다():
    """네 갈래 모두에 도달할 수 있어야 합니다.

    앞 갈래의 조건이 넓어져 뒤 갈래가 도달 불가해지면 여기서 드러납니다.
    """
    reached = {
        control.select_branch(m(blue=config.BLUE_PIXEL_THRESHOLD), DETECTING),
        control.select_branch(m(center=config.CENTER_THRESHOLD + 1), DETECTING),
        control.select_branch(m(left=100, right=100), DETECTING),
        control.select_branch(m(left=5000, right=5000), DETECTING),
    }
    assert reached == set(Branch)


def test_억제_구간에는_정지선도_교차로도_고르지_않는다():
    stopline = m(blue=config.BLUE_PIXEL_THRESHOLD, left=5000, right=5000)
    assert control.select_branch(stopline, COOLDOWN) is Branch.TRACE

    inter = m(center=config.CENTER_THRESHOLD + 1)
    assert control.select_branch(inter, COOLDOWN) is Branch.LINE_LOST


def test_억제_구간에도_차선_이탈_복구는_동작한다():
    """억제되는 것은 정지선·교차로 감지뿐입니다. 원본과 같은 동작입니다."""
    assert control.select_branch(m(), COOLDOWN) is Branch.LINE_LOST


# ============================================================
# 교차로 지시 (plan_intersection)
# ============================================================

def test_표지판이_키보드보다_앞선다():
    st = control.enter_intersection(0.0)
    assert control.plan_intersection(st, 'turn_left', 'd', 0.0) == \
        (Maneuver.LEFT, control.FROM_SIGN)


def test_타임아웃돼도_표지판이_먼저다():
    """둘이 동시에 성립하는 경우입니다. 원본은 표지판을 먼저 확인했습니다.

    다른 테스트는 타임아웃 전 시점만 보므로, 이 경우가 없으면 표지판과 타임아웃의
    순서가 바뀌어도 잡히지 않습니다.
    """
    st = control.enter_intersection(0.0)
    now = config.INTERSECTION_TIMEOUT + 10      # 한참 지남
    assert control.plan_intersection(st, 'turn_left', None, now) == \
        (Maneuver.LEFT, control.FROM_SIGN)


def test_타임아웃이_키보드보다_앞선다():
    """원본이 타임아웃을 먼저 검사하고 continue 했기 때문입니다.

    사람의 키 입력보다 타임아웃이 앞서는 것은 어색하지만 동작을 보존합니다.
    """
    st = control.enter_intersection(0.0)
    now = config.INTERSECTION_TIMEOUT
    assert control.plan_intersection(st, None, 'd', now) == \
        (Maneuver.STRAIGHT, control.FROM_TIMEOUT)


def test_타임아웃_전이면_키보드를_쓴다():
    st = control.enter_intersection(0.0)
    assert control.plan_intersection(st, None, 'd', 1.0) == \
        (Maneuver.RIGHT, control.FROM_KEY)


def test_지시가_없으면_아무것도_하지_않는다():
    st = control.enter_intersection(0.0)
    assert control.plan_intersection(st, None, None, 1.0) == (None, None)


def test_대기_상태가_아니면_지시를_내지_않는다():
    for st in (IntersectionState(), control.exit_intersection(0.0)):
        assert control.plan_intersection(st, 'turn_left', 'd', 999.0) == (None, None)


def test_모르는_표지판은_무시한다():
    """매핑에 없는 표지판은 다음 우선순위로 넘어갑니다."""
    st = control.enter_intersection(0.0)
    assert control.plan_intersection(st, 'unknown_sign', None, 1.0) == (None, None)


def test_모르는_키는_동작_없이_교차로를_탈출한다():
    """원본은 `if user_input:` 로 truthy 판정만 하고, 어느 키에도 맞지 않으면
    모터 명령 없이 교차로를 빠져나갔습니다. 개행 문자 하나로도 회전을 건너뜁니다.

    버그로 보이지만 동작 보존 원칙에 따라 유지하고 사실만 고정합니다.
    """
    st = control.enter_intersection(0.0)
    for key in ('x', '\n', ' ', '1'):
        maneuver, source = control.plan_intersection(st, 'unknown_sign', key, 1.0)
        assert maneuver is Maneuver.IGNORE, key
        assert source is control.FROM_KEY, key
    assert control.steps_for(Maneuver.IGNORE, control.FROM_KEY) == (), \
        "IGNORE 는 모터를 건드리지 않아야 한다"


# 매핑 값을 리터럴로 못 박습니다. 출처만 확인하면 'turn_right' → LEFT 같은
# 잘못된 매핑이 잡히지 않습니다.
@pytest.mark.parametrize("sign,expected", [
    ('go_straight', Maneuver.STRAIGHT),
    ('turn_left', Maneuver.LEFT),
    ('turn_right', Maneuver.RIGHT),
    ('traffic', Maneuver.TRAFFIC_RIGHT),
])
def test_표지판이_올바른_지시로_매핑된다(sign, expected):
    st = control.enter_intersection(0.0)
    assert control.plan_intersection(st, sign, None, 1.0) == (expected, control.FROM_SIGN)
    assert control.SIGN_TO_MANEUVER[sign] is expected


@pytest.mark.parametrize("key,expected", [
    ('w', Maneuver.STRAIGHT),
    ('a', Maneuver.LEFT),
    ('d', Maneuver.RIGHT),
    ('s', Maneuver.HALT),
])
def test_키가_올바른_지시로_매핑된다(key, expected):
    st = control.enter_intersection(0.0)
    assert control.plan_intersection(st, None, key, 1.0) == (expected, control.FROM_KEY)
    assert control.KEY_TO_MANEUVER[key] is expected


def test_정지_키는_전진하지_않는다():
    """s 키가 STRAIGHT 로 바뀌면 정지 대신 6초 전진합니다."""
    steps = control.steps_for(Maneuver.HALT, control.FROM_KEY)
    assert steps == (("STOP", 0),)
    assert control.LINELOST_STEPS[Maneuver.HALT] == (("STOP", 0),)
    assert not any(a == "FORWARD" for a, _ in steps)


# ============================================================
# 시퀀스 표
# ============================================================

@pytest.mark.parametrize("maneuver", [
    Maneuver.STRAIGHT, Maneuver.LEFT, Maneuver.RIGHT,
    Maneuver.TRAFFIC_RIGHT, Maneuver.HALT, Maneuver.IGNORE,
])
@pytest.mark.parametrize("source", [control.FROM_SIGN, control.FROM_KEY])
def test_교차로_지시는_교차로_표를_쓴다(maneuver, source):
    """표가 올바라도 그것을 고르는 함수가 틀리면 리터럴 테스트로는 잡히지 않습니다.

    steps_for 가 LINELOST_STEPS 를 돌려줘도 STRAIGHT 만 보면 통과합니다(두 표의
    STRAIGHT 가 둘 다 6초). 그래서 동일 객체인지 봅니다.
    """
    assert control.steps_for(maneuver, source) is control.INTERSECTION_STEPS[maneuver]


def test_타임아웃은_전용_표를_쓴다():
    assert control.steps_for(Maneuver.STRAIGHT, control.FROM_TIMEOUT) \
        is control.TIMEOUT_STEPS


def test_타임아웃에_직진_아닌_지시가_오면_막는다():
    """타임아웃은 항상 직진입니다. 다른 조합이면 maneuver 가 조용히 무시됩니다."""
    with pytest.raises(AssertionError):
        control.steps_for(Maneuver.LEFT, control.FROM_TIMEOUT)


def test_두_표의_키_집합():
    """차선 이탈에는 신호등이 없습니다. 소비부가 표를 직접 인덱싱하므로
    KEY_TO_MANEUVER 에 항목이 늘면 KeyError 가 납니다."""
    for maneuver in control.KEY_TO_MANEUVER.values():
        assert maneuver in control.INTERSECTION_STEPS, maneuver
        assert maneuver in control.LINELOST_STEPS, maneuver
    for maneuver in control.SIGN_TO_MANEUVER.values():
        assert maneuver in control.INTERSECTION_STEPS, maneuver
    assert Maneuver.TRAFFIC_RIGHT not in control.LINELOST_STEPS
    assert set(control.MANEUVER_NAMES) == set(Maneuver)


def test_이탈_추종_경계():
    """total_pixels == PIXEL_THRESHOLD 는 추종입니다(부등호가 <)."""
    at = m(left=config.PIXEL_THRESHOLD, right=0)
    below = m(left=config.PIXEL_THRESHOLD - 1, right=0)
    assert control.select_branch(at, DETECTING) is Branch.TRACE
    assert control.select_branch(below, DETECTING) is Branch.LINE_LOST


def test_타임아웃_직진은_교차로_직진보다_짧다():
    """원본에서 두 경로의 시간이 달랐습니다. 같은 Maneuver 지만 표가 다릅니다."""
    timeout = control.steps_for(Maneuver.STRAIGHT, control.FROM_TIMEOUT)
    normal = control.steps_for(Maneuver.STRAIGHT, control.FROM_KEY)
    assert timeout != normal
    assert sum(sec for _, sec in timeout) < sum(sec for _, sec in normal)


def test_교차로_시퀀스가_원본_값과_같다():
    """[원위치] salvage/pi-final-2025-12-13:lane_tracer.py
    755-788 (교차로 시퀀스), 743-744 (타임아웃 직진 0.5초)

    리터럴로 못 박습니다. config 를 참조하면 값이 바뀌어도 통과합니다.
    """
    assert control.INTERSECTION_STEPS[Maneuver.STRAIGHT] == (("FORWARD", 6),)
    assert control.INTERSECTION_STEPS[Maneuver.LEFT] == (
        ("FORWARD", 4), ("LEFT", 5), ("FORWARD", 0.8))
    assert control.INTERSECTION_STEPS[Maneuver.RIGHT] == (
        ("FORWARD", 1), ("RIGHT", 5), ("FORWARD", 0.8))
    assert control.INTERSECTION_STEPS[Maneuver.TRAFFIC_RIGHT] == (
        ("STOP", 3.0), ("FORWARD", 4), ("RIGHT", 5), ("FORWARD", 0.8))
    assert control.INTERSECTION_STEPS[Maneuver.HALT] == (("STOP", 0),)
    assert control.INTERSECTION_STEPS[Maneuver.IGNORE] == ()
    assert control.TIMEOUT_STEPS == (("FORWARD", 0.5),)


def test_차선_이탈_시퀀스가_원본_값과_같다():
    """[원위치] salvage:lane_tracer.py 860-878"""
    assert control.LINELOST_STEPS[Maneuver.STRAIGHT] == (("FORWARD", 6),)
    assert control.LINELOST_STEPS[Maneuver.LEFT] == (
        ("FORWARD", 5), ("LEFT", 5), ("FORWARD", 0.8))
    assert control.LINELOST_STEPS[Maneuver.RIGHT] == (
        ("FORWARD", 4), ("RIGHT", 5), ("FORWARD", 0.8))
    assert control.LINELOST_STEPS[Maneuver.HALT] == (("STOP", 0),)


def test_신호등은_우회전_앞에_정지를_붙인_것이다():
    """원본은 별개 분기에 우회전 코드를 복사해 뒀습니다."""
    traffic = control.INTERSECTION_STEPS[Maneuver.TRAFFIC_RIGHT]
    assert traffic[0][0] == "STOP"
    assert [a for a, _ in traffic[1:]] == [
        a for a, _ in control.INTERSECTION_STEPS[Maneuver.RIGHT]]


def test_대기_시간은_음수가_아니다():
    """음수면 interruptible_sleep 이 즉시 반환해 단계를 건너뜁니다.

    동작 이름이 유효한지는 검사하지 않습니다. 가능한 이름 전체를 허용 집합으로
    두면 어떤 교체도 통과해서 아무것도 배제하지 못합니다. 이름과 순서는 위
    리터럴 테스트가 못 박습니다.
    """
    for table in (control.INTERSECTION_STEPS, control.LINELOST_STEPS):
        for maneuver, steps in table.items():
            for _, seconds in steps:
                assert seconds >= 0, (maneuver, seconds)


# ============================================================
# 속도 프로파일
# ============================================================

def test_두_프로파일의_값이_원본과_같다():
    assert control.NORMAL_SPEED.forward == 0.225
    assert control.NORMAL_SPEED.turn == 0.30
    assert control.SLOW_SPEED.forward == 0.20
    assert control.SLOW_SPEED.turn == 0.16


def test_감속_창의_끝점은_아직_감속이다():
    """now == slow_until 은 감속 안입니다.

    `<=` 가 `<` 로 바뀌면 감속이 한 프레임 일찍 풀립니다.
    """
    assert control.is_slow_mode(100.0, 100.0) is True
    assert control.profile_for(100.0, 100.0) is control.SLOW_SPEED
    # 한 순간 뒤에는 풀립니다
    assert control.is_slow_mode(100.0, 100.0001) is False


def test_감속은_직진과_회전_둘_다_낮춘다():
    """감속 프로파일은 직진과 회전을 모두 낮춥니다."""
    assert control.SLOW_SPEED.forward < control.NORMAL_SPEED.forward
    assert control.SLOW_SPEED.turn < control.NORMAL_SPEED.turn


def test_재출발은_직진만_낮춘다():
    """STOP 표지판 재출발입니다. 감속 모드와 달리 회전 속도는 건드리지 않습니다.

    원본의 비대칭입니다. 원본은 SPEED_FORWARD 만 바꾸고 SPEED_TURN 은 그대로
    뒀습니다.
    """
    restart = control.NORMAL_SPEED.slowed_forward(config.SPEED_SLOW_FORWARD)
    assert restart.forward == config.SPEED_SLOW_FORWARD
    assert restart.turn == control.NORMAL_SPEED.turn, "회전 속도를 건드렸다"


def test_프로파일은_불변이다():
    """값으로 다루므로 공유해도 안전합니다.

    모듈 상수가 아니라 로컬 인스턴스에 대입을 시도합니다. frozen 이 빠진 상태에서
    control.NORMAL_SPEED 에 대입하면 모듈 전역이 실제로 바뀌어 뒤따르는 테스트까지
    무너지기 때문입니다.
    """
    from dataclasses import FrozenInstanceError
    local = control.SpeedProfile(forward=0.1, turn=0.2)
    with pytest.raises(FrozenInstanceError):
        local.forward = 0.9
    with pytest.raises(FrozenInstanceError):
        local.turn = 0.9
