"""
하드웨어 계층의 동작을 고정하는 테스트입니다.

핀에 실제로 어떤 값이 들어가는지 검사합니다. 대상은 모터
(salvage/pi-final-2025-12-13:lane_tracer.py 124-193), 부저(207-240),
카메라 래퍼(373-379)입니다.

기대값은 원본 리터럴이 아니라 config 값으로 쓴 계산식입니다. 그래서 이 파일은
"핀 출력이 config 값을 따라간다"를 고정합니다. 비율 상수 자체를 못 박는 곳은
리터럴을 쓰는 test_hardware_parity.py 입니다.

라즈베리파이가 없으므로 FakePin 으로 검사합니다. 그래서 gpiozero 는
create_gpio_pins() 안에서만 import 합니다.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest

import config
import control
import hardware

NORMAL = control.NORMAL_SPEED
SLOW = control.SLOW_SPEED


@pytest.fixture
def motor():
    return hardware.Motor(hardware.create_fake_pins())


def pin_values(m):
    """(왼쪽 in1, in2, pwm, 오른쪽 in1, in2, pwm)"""
    p = m.pins
    return (p.ain1.value, p.ain2.value, p.pwma.value,
            p.bin1.value, p.bin2.value, p.pwmb.value)


# ============================================================
# 직진 / 정지 / 후진
# ============================================================

def test_전진은_양쪽_모터를_같은_속도로_정방향으로_돌린다(motor):
    motor.forward(NORMAL.forward)
    assert pin_values(motor) == (0, 1, config.SPEED_FORWARD_DEFAULT,
                                 0, 1, config.SPEED_FORWARD_DEFAULT)


def test_정지는_방향핀을_모두_0으로_두고_출력을_끊는다(motor):
    motor.forward(NORMAL.forward)
    motor.stop()
    assert pin_values(motor) == (0, 0, 0.0, 0, 0, 0.0)


def test_후진은_방향을_뒤집고_전진_속도의_절반으로_간다(motor):
    motor.backward(NORMAL.forward)
    half = config.SPEED_FORWARD_DEFAULT * config.REVERSE_SPEED_RATIO
    assert pin_values(motor) == (1, 0, half, 1, 0, half)


def test_감속_프로파일에서는_후진도_느려진다(motor):
    """후진 속도가 직진 속도에 연동돼 있다는 사실을 고정합니다."""
    motor.backward(SLOW.forward)
    half = config.SPEED_SLOW_FORWARD * config.REVERSE_SPEED_RATIO
    assert pin_values(motor)[2] == pytest.approx(half)


# ============================================================
# 회전 — 약한 회전 (안쪽 바퀴 정지)
# ============================================================

def test_약한_좌회전은_왼쪽_바퀴를_세우고_오른쪽만_돌린다(motor):
    motor.left(NORMAL.turn, intensity=0.4)
    expected_right = config.SPEED_TURN_DEFAULT * config.TURN_OUTER_RATIO * 0.4
    assert pin_values(motor) == (0, 1, 0.0,
                                 0, 1, pytest.approx(expected_right))


def test_약한_우회전은_오른쪽_바퀴를_세우고_왼쪽만_돌린다(motor):
    motor.right(NORMAL.turn, intensity=0.4)
    expected_left = config.SPEED_TURN_DEFAULT * config.TURN_OUTER_RATIO * 0.4
    assert pin_values(motor) == (0, 1, pytest.approx(expected_left),
                                 0, 1, 0.0)


def test_기준값_자체는_약한_회전이다(motor):
    """조건이 > 이므로 PIVOT_TURN_THRESHOLD 와 같을 때는 역회전하지 않습니다."""
    motor.left(NORMAL.turn, intensity=config.PIVOT_TURN_THRESHOLD)
    assert pin_values(motor)[0] == 0      # 왼쪽 역회전 안 함
    assert pin_values(motor)[2] == 0.0


# ============================================================
# 회전 — 강한 회전 (안쪽 바퀴 역회전)
# ============================================================

def test_강한_좌회전은_왼쪽_바퀴를_역회전시킨다(motor):
    motor.left(NORMAL.turn, intensity=1.0)
    inner = config.SPEED_TURN_DEFAULT * config.PIVOT_INNER_RATIO
    outer = config.SPEED_TURN_DEFAULT * config.TURN_OUTER_RATIO
    assert pin_values(motor) == (1, 0, pytest.approx(inner),
                                 0, 1, pytest.approx(outer))


def test_강한_우회전은_오른쪽_바퀴를_역회전시킨다(motor):
    motor.right(NORMAL.turn, intensity=1.0)
    inner = config.SPEED_TURN_DEFAULT * config.PIVOT_INNER_RATIO
    outer = config.SPEED_TURN_DEFAULT * config.TURN_OUTER_RATIO
    assert pin_values(motor) == (0, 1, pytest.approx(outer),
                                 1, 0, pytest.approx(inner))


def test_좌회전과_우회전은_좌우_대칭이다(motor):
    for intensity in (0.2, 0.5, 0.6, 1.0):
        left = hardware.Motor(hardware.create_fake_pins())
        right = hardware.Motor(hardware.create_fake_pins())
        left.left(NORMAL.turn, intensity=intensity)
        right.right(NORMAL.turn, intensity=intensity)
        l = pin_values(left)
        r = pin_values(right)
        assert l[:3] == pytest.approx(r[3:]), f"intensity={intensity}"
        assert l[3:] == pytest.approx(r[:3]), f"intensity={intensity}"


def test_회전_출력이_PWM_범위를_벗어나지_않는다(motor):
    """FakePin 이 0~1 밖의 값에 예외를 던지므로 이 테스트가 포화를 잡습니다.

    TURN_OUTER_RATIO 가 1.2 라서 SPEED_TURN 을 0.833 이상으로 올리면 바깥쪽
    바퀴 출력이 1.0 을 넘습니다. 실기에서는 gpiozero 가 예외를 던집니다.
    현재 SPEED_TURN(0.30)에서는 0.36 이라 통과합니다.
    """
    for profile in (NORMAL, SLOW):
        for intensity in (0.01, 0.5, 0.51, 1.0):
            motor.left(profile.turn, intensity=intensity)
            motor.right(profile.turn, intensity=intensity)


def test_프로파일의_두_값이_서로_섞이지_않는다(motor):
    """넘긴 회전 속도가 프로파일별로 핀 출력에 그대로 반영됩니다.

    Motor 가 상수를 쓰거나 한 프로파일의 값을 다른 쪽에 섞으면 깨집니다.

    호출부가 직진 속도와 회전 속도를 바꿔 넘기는 실수는 이 테스트가 잡지
    않습니다. 그쪽은 test_회전_인자를_순서로_넘기면_거부한다 가 막습니다.
    """
    motor.left(SLOW.turn, intensity=1.0)
    slow_outer = config.SPEED_SLOW_TURN * config.TURN_OUTER_RATIO
    assert pin_values(motor)[5] == pytest.approx(slow_outer)

    motor.left(NORMAL.turn, intensity=1.0)
    normal_outer = config.SPEED_TURN_DEFAULT * config.TURN_OUTER_RATIO
    assert pin_values(motor)[5] == pytest.approx(normal_outer)
    assert slow_outer != normal_outer, "두 프로파일의 회전 속도가 달라야 한다"


def test_방향핀을_먼저_쓰고_PWM_을_나중에_쓴다(motor):
    """대입 순서를 고정합니다.

    PWM 을 먼저 올리고 방향을 나중에 바꾸면, 실기에서는 이전 방향으로 새 duty 가
    순간 인가되는 글리치가 생깁니다. 최종 값만 비교해서는 잡히지 않아서 FakePin 이
    쓰기 순서를 기록합니다.
    """
    motor.left(NORMAL.turn, intensity=1.0)                      # 왼쪽 역회전 — 방향이 바뀌는 경우
    written = [name for name, _ in motor.pins.ain1.log]
    assert written == ["ain1", "ain2", "pwma", "bin1", "bin2", "pwmb"]


def test_핀_값_범위를_벗어나면_FakePin_이_막는다():
    """FakePin 이 gpiozero 의 계약을 지키는지 확인합니다."""
    pin = hardware.FakePin("test")
    with pytest.raises(ValueError):
        pin.value = 1.5
    with pytest.raises(ValueError):
        pin.value = -0.1


# ============================================================
# 속도 프로파일
# ============================================================

def test_넘긴_속도가_핀_출력에_그대로_나온다(motor):
    motor.forward(SLOW.forward)
    assert pin_values(motor)[2] == config.SPEED_SLOW_FORWARD

    motor.forward(NORMAL.forward)
    assert pin_values(motor)[2] == config.SPEED_FORWARD_DEFAULT


def test_모터는_속도_상태를_갖지_않는다():
    """속도는 매 호출 인자입니다. 상태가 없으면 한쪽만 갱신돼 어긋날 일이 없습니다."""
    motor = hardware.Motor(hardware.create_fake_pins())
    assert not hasattr(motor, "speed_forward")
    assert not hasattr(motor, "speed_turn")
    assert not hasattr(motor, "set_slow_mode")
    assert not hasattr(motor, "restore_speed")


def test_close_는_정지시킨_뒤_모든_핀을_닫는다(motor):
    motor.forward(NORMAL.forward)
    motor.close()
    assert all(p.closed for p in motor.pins.all())
    # 닫기 직전 마지막 쓰기가 정지여야 합니다
    assert motor.pins.ain1.log[-6:] == [
        ("ain1", 0), ("ain2", 0), ("pwma", 0.0),
        ("bin1", 0), ("bin2", 0), ("pwmb", 0.0)]


class ExplodingPin(hardware.FakePin):
    def close(self):
        raise RuntimeError("이미 닫힌 핀")


def test_한_핀이_닫기에_실패해도_나머지는_닫는다():
    """정리 중 예외가 나머지 핀 정리를 막으면 GPIO 가 붙잡힌 채 남습니다."""
    pins = hardware.create_fake_pins()
    pins.ain1 = ExplodingPin("ain1")
    pins.close()          # 예외가 전파되면 안 됩니다
    assert all(p.closed for p in pins.all() if not isinstance(p, ExplodingPin))


def test_부저_close_는_장치를_닫고_없으면_아무것도_안_한다():
    pin = hardware.FakePin("buzzer")
    hardware.Buzzer(pin).close()
    assert pin.closed
    hardware.Buzzer(None).close()      # 예외 없이 끝나야 합니다


# ============================================================
# 부저
# ============================================================

class RecordingSleep:
    def __init__(self):
        self.calls = []

    def __call__(self, sec):
        self.calls.append(sec)


def test_부저가_있으면_울리고_끈다():
    pin = hardware.FakePin()
    sleep = RecordingSleep()
    hardware.Buzzer(pin, sleep=sleep).beep(0.3)
    assert sleep.calls == [0.3]
    assert pin.value == 0          # 끝나면 꺼져 있어야 합니다


def test_부저가_없어도_같은_시간을_기다린다():
    """부저 유무로 비프음 대기 시간이 달라지지 않습니다."""
    sleep = RecordingSleep()
    hardware.Buzzer(None, sleep=sleep).beep(0.3)
    assert sleep.calls == [0.3]


def test_학교종은_네_음을_울린다():
    pin = hardware.FakePin()
    sleep = RecordingSleep()
    hardware.Buzzer(pin, sleep=sleep).play_school_bell()
    # 솔 솔 라(길게) 솔 + 사이 쉼 3번 = 7회 대기
    assert sleep.calls == [
        config.BELL_SHORT_BEEP_SEC, config.BELL_PAUSE_SEC,
        config.BELL_SHORT_BEEP_SEC, config.BELL_PAUSE_SEC,
        config.BELL_LONG_BEEP_SEC, config.BELL_PAUSE_SEC,
        config.BELL_SHORT_BEEP_SEC,
    ]
    assert pin.value == 0


def test_부저가_없으면_학교종은_대체_시간만_기다린다():
    sleep = RecordingSleep()
    hardware.Buzzer(None, sleep=sleep).play_school_bell()
    assert sleep.calls == [config.BELL_FALLBACK_SEC]


# ============================================================
# 카메라
# ============================================================

class StubPicam2:
    """picamera2 의 최소 계약만 흉내 냅니다."""

    def __init__(self):
        self.stopped = False

    def capture_array(self):
        return "FRAME"

    def stop(self):
        self.stopped = True


def test_PiCamera_는_프레임을_성공_플래그와_함께_돌려준다():
    assert hardware.PiCamera(StubPicam2()).read() == (True, "FRAME")


def test_PiCamera_해제는_stop_을_부른다():
    """원본 CameraWrapper.release() 가 close() 가 아니라 stop() 이었습니다."""
    stub = StubPicam2()
    hardware.PiCamera(stub).release()
    assert stub.stopped


def test_가짜_카메라는_프레임을_순서대로_내보낸다():
    cam = hardware.FakeCamera(["a", "b"])
    assert cam.read() == (True, "a")
    assert cam.read() == (True, "b")
    assert cam.read() == (False, None)


def test_가짜_카메라_해제():
    cam = hardware.FakeCamera([])
    cam.release()
    assert cam.released


def test_회전_인자를_순서로_넘기면_거부한다(motor):
    """intensity 는 키워드 전용입니다.

    speed_turn(0.16~0.30)과 intensity(0.0~1.0)가 둘 다 float 이고 범위가 겹칩니다.
    순서를 바꿔 넘기면 오류 없이 그럴듯한 PWM 값이 나와 조용히 잘못 주행합니다.
    """
    with pytest.raises(TypeError):
        motor.left(NORMAL.turn, 0.5)
    with pytest.raises(TypeError):
        motor.right(NORMAL.turn, 0.5)
    # 키워드로는 정상 동작합니다
    motor.left(NORMAL.turn, intensity=0.5)
