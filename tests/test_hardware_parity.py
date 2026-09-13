"""
리팩토링 전후 동등성 테스트입니다.

아래 legacy_* 함수는 리팩토링 전
salvage/pi-final-2025-12-13:lane_tracer.py 124-193 의 코드를
**한 줄도 고치지 않고** 옮긴 것입니다(전역 참조만 인자로 바꿨습니다).
새 Motor 가 같은 핀 값을 쓰는지 모든 조합에서 비교합니다.

라즈베리파이가 없어 실주행 검증을 할 수 없으므로, 구조를 옮기면서 모터 계산을
망가뜨리지 않았다는 근거는 이 비교입니다. 구조가 더 바뀌어도 이 파일은 원본을
기준점으로 남깁니다.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest

import control
import hardware


class LegacyPins:
    """원본이 쓰던 모듈 전역 6개를 그대로 흉내 낸 컨테이너입니다."""

    def __init__(self):
        self.AIN1 = self.AIN2 = self.BIN1 = self.BIN2 = 0
        self.PWMA = self.PWMB = 0.0

    def tuple(self):
        return (self.AIN1, self.AIN2, self.PWMA, self.BIN1, self.BIN2, self.PWMB)


def legacy_forward(p, SPEED_FORWARD):
    p.AIN1 = 0
    p.AIN2 = 1
    p.PWMA = SPEED_FORWARD
    p.BIN1 = 0
    p.BIN2 = 1
    p.PWMB = SPEED_FORWARD


def legacy_left(p, SPEED_TURN, intensity=1.0):
    if intensity > 0.5:
        p.AIN1 = 1
        p.AIN2 = 0
        p.PWMA = SPEED_TURN * 0.3 * intensity
        p.BIN1 = 0
        p.BIN2 = 1
        p.PWMB = SPEED_TURN * 1.2 * intensity
    else:
        left_ratio = 0.0
        right_ratio = 1.2 * intensity
        p.AIN1 = 0
        p.AIN2 = 1
        p.PWMA = SPEED_TURN * left_ratio
        p.BIN1 = 0
        p.BIN2 = 1
        p.PWMB = SPEED_TURN * right_ratio


def legacy_right(p, SPEED_TURN, intensity=1.0):
    if intensity > 0.5:
        p.AIN1 = 0
        p.AIN2 = 1
        p.PWMA = SPEED_TURN * 1.2 * intensity
        p.BIN1 = 1
        p.BIN2 = 0
        p.PWMB = SPEED_TURN * 0.3 * intensity
    else:
        left_ratio = 1.2 * intensity
        right_ratio = 0.0
        p.AIN1 = 0
        p.AIN2 = 1
        p.PWMA = SPEED_TURN * left_ratio
        p.BIN1 = 0
        p.BIN2 = 1
        p.PWMB = SPEED_TURN * right_ratio


def legacy_stop(p):
    p.AIN1 = 0
    p.AIN2 = 0
    p.PWMA = 0.0
    p.BIN1 = 0
    p.BIN2 = 0
    p.PWMB = 0.0


def legacy_backward(p, SPEED_FORWARD):
    p.AIN1 = 1
    p.AIN2 = 0
    p.PWMA = SPEED_FORWARD * 0.5
    p.BIN1 = 1
    p.BIN2 = 0
    p.PWMB = SPEED_FORWARD * 0.5


# ============================================================

def new_pins(motor):
    p = motor.pins
    return (p.ain1.value, p.ain2.value, p.pwma.value,
            p.bin1.value, p.bin2.value, p.pwmb.value)


# 원본에서 SPEED_FORWARD / SPEED_TURN 이 가질 수 있던 두 가지 조합입니다.
# control 의 프로파일을 직접 넘기면, 두 값의 짝이 원본 set_slow_mode() 와
# 같은지도 동등성 비교에 들어옵니다.
#
# 두 번째 행이 중요합니다. 기본값 행만으로는 Motor 가 인자를 무시하고 상수를
# 쓰는 경우가 드러나지 않습니다.
SPEED_PROFILES = [control.NORMAL_SPEED, control.SLOW_SPEED]

# 경계(0.5)를 포함해 촘촘히 훑습니다
INTENSITIES = [0.0, 0.01, 0.1, 0.25, 0.49, 0.5, 0.51, 0.75, 0.99, 1.0]


@pytest.mark.parametrize("profile", SPEED_PROFILES, ids=lambda p: f"f{p.forward}")
def test_직진_정지_후진이_원본과_같다(profile):
    speed_forward = profile.forward
    motor = hardware.Motor(hardware.create_fake_pins())
    legacy = LegacyPins()

    motor.forward(speed_forward)
    legacy_forward(legacy, speed_forward)
    assert new_pins(motor) == legacy.tuple()

    motor.stop()
    legacy_stop(legacy)
    assert new_pins(motor) == legacy.tuple()

    motor.backward(speed_forward)
    legacy_backward(legacy, speed_forward)
    assert new_pins(motor) == legacy.tuple()


@pytest.mark.parametrize("profile", SPEED_PROFILES, ids=lambda p: f"t{p.turn}")
@pytest.mark.parametrize("intensity", INTENSITIES)
def test_회전이_모든_강도에서_원본과_같다(profile, intensity):
    speed_turn = profile.turn
    for new_turn, legacy_turn in ((hardware.Motor.left, legacy_left),
                                  (hardware.Motor.right, legacy_right)):
        motor = hardware.Motor(hardware.create_fake_pins())
        legacy = LegacyPins()

        new_turn(motor, speed_turn, intensity=intensity)
        legacy_turn(legacy, speed_turn, intensity)
        assert new_pins(motor) == legacy.tuple(), (
            f"turn={speed_turn} intensity={intensity} {new_turn.__name__}")
