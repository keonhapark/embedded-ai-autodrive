"""
hardware.py
-----------
하드웨어 경계입니다. GPIO·카메라를 다루는 코드를 여기에 모읍니다.

## 구조

핀을 직접 만들지 않고 주입받습니다(MotorPins). 실제 핀은 gpiozero, 테스트용
핀은 FakePin 입니다. 이렇게 하면 intensity 에 따라 좌우 바퀴 출력을 나누는
계산이 테스트 대상에 들어옵니다. FakeMotor 를 따로 만들면 그 계산이 테스트에서
빠집니다.

gpiozero 와 picamera2 는 팩토리 함수 안에서 import 합니다. 모듈을 import 하는
것만으로 라즈베리파이 패키지가 필요하면 테스트를 돌릴 수 없습니다. 대신 실패
시점이 import 에서 실행 중으로 밀리므로 팩토리에서 실패 사유를 출력합니다.

핀은 이 모듈의 객체를 만든 쪽(main.py 또는 lane_follow_loop)이 소유하고
닫습니다. 원본은 핀이 lane_tracer.py 의 모듈 전역이어서 소유자가 드러나지
않았고, main.py 의 종료 처리가 이미 점유된 핀을 새로 열어 정지를 시도하는
버그가 있었습니다.

속도는 Motor 가 들고 있지 않고 호출마다 인자로 받습니다. "감속 표지판을 봤으니
느리게 간다"는 판단(control.SpeedProfile)이지 구동이 아니기 때문입니다.

## 옮기지 않은 것

원본 init_gpio() 는 새 핀을 열기 전에 기존 핀 7개를 close() 했습니다. 전역
변수로 핸들을 들고 있어서 재호출로 회수할 수 있었기 때문입니다. 지금은 만든
쪽이 닫으므로 필요하지 않습니다.
"""

import time

import config


# ============================================================
# 핀 (포트)
# ============================================================

class FakePin:
    """테스트용 핀입니다. 실제 핀의 계약 두 가지를 지킵니다.

    1) 값 범위. gpiozero 의 PWMOutputDevice 는 0~1 밖의 값에 예외를 던집니다.
       FakePin 이 조용히 받아주면 테스트는 통과하는데 실기에서 터집니다.
    2) 쓰기 순서 기록. 방향 핀과 PWM 핀의 대입 순서가 바뀌면 실기에서는
       이전 방향으로 새 duty 가 순간 인가되는 글리치가 생깁니다. 최종 값만
       비교해서는 잡히지 않습니다.
    """

    def __init__(self, name="pin", log=None):
        self.name = name
        self.log = [] if log is None else log
        self.closed = False
        self._value = 0

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, new_value):
        if not 0 <= new_value <= 1:
            raise ValueError(
                f"{self.name}: gpiozero 는 0~1 만 받는다 (받은 값 {new_value})")
        self._value = new_value
        self.log.append((self.name, new_value))

    def close(self):
        self.closed = True


class MotorPins:
    """모터 채널 한 쌍에 필요한 핀 묶음입니다.

    A 는 왼쪽 모터, B 는 오른쪽 모터입니다.
    AIN1/AIN2 는 회전 방향, PWMA 는 출력 세기입니다.
    """

    def __init__(self, pwma, ain1, ain2, pwmb, bin1, bin2):
        self.pwma = pwma
        self.ain1 = ain1
        self.ain2 = ain2
        self.pwmb = pwmb
        self.bin1 = bin1
        self.bin2 = bin2

    def all(self):
        return (self.pwma, self.ain1, self.ain2, self.pwmb, self.bin1, self.bin2)

    def close(self):
        """모든 핀을 닫습니다. 하나가 실패해도 나머지를 계속 닫습니다.

        여기서 중단하면 GPIO 가 붙잡힌 채 남아 다음 실행이 실패합니다.
        """
        for pin in self.all():
            try:
                pin.close()
            except Exception as e:
                print(f"[정리] 핀 닫기 실패: {e}")


def create_gpio_pins():
    """실제 GPIO 핀을 엽니다. 라즈베리파이에서만 동작합니다."""
    from gpiozero import DigitalOutputDevice, PWMOutputDevice

    # 중간에 실패하면 이미 연 핀을 닫고 예외를 올립니다. 닫지 않으면 참조가
    # 사라져 회수할 수 없고 다음 실행이 GPIO busy 로 실패합니다.
    opened = []
    try:
        for factory, pin in ((PWMOutputDevice, config.MOTOR_PWMA),
                             (DigitalOutputDevice, config.MOTOR_AIN1),
                             (DigitalOutputDevice, config.MOTOR_AIN2),
                             (PWMOutputDevice, config.MOTOR_PWMB),
                             (DigitalOutputDevice, config.MOTOR_BIN1),
                             (DigitalOutputDevice, config.MOTOR_BIN2)):
            opened.append(factory(pin))
    except Exception:
        for device in opened:
            try:
                device.close()
            except Exception:
                pass
        raise

    return MotorPins(*opened)


def create_fake_pins():
    """테스트용 핀 묶음입니다. 6개가 쓰기 로그 하나를 공유합니다(순서 검증용)."""
    log = []
    names = ("pwma", "ain1", "ain2", "pwmb", "bin1", "bin2")
    return MotorPins(*(FakePin(name, log) for name in names))


# ============================================================
# 모터
# ============================================================

class Motor:
    """차동 구동 모터입니다. 핀만 소유하고 속도는 호출마다 인자로 받습니다.

    intensity 는 키워드 전용입니다. speed_turn(0.16~0.30)과 intensity
    (0.0~1.0)가 둘 다 float 이고 범위가 겹쳐서, 순서를 바꿔 넘기면 오류 없이
    그럴듯한 PWM 값이 나옵니다. 실주행에서는 "좀 약하게 도네" 로만 보여 원인을
    찾기 어렵습니다.

    [원위치] salvage/pi-final-2025-12-13:lane_tracer.py 124-193
    (전진·좌회전·우회전·정지·후진 5개 함수. 195-205 의 set_slow_mode /
    restore_speed 는 control.SpeedProfile 이 대신합니다.)
    """

    def __init__(self, pins):
        self.pins = pins

    def forward(self, speed):
        self._drive(left=(0, 1, speed), right=(0, 1, speed))

    def backward(self, speed):
        """후진합니다. 차선 픽셀 급증(벽 충돌 추정) 시 복구용이라 절반 속도로 갑니다.

        speed 는 직진 속도이고 여기서 REVERSE_SPEED_RATIO 를 곱합니다.
        """
        half = speed * config.REVERSE_SPEED_RATIO
        self._drive(left=(1, 0, half), right=(1, 0, half))

    def left(self, speed_turn, *, intensity=1.0):
        """좌회전합니다. intensity 는 0.0~1.0 입니다.

        0.5 를 넘으면 안쪽(왼쪽) 바퀴를 역회전시켜 제자리 회전에 가깝게 돕니다.
        그 이하면 안쪽 바퀴를 세우고 바깥쪽만 돌립니다.
        """
        if intensity > config.PIVOT_TURN_THRESHOLD:
            self._drive(
                left=(1, 0, speed_turn * config.PIVOT_INNER_RATIO * intensity),
                right=(0, 1, speed_turn * config.TURN_OUTER_RATIO * intensity))
        else:
            self._drive(
                left=(0, 1, 0.0),
                right=(0, 1, self._outer(speed_turn, intensity)))

    def right(self, speed_turn, *, intensity=1.0):
        """우회전합니다. left() 의 좌우 대칭입니다."""
        if intensity > config.PIVOT_TURN_THRESHOLD:
            self._drive(
                left=(0, 1, speed_turn * config.TURN_OUTER_RATIO * intensity),
                right=(1, 0, speed_turn * config.PIVOT_INNER_RATIO * intensity))
        else:
            self._drive(
                left=(0, 1, self._outer(speed_turn, intensity)),
                right=(0, 1, 0.0))

    def stop(self):
        """정지합니다. 방향 핀과 PWM 을 모두 0 으로 둡니다.

        원본에 드라이버 칩이 기록돼 있지 않아 이것이 제동인지 코스팅인지는 알 수
        없습니다.
        """
        self._drive(left=(0, 0, 0.0), right=(0, 0, 0.0))

    def _outer(self, speed_turn, intensity):
        """약한 회전에서 바깥쪽 바퀴 출력입니다.

        원본은 강한 회전 분기에서 SPEED_TURN * 1.2 * intensity 로, 약한 회전
        분기에서는 SPEED_TURN * (1.2 * intensity) 순서로 계산했습니다. 곱셈
        순서에 따라 결과가 1e-17 정도 달라집니다(speed_turn=0.30,
        intensity=0.49 에서 0.1764 와 0.17639999999999997). 동등성 테스트
        (test_hardware_parity.py)가 정확 비교를 하므로 원본 순서를 유지하려고
        따로 둡니다.
        """
        return speed_turn * (config.TURN_OUTER_RATIO * intensity)

    def _drive(self, left, right):
        self.pins.ain1.value, self.pins.ain2.value, self.pins.pwma.value = left
        self.pins.bin1.value, self.pins.bin2.value, self.pins.pwmb.value = right

    def close(self):
        """정지시킨 뒤 핀을 닫습니다.

        닫힌 핀에는 더 이상 정지 명령을 보낼 수 없으므로 닫기 전에 정지합니다.
        호출부가 순서를 기억하지 않도록 여기서 묶습니다.
        """
        try:
            self.stop()
        except Exception as e:
            print(f"[정리] 모터 정지 실패: {e}")
        self.pins.close()


# ============================================================
# 부저
# ============================================================

class Buzzer:
    """부저입니다. device 가 None 이면 소리 없이 정해진 시간만 기다립니다.

    부저가 없는 기기에서도 주행 흐름이 크게 달라지지 않도록 무음일 때도
    기다립니다. 원본의 동작입니다. sleep 은 테스트에서 실제로 기다리지 않도록
    주입받습니다.

    [원위치] salvage/pi-final-2025-12-13:lane_tracer.py 207-240
    (beep 207-214, play_school_bell 216-240)
    """

    def __init__(self, device, sleep=time.sleep):
        self.device = device
        self._sleep = sleep

    def beep(self, sec=config.BEEP_DEFAULT_SEC):
        """단순 비프음입니다.

        원본에도 호출부가 없었지만 함께 옮겨 두었습니다. 원본 시그니처의
        volume 인자는 쓰이지 않아 뺐습니다.
        """
        if not self.device:
            self._sleep(sec)
            return
        self.device.value = 1.0
        self._sleep(sec)
        self.device.value = 0

    def play_school_bell(self):
        """학교종(솔-솔-라-솔)을 울립니다. 경적 표지판에 반응할 때 씁니다."""
        if not self.device:
            self._sleep(config.BELL_FALLBACK_SEC)
            return

        short = config.BELL_SHORT_BEEP_SEC
        long = config.BELL_LONG_BEEP_SEC
        pause = config.BELL_PAUSE_SEC
        melody = [(short, pause), (short, pause), (long, pause), (short, 0)]

        for beep_sec, rest_sec in melody:
            self.device.value = 1.0
            self._sleep(beep_sec)
            self.device.value = 0
            if rest_sec > 0:
                self._sleep(rest_sec)

    def close(self):
        """부저를 닫습니다. 장치가 없으면 아무것도 하지 않습니다."""
        if self.device:
            try:
                self.device.close()
            except Exception as e:
                print(f"[정리] 부저 닫기 실패: {e}")


def create_gpio_buzzer():
    """실제 부저를 엽니다. 실패하면 None 을 돌려줍니다. 부저 없이도 주행은 합니다.

    ImportError 는 삼키지 않습니다. gpiozero 자체가 없으면 모터도 열 수 없으므로
    부저 문제로 처리하면 안 됩니다.
    """
    from gpiozero import PWMOutputDevice
    try:
        return PWMOutputDevice(config.BUZZER_PIN,
                               frequency=config.BUZZER_FREQUENCY,
                               initial_value=0)
    except Exception as e:
        print(f"[경고] 부저를 열지 못했다. 무음으로 진행한다: {e}")
        return None


# ============================================================
# 카메라
# ============================================================

class PiCamera:
    """picamera2 를 read()/release() 만 노출하도록 감쌉니다.

    [원위치] salvage/pi-final-2025-12-13:lane_tracer.py 373-379
    (init_camera 350-387 안에 중첩 정의돼 있던 CameraWrapper)
    """

    def __init__(self, picam2):
        self._picam2 = picam2

    def read(self):
        return True, self._picam2.capture_array()

    def release(self):
        self._picam2.stop()


class FakeCamera:
    """정해진 프레임을 순서대로 내보냅니다. 다 쓰면 (False, None) 을 돌려줍니다."""

    def __init__(self, frames):
        self._frames = list(frames)
        self.released = False

    def read(self):
        if not self._frames:
            return False, None
        return True, self._frames.pop(0)

    def release(self):
        self.released = True


def create_camera():
    """카메라를 엽니다. 실패하면 None 을 돌려줍니다.

    'Pipeline handler in use' 는 이전 프로세스가 카메라를 잡고 있을 때 나므로,
    재시도 전에 libcamera 프로세스를 정리합니다.
    """
    last_error = None
    for attempt in range(config.CAMERA_MAX_RETRIES):
        picam2 = None
        try:
            from picamera2 import Picamera2

            if attempt > 0:
                time.sleep(config.CAMERA_RETRY_DELAY_SEC)
                import subprocess
                subprocess.run(['pkill', '-f', 'libcamera'], capture_output=True)
                time.sleep(config.CAMERA_KILL_SETTLE_SEC)

            picam2 = Picamera2()
            # 지역 변수명을 config 로 두면 import 한 config 모듈을 가려
            # UnboundLocalError 가 납니다.
            cam_config = picam2.create_preview_configuration(
                main={"format": config.CAMERA_FORMAT,
                      "size": (config.CAMERA_WIDTH, config.CAMERA_HEIGHT)}
            )
            picam2.configure(cam_config)
            picam2.start()
            time.sleep(config.CAMERA_WARMUP_SEC)
            return PiCamera(picam2)

        except Exception as e:
            last_error = e
            # 생성은 됐는데 configure/start 에서 실패하면 이 객체가 파이프라인을
            # 계속 점유합니다. 점유자가 같은 프로세스라 pkill 로도 풀리지 않습니다.
            if picam2 is not None:
                try:
                    picam2.close()
                except Exception:
                    pass

    print(f"[오류] 카메라를 열지 못했다 ({config.CAMERA_MAX_RETRIES}회 시도): {last_error}")
    return None
