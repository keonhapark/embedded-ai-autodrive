"""
main.shutdown() 의 종료 처리를 검사합니다.

원본은 주행 중 Ctrl+C 를 눌러도 모터가 멈추지 않았습니다(main.py 의 docstring
참고). 실기 없이 확인할 수 있는 것은 정지가 호출되는지, 스레드를 기다리는지,
정리가 한 자원의 실패로 중단되지 않는지입니다.
"""

import os
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import hardware
import main as main_module


class SpyMotor(hardware.Motor):
    def __init__(self):
        super().__init__(hardware.create_fake_pins())
        self.stopped = 0
        self.closed = False

    def stop(self):
        self.stopped += 1
        super().stop()

    def close(self):
        self.closed = True


class SpyBuzzer:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


class SpyCamera:
    def __init__(self):
        self.released = False

    def release(self):
        self.released = True


class FakeThread:
    """join 이 호출됐는지, 살아남았는지를 흉내 냅니다."""

    def __init__(self, name, alive_after_join=False):
        self.name = name
        self.joined_with = None
        self._alive = alive_after_join

    def join(self, timeout=None):
        self.joined_with = timeout

    def is_alive(self):
        return self._alive


def test_종료_신호를_세우고_스레드를_기다린_뒤_정지시킨다():
    stop_event = threading.Event()
    threads = (FakeThread("lane"), FakeThread("detect"))
    motor, buzzer, camera = SpyMotor(), SpyBuzzer(), SpyCamera()

    main_module.shutdown(stop_event, threads, motor, buzzer, camera)

    assert stop_event.is_set(), "종료 신호를 세우지 않았다"
    for t in threads:
        assert t.joined_with == main_module.SHUTDOWN_TIMEOUT_SEC, f"{t.name} 을 기다리지 않았다"
    assert motor.stopped >= 1, "모터를 정지시키지 않았다"
    assert motor.closed and buzzer.closed and camera.released


def test_스레드가_시간_안에_끝나지_않아도_모터를_정지시킨다(capsys):
    """join 이 타임아웃돼도 주입받은 모터를 정지시킵니다."""
    motor = SpyMotor()
    threads = (FakeThread("lane", alive_after_join=True),)

    main_module.shutdown(threading.Event(), threads, motor, SpyBuzzer(), SpyCamera())

    assert motor.stopped >= 1
    assert "끝나지 않았다" in capsys.readouterr().out


def test_한_자원_정리가_실패해도_나머지를_정리한다(capsys):
    class ExplodingBuzzer(SpyBuzzer):
        def close(self):
            raise RuntimeError("부저 닫기 실패")

    motor, camera = SpyMotor(), SpyCamera()
    main_module.shutdown(threading.Event(), (), motor, ExplodingBuzzer(), camera)

    assert motor.closed, "부저 실패가 앞선 정리를 되돌렸다"
    assert camera.released, "부저 실패로 카메라 해제가 스킵됐다"
    assert "정리 실패" in capsys.readouterr().out


def test_모터_정지가_실패해도_정리는_계속한다(capsys):
    class BrokenMotor(SpyMotor):
        def stop(self):
            raise RuntimeError("핀 접근 실패")

    motor, buzzer, camera = BrokenMotor(), SpyBuzzer(), SpyCamera()
    main_module.shutdown(threading.Event(), (), motor, buzzer, camera)

    assert motor.closed and buzzer.closed and camera.released
    assert "모터 정지 실패" in capsys.readouterr().out
