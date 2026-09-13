"""
tools/key_remote.py (SSH 수동 조작 스크립트) 테스트입니다.

이 파일은 import 하는 순간 모듈 최상위에서 GPIO 핀을 엽니다. 그래서 gpiozero 를
가짜 모듈로 바꿔 끼운 뒤 import 합니다. 키 입력은 get_key 대신 미리 정한 순서로
넣습니다.
"""
import importlib
import sys
import types

import pytest


class FakeDevice:
    def __init__(self, pin, **kwargs):
        self.pin = pin
        self.value = 0
        self.closed = False

    def on(self):
        self.value = 1

    def off(self):
        self.value = 0

    def close(self):
        self.closed = True


@pytest.fixture
def key_remote(monkeypatch):
    fake = types.ModuleType("gpiozero")
    fake.DigitalOutputDevice = FakeDevice
    fake.PWMOutputDevice = FakeDevice
    monkeypatch.setitem(sys.modules, "gpiozero", fake)
    monkeypatch.delitem(sys.modules, "tools.key_remote", raising=False)
    module = importlib.import_module("tools.key_remote")
    yield module
    sys.modules.pop("tools.key_remote", None)


def drive_keys(module, monkeypatch, keys):
    """keys 를 순서대로 입력하고 호출된 모터 함수 이름을 돌려줍니다."""
    queue = list(keys) + ["q"]
    monkeypatch.setattr(module, "get_key", lambda: queue.pop(0))
    called = []
    for name in ("motor_forward", "motor_backward", "motor_left",
                 "motor_right", "motor_stop"):
        monkeypatch.setattr(module, name,
                            lambda name=name: called.append(name))
    module.main()
    return called


def test_화살표_키가_각_방향으로_간다(key_remote, monkeypatch):
    """get_key 는 화살표를 'UP'/'DOWN'/'LEFT'/'RIGHT' 로 돌려줍니다.

    원본은 오른쪽 분기에 'RIGsT' 라는 오타가 있어 오른쪽 화살표가 동작하지
    않았고, 화면 안내("D / → : Right")와도 어긋났습니다.
    """
    called = drive_keys(key_remote, monkeypatch, ["UP", "DOWN", "LEFT", "RIGHT"])
    # 마지막 motor_stop 은 main 의 finally 가 부릅니다
    assert called == ["motor_forward", "motor_backward", "motor_left",
                      "motor_right", "motor_stop"], called


def test_문자_키도_같은_방향으로_간다(key_remote, monkeypatch):
    called = drive_keys(key_remote, monkeypatch, ["w", "s", "a", "d", "x"])
    assert called == ["motor_forward", "motor_backward", "motor_left",
                      "motor_right", "motor_stop", "motor_stop"], called
