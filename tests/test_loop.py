"""
주행 루프를 실제로 돌리는 테스트입니다.

FakePin 과 FakeCamera 로 lane_follow_loop() 을 실행하고, 합성 프레임을 넣었을 때
모터가 어떤 호출을 받는지 봅니다. 판단 결과가 실제로 모터까지 가는지 확인하는
것이 목적입니다. 순수 함수 테스트는 호출부를 실행하지 않으므로, if/elif 체인의
뒤 가지가 죽어 차가 주행하지 않아도 통과할 수 있습니다.

cv2 와 numpy 가 필요합니다(requirements-dev.txt). 라즈베리파이 전용 패키지인
picamera2·gpiozero 는 필요 없습니다.
"""

import os
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
import pytest

import config
import control
import hardware
import lane_tracer
import scenes
import shared_state
import vision


class RecordingMotor(hardware.Motor):
    """실제 Motor 그대로 동작하면서 호출을 기록합니다."""

    def __init__(self):
        super().__init__(hardware.create_fake_pins())
        self.calls = []
        self.turns = []        # (방향, 강도). 강도 전달까지 확인할 때 씁니다
        self.speeds = []       # (동작, 속도)
        self.log = []          # 정지까지 포함한 (동작, 속도) 순서. 순서를 봐야
                               # finally 의 무조건 정지와 구분됩니다

    def forward(self, speed):
        self.calls.append("forward")
        self.speeds.append(("forward", speed))
        self.log.append(("forward", speed))
        super().forward(speed)

    def backward(self, speed):
        self.calls.append("backward")
        self.speeds.append(("backward", speed))
        self.log.append(("backward", speed))
        super().backward(speed)

    def left(self, speed_turn, *, intensity=1.0):
        self.calls.append("left")
        self.turns.append(("left", intensity))
        self.speeds.append(("left", speed_turn))
        self.log.append(("left", speed_turn))
        super().left(speed_turn, intensity=intensity)

    def right(self, speed_turn, *, intensity=1.0):
        self.calls.append("right")
        self.turns.append(("right", intensity))
        self.speeds.append(("right", speed_turn))
        self.log.append(("right", speed_turn))
        super().right(speed_turn, intensity=intensity)

    def stop(self):
        self.calls.append("stop")
        self.log.append(("stop", None))
        super().stop()


# 장면은 tests/scenes.py 에 모여 있습니다. 루프는 프레임을 180도 뒤집으므로
# as_camera_frame() 으로 미리 뒤집어 넣습니다.
CYAN = scenes.CYAN
BLACK = scenes.BLACK


def lane_frame():
    return scenes.as_camera_frame(scenes.both_lanes())


def intersection_frame():
    return scenes.as_camera_frame(scenes.intersection())


def stopline_frame():
    return scenes.as_camera_frame(scenes.stopline())


def blank_frame():
    return scenes.blank()


def moderate_frame():
    """급증으로 잡히지 않는 밀도입니다. 정상 추종이 여러 프레임 이어집니다."""
    return scenes.as_camera_frame(scenes.moderate_lanes())


def uneven_frame():
    return scenes.as_camera_frame(scenes.uneven_lanes())


def uneven_mirrored_frame():
    return scenes.as_camera_frame(scenes.uneven_lanes_mirrored())


class CountingCamera(hardware.FakeCamera):
    """읽은 프레임 수를 셉니다. 종료 신호가 언제 먹혔는지 확인하는 데 씁니다."""

    def __init__(self, frames):
        super().__init__(frames)
        self.reads = 0

    def read(self):
        self.reads += 1
        return super().read()


class FakeClock:
    """루프의 time 모듈을 대신합니다.

    sleep 이 실제로 기다리지 않고 시계를 앞으로 돌립니다. 교차로 지시 수행부의
    대기가 6~12.8초라 실시간으로는 테스트할 수 없습니다. strftime 등 나머지
    속성은 진짜 time 모듈에 위임합니다.
    """

    def __init__(self, start=1_000_000.0):
        self.now = start

    def time(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds

    def __getattr__(self, name):
        import time as real_time
        return getattr(real_time, name)


class ClockCamera(CountingCamera):
    """프레임을 읽을 때마다 시계를 조금 앞으로 돌립니다(프레임 간 시간)."""

    def __init__(self, frames, clock, step):
        super().__init__(frames)
        self._clock = clock
        self._step = step

    def read(self):
        self._clock.now += self._step
        return super().read()


@pytest.fixture
def clock(monkeypatch):
    """가짜 시계입니다. sleep 과 시퀀스 대기가 실제로 기다리지 않습니다.

    interruptible_sleep 도 대신합니다. 그대로 두면 stop_event.wait 이 실제로
    6~12.8초를 기다립니다. 실제 wait 경로는 아래
    test_종료_요청이_서_있으면_시퀀스를_중단한다 가 따로 검증합니다.
    """
    c = FakeClock()
    monkeypatch.setattr(lane_tracer, "time", c)
    monkeypatch.setattr(lane_tracer, "interruptible_sleep",
                        lambda stop_event, seconds: (c.sleep(seconds),
                                                     stop_event.is_set())[1])
    return c


@pytest.fixture
def keys(monkeypatch):
    """프레임별 키보드 입력을 미리 넣습니다. 비면 None 입니다."""
    queue = []
    monkeypatch.setattr(lane_tracer, "get_user_input",
                        lambda: queue.pop(0) if queue else None)
    return queue


@pytest.fixture(autouse=True)
def _no_stdin(monkeypatch):
    """루프가 select 로 stdin 을 읽는데, pytest 의 가짜 stdin 은 fileno 가 없습니다."""
    monkeypatch.setattr(lane_tracer, "get_user_input", lambda: None)


@pytest.fixture
def fast_sequences(monkeypatch):
    """개루프 시퀀스가 실제로 기다리지 않게 합니다(최대 12.8초).

    autouse 가 아닙니다. autouse 로 두면 이 파일의 모든 테스트가
    interruptible_sleep 을 가짜로 덮어써, 실제 stop_event.wait 경로가 한 번도
    실행되지 않습니다. 명시적으로 요청하게 해서 어느 테스트가 가짜를 쓰는지
    호출부에 드러나게 합니다.
    """
    monkeypatch.setattr(lane_tracer, "interruptible_sleep",
                        lambda stop_event, seconds: stop_event.is_set())


def run_loop(frames):
    motor = RecordingMotor()
    camera = hardware.FakeCamera(frames)
    lane_tracer.lane_follow_loop(
        motor=motor,
        buzzer=hardware.Buzzer(None, sleep=lambda _: None),
        camera=camera)
    return motor, camera


# ============================================================

def test_차선이_보이면_주행_명령이_모터까지_간다():
    """라인 트레이싱 갈래에 도달하는지 확인합니다."""
    motor, _ = run_loop([lane_frame()] * 3)
    drive_calls = [c for c in motor.calls if c in ("forward", "left", "right")]
    assert drive_calls, f"주행 명령이 한 번도 가지 않았다. 호출: {motor.calls}"


def test_차선이_없으면_정지한다(fast_sequences):
    """라인 이탈 갈래에 도달하는지 확인합니다.

    `"stop" in motor.calls` 로는 검증되지 않습니다. finally 가 소유와 무관하게
    항상 motor.stop() 을 부르므로 그 단정은 루프가 무엇을 했든 참입니다. 그래서
    첫 호출을 봅니다 — 라인 이탈 갈래만 프레임 처리 중에 정지를 부릅니다.
    """
    motor, _ = run_loop([blank_frame()] * 3)
    assert motor.calls[0] == "stop", f"라인 이탈 가지에 도달하지 않았다: {motor.calls}"
    assert motor.calls.count("stop") >= 2, (
        f"finally 의 정지 하나뿐이다. 가지가 죽었다: {motor.calls}")


def test_프레임이_끝나면_정지하고_빠져나온다():
    """finally 가 실행된다는 것만 확인합니다(마지막 호출이 정지).

    주의: 이 단정은 루프가 무엇을 했는지는 말해 주지 않습니다. finally 가 항상
    정지를 부르기 때문입니다. 갈래 도달은 아래 전용 테스트들이 봅니다.
    """
    motor, camera = run_loop([lane_frame()])
    assert motor.calls[-1] == "stop"


# ============================================================
# 소유권 — 만든 쪽이 닫습니다
# ============================================================

def test_넘겨받은_하드웨어는_닫지_않는다():
    """main.py 가 소유한 장치를 루프가 닫아 버리면 main 의 종료 처리가 닫힌
    핀을 만집니다. 정지는 하되 닫지는 않아야 합니다."""
    motor, camera = run_loop([lane_frame()])
    assert not any(p.closed for p in motor.pins.all()), "넘겨받은 핀을 닫았다"
    assert not camera.released, "넘겨받은 카메라를 해제했다"
    assert "stop" in motor.calls, "정지는 소유와 무관하게 해야 한다"


def test_직접_연_하드웨어는_닫는다(monkeypatch):
    """motor 를 넘기지 않으면 루프가 만들고, 그것은 루프가 닫습니다."""
    made = hardware.Motor(hardware.create_fake_pins())
    monkeypatch.setattr(hardware, "create_gpio_pins", lambda: made.pins)
    monkeypatch.setattr(hardware, "create_gpio_buzzer", lambda: None)

    lane_tracer.lane_follow_loop(camera=hardware.FakeCamera([lane_frame()]))
    assert all(p.closed for p in made.pins.all())


# ============================================================
# 협조적 종료
# ============================================================

def test_종료_신호가_이미_서_있으면_프레임을_읽지_않는다():
    stop_event = threading.Event()
    stop_event.set()
    motor = RecordingMotor()
    camera = CountingCamera([lane_frame()] * 5)

    lane_tracer.lane_follow_loop(
        motor=motor, buzzer=hardware.Buzzer(None, sleep=lambda _: None),
        camera=camera, stop_event=stop_event)

    assert camera.reads == 0, f"종료 상태인데 프레임을 읽었다 ({camera.reads}회)"
    assert motor.calls == ["stop"], f"주행하지 않고 정지만 해야 한다: {motor.calls}"


def test_주행_중_종료_신호를_받으면_멈춘다():
    """프레임이 남아 있어도 stop_event 가 서면 루프를 빠져나옵니다.

    이 신호가 없으면 스레드가 강제 종료될 때까지 돌고, finally 가 실행되지 않아
    모터가 멈추지 않습니다.
    """
    stop_event = threading.Event()
    motor = RecordingMotor()

    class StopAfterTwo(CountingCamera):
        def read(self):
            if self.reads == 1:
                stop_event.set()      # 두 번째 읽기 직전에 종료 요청
            return super().read()

    camera = StopAfterTwo([lane_frame()] * 50)
    lane_tracer.lane_follow_loop(
        motor=motor, buzzer=hardware.Buzzer(None, sleep=lambda _: None),
        camera=camera, stop_event=stop_event)

    assert camera.reads == 2, f"종료 후에도 프레임을 읽었다 ({camera.reads}회)"
    assert motor.calls[-1] == "stop", f"정지하지 않고 끝났다: {motor.calls}"


def test_종료_신호가_없으면_자체_Event_로_돌아간다():
    """단독 실행 호환입니다. 프레임이 소진되면 끝납니다."""
    motor, _ = run_loop([lane_frame()] * 2)
    assert motor.calls[-1] == "stop"


# ============================================================
# 각 갈래에 실제로 도달하는가
#
# 갈래마다 그 갈래에만 있는 흔적으로 도달을 확인합니다.
# ============================================================

def test_교차로_장면에서_교차로_가지로_들어간다(capsys):
    """교차로 갈래에만 있는 흔적으로 확인합니다.

    교차로 장면은 total_pixels 가 0 이라, 교차로 갈래가 죽어도 라인 이탈 갈래가
    같은 정지를 부르고 finally 도 정지를 부릅니다. 그래서 정지 호출이 아니라 이
    갈래에만 있는 "[교차로]" 출력을 봅니다.
    """
    motor, _ = run_loop([intersection_frame()] * 2)
    assert "[교차로]" in capsys.readouterr().out
    assert not [c for c in motor.calls if c in ("forward", "left", "right")], (
        f"교차로에서 주행 명령이 나갔다: {motor.calls}")


def test_정지선_장면에서_정지선_가지로_들어간다(capsys):
    motor, _ = run_loop([stopline_frame()])
    assert "첫 번째 정지선" in capsys.readouterr().out


def test_두_번째_정지선에서_주행을_끝낸다(capsys):
    """정지선 사이 간격을 두고 두 번 보여 주면 루프가 break 로 끝납니다."""
    frames = [stopline_frame()]
    # 첫 정지선 이후 STOPLINE_MIN_GAP_SEC 이상 지나야 두 번째로 인정됩니다.
    # 프레임 사이 간격이 0.02초라 그만큼 채웁니다.
    gap_frames = int(config.STOPLINE_MIN_GAP_SEC / config.LANE_LOOP_DELAY_SEC) + 5
    frames += [lane_frame()] * gap_frames
    frames += [stopline_frame()]
    frames += [lane_frame()] * 5      # 여기까지 오면 break 가 안 된 것

    motor = RecordingMotor()
    camera = CountingCamera(frames)
    lane_tracer.lane_follow_loop(
        motor=motor, buzzer=hardware.Buzzer(None, sleep=lambda _: None),
        camera=camera)

    out = capsys.readouterr().out
    assert "두 번째 정지선" in out, out[-400:]
    assert camera.reads <= len(frames), "break 하지 않고 프레임을 다 읽었다"


def test_차선이_보이는_동안은_교차로로_가지_않는다(capsys):
    """중앙 박스가 비어 있으면 교차로가 아닙니다.

    is_intersection 의 두 조건 중 center_pixels 쪽만 검증합니다. total_pixels
    가드는 아래 테스트가 봅니다.
    """
    run_loop([lane_frame()] * 3)
    assert "[교차로]" not in capsys.readouterr().out


def test_중앙이_차도_주변이_함께_차면_교차로가_아니다(capsys):
    """is_intersection 의 total_pixels 가드를 검증합니다.

    화면 전체가 차선 색이면 중앙 박스는 가득 차지만(27648 > CENTER_THRESHOLD)
    총 픽셀이 38400 으로 PIXEL_THRESHOLD 의 두 배를 넘습니다. 굵은 직선 구간을
    교차로로 오인하지 않게 막는 것이 이 가드입니다. 차선 장면은 중앙이 비어 있어
    앞 조건에서 이미 걸러지므로 위 테스트로는 이 가드를 검증할 수 없습니다.

    프레임 수를 적게 두는 이유: 이 장면은 픽셀이 많아 HIGH_PIXEL_DURATION(0.5초)
    이 지나면 후진 복구로 들어가고, 그러면 갈래 선택 전에 continue 해서 검증이
    무의미해집니다. 가짜 시계를 쓰지 않으므로 두 프레임은 그 안에 끝납니다.
    """
    run_loop([scenes.as_camera_frame(scenes.all_lane_color())] * 2)
    assert "[교차로]" not in capsys.readouterr().out


# --- 조향 판단이 올바른 모터 메서드로 가는가 ---
#
# 좌우 대칭 장면은 편차가 0 이라 항상 FORWARD 가 되고 좌우 디스패치가 실행되지
# 않습니다. 비대칭 장면으로 다음 잘못을 잡습니다.
#   - 조향 결과를 좌우 반대로 전달 (차가 코스 반대로 돕니다)
#   - 조향 결과를 무시하고 항상 전진
#   - 회전 강도를 무시하고 항상 1.0

def test_왼쪽이_많으면_우회전_명령이_나간다():
    motor, _ = run_loop([uneven_frame()] * 2)
    assert "right" in motor.calls, f"우회전이 나가지 않았다: {motor.calls}"
    assert "left" not in motor.calls, f"반대로 돌았다: {motor.calls}"


def test_오른쪽이_많으면_좌회전_명령이_나간다():
    motor, _ = run_loop([uneven_mirrored_frame()] * 2)
    assert "left" in motor.calls, f"좌회전이 나가지 않았다: {motor.calls}"
    assert "right" not in motor.calls, f"반대로 돌았다: {motor.calls}"


def test_회전_강도가_그대로_전달된다():
    """판단이 계산한 강도가 모터까지 가는지 확인합니다.

    비대칭 장면의 편차가 약 0.397 이고 정규화 계수가 0.5 라 강도는 약 0.795
    입니다. 1.0 으로 포화되지 않는 값이라 "항상 1.0 을 넘긴다"는 잘못을 잡습니다.
    """
    motor, _ = run_loop([uneven_frame()] * 2)
    assert motor.turns, "회전 호출이 없다"
    for direction, intensity in motor.turns:
        assert direction == "right"
        assert 0.0 < intensity < 1.0, f"강도가 포화됐다: {intensity}"
        assert intensity == pytest.approx(0.795, abs=0.02), intensity


# ============================================================
# 교차로 상태 기계가 루프에서 실제로 도는가
# ============================================================

def run_with_clock(frames, clock, step=0.5, motor=None):
    motor = motor or RecordingMotor()
    camera = ClockCamera(frames, clock, step)
    lane_tracer.lane_follow_loop(
        motor=motor, buzzer=hardware.Buzzer(None, sleep=lambda _: None),
        camera=camera)
    return motor, camera


def test_교차로에서_키_입력으로_지시를_수행하고_빠져나온다(clock, keys, capsys):
    """WAITING 블록(키 처리)에 도달하는지 확인합니다.

    'w' 처리가 6초를 기다리므로 가짜 시계가 필요합니다.
    """
    keys.extend([None, "w"])
    before = clock.now
    # 교차로 장면만 줍니다. 차선 장면을 뒤에 붙이면 정상 추종이 forward 를
    # 넣어서 'w' 시퀀스가 실행되지 않아도 단정이 통과합니다.
    motor, _ = run_with_clock([intersection_frame()] * 4, clock)
    out = capsys.readouterr().out
    assert "[교차로]" in out
    # 시계가 시퀀스 시간만큼 흘렀는지로 확인합니다
    assert clock.now - before >= config.TURN_STRAIGHT_FORWARD_SEC, \
        f"직진 시퀀스가 실행되지 않았다 (경과 {clock.now - before}초)"
    assert "forward" in motor.calls


def test_교차로_통과_직후에는_정지선을_감지하지_않는다(clock, keys, capsys):
    """COOLDOWN 억제가 실제로 작동하는지 확인합니다.

    정지선 갈래의 `inter.detecting` 가드가 없으면 교차로를 빠져나오는 순간 바로
    정지선으로 오인해 주행이 끝납니다. 순수 함수 쪽은 test_control.py 의
    select_branch 테스트가 보고, 이 테스트는 그 가드가 실제 루프에서도 걸리는지를
    봅니다.

    프레임당 0.5초씩 흐르므로 통과 직후의 정지선 프레임들은 억제 구간
    (INTERSECTION_EXIT_DURATION = 2.0초) 안입니다.
    """
    keys.extend([None, "w"])
    frames = [intersection_frame(), intersection_frame()] + [stopline_frame()] * 3
    run_with_clock(frames, clock, step=0.5)
    out = capsys.readouterr().out
    assert "[교차로]" in out
    assert "정지선" not in out, f"억제 구간에서 정지선을 감지했다:\n{out}"


def test_교차로_통과_직후에는_같은_교차로를_다시_잡지_않는다(clock, keys, capsys):
    """억제 구간에 교차로 장면을 계속 줘도 다시 진입하지 않아야 합니다.

    이것이 COOLDOWN 의 존재 이유입니다. 교차로 갈래의 `inter.detecting` 가드가
    없으면 차가 같은 교차로에서 계속 멈춰 섭니다. 순수 함수 쪽은
    test_control.py 의 select_branch 테스트가 보고, 이 테스트는 그 가드가 실제
    루프에서도 걸리는지를 봅니다.
    """
    keys.extend([None, "w"])
    # 통과 후에도 계속 교차로 장면 (프레임당 0.5초, 억제는 2.0초)
    frames = [intersection_frame()] * 5
    run_with_clock(frames, clock, step=0.5)
    out = capsys.readouterr().out
    assert out.count("[교차로]") == 1, (
        f"억제 구간에서 교차로를 다시 잡았다 ({out.count('[교차로]')}회):\n{out}")


def test_억제_시간이_지나면_정지선을_다시_감지한다(clock, keys, capsys):
    keys.extend([None, "w"])
    # 억제 2.0초를 넘기도록 프레임을 넉넉히 줍니다 (프레임당 0.5초)
    frames = [intersection_frame(), intersection_frame()] + [stopline_frame()] * 8
    run_with_clock(frames, clock, step=0.5)
    out = capsys.readouterr().out
    assert "첫 번째 정지선" in out, f"억제가 풀리지 않았다:\n{out}"


def test_교차로에서_지시가_없으면_타임아웃으로_직진한다(clock, keys, capsys):
    """INTERSECTION_TIMEOUT(3.0초)이 지나면 자동으로 직진합니다."""
    motor, _ = run_with_clock([intersection_frame()] * 12, clock, step=0.5)
    out = capsys.readouterr().out
    assert "[타임아웃]" in out, f"타임아웃 처리가 없다:\n{out}"
    assert "forward" in motor.calls


# ============================================================
# 시퀀스 실행기 — 종료 요청에 반응하는 대기
# ============================================================

# 아래 테스트들은 fast_sequences 를 요청하지 않습니다. 실제
# interruptible_sleep(= stop_event.wait)을 거쳐야 종료 반응이 검증됩니다.

def test_실제_대기는_지정된_시간만큼_걸린다():
    """interruptible_sleep 이 실제로 기다리는지 확인합니다.

    대기가 사라지면(함수가 stop_event.is_set() 만 돌려주면) 시퀀스 표의 초 값이
    무의미해지고 차가 회전 없이 교차로를 지나갑니다.
    """
    stop_event = threading.Event()
    started = time.monotonic()
    interrupted = lane_tracer.interruptible_sleep(stop_event, 0.05)
    elapsed = time.monotonic() - started

    assert interrupted is False, "종료 요청이 없는데 True 를 반환했다"
    assert elapsed >= 0.05, f"기다리지 않았다 ({elapsed:.4f}초)"


def test_실제_대기는_종료_요청에_즉시_반응한다():
    stop_event = threading.Event()
    stop_event.set()
    started = time.monotonic()
    interrupted = lane_tracer.interruptible_sleep(stop_event, 5.0)
    elapsed = time.monotonic() - started

    assert interrupted is True
    assert elapsed < 0.5, f"종료 요청을 받고도 기다렸다 ({elapsed:.2f}초)"


def test_종료_요청이_서_있으면_시퀀스를_중단한다():
    """실제 wait 경로로 검증합니다. 가짜를 요청하지 않습니다.

    time.sleep 이면 최대 12.8초를 잠들어, main 의 join 이 6초에 타임아웃된 뒤
    스레드가 깨어나 이미 닫힌 핀에 모터 명령을 보낼 수 있습니다.
    """
    stop_event = threading.Event()
    stop_event.set()
    motor = RecordingMotor()

    started = time.monotonic()
    finished = lane_tracer.run_steps(
        motor, control.INTERSECTION_STEPS[control.Maneuver.LEFT], stop_event,
        control.NORMAL_SPEED)
    elapsed = time.monotonic() - started

    assert finished is False, "종료 요청을 무시하고 시퀀스를 끝냈다"
    assert motor.calls == [], f"종료 상태인데 모터를 움직였다: {motor.calls}"
    assert elapsed < 0.5, f"9.8초 시퀀스를 기다렸다 ({elapsed:.2f}초)"


def test_시퀀스가_실제로_각_단계만큼_기다린다():
    """표의 초 값이 실제 대기로 이어지는지 확인합니다."""
    stop_event = threading.Event()
    motor = RecordingMotor()
    steps = ((control.FORWARD, 0.03), (control.LEFT, 0.03))

    started = time.monotonic()
    assert lane_tracer.run_steps(motor, steps, stop_event, control.NORMAL_SPEED) is True
    elapsed = time.monotonic() - started

    assert motor.calls == ["forward", "left"]
    assert elapsed >= 0.06, f"두 단계를 다 기다리지 않았다 ({elapsed:.4f}초)"


def test_종료_요청이_없으면_시퀀스를_끝까지_실행한다():
    stop_event = threading.Event()
    motor = RecordingMotor()
    steps = ((control.FORWARD, 0), (control.LEFT, 0), (control.FORWARD, 0))

    assert lane_tracer.run_steps(motor, steps, stop_event, control.NORMAL_SPEED) is True
    assert motor.calls == ["forward", "left", "forward"]


def test_시퀀스_중간에_종료_요청이_오면_남은_단계를_버린다():
    stop_event = threading.Event()
    motor = RecordingMotor()

    calls = {"n": 0}

    def wait(event, seconds):
        calls["n"] += 1
        if calls["n"] == 2:      # 두 번째 대기에서 종료 요청
            event.set()
        return event.is_set()

    import unittest.mock as mock
    with mock.patch.object(lane_tracer, "interruptible_sleep", wait):
        finished = lane_tracer.run_steps(
            motor, control.INTERSECTION_STEPS[control.Maneuver.LEFT], stop_event,
            control.NORMAL_SPEED)

    assert finished is False
    assert motor.calls == ["forward", "left"], f"3단계를 실행했다: {motor.calls}"


def test_drive_는_모든_동작을_모터_호출로_옮긴다():
    motor = RecordingMotor()
    for action, expected in ((control.FORWARD, "forward"), (control.LEFT, "left"),
                             (control.RIGHT, "right"), (control.STOP, "stop")):
        motor.calls.clear()
        lane_tracer.drive(motor, action, control.NORMAL_SPEED)
        assert motor.calls == [expected], action


def test_개루프_회전은_최대_강도다():
    """run_steps 독스트링이 "항상 최대 강도(1.0)"라고 선언합니다.

    호출 이름(calls)만 보고 강도(turns)를 보지 않으면 drive 의 intensity
    기본값이 바뀌어도 잡히지 않습니다.
    """
    motor = RecordingMotor()
    lane_tracer.run_steps(motor, ((control.LEFT, 0), (control.RIGHT, 0)),
                          threading.Event(), control.NORMAL_SPEED)
    assert motor.turns == [("left", 1.0), ("right", 1.0)]


def test_drive_는_강도를_그대로_넘긴다():
    motor = RecordingMotor()
    lane_tracer.drive(motor, control.RIGHT, control.NORMAL_SPEED, 0.42)
    assert motor.turns == [("right", 0.42)]


def test_모르는_동작은_예외다():
    """시퀀스 표에 오타가 나면 조용히 무시되지 않아야 합니다."""
    with pytest.raises(ValueError):
        lane_tracer.drive(RecordingMotor(), "SPIN", control.NORMAL_SPEED)


def test_차선_이탈에서_키_입력으로_복구한다(clock, keys, capsys):
    """LINE_LOST 갈래의 수동 복구가 실제로 모터까지 가는지 확인합니다."""
    keys.extend(["a"])
    motor, _ = run_with_clock([blank_frame()] * 3, clock)
    assert "left" in motor.calls, f"좌회전 복구가 실행되지 않았다: {motor.calls}"


# ============================================================
# 표지판 주행 — 교차로에서 표지판 큐를 소비하는 쪽
#
# 큐에 지시가 있을 때만 실행되는 경로입니다(popleft, 쿨다운 갱신, clear).
# 다음 잘못을 잡습니다.
#   - FROM_SIGN → FROM_KEY (쿨다운 대상이 바뀌어 같은 표지판이 반복 실행)
#   - recognized_signs.clear() 누락 (오래된 지시가 다음 교차로에서 실행)
#   - popleft() → [-1] (FIFO 가 LIFO 가 되고 큐가 줄지 않음)
#   - plan_intersection 에 sign_type 대신 None 전달 (표지판을 쓰지 않음)
# ============================================================

@pytest.fixture
def sign_queue():
    """표지판 큐를 비운 상태로 시작하고 끝난 뒤에도 비웁니다."""
    lane_tracer.recognized_signs.clear()
    yield lane_tracer.recognized_signs
    lane_tracer.recognized_signs.clear()


@pytest.fixture
def action_times():
    with shared_state.lock:
        shared_state.action_last_time.clear()
    yield shared_state.action_last_time
    with shared_state.lock:
        shared_state.action_last_time.clear()


def test_교차로에서_표지판_지시를_수행한다(clock, keys, sign_queue, action_times, capsys):
    sign_queue.append({'type': 'turn_left'})
    motor, _ = run_with_clock([intersection_frame()] * 4, clock)

    out = capsys.readouterr().out
    assert "[실행] 좌회전" in out, out
    assert ("left", 1.0) in motor.turns, f"좌회전이 모터까지 가지 않았다: {motor.turns}"


def test_표지판_수행_후_쿨다운을_갱신하고_큐를_비운다(clock, keys, sign_queue, action_times):
    sign_queue.append({'type': 'turn_right'})
    sign_queue.append({'type': 'turn_left'})     # 뒤에 남은 지시
    run_with_clock([intersection_frame()] * 4, clock)

    assert 'turn_right' in action_times, "수행한 표지판의 쿨다운이 갱신되지 않았다"
    assert 'turn_left' not in action_times, "수행하지 않은 표지판의 쿨다운을 찍었다"
    assert len(sign_queue) == 0, f"큐를 비우지 않았다: {list(sign_queue)}"


def test_표지판은_먼저_들어온_것부터_수행한다(clock, keys, sign_queue, action_times, capsys):
    """popleft 가 [-1] 로 바뀌면 LIFO 가 됩니다."""
    sign_queue.append({'type': 'turn_left'})
    sign_queue.append({'type': 'turn_right'})
    run_with_clock([intersection_frame()] * 4, clock)

    out = capsys.readouterr().out
    assert "[실행] 좌회전" in out, out
    assert "우회전" not in out, f"나중에 들어온 지시를 먼저 수행했다:\n{out}"
    assert 'turn_left' in action_times


def test_표지판이_키보드보다_먼저_쓰인다(clock, keys, sign_queue, action_times, capsys):
    sign_queue.append({'type': 'turn_left'})
    keys.extend(['d'])            # 우회전 키 — 무시돼야 합니다
    run_with_clock([intersection_frame()] * 4, clock)

    out = capsys.readouterr().out
    assert "[실행] 좌회전" in out, out


def test_표지판이_있으면_키를_읽지_않는다(clock, sign_queue, action_times, monkeypatch):
    """get_user_input 은 stdin 에서 글자를 소비합니다. 표지판이 이기는 프레임에서
    읽으면 조작자가 누른 키가 조용히 사라집니다. 원본도 읽지 않았습니다."""
    reads = []
    monkeypatch.setattr(lane_tracer, "get_user_input",
                        lambda: (reads.append(1), None)[1])
    sign_queue.append({'type': 'turn_left'})
    # 프레임 2장. 1장은 교차로 감지, 2장은 대기(표지판 수행)입니다. 여기서 끝내야
    # 차선 이탈 갈래의 정상적인 stdin 읽기가 섞이지 않습니다.
    run_with_clock([intersection_frame()] * 2, clock)

    assert reads == [], f"표지판이 있는데 stdin 을 {len(reads)}회 읽었다"


def test_인식되지_않는_표지판은_큐에서_버려진다(clock, keys, sign_queue, action_times, capsys):
    """원본과 같은 동작입니다. 프레임마다 하나씩 조용히 버려집니다."""
    sign_queue.append({'type': 'unknown_sign'})
    run_with_clock([intersection_frame()] * 2, clock)

    assert len(sign_queue) == 0, "큐에서 꺼내지 않았다"
    assert "[실행]" not in capsys.readouterr().out
    assert 'unknown_sign' not in action_times


# ============================================================
# 루프 안에서 시퀀스 중 종료
# ============================================================

def test_시퀀스_중_종료_요청이_오면_상태를_갱신하지_않는다(clock, keys, sign_queue,
                                                action_times, monkeypatch):
    """중단된 지시를 "수행 완료"로 기록하면 안 됩니다.

    run_steps 반환값을 호출부가 무시하면 쿨다운 갱신과 교차로 통과 기록이
    그대로 실행됩니다. 이 테스트가 그 배선을 봅니다.
    """
    stop_event = threading.Event()
    calls = {"n": 0}

    def waiting(event, seconds):
        calls["n"] += 1
        clock.sleep(seconds)
        if calls["n"] == 2:          # 좌회전 시퀀스 두 번째 단계에서 종료 요청
            stop_event.set()
        return stop_event.is_set()

    monkeypatch.setattr(lane_tracer, "interruptible_sleep", waiting)
    sign_queue.append({'type': 'turn_left'})
    sign_queue.append({'type': 'turn_right'})   # clear() 가 돌았는지 보려면 2개가 필요합니다

    motor = RecordingMotor()
    lane_tracer.lane_follow_loop(
        motor=motor, buzzer=hardware.Buzzer(None, sleep=lambda _: None),
        camera=ClockCamera([intersection_frame()] * 6, clock, 0.5),
        stop_event=stop_event)

    assert motor.calls[:2] == ["stop", "forward"], motor.calls
    assert "right" not in motor.calls
    assert 'turn_left' not in action_times, "중단된 지시의 쿨다운을 찍었다"
    # popleft 는 중단과 무관하게 이미 꺼냈습니다. clear() 가 돌았으면 0 이 됩니다.
    assert len(sign_queue) == 1, "중단됐는데 남은 큐를 비웠다"
    assert motor.calls[-1] == "stop", "정리하면서 정지하지 않았다"


def test_차선이탈_복구_중에도_종료_요청이_먹는다(clock, keys, monkeypatch):
    """이탈 복구 시퀀스에 stop_event 가 제대로 배선됐는지 확인합니다.

    run_steps 에 새 Event 를 넘기면 이 테스트가 잡습니다. 이탈 좌회전은 10.8초로
    이탈 시퀀스 중 가장 깁니다(전체 최장은 신호등 우회전 12.8초). 배선이 끊기면
    종료가 그만큼 밀립니다.
    """
    stop_event = threading.Event()
    waits = {"n": 0}

    def waiting(event, seconds):
        waits["n"] += 1
        clock.sleep(seconds)
        if waits["n"] == 1:          # 첫 단계 대기 중 종료 요청
            stop_event.set()
        return event.is_set()        # 넘겨받은 event 를 봅니다

    monkeypatch.setattr(lane_tracer, "interruptible_sleep", waiting)
    keys.extend(['a'])               # 좌회전 복구

    motor = RecordingMotor()
    lane_tracer.lane_follow_loop(
        motor=motor, buzzer=hardware.Buzzer(None, sleep=lambda _: None),
        camera=ClockCamera([blank_frame()] * 6, clock, 0.5),
        stop_event=stop_event)

    assert "forward" in motor.calls, motor.calls
    assert "left" not in motor.calls, (
        f"종료 요청 후에도 회전 단계를 실행했다: {motor.calls}")


# ============================================================
# 속도 프로파일
# ============================================================

def test_정상_주행은_기본_속도로_간다(clock, keys):
    motor, _ = run_with_clock([moderate_frame()] * 3, clock, step=0.1)
    forwards = [sp for act, sp in motor.speeds if act == "forward"]
    assert forwards, motor.speeds
    assert all(sp == control.NORMAL_SPEED.forward for sp in forwards), motor.speeds


def test_개루프_회전도_현재_프로파일을_따른다(clock, keys, sign_queue, action_times):
    """개루프 시퀀스는 그 순간의 프로파일을 씁니다.

    표에 속도 칸을 두지 않았기 때문입니다. 원본도 회전 시 전역 SPEED_TURN 을
    읽어 같은 동작이었습니다.
    """
    sign_queue.append({'type': 'turn_left'})
    motor, _ = run_with_clock([intersection_frame()] * 4, clock)
    turn_speeds = [sp for act, sp in motor.speeds if act == "left"]
    assert turn_speeds == [control.NORMAL_SPEED.turn], motor.speeds


def test_후진_복구는_직진_속도에_연동된다(clock, keys):
    """차선 픽셀이 급증하면 후진하고, 그때 넘기는 값은 직진 속도입니다.

    절반으로 줄이는 것은 Motor.backward 안에서 일어나므로 이 테스트는 그 비율을
    잡지 않습니다. 비율을 못 박는 곳은 리터럴 0.5 를 쓰는
    test_hardware_parity.py 입니다.
    """
    frame = scenes.as_camera_frame(scenes.all_lane_color())
    motor = RecordingMotor()
    camera = ClockCamera([frame] * 60, clock, 0.5)   # 지속시간 0.5초를 넘기도록 합니다
    lane_tracer.lane_follow_loop(
        motor=motor, buzzer=hardware.Buzzer(None, sleep=lambda _: None),
        camera=camera)
    backwards = [sp for act, sp in motor.speeds if act == "backward"]
    assert backwards, f"후진하지 않았다: {set(a for a, _ in motor.speeds)}"
    assert all(sp == control.NORMAL_SPEED.forward for sp in backwards)


def test_모터_동작은_속도_인자가_필수다():
    """속도를 빼먹으면 TypeError 로 즉시 드러나야 합니다.

    hasattr 로 "상태가 없다"를 보는 것은 test_hardware.py 에 있습니다. 여기서는
    인자가 실제로 필수인지 시그니처로 확인합니다. 기본값이 붙으면 호출부가 속도를
    빼먹어도 조용히 통과합니다.
    """
    import inspect
    for name in ("forward", "backward"):
        params = inspect.signature(getattr(hardware.Motor, name)).parameters
        speed = list(params.values())[1]
        assert speed.default is inspect.Parameter.empty, f"{name} 의 속도에 기본값이 있다"
    for name in ("left", "right"):
        params = inspect.signature(getattr(hardware.Motor, name)).parameters
        assert list(params.values())[1].default is inspect.Parameter.empty


@pytest.fixture
def object_state():
    """표지판 감지 상태입니다. 감속 창까지 되돌립니다."""
    with shared_state.lock:
        saved = shared_state.object_state.copy()
        for name in shared_state.object_state:
            shared_state.object_state[name] = False
        shared_state.slow_mode_until = None
    yield shared_state.object_state
    with shared_state.lock:
        shared_state.object_state.update(saved)
        shared_state.slow_mode_until = None


def test_SLOW_표지판이_속도를_낮춘다(clock, keys, object_state, action_times, capsys):
    """handle_runtime_triggers 가 돌려준 프로파일을 호출부가 받아 쓰는지 확인합니다.

    반환값을 버리면 감속이 아무 효과도 내지 못합니다.
    """
    object_state["slow"] = True
    motor, _ = run_with_clock([moderate_frame()] * 4, clock, step=0.1)

    assert "SLOW 감속" in capsys.readouterr().out
    forwards = [sp for act, sp in motor.speeds if act == "forward"]
    assert control.SLOW_SPEED.forward in forwards, (
        f"감속 속도가 모터까지 가지 않았다: {sorted(set(forwards))}")


def test_감속_중_개루프_회전도_느리다(clock, keys, object_state, sign_queue,
                                 action_times, capsys):
    """시퀀스가 그 순간의 프로파일을 쓰는지 확인합니다.

    표에 속도 칸이 없으므로 개루프 회전은 현재 프로파일을 따릅니다.
    """
    object_state["slow"] = True
    # 첫 프레임에서 감속을 걸고, 그다음 교차로에서 좌회전합니다.
    motor = RecordingMotor()
    sign_queue.append({'type': 'turn_left'})
    camera = ClockCamera([moderate_frame()] + [intersection_frame()] * 4, clock, 0.1)

    lane_tracer.lane_follow_loop(
        motor=motor, buzzer=hardware.Buzzer(None, sleep=lambda _: None),
        camera=camera)
    # 감속이 먼저 적용됐는지 확인한 뒤 회전 속도를 봅니다
    assert "SLOW 감속" in capsys.readouterr().out
    turn_speeds = [sp for act, sp in motor.speeds if act in ("left", "right")]
    assert turn_speeds, f"회전이 일어나지 않았다: {motor.speeds}"
    assert all(sp == control.SLOW_SPEED.turn for sp in turn_speeds), (
        f"감속 중인데 회전이 기본 속도다: {sorted(set(turn_speeds))}")


def test_판단에는_직진_속도를_넘긴다(clock, keys, monkeypatch):
    """decide_steering 의 세 번째 인자는 직진 속도입니다.

    회전 속도를 넘겨도 현재 설정에서는 관측되지 않습니다. 균형 임계값이
    HIGH_SPEED_THRESHOLD(0.6)를 기준으로 갈리는데 0.225 와 0.30 이 둘 다 그보다
    작기 때문입니다(도달할 수 없는 분기). 그래서 배선을 직접 봅니다.
    """
    seen = []
    original = control.decide_steering

    def spy(left, right, speed_forward, one_side, now):
        seen.append(speed_forward)
        return original(left, right, speed_forward, one_side, now)

    monkeypatch.setattr(lane_tracer.control, "decide_steering", spy)
    run_with_clock([moderate_frame()] * 3, clock, step=0.1)

    assert seen, "decide_steering 이 호출되지 않았다"
    assert all(sp == control.NORMAL_SPEED.forward for sp in seen), (
        f"직진 속도가 아닌 값을 넘겼다: {sorted(set(seen))}")


def test_감속은_시간이_지나면_스스로_풀린다(clock, keys, object_state, action_times):
    """SLOW → NORMAL 복원입니다.

    복원이 빠지면 slow 표지판을 한 번 본 뒤 남은 주행 전체가 감속됩니다.
    """
    object_state["slow"] = True
    # 프레임당 1초씩 흘려 SLOW_MODE_DURATION(3.0)을 넘깁니다
    motor, _ = run_with_clock([moderate_frame()] * 8, clock, step=1.0)

    forwards = [sp for act, sp in motor.speeds if act == "forward"]
    assert control.SLOW_SPEED.forward in forwards, f"감속하지 않았다: {forwards}"
    assert forwards[-1] == control.NORMAL_SPEED.forward, (
        f"감속이 풀리지 않았다: {forwards}")


def test_감속_복원은_회전_속도까지_되돌린다(clock, keys, object_state, sign_queue,
                                    action_times):
    """직진만 복원하고 회전을 감속으로 남기는 경우를 잡습니다."""
    object_state["slow"] = True
    motor = RecordingMotor()
    # 감속 → 만료 → 그 뒤 교차로 좌회전
    frames = [moderate_frame()] * 5 + [intersection_frame()] * 4
    camera = ClockCamera(frames, clock, 1.0)

    def queue_sign():
        sign_queue.append({'type': 'turn_left'})
    queue_sign()

    lane_tracer.lane_follow_loop(
        motor=motor, buzzer=hardware.Buzzer(None, sleep=lambda _: None),
        camera=camera)

    turn_speeds = [sp for act, sp in motor.speeds if act == "left"]
    assert turn_speeds, f"회전이 없었다: {motor.speeds}"
    assert turn_speeds[-1] == control.NORMAL_SPEED.turn, (
        f"복원 후에도 회전이 감속 속도다: {turn_speeds}")


class ReSlowingCamera(ClockCamera):
    """프레임마다 SLOW 표지판이 다시 잡히는 카메라입니다.

    감속 진입부가 object_state["slow"] 를 즉시 내리므로, 표지판이 계속 보이는
    상황을 만들려면 프레임마다 다시 세워야 합니다.
    """

    def read(self):
        with shared_state.lock:
            shared_state.object_state["slow"] = True
        return super().read()


def test_감속은_표지판이_계속_보여도_만료_프레임에_풀린다(clock, keys, object_state,
                                                action_times):
    """만료되는 프레임에는 감속을 다시 걸지 않습니다 — 원본의 타이밍입니다.

    원본의 진입 가드는 slow_mode_active 플래그를 읽었고, 그 플래그를 내리는 만료
    블록은 if/elif 체인보다 **뒤**에 있었습니다. 그래서 만료 프레임에는 가드가
    이전 값(True)을 읽어 진입이 막혔고 만료 블록이 정상 속도를 복원했습니다.
    다시 걸리는 것은 그다음 프레임입니다.

    가드를 시각 비교(is_slow_mode)로 판정하면 만료 프레임에 곧바로 다시 걸려,
    표지판이 계속 보이는 동안 감속이 **풀리지 않습니다**.
    """
    motor = RecordingMotor()
    # 프레임당 1초. SLOW_MODE_DURATION(3.0)을 두 번 넘길 만큼 돌립니다.
    camera = ReSlowingCamera([moderate_frame()] * 10, clock, 1.0)

    lane_tracer.lane_follow_loop(
        motor=motor, buzzer=hardware.Buzzer(None, sleep=lambda _: None),
        camera=camera)

    forwards = [sp for act, sp in motor.speeds if act == "forward"]
    assert control.SLOW_SPEED.forward in forwards, f"감속하지 않았다: {forwards}"

    # 첫 프레임은 감속이 걸리기 전이라 정상 속도입니다. 그 값으로 단정이
    # 충족되지 않도록 감속이 시작된 뒤 구간만 봅니다.
    after = forwards[forwards.index(control.SLOW_SPEED.forward):]
    assert control.NORMAL_SPEED.forward in after, (
        "표지판이 계속 보이는 동안 감속이 한 번도 풀리지 않았다. 만료 프레임에 "
        f"창을 재무장하고 있다: {forwards}")


def test_감속은_만료_다음_프레임에_다시_걸린다(clock, keys, object_state, action_times):
    """해제가 한 프레임짜리 창이라는 것을 반대쪽에서 고정합니다.

    위 테스트만 있으면 "만료 후 다시는 감속하지 않는" 경우가 통과합니다. 원본은
    다음 프레임에 표지판이 여전히 보이면 곧바로 다시 감속했습니다.
    """
    motor = RecordingMotor()
    camera = ReSlowingCamera([moderate_frame()] * 10, clock, 1.0)

    lane_tracer.lane_follow_loop(
        motor=motor, buzzer=hardware.Buzzer(None, sleep=lambda _: None),
        camera=camera)

    forwards = [sp for act, sp in motor.speeds if act == "forward"]
    # 감속 → 정상 → 감속으로 돌아오는 구간이 있어야 합니다. 첫 프레임의 정상
    # 속도를 해제로 착각하지 않도록 감속이 시작된 뒤에서 해제 지점을 찾습니다.
    after = forwards[forwards.index(control.SLOW_SPEED.forward):]
    restored = after.index(control.NORMAL_SPEED.forward)
    assert control.SLOW_SPEED.forward in after[restored + 1:], (
        f"만료 후 다시 감속하지 않았다: {forwards}")


# ============================================================
# STOP 표지판
#
# 정지(2초), 저속 재출발(0.5초), 쿨다운 기록을 봅니다.
# ============================================================

def test_STOP_표지판은_멈춘_뒤_느리게_재출발한다(clock, keys, object_state,
                                          action_times, capsys):
    object_state["stop"] = True
    before = clock.now
    motor, _ = run_with_clock([moderate_frame()] * 3, clock, step=0.1)

    out = capsys.readouterr().out
    assert "STOP 정지" in out, out

    # `"stop" in motor.calls` 로는 검증되지 않습니다. finally 가 소유와 무관하게
    # 항상 정지를 부르므로 그 단정은 STOP 분기를 지워도 참입니다.
    # 정지 바로 뒤에 저속 재출발이 오는 **순서**를 봅니다.
    restart = ("forward", config.SPEED_SLOW_FORWARD)
    assert restart in motor.log, f"저속 재출발이 없다: {motor.log}"
    i = motor.log.index(restart)
    assert i > 0 and motor.log[i - 1] == ("stop", None), (
        f"재출발 직전에 정지가 없다: {motor.log}")

    # 정지 2초 + 저속 재출발 0.5초가 실제로 흘렀는지 가짜 시계로 봅니다.
    # 호출 이름만 보면 sleep 을 지워도 통과합니다.
    assert clock.now - before >= (config.STOP_SIGN_HOLD_SEC
                                 + config.STOP_SIGN_RESTART_SLOW_SEC), (
        f"정지·재출발 시간이 흐르지 않았다 (경과 {clock.now - before}초)")

    # 재출발은 감속 직진 속도입니다. 회전 속도는 건드리지 않습니다.
    forwards = [sp for act, sp in motor.speeds if act == "forward"]
    assert config.SPEED_SLOW_FORWARD in forwards, (
        f"저속 재출발이 없다: {sorted(set(forwards))}")
    assert control.NORMAL_SPEED.forward in forwards, (
        f"재출발 뒤 기본 속도로 돌아오지 않았다: {sorted(set(forwards))}")


def test_STOP_표지판이_쿨다운을_기록한다(clock, keys, object_state, action_times):
    object_state["stop"] = True
    run_with_clock([moderate_frame()] * 3, clock, step=0.1)
    assert "stop" in action_times, "쿨다운을 기록하지 않았다"


def test_STOP_재출발은_회전_속도를_건드리지_않는다(clock, keys, object_state,
                                          sign_queue, action_times):
    """감속 모드(SLOW)와 달리 재출발은 직진만 낮춥니다. 원본의 비대칭입니다."""
    object_state["stop"] = True
    sign_queue.append({'type': 'turn_left'})
    motor = RecordingMotor()
    camera = ClockCamera([moderate_frame()] + [intersection_frame()] * 4,
                         clock, 0.1)
    lane_tracer.lane_follow_loop(
        motor=motor, buzzer=hardware.Buzzer(None, sleep=lambda _: None),
        camera=camera)

    turn_speeds = [sp for act, sp in motor.speeds if act == "left"]
    assert turn_speeds, f"회전이 없었다: {motor.speeds}"
    assert all(sp == control.NORMAL_SPEED.turn for sp in turn_speeds), (
        f"STOP 재출발이 회전 속도까지 낮췄다: {turn_speeds}")


def test_HORN_표지판은_부저를_울린다(clock, keys, object_state, action_times, capsys):
    class RecordingBuzzer(hardware.Buzzer):
        def __init__(self):
            super().__init__(None, sleep=lambda _: None)
            self.bells = 0

        def play_school_bell(self):
            self.bells += 1
            super().play_school_bell()

    object_state["horn"] = True
    buzzer = RecordingBuzzer()
    lane_tracer.lane_follow_loop(
        motor=RecordingMotor(), buzzer=buzzer,
        camera=ClockCamera([moderate_frame()] * 3, clock, 0.1))

    assert buzzer.bells >= 1, "학교종이 울리지 않았다"
    assert "학교종" in capsys.readouterr().out


def test_개루프_우회전도_현재_프로파일을_따른다(clock, keys, sign_queue, action_times):
    """좌회전만 검증하면 drive 의 RIGHT 분기에 잘못된 속도를 넘겨도 통과합니다."""
    sign_queue.append({'type': 'turn_right'})
    motor, _ = run_with_clock([intersection_frame()] * 4, clock)
    turn_speeds = [sp for act, sp in motor.speeds if act == "right"]
    assert turn_speeds == [control.NORMAL_SPEED.turn], motor.speeds


def test_감속_중_우회전도_느리다(clock, keys, object_state, sign_queue, action_times):
    object_state["slow"] = True
    sign_queue.append({'type': 'turn_right'})
    motor = RecordingMotor()
    camera = ClockCamera([moderate_frame()] + [intersection_frame()] * 4, clock, 0.1)
    lane_tracer.lane_follow_loop(
        motor=motor, buzzer=hardware.Buzzer(None, sleep=lambda _: None),
        camera=camera)
    turn_speeds = [sp for act, sp in motor.speeds if act == "right"]
    assert turn_speeds, f"우회전이 없었다: {motor.speeds}"
    assert all(sp == control.SLOW_SPEED.turn for sp in turn_speeds), turn_speeds


def test_감속_중_판단에도_감속_직진_속도를_넘긴다(clock, keys, object_state,
                                          action_times, monkeypatch):
    """기본 속도에서만 보면 상수를 넘기는 잘못이 통과합니다. 감속 중에도 확인합니다."""
    seen = []
    original = control.decide_steering

    def spy(left, right, speed_forward, one_side, now):
        seen.append(speed_forward)
        return original(left, right, speed_forward, one_side, now)

    monkeypatch.setattr(lane_tracer.control, "decide_steering", spy)
    object_state["slow"] = True
    run_with_clock([moderate_frame()] * 4, clock, step=0.1)

    assert control.SLOW_SPEED.forward in seen, (
        f"감속 중인데 판단에 기본 속도를 넘겼다: {sorted(set(seen))}")


def test_감속_중_후진도_느리다(clock, keys, object_state, action_times):
    """후진 복구가 현재 프로파일의 직진 속도에 연동되는지 확인합니다."""
    object_state["slow"] = True
    motor = RecordingMotor()
    # 먼저 감속을 건 뒤 픽셀 급증 장면으로 후진을 유도합니다
    frames = ([moderate_frame()] * 2
              + [scenes.as_camera_frame(scenes.all_lane_color())] * 40)
    lane_tracer.lane_follow_loop(
        motor=motor, buzzer=hardware.Buzzer(None, sleep=lambda _: None),
        camera=ClockCamera(frames, clock, 0.2))

    backwards = [sp for act, sp in motor.speeds if act == "backward"]
    assert backwards, f"후진하지 않았다: {set(a for a, _ in motor.speeds)}"
    assert all(sp == control.SLOW_SPEED.forward for sp in backwards), (
        f"감속 중인데 후진이 기본 속도다: {sorted(set(backwards))}")

# ============================================================
# 표지판 저장부 (store_direction_signs) — 큐를 채우는 쪽
#
# 인식된 방향 표지판을 교차로용 큐에 넣는 경로입니다. 위의 표지판 주행
# 테스트는 큐에 값을 직접 넣으므로 이 경로를 실행하지 않습니다. 여기서 신호등
# 90% 신뢰도 게이트, 표지판별 쿨다운, SIGN_COOLDOWN, 루프의 호출을 봅니다.
# 이 경로가 끊기면 교차로에서 항상 타임아웃 직진합니다.
# ============================================================

@pytest.fixture
def sign_clock(monkeypatch):
    """store_direction_signs 를 단독으로 부를 때 쓰는 시계입니다.

    이 함수는 모듈 전역 last_sign_time 과 time.time() 을 함께 읽으므로,
    SIGN_COOLDOWN(3초)을 넘기려면 둘을 같이 통제해야 합니다.
    """
    c = FakeClock(start=10_000.0)
    monkeypatch.setattr(lane_tracer, "time", c)
    monkeypatch.setattr(lane_tracer, "last_sign_time", 0.0)
    return c


def test_방향_표지판을_큐에_저장한다(object_state, action_times, sign_queue, sign_clock):
    object_state["turn_left"] = True
    lane_tracer.store_direction_signs()

    assert [s["type"] for s in sign_queue] == ["turn_left"], list(sign_queue)


def test_신호등은_신뢰도가_낮으면_저장하지_않는다(object_state, action_times,
                                        sign_queue, sign_clock):
    """오인식한 신호등으로 우회전 시퀀스가 도는 것을 막는 게이트입니다."""
    object_state["traffic"] = True
    with shared_state.lock:
        shared_state.confidence["traffic"] = 0.89
    lane_tracer.store_direction_signs()
    assert list(sign_queue) == [], "신뢰도 미달 신호등을 저장했다"

    # 쿨다운을 넘긴 뒤 신뢰도만 올리면 저장됩니다
    sign_clock.now += shared_state.ACTION_COOLDOWN + 1
    lane_tracer.last_sign_time = 0.0
    with shared_state.lock:
        shared_state.confidence["traffic"] = 0.95
    lane_tracer.store_direction_signs()
    assert [s["type"] for s in sign_queue] == ["traffic"]


def test_쿨다운_중_같은_표지판은_다시_저장하지_않는다(object_state, action_times,
                                           sign_queue, sign_clock):
    """교차로에서 지시를 소비한 직후 같은 표지판을 다시 저장하지 않습니다.

    큐를 비우지 않고 두 번 부르면 큐 꼬리 중복 검사(recognized_signs[-1])가 먼저
    막아, 쿨다운 게이트가 없어도 통과합니다. 그래서 교차로가 지시를 꺼내 간
    상황을 만들어 쿨다운 게이트만 남깁니다.
    """
    object_state["turn_right"] = True
    lane_tracer.store_direction_signs()
    assert len(sign_queue) == 1
    sign_queue.popleft()                             # 교차로가 지시를 소비함

    # SIGN_COOLDOWN 은 우회하고 ACTION_COOLDOWN 안에서 다시 부릅니다
    lane_tracer.last_sign_time = 0.0
    sign_clock.now += 1.0
    lane_tracer.store_direction_signs()
    assert list(sign_queue) == [], f"쿨다운 중에 다시 저장했다: {list(sign_queue)}"

    # 쿨다운이 지나면 다시 저장합니다 — 게이트가 영구 차단이 아니라는 것도 봅니다
    lane_tracer.last_sign_time = 0.0
    sign_clock.now += shared_state.ACTION_COOLDOWN
    lane_tracer.store_direction_signs()
    assert [s["type"] for s in sign_queue] == ["turn_right"]


def test_직전에_저장했으면_건너뛴다(object_state, action_times, sign_queue, sign_clock):
    """SIGN_COOLDOWN 조기 반환입니다. 이것이 없으면 매 프레임 저장을 시도합니다."""
    object_state["go_straight"] = True
    lane_tracer.last_sign_time = sign_clock.now      # 방금 저장한 상태
    lane_tracer.store_direction_signs()
    assert list(sign_queue) == [], "쿨다운 안인데 저장했다"


def test_루프가_표지판_저장을_호출한다(clock, keys, monkeypatch):
    """루프가 store_direction_signs 를 부르는지 확인합니다."""
    calls = []
    monkeypatch.setattr(lane_tracer, "store_direction_signs",
                        lambda: calls.append(1))
    # 프레임 수를 FRAME_SHARE_INTERVAL 로 계산하면 상수가 바뀔 때 같이 바뀌어
    # 검증이 무의미해집니다. 리터럴로 둡니다(주기 2 → 3프레임이면 1회 이상).
    run_with_clock([moderate_frame()] * 3, clock, step=0.1)
    assert calls, "루프가 store_direction_signs 를 부르지 않았다"


# ============================================================
# 탐지 스레드로 가는 프레임 공유
#
# 탐지 스레드가 받는 유일한 입력입니다. 이 배선이 끊기면 object_detector 가
# 계속 None 프레임을 보고 표지판·신호등·STOP·SLOW·HORN 이 모두 동작하지 않습니다.
# ============================================================

def test_탐지_스레드에_프레임을_공유한다(clock, keys):
    with shared_state.lock:
        shared_state.latest_frame = None

    frame = moderate_frame()
    run_with_clock([frame] * 3, clock, step=0.1)     # 리터럴. 위 테스트 참고

    with shared_state.lock:
        shared = shared_state.latest_frame
    assert shared is not None, "탐지 스레드에 프레임을 공유하지 않았다"
    # 루프가 뒤집은 프레임을 넘겨야 합니다. 카메라 원본을 그대로 넘기면
    # 탐지 스레드가 거꾸로 된 화면을 봅니다.
    assert (shared == vision.upright(frame)).all()
    # 공유할 때의 .copy() 는 검증하지 않습니다. upright 가 매 프레임 새 배열을
    # 만들고 루프와 vision 어디서도 프레임을 제자리 수정하지 않아, 지워도
    # 관측되는 차이가 없습니다.


# ============================================================
# 정리·재진입·교차로 대기
# ============================================================

def test_카메라를_못_열면_직접_연_하드웨어를_닫고_빠져나간다(monkeypatch):
    """원본은 이때 핀을 열어 둔 채 반환했습니다."""
    made = hardware.Motor(hardware.create_fake_pins())
    monkeypatch.setattr(hardware, "create_gpio_pins", lambda: made.pins)
    monkeypatch.setattr(hardware, "create_gpio_buzzer", lambda: None)
    monkeypatch.setattr(hardware, "create_camera", lambda: None)

    lane_tracer.lane_follow_loop()

    assert all(p.closed for p in made.pins.all()), "카메라 실패 후 핀을 열어 뒀다"


def test_cleanup_은_한_자원이_실패해도_나머지를_정리한다(capsys):
    """자원별 예외 격리를 봅니다. main.shutdown 에도 같은 방어가 있습니다."""
    class Exploding:
        def close(self):
            raise RuntimeError("닫기 실패")

    class Closable:
        closed = False
        def close(self):
            self.closed = True

    class Releasable:
        released = False
        def release(self):
            self.released = True

    first, last = Closable(), Releasable()
    lane_tracer.cleanup([first, Exploding(), last])

    assert first.closed
    assert last.released, "앞 자원의 실패로 뒤 자원 정리가 스킵됐다"
    assert "정리 실패" in capsys.readouterr().out


def test_루프에_들어갈_때_진행_중인_감속을_이어받는다(clock, keys, object_state):
    """루프에 들어갈 때 속도를 NORMAL 로 시작하면 진행 중이던 감속 창이 무시됩니다.

    다른 감속 테스트는 모두 slow_mode_until 이 None 인 상태에서 시작합니다.
    """
    with shared_state.lock:
        shared_state.slow_mode_until = clock.now + 100.0
    motor, _ = run_with_clock([moderate_frame()] * 2, clock, step=0.1)

    forwards = [sp for act, sp in motor.speeds if act == "forward"]
    assert forwards, f"직진이 없었다: {motor.speeds}"
    assert forwards[0] == control.SLOW_SPEED.forward, (
        f"진행 중인 감속을 이어받지 않았다: {forwards}")


def test_교차로_대기_중에는_차선_픽셀로_후진하지_않는다(clock, keys):
    """대기 중에는 측정값을 0 으로 둡니다. 원본도 그랬습니다.

    이 가드가 없으면 차선 색이 많은 곳에 멈춰 지시를 기다리는 동안
    HIGH_PIXEL_DURATION(0.5초)이 지나 후진으로 빠집니다. 교차로 장면은 좌우가
    비어 있어 가드가 없어도 드러나지 않으므로 전체가 차선 색인 장면을 씁니다.

    대기는 INTERSECTION_TIMEOUT(3초)까지입니다. 프레임 8장 x 0.2초로 그 안에서
    후진 지속시간만 넘깁니다.
    """
    lanes_everywhere = scenes.as_camera_frame(scenes.all_lane_color())
    frames = [intersection_frame()] + [lanes_everywhere] * 8
    motor, _ = run_with_clock(frames, clock, step=0.2)

    assert "backward" not in motor.calls, "교차로 대기 중에 후진했다"


# ============================================================
# STOP / HORN 중복 실행 방지
#
# 쿨다운을 기록하는지뿐 아니라 그 기록이 다음 발동을 막는지를 봅니다. 기록하는
# 줄과 검사하는 줄이 달라서 기록만 봐서는 검사를 검증할 수 없습니다.
#
# 표지판이 계속 보이는 상황은 카메라가 매 프레임 감지 상태를 다시 세워 만듭니다.
# 분기가 object_state 를 즉시 내리므로, 한 번만 세우면 쿨다운과 무관하게 두 번째
# 발동이 일어나지 않습니다.
# ============================================================

class ReDetectingCamera(ClockCamera):
    """프레임마다 지정한 표지판이 다시 감지되는 카메라입니다."""

    def __init__(self, frames, clock, step, name):
        super().__init__(frames, clock, step)
        self._name = name

    def read(self):
        with shared_state.lock:
            shared_state.object_state[self._name] = True
        return super().read()


class CountingBuzzer(hardware.Buzzer):
    def __init__(self):
        super().__init__(None, sleep=lambda _: None)
        self.bells = 0

    def play_school_bell(self):
        self.bells += 1


def test_STOP_표지판은_쿨다운_안에서_다시_발동하지_않는다(clock, keys, object_state,
                                                  action_times, capsys):
    """STOP 이 시야에 머무를 때 매 프레임 정지 2초 + 재출발을 반복하면 전진하지 못합니다.

    한 번 발동하면 시계가 2.5초 흐르고, 프레임 8장 x 0.1초를 더해도
    ACTION_COOLDOWN(4.0초) 안입니다.
    """
    camera = ReDetectingCamera([moderate_frame()] * 8, clock, 0.1, "stop")
    lane_tracer.lane_follow_loop(
        motor=RecordingMotor(), buzzer=hardware.Buzzer(None, sleep=lambda _: None),
        camera=camera)

    fired = capsys.readouterr().out.count("STOP 정지")
    assert fired == 1, f"쿨다운 안에서 {fired}회 발동했다"


def test_STOP_표지판은_발동_후_감지_상태를_내린다(clock, keys, object_state,
                                          action_times):
    """즉시 리셋입니다. 이것이 없으면 쿨다운이 끝나는 순간 같은 감지로 다시 멈춥니다."""
    object_state["stop"] = True
    run_with_clock([moderate_frame()] * 2, clock, step=0.1)
    assert object_state["stop"] is False, "발동 후에도 STOP 감지 상태가 남았다"


def test_HORN_표지판은_쿨다운_안에서_다시_울리지_않는다(clock, keys, object_state,
                                                 action_times):
    buzzer = CountingBuzzer()
    camera = ReDetectingCamera([moderate_frame()] * 8, clock, 0.1, "horn")
    lane_tracer.lane_follow_loop(motor=RecordingMotor(), buzzer=buzzer, camera=camera)

    assert buzzer.bells == 1, f"쿨다운 안에서 {buzzer.bells}회 울렸다"


def test_시퀀스_마지막_단계에서_중단돼도_수행_완료로_기록하지_않는다(
        clock, keys, sign_queue, action_times, monkeypatch):
    """위의 중단 테스트는 두 번째(중간) 단계에서 멈춥니다.

    중간에서 멈추면 다음 단계 앞의 stop_event 확인이 차이를 흡수해서, run_steps
    가 대기의 반환값을 무시해도 결과가 같습니다. 차이는 마지막 단계의 대기 중에
    종료 요청이 올 때만 드러납니다 — 확인할 다음 단계가 없어 run_steps 가 True 를
    돌려주고 호출부가 쿨다운을 찍습니다.
    """
    stop_event = threading.Event()
    calls = {"n": 0}

    def waiting(event, seconds):
        calls["n"] += 1
        clock.sleep(seconds)
        if calls["n"] == 3:          # 좌회전 시퀀스의 마지막(세 번째) 단계
            stop_event.set()
        return stop_event.is_set()

    monkeypatch.setattr(lane_tracer, "interruptible_sleep", waiting)
    sign_queue.append({'type': 'turn_left'})

    lane_tracer.lane_follow_loop(
        motor=RecordingMotor(), buzzer=hardware.Buzzer(None, sleep=lambda _: None),
        camera=ClockCamera([intersection_frame()] * 6, clock, 0.5),
        stop_event=stop_event)

    assert calls["n"] == 3, f"마지막 단계까지 가지 않았다: 대기 {calls['n']}회"
    assert 'turn_left' not in action_times, "중단된 지시의 쿨다운을 찍었다"
