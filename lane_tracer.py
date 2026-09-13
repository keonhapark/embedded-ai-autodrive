"""
lane_tracer.py
--------------
Act 계층입니다. Sense(vision)와 Plan(control)을 조립해 모터를 호출합니다.

    vision.measure(frame)  ->  Measurements      무엇을 봤는가
    control.*              ->  판단              그래서 무엇을 할 것인가
    hardware.Motor         ->  실행              모터에 전달

이 파일에는 픽셀 연산(vision.py), 순수 판단(control.py), GPIO 접근(hardware.py)이
없습니다. 남은 것은 프레임마다 셋을 잇는 순서와, 시간에 의존하는 개루프 동작
시퀀스입니다.

## 정리하지 않은 것

- handle_runtime_triggers 와 store_direction_signs 가 shared_state 부작용과
  판단을 섞고 있습니다. 표지판 대응 정책을 control.py 로 옮길 수 있습니다.
- 표지판 대응(STOP 정지 2초 + 재출발 0.5초)과 정지선 최종 정지(1.5초)는 아직
  time.sleep 이라 그동안 종료 요청에 바로 반응하지 않습니다. 종료 타임아웃
  (6초) 안이라 실기 피해는 없습니다.
- handle_runtime_triggers 의 bare except 두 곳이 KeyboardInterrupt·SystemExit
  까지 삼킵니다. 원본 그대로입니다.
"""
import time
import sys
import select
import threading
from collections import deque

import hardware
import vision

import config
import control

# shared_state 는 객체인식 스레드와의 공유 상태입니다. 없으면 인식 없이 주행만 합니다.
# config·control 을 같은 try 에 넣으면 이 fallback 이 동작하지 않습니다 — 모듈
# 최상위에서 config 를 읽기 때문에 ImportError 대신 NameError 가 납니다.
try:
    import shared_state
    OBJECT_DETECTION_ENABLED = True
except ImportError:
    OBJECT_DETECTION_ENABLED = False

# ============================================================
# 표지판 인식 큐 시스템
# ============================================================
recognized_signs = deque(maxlen=5)  # 최근 5개 표지판만 저장
last_sign_time = 0  # 마지막 표지판 인식 시간
SIGN_COOLDOWN = config.SIGN_COOLDOWN

# 모터·부저·카메라는 인자로 받습니다. 받지 못하면 lane_follow_loop 가 직접 만들고
# 직접 닫습니다.

# ============================================================
# 유틸리티 함수
# ============================================================
def interruptible_sleep(stop_event, seconds):
    """seconds 동안 기다립니다. 종료 요청이 오면 즉시 True 를 돌려줍니다.

    time.sleep 이면 개루프 시퀀스가 최대 12.8초(신호등 우회전: 정지 3 + 직진 4 +
    회전 5 + 직진 0.8) 동안 종료 신호에 반응하지 않습니다. main 의 join 은 6초에
    타임아웃되므로, 그 뒤 스레드가 깨어나 이미 닫힌 핀에 모터 명령을 보낼 수
    있습니다.

    Event.wait 는 신호가 없으면 타임아웃까지 기다리므로 개루프 타이밍이
    보존됩니다.
    """
    return stop_event.wait(seconds)


def drive(motor, action, speed, intensity=1.0):
    """동작 이름을 모터 호출로 옮깁니다. Plan 과 Act 의 접점입니다.

    speed 는 control.SpeedProfile 입니다. 모터가 속도를 들고 있지 않으므로
    여기서 넘깁니다.
    """
    if action == control.FORWARD:
        motor.forward(speed.forward)
    elif action == control.LEFT:
        motor.left(speed.turn, intensity=intensity)
    elif action == control.RIGHT:
        motor.right(speed.turn, intensity=intensity)
    elif action == control.STOP:
        motor.stop()
    else:
        raise ValueError(f"알 수 없는 동작: {action}")


def run_steps(motor, steps, stop_event, speed):
    """(동작, 초) 열을 순서대로 실행합니다.

    종료 요청이 오면 남은 단계를 버리고 False 를 돌려줍니다.

    speed 는 현재 속도 프로파일이라 개루프 회전도 감속 모드를 따릅니다. 원본도
    그랬습니다. 표에 속도 칸을 두지 않은 것은 원본에 시퀀스별 속도 구분이 없었기
    때문입니다. 회전은 원본처럼 항상 최대 강도(1.0)입니다 — 개루프 시퀀스는
    편차를 보지 않습니다.

    프레임마다 데드라인을 확인하는 상태 기계로 만들지 않은 이유: 그러면 개루프
    타이밍이 루프 주기로 양자화됩니다. 실제 주기는 camera.read() 와
    vision.measure() 가 좌우해 지터가 있고, 트랙에서 0.8초 단위로 맞춘 회전
    시간이 달라질 수 있습니다. 하드웨어 없이는 그 차이를 검증할 수 없어 원본처럼
    개루프 구간에는 센서를 보지 않습니다.
    """
    for action, seconds in steps:
        # 실행하기 전에 확인합니다. 뒤에서만 확인하면 종료 요청이 이미 서 있어도
        # 모터가 한 번 움직입니다.
        if stop_event.is_set():
            return False
        drive(motor, action, speed)
        # 0초 단계(정지 등)는 기다리지 않습니다.
        if seconds > 0 and interruptible_sleep(stop_event, seconds):
            return False
    return True


def cleanup(resources):
    """자원을 하나씩 격리해 정리합니다.

    한 줄로 늘어놓으면 앞의 것이 예외를 던졌을 때 뒤의 것이 모두 건너뛰어집니다.
    """
    for resource in resources:
        closer = getattr(resource, "close", None) or resource.release
        try:
            closer()
        except Exception as e:
            print(f"[정리 실패] {type(resource).__name__}: {e}")

def get_user_input():
    """사용자 입력 확인 (non-blocking)"""
    if select.select([sys.stdin], [], [], 0)[0]:
        try:
            key = sys.stdin.read(1).lower()
            return key
        except:
            return None
    return None

# ============================================================
# 객체 인식 트리거 처리
# ============================================================
def handle_runtime_triggers(motor, buzzer, speed):
    """주행 중 객체 인식 트리거를 처리합니다.

    현재 속도 프로파일을 받아 갱신된 프로파일을 돌려줍니다.
    """
    if not OBJECT_DETECTION_ENABLED:
        return speed

    current_time = time.time()

    with shared_state.lock:
        obj_state = shared_state.object_state.copy()

    # STOP 표지판 - 즉시 정지 (쿨다운만 체크)
    if obj_state.get("stop"):
        # 중복 실행 체크
        can_execute = True
        with shared_state.lock:
            if "stop" in shared_state.action_last_time:
                time_since = current_time - shared_state.action_last_time["stop"]
                if time_since < shared_state.ACTION_COOLDOWN:
                    can_execute = False

        if can_execute:
            print("[동작] STOP 정지 2초")
            motor.stop()
            time.sleep(config.STOP_SIGN_HOLD_SEC)

            # 재출발만 잠깐 느리게 합니다. 감속 모드(SLOW_SPEED)와 달리 회전
            # 속도는 건드리지 않습니다. 이 사본은 이 구간에만 쓰고 루프의 speed 는
            # 그대로입니다.
            restart = speed.slowed_forward(config.SPEED_SLOW_FORWARD)
            drive(motor, control.FORWARD, restart)
            time.sleep(config.STOP_SIGN_RESTART_SLOW_SEC)

            with shared_state.lock:
                shared_state.action_last_time["stop"] = current_time
                shared_state.object_state["stop"] = False  # 즉시 리셋


    # SLOW 표지판
    elif obj_state.get("slow"):
        try:
            with shared_state.lock:
                # 감속 창이 아직 치워지지 않았으면 다시 걸지 않습니다. 창이 만료되는
                # 프레임에는 아래 파생 블록이 창을 치우기 전이라 여기서 걸리지 않고,
                # 다음 프레임에야 다시 걸립니다. 그 한 프레임 동안 정상 속도로
                # 돌아가는 것이 원본 동작입니다. 시각으로 판정(is_slow_mode)하면
                # 만료 프레임에 곧바로 다시 걸려 표지판이 보이는 동안 감속이 풀리지
                # 않습니다.
                if shared_state.slow_mode_until is None:
                    print("[동작] SLOW 감속 3초")
                    shared_state.slow_mode_until = (
                        current_time + config.SLOW_MODE_DURATION)
                    shared_state.object_state["slow"] = False  # 즉시 리셋
        except:
            pass

    # HORN 표지판 - 학교종 멜로디
    elif obj_state.get("horn"):
        can_execute = True
        with shared_state.lock:
            if "horn" in shared_state.action_last_time:
                time_since = current_time - shared_state.action_last_time["horn"]
                if time_since < shared_state.ACTION_COOLDOWN:
                    can_execute = False

        if can_execute:
            print("[동작] 🎵 학교종 (솔솔라솔)")
            buzzer.play_school_bell()

            with shared_state.lock:
                shared_state.action_last_time["horn"] = current_time
                shared_state.object_state["horn"] = False  # 즉시 리셋


    # 현재 속도는 감속 창에서 파생합니다. 따로 대입하지 않으므로 창과 속도가
    # 어긋날 수 없습니다.
    try:
        with shared_state.lock:
            now = time.time()
            if not control.is_slow_mode(shared_state.slow_mode_until, now):
                shared_state.slow_mode_until = None      # 만료된 창을 치웁니다
            speed = control.profile_for(shared_state.slow_mode_until, now)
    except:
        pass

    return speed

# ============================================================
# 표지판 관리 함수
# ============================================================
def store_direction_signs():
    """인식된 방향 표지판을 큐에 저장합니다."""
    if not OBJECT_DETECTION_ENABLED:
        return

    global last_sign_time
    current_time = time.time()

    # 쿨다운 체크
    if current_time - last_sign_time < SIGN_COOLDOWN:
        return

    with shared_state.lock:
        obj_state = shared_state.object_state.copy()
        confidence = getattr(shared_state, 'confidence', {})

    timestamp = time.strftime("%H:%M:%S")
    direction_signs = ["go_straight", "turn_left", "turn_right", "traffic"]

    for sign in direction_signs:
        if obj_state.get(sign):
            conf = confidence.get(sign, 0) if confidence else 0

            # 방향 표지판별 쿨다운
            with shared_state.lock:
                last_time = shared_state.action_last_time.get(sign, 0)

            if current_time - last_time < shared_state.ACTION_COOLDOWN:
                continue  # 쿨다운 중이면 저장하지 않음

            # 방향 표지판의 마지막 인식 시간 갱신
            with shared_state.lock:
                shared_state.action_last_time[sign] = current_time

            # 신호등은 신뢰도 90% 이상만 저장
            if sign == "traffic" and conf < 0.90:
                continue

            sign_info = {
                'type': sign,
                'confidence': conf,
                'time': current_time,
                'timestamp': timestamp
            }

            # 큐에 저장 (직전과 같은 표지판이면 저장하지 않음)
            if not recognized_signs or recognized_signs[-1]['type'] != sign:
                recognized_signs.append(sign_info)
                last_sign_time = current_time

                sign_names = {
                    "go_straight": "직진",
                    "turn_left": "좌회전",
                    "turn_right": "우회전",
                    "traffic": "신호등"
                }
                print(f"[저장] {sign_names.get(sign, sign)} (큐:{len(recognized_signs)})")

                break

            break

# ============================================================
# 메인 루프
# ============================================================
def lane_follow_loop(motor=None, buzzer=None, camera=None, stop_event=None):
    """통합 라인 트레이서 메인 루프입니다.

    하드웨어를 인자로 받고, 넘기지 않으면 직접 장치를 엽니다.

    소유권 규칙: **만든 쪽이 닫습니다.** 넘겨받은 장치는 닫지 않습니다. main.py 가
    소유한 장치를 여기서 닫으면 main 의 종료 처리가 닫힌 핀을 만지게 됩니다.
    정지(stop)는 소유와 무관하게 항상 합니다. 여러 번 해도 안전합니다.

    stop_event 가 서면 루프를 빠져나옵니다. daemon 스레드는 인터프리터 종료 시
    finally 를 실행하지 못하고 죽으므로, 이 신호로 빠져나와야 모터 정지가
    실행됩니다. 개루프 시퀀스(교차로 회전, 차선 이탈 복구)도 stop_event 를 보므로
    좌회전 9.8초 도중에 종료 요청이 와도 남은 단계를 버리고 빠져나옵니다.

    표지판 대응(최대 2.5초)과 정지선 최종 정지(1.5초)는 아직 time.sleep 이라
    그동안에는 종료 요청에 바로 반응하지 않습니다.
    """
    created = []
    if motor is None:
        motor = hardware.Motor(hardware.create_gpio_pins())
        created.append(motor)
    if buzzer is None:
        buzzer = hardware.Buzzer(hardware.create_gpio_buzzer())
        created.append(buzzer)
    if stop_event is None:
        stop_event = threading.Event()
    print(f"[시스템] 객체인식={'ON' if OBJECT_DETECTION_ENABLED else 'OFF'}")

    if OBJECT_DETECTION_ENABLED:
        try:
            with shared_state.lock:
                _ = shared_state.object_state.copy()
        except Exception as e:
            print(f"[오류] shared_state: {e}")

    if camera is None:
        camera = hardware.create_camera()
        if camera:
            created.append(camera)
    if not camera:
        # 카메라 없이는 주행할 수 없습니다. 직접 연 것만 정리하고 빠져나갑니다.
        # 원본은 핀을 열어 둔 채 반환했습니다.
        cleanup(created)
        return

    frame_count = 0


    # 한쪽 라인 없을 때 직진 타이머
    one_side_missing_time = None
    one_side_missing_direction = None

    # 교차로 상태 (control.IntersectionState — NONE / WAITING / COOLDOWN)
    inter = control.IntersectionState()

    # 현재 속도 프로파일입니다. 감속 창(shared_state.slow_mode_until)에서 파생하므로
    # 루프에 다시 들어와도 진행 중인 감속을 이어받습니다.
    speed = control.profile_for(shared_state.slow_mode_until, time.time())

    # 정지선 2단계 판정 상태입니다. 모듈 전역으로 두면 같은 프로세스의 두 번째
    # 주행이 stage=1 로 시작해 첫 정지선을 두 번째로 오인하고 바로 끝나므로
    # 지역 변수로 둡니다.
    stopline = control.StoplineState()

    # 라인 손실 관련
    line_lost_time = None

    # 높은 픽셀 값 감지 및 후진 모드
    high_pixel_start_time = None  # 높은 픽셀 값 감지 시작 시간
    reverse_mode = False          # 후진 모드 플래그

    try:
        while not stop_event.is_set():
            ret, frame = camera.read()
            if not ret:
                break

            frame_count += 1

            # 이미지 뒤집기
            frame = vision.upright(frame)

            # shared_state에 프레임 전달
            if OBJECT_DETECTION_ENABLED and frame_count % config.FRAME_SHARE_INTERVAL == 0:
                try:
                    with shared_state.lock:
                        shared_state.latest_frame = frame.copy()
                except:
                    pass

            # ====== 방향 표지판을 큐에 저장 ======
            if OBJECT_DETECTION_ENABLED and frame_count % config.FRAME_SHARE_INTERVAL == 0:
                store_direction_signs()

            # ====== Sense — 프레임에서 픽셀 수를 읽습니다 (vision.measure) ======
            if inter.waiting:
                # 교차로 대기 중에는 차선을 보지 않습니다. 원본도 이때 모든
                # 측정값을 0 으로 두었고, Measurements() 의 기본값이 같습니다.
                measured = vision.Measurements()
            else:
                measured = vision.measure(frame)

                if (frame_count % config.BLUE_LOG_INTERVAL_FRAMES == 0
                        and measured.blue_pixels > config.BLUE_LOG_MIN_PIXELS):
                    print(f"[파란색 감지] {measured.blue_pixels}px "
                          f"(임계값: {config.BLUE_PIXEL_THRESHOLD}px)")


            # ====== 높은 픽셀 값 감지 및 후진 처리 (control.check_reverse) ======
            reverse_state = control.check_reverse(
                measured.left_pixels, measured.right_pixels,
                control.ReverseState(reverse_mode, high_pixel_start_time),
                time.time())
            reverse_mode = reverse_state.reverse_mode
            high_pixel_start_time = reverse_state.high_since

            # ====== 후진 모드 실행 ======
            if reverse_mode:
                motor.backward(speed.forward)
                time.sleep(config.REVERSE_LOOP_DELAY_SEC)
                continue


            # ====== 교차로 모드에서 키보드 입력 처리 ======
            if inter.waiting:
                # 표지판 큐에서 꺼내는 것은 부작용이라 여기서 합니다.
                # control.plan_intersection 은 꺼낸 문자열만 받습니다.
                sign_type = None
                if OBJECT_DETECTION_ENABLED and recognized_signs:
                    sign_type = recognized_signs.popleft()['type']

                # get_user_input() 은 stdin 에서 한 글자를 소비합니다. 표지판이
                # 있거나 타임아웃이 성립한 프레임에서는 읽지 않습니다. 읽어도
                # 우선순위에서 밀려 버려지므로 조작자가 누른 키가 사라집니다.
                # 원본도 두 경우에 stdin 을 읽지 않았습니다.
                now = time.time()
                key = None
                if sign_type is None and not control.intersection_timed_out(inter, now):
                    key = get_user_input()

                maneuver, source = control.plan_intersection(
                    inter, sign_type, key, now)

                if maneuver is None:
                    motor.stop()          # 지시를 기다리는 중
                    continue

                # 키보드 입력에는 로그를 찍지 않습니다(원본과 같음). 원본의 신호등
                # 로그 두 줄("3초 대기 중", "우회전 시작")은 시퀀스를 표로 바꾸면서
                # 없어졌습니다.
                if source is control.FROM_TIMEOUT:
                    print("[타임아웃] 자동 직진")
                elif source is control.FROM_SIGN:
                    print(f"[실행] {control.MANEUVER_NAMES[maneuver]}")

                # 완주하지 못했으면 쿨다운 갱신과 상태 전이를 하지 않습니다.
                # 수행하지 않은 지시를 수행 완료로 기록하면 안 됩니다.
                if not run_steps(motor, control.steps_for(maneuver, source),
                                 stop_event, speed):
                    break

                if source is control.FROM_SIGN:
                    with shared_state.lock:
                        shared_state.action_last_time[sign_type] = time.time()
                    recognized_signs.clear()

                inter = control.exit_intersection(time.time())
                continue

            # ====== 교차로 통과 직후면 일정 시간 재감지를 억제합니다 ======
            inter = control.tick_intersection(inter, time.time())


            # ====== 이 프레임에서 어느 갈래로 갈지 (control.select_branch) ======
            # 원본은 여기부터 if/elif 체인이었습니다. 갈래 선택을 순수 함수로 분리해
            # 각 갈래에 도달하는지를 테스트로 확인합니다.
            branch = control.select_branch(measured, inter)

            if branch is control.Branch.STOPLINE:
                decision = control.check_stopline(
                    measured.blue_pixels, stopline, time.time())
                stopline = decision.state

                if decision.passed_first:
                    print(f"[정지선] 첫 번째 정지선 감지 ({measured.blue_pixels}px) → 계속 진행")
                if decision.should_stop:
                    print(f"\n[정지선] 두 번째 정지선 감지: {measured.blue_pixels}px")
                    print("[종료] 정지선 사이 → 최종 정지 및 종료")
                    motor.stop()
                    time.sleep(config.STOPLINE_FINAL_STOP_SEC)
                    break

            # ====== 교차로 감지 ======
            elif branch is control.Branch.INTERSECTION:
                # 원본에는 여기 `if not intersection_mode:` 재진입 가드가 있었습니다.
                # inter.detecting 이 이미 NONE 을 보장하므로(WAITING 이면 위에서
                # continue 합니다) 항상 참이라 뺐습니다.
                motor.stop()
                inter = control.enter_intersection(time.time())

                queue_info = f" 큐:{len(recognized_signs)}" if recognized_signs else ""
                print(f"[교차로]{queue_info}")

            # ====== 라인 이탈 ======
            elif branch is control.Branch.LINE_LOST:
                if line_lost_time is None:
                    line_lost_time = time.time()
                    motor.stop()

                # 수동 복구입니다. 교차로와 같은 지시를 쓰지만 시간 표가 다릅니다
                # (control.LINELOST_STEPS 참고). 매핑에 없는 문자는 무시합니다.
                # 원본도 여기서는 탈출 처리가 없어 아무 일도 일어나지 않았습니다.
                maneuver = control.KEY_TO_MANEUVER.get(get_user_input())
                if maneuver is not None:
                    if not run_steps(motor, control.LINELOST_STEPS[maneuver],
                                     stop_event, speed):
                        break

            # ====== 라인 트레이싱 ======
            else:
                if line_lost_time is not None:
                    line_lost_time = None

                # 조향 결정 (control.decide_steering — 순수 함수)
                steering = control.decide_steering(
                    measured.left_pixels, measured.right_pixels, speed.forward,
                    control.OneSideState(one_side_missing_time,
                                         one_side_missing_direction),
                    time.time())
                one_side_missing_time = steering.one_side.missing_since
                one_side_missing_direction = steering.one_side.direction

                # drive() 를 거쳐 동작 디스패치를 한 곳으로 모읍니다.
                drive(motor, steering.action, speed, steering.intensity)

                # 주행 중 객체 인식 트리거 처리
                speed = handle_runtime_triggers(motor, buzzer, speed)

            # time.sleep 대신 wait 를 써서 종료 신호에 바로 반응합니다.
            # 루프는 시간 대부분을 여기서 보냅니다.
            stop_event.wait(config.LANE_LOOP_DELAY_SEC)

    except KeyboardInterrupt:
        # 이 함수를 단독 실행(python3 lane_tracer.py)할 때만 옵니다.
        # main.py 경로에서는 SIGINT 가 메인 스레드로만 갑니다.
        print("\n[종료]")

    finally:
        # stop_event 로 빠져나오므로 종료 시 이 블록이 실제로 실행됩니다.
        # 개루프 시퀀스 중에 종료해도 run_steps 가 중단하고 여기로 옵니다.
        # 정지는 소유와 무관하게 항상, 닫기는 직접 만든 것만 합니다.
        try:
            motor.stop()
        except Exception as e:
            print(f"[정리 실패] motor.stop: {e}")
        cleanup(created)

if __name__ == '__main__':
    lane_follow_loop()
