"""
lane_tracer.py
--------------
"""

import cv2
import numpy as np
import time
import sys
import select
from gpiozero import DigitalOutputDevice, PWMOutputDevice
import shared_state

# ============================================================
# 모터 / 부저 설정
# ============================================================

PWMA = PWMOutputDevice(18)
AIN1 = DigitalOutputDevice(22)
AIN2 = DigitalOutputDevice(27)

PWMB = PWMOutputDevice(23)
BIN1 = DigitalOutputDevice(25)
BIN2 = DigitalOutputDevice(24)

# 부저가 없으면 필요 시 예외 처리
try:
    BUZZER = DigitalOutputDevice(12)
except Exception:
    BUZZER = None

# 기본/감속 속도 프로파일
SPEED_FORWARD_DEFAULT = 0.75
SPEED_TURN_DEFAULT    = 0.55
SPEED_SLOW_FORWARD    = 0.25   # “20으로 줄이기”에 대응하는 PWM 근사치
SPEED_SLOW_TURN       = 0.20

# 현재 속도 설정 (가변)
SPEED_FORWARD = SPEED_FORWARD_DEFAULT
SPEED_TURN    = SPEED_TURN_DEFAULT

# ============================================================
# 모터 제어 함수
# ============================================================

def motor_forward():
    AIN1.value = 0
    AIN2.value = 1
    PWMA.value = SPEED_FORWARD
    BIN1.value = 0
    BIN2.value = 1
    PWMB.value = SPEED_FORWARD

def motor_left():
    AIN1.value = 0
    AIN2.value = 1
    PWMA.value = SPEED_TURN * 0.25
    BIN1.value = 0
    BIN2.value = 1
    PWMB.value = SPEED_TURN

def motor_right():
    AIN1.value = 0
    AIN2.value = 1
    PWMA.value = SPEED_TURN
    BIN1.value = 0
    BIN2.value = 1
    PWMB.value = SPEED_TURN * 0.25

def motor_stop():
    AIN1.value = 0
    AIN2.value = 1
    PWMA.value = 0.0
    BIN1.value = 1
    BIN2.value = 0
    PWMB.value = 0.0

# 속도 변경/복구
def set_slow_mode():
    global SPEED_FORWARD, SPEED_TURN
    SPEED_FORWARD = SPEED_SLOW_FORWARD
    SPEED_TURN    = SPEED_SLOW_TURN

def restore_speed():
    global SPEED_FORWARD, SPEED_TURN
    SPEED_FORWARD = SPEED_FORWARD_DEFAULT
    SPEED_TURN    = SPEED_TURN_DEFAULT

def beep(sec=1.0):
    if BUZZER is None:
        print("🔊 (buzzer not available)")
        time.sleep(sec)
        return
    try:
        BUZZER.value = 1
        time.sleep(sec)
    finally:
        BUZZER.value = 0

def get_user_input():
    """사용자 입력 확인 (non-blocking)"""
    if select.select([sys.stdin], [], [], 0)[0]:
        try:
            return sys.stdin.read(1).lower()
        except:
            return None
    return None

# ============================================================
# 카메라 초기화
# ============================================================

def init_camera():
    try:
        from picamera2 import Picamera2
        print("[INFO] Initializing camera...")

        picam2 = Picamera2()
        config = picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (640, 480)}
        )
        picam2.configure(config)
        picam2.start()
        time.sleep(2)
        print("[✓] Camera ready")

        class CameraWrapper:
            def read(self):
                frame = picam2.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                return True, frame
            def release(self):
                picam2.stop()

        return CameraWrapper()
    except Exception as e:
        print(f"[ERROR] Camera failed: {e}")
        return None

# ============================================================
# 트리거 처리 (일반 주행 중: stop/slow/horn)
# ============================================================

def handle_runtime_triggers():
    """
    일반 주행(CRUISE) 중 수시로 호출.
    stop/slow/horn 처리. 처리했다면 True 리턴.
    """
    handled = False
    with shared_state.lock:
        trig = shared_state.last_trigger

    if not trig:
        return handled

    if trig == "stop":
        print("🛑 STOP sign → 정지 3초")
        motor_stop()
        time.sleep(3)
        handled = True

    elif trig == "slow":
        print("🐢 SLOW sign → 감속 3초")
        set_slow_mode()
        motor_forward()
        time.sleep(3)
        restore_speed()
        handled = True

    elif trig == "horn":
        print("🔊 HORN sign → 경적 1초")
        beep(1.0)
        handled = True

    # 처리한 트리거는 비우기
    if handled:
        with shared_state.lock:
            shared_state.last_trigger = None
    return handled

def try_branch_by_trigger():
    """
    수동 대기 루프에서 호출.
    go_straight / turn_left / turn_right 가 감지되면 해당 동작 수행 후 True.
    """
    acted = False
    with shared_state.lock:
        trig = shared_state.last_trigger

    if trig in ("go_straight", "turn_left", "turn_right"):
        print(f"🚧 갈림길 표지판 인식: {trig} → 자동 수행")
        motor_stop(); time.sleep(1.0)

        if trig == "go_straight":
            motor_forward(); time.sleep(1.5)

        elif trig == "turn_left":
            motor_left(); time.sleep(1.2)
            motor_forward(); time.sleep(1.0)

        elif trig == "turn_right":
            motor_right(); time.sleep(1.2)
            motor_forward(); time.sleep(1.0)

        # 트리거 소모
        with shared_state.lock:
            shared_state.last_trigger = None
        acted = True

    return acted

# ============================================================
# 신호등 상태머신 파라미터
# ============================================================

TL_APPROACH_AREA = 8000     # 신호등 인지 시작
TL_STOPLINE_AREA = 25000    # 정지선 근처(일단 정지)
T_RED_TIMEOUT    = 10.0     # 빨간불 대기 최대 시간 (트랙 한정 백업)
RIGHT_STOP_SEC   = 3.0      # 초록 + 우회전 시 3초 정지
TURN_RIGHT_SEC   = 0.8      # 우회전 유지 시간(코스에 맞게 튜닝)

# ============================================================
# 메인 루프 (스레드용)
# ============================================================

def lane_follow_loop():
    print("=" * 70)
    print(" Line Tracer (Corner Detection) + Traffic Light FSM")
    print("=" * 70)
    print("[INFO] Press Ctrl+C to stop\n")

    camera = init_camera()
    if not camera:
        return

    lower_cyan = np.array([65, 20, 20])
    upper_cyan = np.array([115, 255, 255])

    start_time = time.time()
    frame_count = 0
    action_stats = {"FORWARD": 0, "LEFT": 0, "RIGHT": 0, "STOP": 0, "MANUAL": 0}

    BALANCE_THRESHOLD = 0.35
    BOX_WIDTH, BOX_HEIGHT = 160, 120
    LINE_LOST_THRESHOLD = 2.0
    PIXEL_THRESHOLD = 1200  # updated threshold for 640x480

    last_seen_side = None
    line_lost_time = None

    # 신호등 상태머신
    state = "CRUISE"
    red_since = None
    right_wait_since = None

    try:
        while True:
            ret, frame = camera.read()
            if not ret:
                print("[ERROR] Failed to read frame")
                break
            # 최신 프레임 공유
            with shared_state.lock:
                shared_state.latest_frame = frame.copy()


            frame_count += 1
            frame = cv2.flip(frame, -1)
            height, width = frame.shape[:2]

            # 좌/우 하단 박스 추출
            left_box  = frame[height-BOX_HEIGHT:height, 0:BOX_WIDTH]
            right_box = frame[height-BOX_HEIGHT:height, width-BOX_WIDTH:width]

            hsv_left  = cv2.cvtColor(left_box,  cv2.COLOR_BGR2HSV)
            hsv_right = cv2.cvtColor(right_box, cv2.COLOR_BGR2HSV)
            mask_left  = cv2.inRange(hsv_left,  lower_cyan, upper_cyan)
            mask_right = cv2.inRange(hsv_right, lower_cyan, upper_cyan)

            kernel = np.ones((3, 3), np.uint8)
            mask_left  = cv2.erode(mask_left,  kernel, iterations=2)
            mask_left  = cv2.dilate(mask_left, kernel, iterations=3)
            mask_right = cv2.erode(mask_right, kernel, iterations=2)
            mask_right = cv2.dilate(mask_right, kernel, iterations=3)

            left_pixels  = cv2.countNonZero(mask_left)
            right_pixels = cv2.countNonZero(mask_right)
            total_pixels = left_pixels + right_pixels

            if total_pixels > 0:
                left_ratio  = left_pixels / total_pixels
                right_ratio = right_pixels / total_pixels
            else:
                left_ratio = right_ratio = 0.0

            diff = abs(left_ratio - right_ratio)
            action = "STOP"

            # -----------------------------
            # 신호등 상태 스냅샷
            # -----------------------------
            with shared_state.lock:
                tl       = getattr(shared_state, "traffic_light", None)           # 'red' | 'green' | None
                tl_vis   = getattr(shared_state, "traffic_light_visible", False)  # 초록 가시성
                tl_area  = getattr(shared_state, "traffic_light_area", 0)
                tl_ts    = getattr(shared_state, "traffic_light_last_ts", 0.0)
                trig_now = shared_state.last_trigger

            # -----------------------------
            # 상태머신 전이
            # -----------------------------
            if state == "CRUISE":
                # 일반 표지판(정지/감속/경적)은 CRUISE에서만 즉시 처리
                handle_runtime_triggers()

                # 신호등 접근 판단
                if tl is not None and tl_area >= TL_APPROACH_AREA:
                    print("🚦 TL_APPROACH 진입")
                    state = "TL_APPROACH"

                # 라인이 보일 때 일반 주행
                elif total_pixels >= PIXEL_THRESHOLD:
                    if diff < BALANCE_THRESHOLD:
                        motor_forward(); action = "FORWARD"
                    elif left_pixels > right_pixels:
                        motor_right();  action = "RIGHT"
                        last_seen_side = 'LEFT'
                    else:
                        motor_left();   action = "LEFT"
                        last_seen_side = 'RIGHT'
                else:
                    # 라인 이탈 처리 (갈림길/수동모드)
                    if line_lost_time is None:
                        line_lost_time = time.time()
                    lost_duration = time.time() - line_lost_time

                    if lost_duration >= LINE_LOST_THRESHOLD:
                        motor_stop()
                        action = "WAIT_INPUT"
                        print(f"\n⚠ 라인 {lost_duration:.1f}초 이탈 - 수동/표지판 모드 전환")
                        waiting = True

                        while waiting:
                            # 갈림길 표지판 자동 처리 시도
                            if try_branch_by_trigger():
                                print("✓ 표지판 지시 수행 → 자동 모드 복귀")
                                line_lost_time = None
                                waiting = False
                                break

                            # 수동 입력도 백업 유지
                            user_input = get_user_input()
                            if user_input == 'a':
                                print("→ 좌회전(수동)"); motor_forward(); time.sleep(0.5); motor_left()
                            elif user_input == 'd':
                                print("→ 우회전(수동)"); motor_forward(); time.sleep(0.5); motor_right()
                            elif user_input == 'w':
                                print("→ 직진(수동)"); motor_forward()

                            # 라인 복귀 확인
                            ret2, temp = camera.read()
                            if not ret2: break
                            temp = cv2.flip(temp, -1)
                            th, tw = temp.shape[:2]
                            t_left  = temp[th-BOX_HEIGHT:th, 0:BOX_WIDTH]
                            t_right = temp[th-BOX_HEIGHT:th, tw-BOX_WIDTH:tw]
                            t_left_pixels  = cv2.countNonZero(cv2.inRange(cv2.cvtColor(t_left,  cv2.COLOR_BGR2HSV), lower_cyan, upper_cyan))
                            t_right_pixels = cv2.countNonZero(cv2.inRange(cv2.cvtColor(t_right, cv2.COLOR_BGR2HSV), lower_cyan, upper_cyan))
                            t_total = t_left_pixels + t_right_pixels
                            if t_total >= PIXEL_THRESHOLD:
                                print("✓ 라인 재발견! 자동 모드 복귀")
                                line_lost_time = None
                                waiting = False
                            time.sleep(0.05)
                    else:
                        motor_stop()
                        action = "STOP"

            elif state == "TL_APPROACH":
                set_slow_mode()
                motor_forward()
                # 정지선 근처 도달 → 일단 정지하고 신호 판단
                if tl_area >= TL_STOPLINE_AREA:
                    motor_stop()
                    restore_speed()
                    if tl == "red":
                        print("🛑 빨간불 → 대기 진입")
                        red_since = time.time()
                        state = "TL_STOP_RED"
                    elif tl == "green":
                        # 초록이라도 가시성 필터 확인
                        if tl_vis:
                            # 우회전 의도면 규칙② 적용 (3초 정지)
                            if trig_now == "turn_right":
                                print("🟢 초록 + 우회전 표지판 → 3초 정지 후 우회전")
                                right_wait_since = time.time()
                                state = "TL_RIGHT_WAIT"
                            else:
                                print("🟢 초록 확인 → 직진 출발")
                                state = "TL_GO_GREEN"
                        else:
                            print("🟢 초록 분류되었지만 가시성 부족 → 대기")
                            red_since = time.time()
                            state = "TL_STOP_RED"
                else:
                    # 정지선 전까지는 저속 주행 or 정지 유지 (필요 시 보정)
                    continue

            elif state == "TL_STOP_RED":
                motor_stop()
                # 백업 플랜: 10초 대기 후 출발 (트랙 한정)
                if red_since and (time.time() - red_since >= T_RED_TIMEOUT):
                    print("⏱️ 빨간불 10초 경과 → 백업 출발")
                    state = "TL_GO_GREEN"
                # 정상 전이: 초록 + 가시성 OK
                elif tl == "green" and tl_vis:
                    print("🟢 초록 전환 확인 → 출발")
                    state = "TL_GO_GREEN"

            elif state == "TL_GO_GREEN":
                motor_forward(); action = "FORWARD"
                # 라인이 충분히 보이면 CRUISE 복귀
                if total_pixels >= PIXEL_THRESHOLD:
                    print("↩️ 라인 복귀 → CRUISE")
                    state = "CRUISE"

            elif state == "TL_RIGHT_WAIT":
                motor_stop(); action = "STOP"
                if time.time() - right_wait_since >= RIGHT_STOP_SEC:
                    print("↪️ 우회전 실행")
                    state = "TL_RIGHT_TURN"

            elif state == "TL_RIGHT_TURN":
                motor_right(); time.sleep(TURN_RIGHT_SEC)
                motor_forward(); time.sleep(0.5)
                if total_pixels >= PIXEL_THRESHOLD:
                    print("↩️ 라인 복귀 → CRUISE")
                    state = "CRUISE"

            # 통계/로그
            action_stats[action if not action.startswith("WAIT") else "MANUAL"] += 1
            runtime = int(time.time() - start_time)
            icon = {"FORWARD":"↑","LEFT":"←","RIGHT":"→","STOP":"■","WAIT_INPUT":"⏸",
                    "TL_APPROACH":"🅿", "TL_STOP_RED":"■", "TL_GO_GREEN":"↑",
                    "TL_RIGHT_WAIT":"⏸", "TL_RIGHT_TURN":"↪"}.get(state, "■")
            balance_bar = create_balance_bar(left_ratio, right_ratio)
            print(f"[{runtime:3d}s] F:{frame_count:5d} | "
                  f"L:{left_pixels:4d} R:{right_pixels:4d} Tot:{total_pixels:4d} | "
                  f"{balance_bar} | D:{diff:.2f} | {icon} {state:12s}")

            time.sleep(0.03)

    except KeyboardInterrupt:
        print("\n[INFO] Lane tracer stopped by user.")

    finally:
        motor_stop()
        camera.release()
        print("[✓] Lane tracer cleanup complete")

# ============================================================
# 보조 함수
# ============================================================

def create_balance_bar(left_ratio, right_ratio):
    bar_length = 20
    left_bars = int(left_ratio * bar_length)
    right_bars = int(right_ratio * bar_length)
    return "L[" + "█"*left_bars + " "*(bar_length-left_bars) + "|" + \
           "█"*right_bars + " "*(bar_length-right_bars) + "]R"
