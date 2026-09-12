"""
Line Tracer Integrated - 통합 라인 트레이서
교차로 감지 + 객체 인식 + 키보드 제어 통합 시스템

수정 사항 (2024-11-12):
- main() 함수명을 lane_follow_loop()으로 변경 (main.py와 연동)
- HSV 색상 범위 확장: [65, 20, 20] ~ [115, 255, 255]
- 픽셀 임계값 고정값 사용: 1200
- 라인 트레이싱 로직 단순화

수정 사항 (최신):
- 파란색 픽셀 감지 기능 추가 (종료 조건)
  * BLUE_PIXEL_THRESHOLD: 파란색 픽셀 임계값 (기본값: 5000px, 사용자 수정 가능)
  * 감지 영역: 화면 하단 1/4 (75% ~ 100% 영역)
  * 교차로에서 파란색 종료선 감지 시 프로그램 자동 종료
  * 파란색 감지 시 픽셀 값을 로그에 실시간 출력
  
- 부저를 PWM 방식으로 변경하여 큰 소리 출력
- 로깅 시스템 추가
  * 파일 로그: /home/keonha/AI_CAR/logs/ (DEBUG 레벨, 모든 상세 정보)
  * 콘솔 로그: INFO 레벨 (주요 이벤트만)
  * 타임스탬프, 로그 레벨, 메시지 포함
"""
import cv2
import numpy as np
import time
import sys
import select
from gpiozero import DigitalOutputDevice, PWMOutputDevice
from collections import deque

# shared_state import 시도
try:
    import shared_state
    OBJECT_DETECTION_ENABLED = True
except ImportError:
    OBJECT_DETECTION_ENABLED = False
    pass

# ============================================================
# 표지판 인식 큐 시스템
# ============================================================
recognized_signs = deque(maxlen=5)  # 최근 5개 표지판만 저장
last_sign_time = 0  # 마지막 표지판 인식 시간
SIGN_COOLDOWN = 3.0  # 동일 표지판 재인식 방지 시간 (초)

# ============================================================
# 파란색 픽셀 감지 (프로그램 종료 조건)
# ============================================================
BLUE_PIXEL_THRESHOLD = 25000  # 파란색 픽셀 임계값 (사용자 수정 가능)
stopline_stage = 0
first_stopline_time = 0

# ============================================================
# (삭제됨) 객체 감지는 바로 실행 - 프레임 체크 없음
# ============================================================

# ============================================================
# 모터 / 부저 설정 (Lazy Initialization)
# ============================================================
# GPIO 객체들을 None으로 초기화 (실제 초기화는 init_gpio에서)
PWMA = None
AIN1 = None
AIN2 = None

PWMB = None
BIN1 = None
BIN2 = None

BUZZER = None

def init_gpio():
    """GPIO 초기화 - 프로그램 시작 시 한 번 호출"""
    global PWMA, AIN1, AIN2, PWMB, BIN1, BIN2, BUZZER

    try:
        # 기존 GPIO 정리 (있다면)
        if PWMA is not None:
            PWMA.close()
        if AIN1 is not None:
            AIN1.close()
        if AIN2 is not None:
            AIN2.close()
        if PWMB is not None:
            PWMB.close()
        if BIN1 is not None:
            BIN1.close()
        if BIN2 is not None:
            BIN2.close()
        if BUZZER is not None:
            BUZZER.close()
    except:
        pass

    # 새로 초기화
    PWMA = PWMOutputDevice(18)
    AIN1 = DigitalOutputDevice(22)
    AIN2 = DigitalOutputDevice(27)

    PWMB = PWMOutputDevice(23)
    BIN1 = DigitalOutputDevice(25)
    BIN2 = DigitalOutputDevice(24)

    # 부저 설정 (PWM으로 큰 소리)
    try:
        BUZZER = PWMOutputDevice(12, frequency=2000, initial_value=0)
    except Exception:
        BUZZER = None
        pass

# 속도 프로파일 (최적화 버전 - 안정성과 회전력 균형)
SPEED_FORWARD_DEFAULT = 0.225  # 기본 직진 속도 (0.40 → 0.45, 약간 증가)
SPEED_TURN_DEFAULT = 0.30     # 기본 회전 속도 (0.55 → 0.60, 회전력 강화)
SPEED_SLOW_FORWARD = 0.20     # 감속 직진 (기존 0.25)
SPEED_SLOW_TURN = 0.16         # 감속 회전 (기존 0.20)

# 현재 속도 (동적 변경용)
SPEED_FORWARD = SPEED_FORWARD_DEFAULT
SPEED_TURN = SPEED_TURN_DEFAULT

# ============================================================
# 모터 제어 함수
# ============================================================
def motor_forward():
    """전진"""
    AIN1.value = 0
    AIN2.value = 1
    PWMA.value = SPEED_FORWARD
    BIN1.value = 0
    BIN2.value = 1
    PWMB.value = SPEED_FORWARD

def motor_left(intensity=1.0):
    """좌회전 - intensity로 회전 강도 조절 (0.0~1.0)"""
    # 급격한 회전: 안쪽 바퀴를 후진시킴 (intensity > 0.5일 때)
    if intensity > 0.5:
        # 제자리 회전에 가까운 동작
        AIN1.value = 1  # 왼쪽 후진
        AIN2.value = 0
        PWMA.value = SPEED_TURN * 0.3 * intensity
        BIN1.value = 0  # 오른쪽 전진
        BIN2.value = 1
        PWMB.value = SPEED_TURN * 1.2 * intensity
    else:
        # 일반 회전: 안쪽 바퀴 느리게
        left_ratio = 0.0  # 안쪽 바퀴 정지
        right_ratio = 1.2 * intensity  # 바깥쪽 바퀴 더 빠르게
        AIN1.value = 0
        AIN2.value = 1
        PWMA.value = SPEED_TURN * left_ratio
        BIN1.value = 0
        BIN2.value = 1
        PWMB.value = SPEED_TURN * right_ratio

def motor_right(intensity=1.0):
    """우회전 - intensity로 회전 강도 조절 (0.0~1.0)"""
    # 급격한 회전: 안쪽 바퀴를 후진시킴 (intensity > 0.5일 때)
    if intensity > 0.5:
        # 제자리 회전에 가까운 동작
        AIN1.value = 0  # 왼쪽 전진
        AIN2.value = 1
        PWMA.value = SPEED_TURN * 1.2 * intensity
        BIN1.value = 1  # 오른쪽 후진
        BIN2.value = 0
        PWMB.value = SPEED_TURN * 0.3 * intensity
    else:
        # 일반 회전: 안쪽 바퀴 느리게
        left_ratio = 1.2 * intensity  # 바깥쪽 바퀴 더 빠르게
        right_ratio = 0.0  # 안쪽 바퀴 정지
        AIN1.value = 0
        AIN2.value = 1
        PWMA.value = SPEED_TURN * left_ratio
        BIN1.value = 0
        BIN2.value = 1
        PWMB.value = SPEED_TURN * right_ratio

def motor_stop():
    """정지 - 완전한 브레이크 모드"""
    AIN1.value = 0
    AIN2.value = 0  # 왼쪽 모터 브레이크
    PWMA.value = 0.0
    BIN1.value = 0
    BIN2.value = 0  # 오른쪽 모터 브레이크
    PWMB.value = 0.0

def motor_backward():
    """후진 - 비정상 픽셀 값 감지 시"""
    AIN1.value = 1
    AIN2.value = 0
    PWMA.value = SPEED_FORWARD * 0.5  # 느리게 후진
    BIN1.value = 1
    BIN2.value = 0
    PWMB.value = SPEED_FORWARD * 0.5

def set_slow_mode():
    """감속 모드 설정"""
    global SPEED_FORWARD, SPEED_TURN
    SPEED_FORWARD = SPEED_SLOW_FORWARD
    SPEED_TURN = SPEED_SLOW_TURN

def restore_speed():
    """정상 속도 복원"""
    global SPEED_FORWARD, SPEED_TURN
    SPEED_FORWARD = SPEED_FORWARD_DEFAULT
    SPEED_TURN = SPEED_TURN_DEFAULT

def beep(sec=3.0, volume=1.0):
    """부저 울리기 - 단순 비프음 (최대 볼륨)"""
    if BUZZER:
        BUZZER.value = 1.0  # 최대 볼륨 고정
        time.sleep(sec)
        BUZZER.value = 0
    else:
        time.sleep(sec)

def play_school_bell():
    """학교종 - 매우 크고 또렷한 버전! 🔊"""
    if not BUZZER:
        time.sleep(1.5)
        return

    # 음표 길이 대폭 증가 (훨씬 더 길고 크게!)
    short_beep = 0.45   # 짧은 음 (0.25 → 0.45초, +80%)
    long_beep = 0.60    # 긴 음 (0.35 → 0.60초, +71%)
    pause = 0.05        # 음 사이 쉼표 (짧게 → 빠른 템포로 강렬하게)

    # 학교종: 솔 솔 라(길게) 솔 - 4음
    melody = [
        (short_beep, pause),  # 솔
        (short_beep, pause),  # 솔
        (long_beep, pause),   # 라 (강조)
        (short_beep, 0)       # 솔
    ]

    for beep_time, rest_time in melody:
        BUZZER.value = 1.0  # 최대 볼륨 (100% duty cycle)
        time.sleep(beep_time)
        BUZZER.value = 0
        if rest_time > 0:
            time.sleep(rest_time)



# ============================================================
# 유틸리티 함수
# ============================================================
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
# 객체 인식 트리거 처리 (간소화 버전)
# ============================================================
def handle_runtime_triggers():
    """주행 중 객체 인식 트리거 처리 - 즉시 실행"""
    global SPEED_FORWARD

    if not OBJECT_DETECTION_ENABLED:
        return False

    handled = False
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
            motor_stop()
            time.sleep(2.0)

            old_speed = SPEED_FORWARD
            SPEED_FORWARD = SPEED_SLOW_FORWARD
            motor_forward()
            time.sleep(0.5)
            SPEED_FORWARD = old_speed

            with shared_state.lock:
                shared_state.action_last_time["stop"] = current_time
                shared_state.object_state["stop"] = False  # 즉시 리셋

        handled = True

    # SLOW 표지판
    elif obj_state.get("slow"):
        try:
            with shared_state.lock:
                if not getattr(shared_state, 'slow_mode_active', False):
                    print("[동작] SLOW 감속 3초")
                    set_slow_mode()
                    shared_state.slow_mode_until = time.time() + 3.0
                    shared_state.slow_mode_active = True
                    shared_state.object_state["slow"] = False  # 즉시 리셋
        except:
            pass
        handled = True

    # HORN 표지판 - 재밌는 멜로디 재생! 🎵
    elif obj_state.get("horn"):
        can_execute = True
        with shared_state.lock:
            if "horn" in shared_state.action_last_time:
                time_since = current_time - shared_state.action_last_time["horn"]
                if time_since < shared_state.ACTION_COOLDOWN:
                    can_execute = False

        if can_execute:
            print("[동작] 🎵 학교종 (솔솔라솔)")
            #honk_strong()
            play_school_bell()  # 짧은 학교종 멜로디

            with shared_state.lock:
                shared_state.action_last_time["horn"] = current_time
                shared_state.object_state["horn"] = False  # 즉시 리셋

        handled = True

    # SLOW 모드 자동 해제 체크
    try:
        with shared_state.lock:
            slow_until = getattr(shared_state, 'slow_mode_until', None)
            if slow_until and time.time() > slow_until:
                restore_speed()
                shared_state.slow_mode_until = None
                shared_state.slow_mode_active = False
    except:
        pass

    return handled

# ============================================================
# 카메라 초기화
# ============================================================
def init_camera():
    """카메라 초기화 - 640x480 해상도"""
    max_retries = 3
    retry_delay = 2

    for attempt in range(max_retries):
        try:
            from picamera2 import Picamera2

            if attempt > 0:
                time.sleep(retry_delay)
                import subprocess
                subprocess.run(['pkill', '-f', 'libcamera'], capture_output=True)
                time.sleep(0.5)

            picam2 = Picamera2()
            config = picam2.create_preview_configuration(
                main={"format": "RGB888", "size": (640, 480)}
            )
            picam2.configure(config)
            picam2.start()
            time.sleep(2)

            class CameraWrapper:
                def read(self):
                    frame = picam2.capture_array()
                    return True, frame

                def release(self):
                    picam2.stop()

            return CameraWrapper()

        except Exception as e:
            if "Pipeline handler in use" in str(e) and attempt == max_retries - 1:
                print(f"[오류] 카메라 사용중")

    return None

# ============================================================
# 표지판 관리 함수 (간소화)
# ============================================================
def store_direction_signs():
    """방향 표지판을 인식하여 큐에 저장 - 즉시 저장"""
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

            # -------------------------
            # 🔥 방향 표지판 쿨다운 추가
            # -------------------------
            with shared_state.lock:
                last_time = shared_state.action_last_time.get(sign, 0)

            if current_time - last_time < shared_state.ACTION_COOLDOWN:
                continue  # 쿨다운 중이면 저장하지 않음

            # 🔥 방향 표지판의 마지막 인식 시간 갱신
            with shared_state.lock:
                shared_state.action_last_time[sign] = current_time
            # -------------------------

            # 신호등은 신뢰도 90% 이상만 저장
            if sign == "traffic" and conf < 0.90:
                continue

            sign_info = {
                'type': sign,
                'confidence': conf,
                'time': current_time,
                'timestamp': timestamp
            }

            # 큐에 저장 (중복 방지)
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

                # 저장 후 즉시 리셋하여 중복 방지
                #with shared_state.lock:
                    #shared_state.object_state[sign] = False

                break
            

            break

# ============================================================
# 메인 루프
# ============================================================
def lane_follow_loop():
    """통합 라인 트레이서 메인 루프"""
    # GPIO 초기화
    init_gpio()
    print(f"[시스템] 객체인식={'ON' if OBJECT_DETECTION_ENABLED else 'OFF'}")

    if OBJECT_DETECTION_ENABLED:
        try:
            with shared_state.lock:
                _ = shared_state.object_state.copy()
        except Exception as e:
            print(f"[오류] shared_state: {e}")

    camera = init_camera()
    if not camera:
        return

    # HSV 범위 - 청록색(Cyan) 라인용 (확장된 범위)
    lower_cyan = np.array([65, 20, 20])
    upper_cyan = np.array([115, 255, 255])

    # HSV 범위 - 파란색 감지용 (종료 조건)
    # 파란색은 HSV에서 100-130도 범위
    lower_blue = np.array([100, 100, 100])   # 파란색 범위
    upper_blue = np.array([130, 255, 255])

    frame_count = 0

    # 동적 균형 임계값 (속도 기반 - 개선된 버전)
    BASE_BALANCE_THRESHOLD = 0.35  # 기본 균형 임계값 (저속/중속)
    HIGH_SPEED_BALANCE_THRESHOLD = 0.25  # 고속 시 균형 임계값 (더 민감)

    # 박스 크기 설정 (해상도에 맞춰)
    BOX_WIDTH_RATIO = 0.25   # 화면 너비의 25%
    BOX_HEIGHT_RATIO = 0.25  # 화면 높이의 25%

    # 박스 크기 초기화 (640x480 기준)
    width = 640
    height = 480
    BOX_WIDTH = int(width * BOX_WIDTH_RATIO)
    BOX_HEIGHT = int(height * BOX_HEIGHT_RATIO)

    # 픽셀 임계값 (고정값)
    PIXEL_THRESHOLD = 800  # 라인 감지 임계값 (더 민감하게 조정)
    CENTER_THRESHOLD = 5000  # 교차로 감지 임계값 (고정) (5000->9000)

    # 한쪽 라인 없을 때 직진 타이머
    one_side_missing_time = None
    one_side_missing_direction = None
    STRAIGHT_DURATION = 0.5

    # 교차로 모드 관련 변수
    intersection_mode = False
    intersection_exit_time = None
    intersection_wait_start = None  # 교차로 대기 시작 시간
    INTERSECTION_EXIT_DURATION = 2.0
    INTERSECTION_TIMEOUT = 3.0  # 교차로 대기 타임아웃 (3초)

    # 라인 손실 관련
    line_lost_time = None

    # 차량 상태 관련
    vehicle_stopped = False  # 차량 정지 상태
    stop_reason = None  # 정지 이유

    # 높은 픽셀 값 감지 및 후진 모드
    HIGH_PIXEL_THRESHOLD = 12000  # 비정상 픽셀 값 임계값
    HIGH_PIXEL_DURATION = 0.5     # 0.5초 이상 지속 시 후진
    high_pixel_start_time = None  # 높은 픽셀 값 감지 시작 시간
    reverse_mode = False          # 후진 모드 플래그

    try:
        while True:
            ret, frame = camera.read()
            if not ret:
                break

            frame_count += 1

            # 이미지 뒤집기
            frame = cv2.flip(frame, -1)

            # 전체 프레임 크기
            height, width = frame.shape[:2]

            # ====== 차량 정지 상태 판단 ======
            # 교차로 모드일 때만 vehicle_stopped 사용
            if intersection_mode:
                if not vehicle_stopped:
                    vehicle_stopped = True
                    stop_reason = "교차로 대기"
            else:
                if vehicle_stopped and stop_reason == "교차로 대기":
                    vehicle_stopped = False
                    stop_reason = None

            # shared_state에 프레임 전달
            if OBJECT_DETECTION_ENABLED and frame_count % 2 == 0:
                try:
                    with shared_state.lock:
                        shared_state.latest_frame = frame.copy()
                except:
                    pass

            # ====== 방향 표지판을 큐에 저장 ======
            if OBJECT_DETECTION_ENABLED and frame_count % 2 == 0:
                store_direction_signs()

            # ====== 교차로에서만 특별 처리 ======
            if vehicle_stopped and stop_reason == "교차로 대기":
                # 교차로에서는 라인 인식 건너뛰기
                left_pixels = 0
                right_pixels = 0
                center_pixels = 0
                total_pixels = 0
                left_ratio = 0.0
                right_ratio = 0.0
                diff = 0.0
                blue_pixels = 0  # 파란색 픽셀도 초기화

                # 박스 크기 계산
                BOX_WIDTH = int(width * BOX_WIDTH_RATIO)
                BOX_HEIGHT = int(height * BOX_HEIGHT_RATIO)

            else:
                # ====== 정상 주행 - 라인 인식 수행 ======
                # 동적 박스 크기 계산
                BOX_WIDTH = int(width * BOX_WIDTH_RATIO)
                BOX_HEIGHT = int(height * BOX_HEIGHT_RATIO)

                # PIXEL_THRESHOLD는 이미 고정값으로 설정됨 (1200)

                # 좌하단 박스
                left_box_x1 = 0
                left_box_y1 = height - BOX_HEIGHT
                left_box_x2 = BOX_WIDTH
                left_box_y2 = height

                # 우하단 박스
                right_box_x1 = width - BOX_WIDTH
                right_box_y1 = height - BOX_HEIGHT
                right_box_x2 = width
                right_box_y2 = height

                # 전방 중앙 박스 (교차로 감지용)
                center_box_width = int(width * 0.6)  # 화면 너비의 60%
                center_box_height = int(height * 0.15)  # 화면 높이의 15%
                center_box_x1 = (width - center_box_width) // 2
                center_box_y1 = int(height * 0.3)  # 화면 상단 30% 위치
                center_box_x2 = center_box_x1 + center_box_width
                center_box_y2 = center_box_y1 + center_box_height

                # ====== HSV 변환 최적화: 전체 프레임 1회 변환 ======
                hsv_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

                # 좌측 박스 처리 (HSV 프레임에서 슬라이싱)
                hsv_left = hsv_frame[left_box_y1:left_box_y2, left_box_x1:left_box_x2]
                mask_left = cv2.inRange(hsv_left, lower_cyan, upper_cyan)

                # 노이즈 제거
                kernel = np.ones((3, 3), np.uint8)
                mask_left = cv2.erode(mask_left, kernel, iterations=2)
                mask_left = cv2.dilate(mask_left, kernel, iterations=3)

                # 우측 박스 처리 (HSV 프레임에서 슬라이싱)
                hsv_right = hsv_frame[right_box_y1:right_box_y2, right_box_x1:right_box_x2]
                mask_right = cv2.inRange(hsv_right, lower_cyan, upper_cyan)
                mask_right = cv2.erode(mask_right, kernel, iterations=2)
                mask_right = cv2.dilate(mask_right, kernel, iterations=3)

                # 전방 중앙 박스 처리 (HSV 프레임에서 슬라이싱)
                hsv_center = hsv_frame[center_box_y1:center_box_y2, center_box_x1:center_box_x2]
                mask_center = cv2.inRange(hsv_center, lower_cyan, upper_cyan)
                mask_center = cv2.erode(mask_center, kernel, iterations=2)
                mask_center = cv2.dilate(mask_center, kernel, iterations=3)

                # 파란색 픽셀 감지 (화면 하단 1/4 영역)
                blue_box_x1 = 0
                blue_box_y1 = int(height * 0.75)  # 75% 위치부터
                blue_box_x2 = width
                blue_box_y2 = height

                hsv_blue_area = hsv_frame[blue_box_y1:blue_box_y2, blue_box_x1:blue_box_x2]
                mask_blue = cv2.inRange(hsv_blue_area, lower_blue, upper_blue)
                mask_blue = cv2.erode(mask_blue, kernel, iterations=2)
                mask_blue = cv2.dilate(mask_blue, kernel, iterations=3)
                blue_pixels = cv2.countNonZero(mask_blue)

                # 파란색 픽셀 로그 출력 (10프레임마다, 500px 이상일 때만)
                if frame_count % 10 == 0 and blue_pixels > 500:
                    print(f"[파란색 감지] {blue_pixels}px (임계값: {BLUE_PIXEL_THRESHOLD}px)")

                # 픽셀 수 계산
                left_pixels = cv2.countNonZero(mask_left)
                right_pixels = cv2.countNonZero(mask_right)
                center_pixels = cv2.countNonZero(mask_center)
                total_pixels = left_pixels + right_pixels

                # CENTER_THRESHOLD는 이미 고정값으로 설정됨 (5000)

                # 좌우 비율 계산
                if total_pixels > 0:
                    left_ratio = left_pixels / total_pixels
                    right_ratio = right_pixels / total_pixels
                else:
                    left_ratio = 0.0
                    right_ratio = 0.0

                # 좌우 차이
                diff = abs(left_ratio - right_ratio)

            # 균형 임계값은 고정값 사용 (BALANCE_THRESHOLD)

            # ====== 높은 픽셀 값 감지 및 후진 처리 ======
            if left_pixels > HIGH_PIXEL_THRESHOLD or right_pixels > HIGH_PIXEL_THRESHOLD:
                # 높은 픽셀 값 감지
                if high_pixel_start_time is None:
                    high_pixel_start_time = time.time()
                elif time.time() - high_pixel_start_time >= HIGH_PIXEL_DURATION:
                    # 0.5초 이상 지속됨 → 후진 모드 활성화
                    if not reverse_mode:
                        reverse_mode = True
            else:
                # 정상 픽셀 값으로 복귀
                if reverse_mode:
                    reverse_mode = False
                    high_pixel_start_time = None
                elif high_pixel_start_time is not None:
                    # 타이머만 리셋 (0.5초 전에 정상 복귀)
                    high_pixel_start_time = None

            # ====== 후진 모드 실행 ======
            if reverse_mode:
                motor_backward()
                time.sleep(0.1)
                continue


            # ====== 교차로 모드에서 키보드 입력 처리 ======
            if intersection_mode:
                # 먼저 저장된 표지판 확인하여 자동 키 입력으로 변환
                auto_input = False
                user_input = None
                if OBJECT_DETECTION_ENABLED and recognized_signs and user_input is None:

                    # sign_info = recognized_signs[-1]  # 가장 먼저 저장된 표지판 확인
                    # sign_type = sign_info['type']

                    sign_info = recognized_signs.popleft()   # 가장 먼저 들어온 명령
                    sign_type = sign_info['type']


                    # 표지판을 키 입력으로 변환
                    sign_to_key = {
                        'go_straight': 'w',
                        'turn_left': 'a',
                        'turn_right': 'd',
                        'traffic': 'traffic'  # 신호등은 특별 처리
                    }

                    if sign_type in sign_to_key:
                        user_input = sign_to_key[sign_type]
                        auto_input = True

                        sign_names = {
                            'go_straight': '직진',
                            'turn_left': '좌회전',
                            'turn_right': '우회전',
                            'traffic': '신호등 (3초 대기)'
                        }
                        print(f"[실행] {sign_names.get(sign_type, sign_type)}")

                # 타임아웃 체크
                if not user_input and intersection_wait_start:
                    wait_time = time.time() - intersection_wait_start

                    if wait_time >= INTERSECTION_TIMEOUT:
                        print("[타임아웃] 자동 직진")
                        motor_forward()
                        time.sleep(0.5) #추가
                        intersection_mode = False
                        intersection_exit_time = time.time()
                        intersection_wait_start = None
                        vehicle_stopped = False
                        continue

                if not user_input:
                    user_input = get_user_input()
                if user_input:

                    if user_input == 'w':
                        motor_forward()
                        time.sleep(6)

                    elif user_input == 'a':
                        motor_forward()
                        time.sleep(4)  # 진입 시간
                        motor_left(1.0)
                        time.sleep(5)  # 회전 시간
                        motor_forward()
                        time.sleep(0.8)  # 탈출 시간

                    elif user_input == 'd':
                        motor_forward()
                        time.sleep(1)  # 진입 시간
                        motor_right(1.0)
                        time.sleep(5)  # 회전 시간
                        motor_forward()
                        time.sleep(0.8)  # 탈출 시간
                    elif user_input == 'traffic':
                        # 신호등: 3초 대기 후 'd' (우회전) 실행
                        print("🚦 [신호등] 3초 대기 중...")
                        motor_stop()
                        time.sleep(3.0)  # 3초 정지
                        print("🚦 [신호등] 우회전 시작!")
                        # 'd' 케이스와 동일한 우회전 실행
                        motor_forward()
                        time.sleep(4)  # 진입 시간
                        motor_right(1.0)
                        time.sleep(5)  # 회전 시간
                        motor_forward()
                        time.sleep(0.8)  # 탈출 시간
                    elif user_input == 's':
                        motor_stop()

                    # ===================================
                    # 🔥 실행된 표지판 쿨다운 시간 갱신 (여기에 넣음)
                    # ===================================
                    if auto_input and sign_type:
                        with shared_state.lock:
                            shared_state.action_last_time[sign_type] = time.time()

                    # 자동 명령이면 큐 비우기
                    if auto_input:
                        recognized_signs.clear()
                    
                    intersection_mode = False
                    intersection_exit_time = time.time()
                    intersection_wait_start = None
                    vehicle_stopped = False
                else:
                    # 키보드 입력 대기 중
                    motor_stop()
                continue

            # ====== 교차로 탈출 중이면 일정 시간 교차로 감지 무시 ======
            if intersection_exit_time:
                elapsed = time.time() - intersection_exit_time
                if elapsed >= INTERSECTION_EXIT_DURATION:
                    intersection_exit_time = None



            # ====== 파란색 픽셀 감지 (항상 체크 - 교차로와 무관) ======
            if not intersection_exit_time and blue_pixels >= BLUE_PIXEL_THRESHOLD:
                global stopline_stage, first_stopline_time
                # 1단계: 첫 번째 정지선 감지 → 통과
                if stopline_stage == 0:
                    print(f"[정지선] 첫 번째 정지선 감지 ({blue_pixels}px) → 계속 진행")
                    stopline_stage = 1
                    first_stopline_time = time.time()
                    # 멈추지 않고 그냥 직진하도록 처리
                    # 필요하면 살짝 감속 가능: SPEED_FORWARD = SPEED_SLOW_FORWARD

                # 2단계: 두 번째 정지선 감지 → 정지 및 종료
                elif stopline_stage == 1:
                # 첫 정지선과 너무 가까우면 노이즈 → 무시
                    if time.time() - first_stopline_time < 0.3:
                        pass
                    else:
                        print(f"\n[정지선] 두 번째 정지선 감지: {blue_pixels}px")
                        print("[종료] 정지선 사이 → 최종 정지 및 종료")
                        motor_stop()
                        time.sleep(1.5)
                        break

            # ====== 교차로 감지 ======
            elif not intersection_exit_time and center_pixels > CENTER_THRESHOLD and total_pixels < PIXEL_THRESHOLD * 2:
                if not intersection_mode:
                    motor_stop()
                    intersection_mode = True
                    intersection_wait_start = time.time()

                    queue_info = f" 큐:{len(recognized_signs)}" if recognized_signs else ""
                    print(f"[교차로]{queue_info}")

            # ====== 라인 이탈 ======
            elif total_pixels < PIXEL_THRESHOLD:
                if line_lost_time is None:
                    line_lost_time = time.time()
                    motor_stop()

                # 키보드 입력 확인
                user_input = get_user_input()
                if user_input:
                    if user_input == 'w':
                        motor_forward()
                        time.sleep(6)
                    elif user_input == 'a':
                        motor_forward()
                        time.sleep(5)  # 진입 시간 (교차로와 동일)
                        motor_left(1.0)
                        time.sleep(5)  # 회전 시간 (교차로와 동일)
                        motor_forward()
                        time.sleep(0.8)  # 탈출 시간 (교차로와 동일)
                    elif user_input == 'd':
                        motor_forward()
                        time.sleep(4)  # 진입 시간 (교차로와 동일)
                        motor_right(1.0)
                        time.sleep(5)  # 회전 시간 (교차로와 동일)
                        motor_forward()
                        time.sleep(0.8)  # 탈출 시간 (교차로와 동일)
                    elif user_input == 's':
                        motor_stop()

            # ====== 라인 트레이싱 ======
            elif total_pixels >= PIXEL_THRESHOLD:
                if line_lost_time is not None:
                    line_lost_time = None

                vehicle_stopped = False

                # 동적 임계값 계산 (속도 기반)
                is_high_speed = SPEED_FORWARD > 0.6
                current_balance_threshold = HIGH_SPEED_BALANCE_THRESHOLD if is_high_speed else BASE_BALANCE_THRESHOLD

                if diff < current_balance_threshold:
                    # 좌우 균형 잡힘 → 전진
                    motor_forward()
                    one_side_missing_time = None
                    one_side_missing_direction = None

                elif left_pixels > right_pixels:
                    # 왼쪽에 청록색이 많음 → 우회전 필요
                    # 편차에 비례한 회전 강도 계산 (최대 편차 50%로 정규화)
                    turn_intensity = min(1.0, diff / 0.5)

                    if right_pixels < 50:
                        # 오른쪽 라인이 거의 없음
                        if one_side_missing_time is None or one_side_missing_direction != 'RIGHT':
                            one_side_missing_time = time.time()
                            one_side_missing_direction = 'RIGHT'

                        elapsed = time.time() - one_side_missing_time
                        if elapsed < STRAIGHT_DURATION:
                            # 직진 유지 (0.5초)
                            motor_forward()
                        else:
                            # 강한 우회전 (intensity * 1.25로 완화 - 과도한 회전 방지)
                            motor_right(min(1.0, turn_intensity * 1.25))
                    else:
                        # 일반 우회전 (비례 제어)
                        motor_right(turn_intensity)
                        one_side_missing_time = None
                        one_side_missing_direction = None

                else:
                    # 오른쪽에 청록색이 많음 → 좌회전 필요
                    # 편차에 비례한 회전 강도 계산 (최대 편차 50%로 정규화)
                    turn_intensity = min(1.0, diff / 0.5)

                    if left_pixels < 50:
                        # 왼쪽 라인이 거의 없음
                        if one_side_missing_time is None or one_side_missing_direction != 'LEFT':
                            one_side_missing_time = time.time()
                            one_side_missing_direction = 'LEFT'

                        elapsed = time.time() - one_side_missing_time
                        if elapsed < STRAIGHT_DURATION:
                            # 직진 유지 (0.5초)
                            motor_forward()
                        else:
                            # 강한 좌회전 (intensity * 1.25로 완화 - 과도한 회전 방지)
                            motor_left(min(1.0, turn_intensity * 1.25))
                    else:
                        # 일반 좌회전 (비례 제어)
                        motor_left(turn_intensity)
                        one_side_missing_time = None
                        one_side_missing_direction = None

                # 주행 중 객체 인식 트리거 처리
                handle_runtime_triggers()

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[종료]")

    finally:
        motor_stop()
        if PWMA:
            PWMA.value = 0.0
        if PWMB:
            PWMB.value = 0.0
        camera.release()

if __name__ == '__main__':
    lane_follow_loop()