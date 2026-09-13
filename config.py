"""
config.py
---------
주행 파라미터를 한곳에 모읍니다.

값 대부분은 실제 트랙에서 주행하며 조정한 결과입니다. 근거를 아는 값에는 이유를
적었고, 모르는 값은 "근거 미확인"으로 표시했습니다. 추정은 "~로 보입니다"로
구분해 적었습니다.

이 파일은 값을 옮기기만 했으며 동작을 바꾸지 않습니다.
원래 위치는 각 항목의 [원위치] 에 적었습니다.

## [원위치] 표기

값 옆의 `[원위치] lane_tracer.py NNN` 은 리팩토링 이전 원본의 줄번호입니다.
기준은 라즈베리파이에서 회수한 `salvage/pi-final-2025-12-13` 브랜치의
lane_tracer.py(961줄)이며, 현재 main 의 줄번호가 아닙니다.
"""

# ============================================================
# GPIO 핀 (BCM 번호)
# ============================================================
# [원위치] lane_tracer.py init_gpio() 안에 리터럴로 들어 있었습니다.
# 모터 채널 A/B 는 각각 PWM(속도) 1핀 + 방향 2핀으로 구성됩니다.

MOTOR_PWMA = 18   # 좌측 모터 속도
MOTOR_AIN1 = 22   # 좌측 방향 1
MOTOR_AIN2 = 27   # 좌측 방향 2
MOTOR_PWMB = 23   # 우측 모터 속도
MOTOR_BIN1 = 25   # 우측 방향 1
MOTOR_BIN2 = 24   # 우측 방향 2

BUZZER_PIN = 12
BUZZER_FREQUENCY = 2000   # Hz. 원본 주석: "PWM으로 큰 소리"


# ============================================================
# 모터 속도 (PWM duty, 0.0 ~ 1.0)
# ============================================================
# [원위치] lane_tracer.py 112-115
# 원본 주석에 남은 조정 이력: 직진 "0.40 → 0.45, 약간 증가",
# 회전 "0.55 → 0.60, 회전력 강화", 감속 직진 "기존 0.25", 감속 회전 "기존 0.20".
# 앞의 두 이력은 실제 값(0.225 / 0.30)과 맞지 않습니다. 이력을 적은 뒤 값을 더
# 낮추고 주석을 갱신하지 않은 것으로 보입니다. 값을 기준으로 삼았습니다.

SPEED_FORWARD_DEFAULT = 0.225
SPEED_TURN_DEFAULT = 0.30     # 직진보다 높습니다. 원본 주석은 "회전력 강화"
SPEED_SLOW_FORWARD = 0.20     # slow 표지판 인식 시
SPEED_SLOW_TURN = 0.16

# 후진 복구는 전진 속도의 절반으로 합니다. 벽에서 떨어지는 것이 목적이라
# 빠를 필요가 없습니다.
# [원위치] lane_tracer.py 186-193 (motor_backward). 190·193 에
#          PWMA/PWMB.value = SPEED_FORWARD * 0.5 로 들어 있었습니다.
REVERSE_SPEED_RATIO = 0.5

# 회전 시 좌우 바퀴 출력 배분.
# intensity 가 PIVOT_TURN_THRESHOLD 를 넘으면 안쪽 바퀴를 역회전시켜 제자리
# 회전에 가깝게 만듭니다. 그 이하면 안쪽 바퀴를 세우고 바깥쪽만 돌립니다.
# [원위치] lane_tracer.py 133-175 (motor_left 133-153, motor_right 155-175)
PIVOT_TURN_THRESHOLD = 0.5    # 이 값을 넘는 intensity 는 제자리 회전으로 전환
PIVOT_INNER_RATIO = 0.3       # 제자리 회전 시 안쪽 바퀴(역회전) 출력 배율.
                              # 제자리 회전 분기에만 적용됩니다.
TURN_OUTER_RATIO = 1.2        # 바깥쪽 바퀴 출력 배율. 제자리 회전과 약한 회전
                              # 두 분기 모두에 적용됩니다.
                              #
                              # 주의: 1 을 넘는 값이라 SPEED_TURN 을 올리면
                              # PWM 이 포화됩니다. SPEED_TURN * 1.2 > 1.0 이 되는
                              # 순간(SPEED_TURN > 0.833) 바깥쪽 바퀴만 1.0 에
                              # 잘려 좌우 대칭이 깨집니다. 현재는 0.30 이라 0.36 입니다.


# ============================================================
# 카메라
# ============================================================
# [원위치] lane_tracer.py init_camera() 366-371

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FORMAT = "RGB888"
CAMERA_WARMUP_SEC = 2         # start() 직후 안정화 대기
CAMERA_MAX_RETRIES = 3        # 초기화 재시도 횟수
CAMERA_RETRY_DELAY_SEC = 2    # 재시도 간격
# 재시도 시 libcamera 프로세스를 kill 한 뒤 기다리는 시간
CAMERA_KILL_SETTLE_SEC = 0.5

# 카메라가 거꾸로 장착돼 있어 프레임을 상하좌우 반전합니다. cv2.flip(frame, -1)
CAMERA_FLIP = True


# ============================================================
# 차선 인식 — HSV 범위
# ============================================================
# [원위치] lane_tracer.py 483-489
# 트랙 차선은 청록색(cyan)입니다. 채도·명도 하한을 20까지 낮춘 것은 조명이 약한
# 구간에서 선을 놓쳤기 때문으로 보입니다(원본 주석: "확장된 범위").

LANE_HSV_LOWER = (65, 20, 20)
LANE_HSV_UPPER = (115, 255, 255)

# 정지선은 파란색입니다. 색상(Hue) 100~130 이 파란 계열입니다.
STOPLINE_HSV_LOWER = (100, 100, 100)
STOPLINE_HSV_UPPER = (130, 255, 255)


# ============================================================
# 차선 인식 — ROI (관심 영역)
# ============================================================
# [원위치] lane_tracer.py 498-505
# 전체 프레임을 처리하지 않고 박스 3개만 봅니다. 라즈베리파이에서 HSV 변환과
# 마스킹 비용을 줄이기 위한 것입니다.
#   좌하단 / 우하단 박스 → 조향 판단
#   전방 중앙 박스      → 교차로 감지

ROI_BOX_WIDTH_RATIO = 0.25    # 화면 너비의 25%
ROI_BOX_HEIGHT_RATIO = 0.25   # 화면 높이의 25%

# 전방 중앙 박스 (교차로 감지용)
# [원위치] lane_tracer.py 610-614
CENTER_BOX_WIDTH_RATIO = 0.6    # 화면 너비의 60%
CENTER_BOX_HEIGHT_RATIO = 0.15  # 화면 높이의 15%
CENTER_BOX_TOP_RATIO = 0.3      # 화면 상단에서 30% 지점부터

# 정지선 감지 영역: 화면 하단 1/4 (75% ~ 100%)
STOPLINE_ROI_TOP_RATIO = 0.75


# ============================================================
# 차선 인식 — 노이즈 제거
# ============================================================
# [원위치] lane_tracer.py 625-639
# erode 로 작은 점을 지우고 dilate 로 남은 선을 다시 굵게 만듭니다.
# dilate 반복이 erode 보다 많아(2 → 3) 선이 끊기지 않습니다.

MORPH_KERNEL_SIZE = 3
MORPH_ERODE_ITERATIONS = 2
MORPH_DILATE_ITERATIONS = 3


# ============================================================
# 차선 인식 — 판정 임계값
# ============================================================
# [원위치] lane_tracer.py 494-495, 508-509

# 좌우 픽셀 비율 차이가 이보다 작으면 직진으로 봅니다.
BASE_BALANCE_THRESHOLD = 0.35
HIGH_SPEED_BALANCE_THRESHOLD = 0.25   # 고속에서는 더 민감하게

# 고속 판정 기준.
# [원위치] lane_tracer.py 888  is_high_speed = SPEED_FORWARD > 0.6
#
# 주의: 현재 SPEED_FORWARD 는 0.225(기본) 또는 0.20(감속)뿐이라 이 기준을 넘지
# 않습니다. 즉 HIGH_SPEED_BALANCE_THRESHOLD 에는 도달할 수 없습니다. 기본 속도가
# 더 높던 시절의 잔재로 보입니다. 동작 보존을 위해 값은 유지하고 사실만
# 기록합니다.
HIGH_SPEED_THRESHOLD = 0.6

# 한쪽 ROI 픽셀이 이보다 적으면 그쪽 차선이 없는 것으로 봅니다.
# [원위치] lane_tracer.py 902 (right_pixels < 50), 926 (left_pixels < 50)
ONE_SIDE_MISSING_PIXELS = 50

# 회전 강도를 비율 편차로 정규화할 때의 분모.
# 편차 0.5(50%)에서 강도가 1.0 이 됩니다.
# [원위치] lane_tracer.py 900, 924  min(1.0, diff / 0.5)
TURN_INTENSITY_NORMALIZER = 0.5

# 한쪽 차선이 사라진 채 STRAIGHT_DURATION 을 넘겼을 때의 회전 강도 배율.
# 원본 주석: "과도한 회전 방지"
# [원위치] lane_tracer.py 914, 938  min(1.0, turn_intensity * 1.25)
STRONG_TURN_MULTIPLIER = 1.25

# 전체 픽셀이 이보다 적으면 차선을 잃은 것으로 판단합니다.
# 원본 주석: "더 민감하게 조정"
PIXEL_THRESHOLD = 800

# 전방 중앙 박스 픽셀이 이보다 많으면 교차로 후보입니다.
# 원본 주석에 "(5000->9000)" 메모가 남아 있으나 값은 5000 그대로입니다.
# 9000 으로 올리려다 되돌린 것으로 보입니다. 근거 미확인.
CENTER_THRESHOLD = 5000


# ============================================================
# 정지선 (주행 종료 판정)
# ============================================================
# [원위치] lane_tracer.py 50, 832
# 트랙에 정지선이 두 개 있습니다. 첫 번째는 통과하고 두 번째에서 멈춥니다.
# 초기값 3000 에서 25000 으로 올렸습니다. 낮은 값에서는 파란 계열 반사나 배경이
# 잡혀 일찍 종료됐던 것으로 보입니다.

BLUE_PIXEL_THRESHOLD = 25000

# 파란색 픽셀 로그 출력 조건 (디버깅용)
# [원위치] lane_tracer.py 653-655
BLUE_LOG_INTERVAL_FRAMES = 10
BLUE_LOG_MIN_PIXELS = 500

# 첫 정지선 감지 직후 같은 선을 다시 잡지 않도록 두는 최소 간격
STOPLINE_MIN_GAP_SEC = 0.3

# 최종 정지 후 대기
STOPLINE_FINAL_STOP_SEC = 1.5


# ============================================================
# 교차로
# ============================================================
# [원위치] lane_tracer.py 514, 520-521

# 교차로 통과 직후 이 시간 동안 재감지를 억제합니다.
INTERSECTION_EXIT_DURATION = 2.0

# 교차로에서 지시를 기다리는 최대 시간. 넘으면 직진합니다.
INTERSECTION_TIMEOUT = 3.0

# 타임아웃 직진 시 전진 유지 시간. 원본에 "#추가" 주석과 함께 뒤늦게 들어간 값입니다.
INTERSECTION_TIMEOUT_FORWARD_SEC = 0.5

# 한쪽 차선만 보일 때 직진을 유지하는 시간
STRAIGHT_DURATION = 0.5


# ============================================================
# 교차로 회전 동작 타이밍
# ============================================================
# [원위치] lane_tracer.py 755-788
# 표지판별로 정해진 시간만큼 모터를 돌리는 개루프(open-loop) 방식입니다.
# 피드백이 없으므로 트랙 형상에 맞춰 초 단위로 맞췄습니다.
#
# 주의: 좌회전과 우회전의 진입 시간이 다르고(4초 vs 1초), 신호등 우회전은 일반
# 우회전과 같은 동작인데 진입 시간이 4초입니다. 이유는 근거 미확인이며, 트랙에서
# 해당 지점의 형상이 달랐을 가능성이 있습니다.

TURN_STRAIGHT_FORWARD_SEC = 6      # go_straight 표지판: 직진 유지

TURN_LEFT_ENTER_SEC = 4            # 좌회전 진입
TURN_LEFT_ROTATE_SEC = 5           # 좌회전 회전
TURN_LEFT_EXIT_SEC = 0.8           # 좌회전 탈출

TURN_RIGHT_ENTER_SEC = 1           # 우회전 진입 (좌회전보다 짧음)
TURN_RIGHT_ROTATE_SEC = 5
TURN_RIGHT_EXIT_SEC = 0.8

TRAFFIC_STOP_SEC = 3.0             # 신호등 인식 후 정지 대기
TRAFFIC_RIGHT_ENTER_SEC = 4        # 신호등 우회전 진입 (일반 우회전은 1초)
TRAFFIC_RIGHT_ROTATE_SEC = 5
TRAFFIC_RIGHT_EXIT_SEC = 0.8


# ============================================================
# 라인 이탈 복구 시 회전 동작 타이밍
# ============================================================
# [원위치] lane_tracer.py 860-878
# 차선을 완전히 잃었을 때(total_pixels < PIXEL_THRESHOLD) 키 입력으로 수동
# 회전하는 경로입니다. 교차로 블록과 코드가 거의 같습니다.
#
# 주의: 원본 주석은 "(교차로와 동일)"이지만 진입 시간이 실제로 다릅니다.
# 좌회전 4초 vs 5초, 우회전 1초 vs 4초. 복사한 뒤 한쪽만 조정해 갈라진 것으로
# 보입니다. 어느 쪽이 의도인지 근거 미확인이라 통일하지 않고 원래 값을
# 유지합니다. 라즈베리파이에서 검증할 수 있을 때 판단할 항목입니다.

LINELOST_STRAIGHT_FORWARD_SEC = 6

LINELOST_LEFT_ENTER_SEC = 5        # 교차로는 4
LINELOST_LEFT_ROTATE_SEC = 5
LINELOST_LEFT_EXIT_SEC = 0.8

LINELOST_RIGHT_ENTER_SEC = 4       # 교차로는 1
LINELOST_RIGHT_ROTATE_SEC = 5
LINELOST_RIGHT_EXIT_SEC = 0.8


# ============================================================
# 후진 복구
# ============================================================
# [원위치] lane_tracer.py 531-532
# 차선 픽셀이 비정상적으로 많으면 벽이나 차선 뭉치에 근접한 상태로 봅니다.
# 순간적인 값 튐과 구분하려고 일정 시간 지속될 때만 후진합니다.

HIGH_PIXEL_THRESHOLD = 12000
HIGH_PIXEL_DURATION = 0.5


# ============================================================
# 표지판 대응
# ============================================================
# [원위치] lane_tracer.py 45, shared_state.py 51

# 같은 표지판을 연속 인식했을 때 동작을 반복하지 않는 간격
SIGN_COOLDOWN = 3.0

# stop 표지판 인식 시 정지 유지 시간과, 재출발 시 저속 구간 길이
# [원위치] lane_tracer.py 286 (정지 2.0초), 291 (저속 재출발 0.5초)
STOP_SIGN_HOLD_SEC = 2.0
STOP_SIGN_RESTART_SLOW_SEC = 0.5

# 객체별 동작 재실행 금지 간격.
# 원본 주석은 "5초간 재실행 금지"인데 값은 4.0 이었습니다. 주석이 값을 따라가지
# 못한 것으로 보고 값을 유지했습니다.
ACTION_COOLDOWN = 4.0

# slow 표지판 인식 시 감속 유지 시간. 종료 시각을 shared_state.slow_mode_until 에
# 기록합니다.
# [원위치] lane_tracer.py 307
SLOW_MODE_DURATION = 3.0

# 인식 가능한 클래스
# [원위치] shared_state.py 17-21
KNOWN_OBJECTS = [
    "go_straight", "turn_left", "turn_right",
    "stop", "slow", "horn",
    "traffic",
]


# ============================================================
# 부저
# ============================================================
# [원위치] lane_tracer.py 216-240
# 원본 주석 이력: 짧은 음 0.25 → 0.45초, 긴 음 0.35 → 0.60초.
# 트랙에서 소리가 잘 들리지 않아 길이를 늘린 것으로 보입니다.

BEEP_DEFAULT_SEC = 3.0

# 부저가 없을 때 멜로디 대신 기다리는 시간. 실제 멜로디(약 2.1초)보다 짧습니다.
# [원위치] lane_tracer.py 219
BELL_FALLBACK_SEC = 1.5

BELL_SHORT_BEEP_SEC = 0.45
BELL_LONG_BEEP_SEC = 0.60
BELL_PAUSE_SEC = 0.05


# ============================================================
# 객체 검출 (YOLOv8 2단계)
# ============================================================
# [원위치] object_detector.py 24-34
# Detector 가 표지판/신호등 영역을 찾고, Classifier 가 잘라낸 영역을 8클래스로
# 분류합니다. 단일 Detection 모델 대신 2단계로 나눈 이유는 배경 노이즈의 영향을
# 줄이고 각 단계에 따로 신뢰도 임계값을 적용하기 위해서입니다.

DETECTOR_MODEL = "models/sign_traffic_detectorV3.pt"
CLASSIFIER_MODEL = "models/sign_traffic_classifierV3.pt"

# 박스 면적이 이보다 작으면 무시합니다. 초기 5000 에서 올렸습니다.
# 멀리 있는 표지판에 미리 반응하거나 오탐하는 것을 막으려는 것으로 보입니다.
MIN_AREA = 10000

# 근접 판정 임계값. object_detector 가 상수로 받아 두기만 하고 실제로 비교하는
# 곳은 없습니다(MIN_AREA 만 씁니다). 원본 그대로 남깁니다.
NEAR_AREA = 20000

# Detector 신뢰도. 초기 0.7 에서 올렸습니다.
CONF_THRESHOLD = 0.8

# Classifier 신뢰도. Detector 보다 높게 잡아 오분류를 한 번 더 거릅니다.
CLASSIFIER_CONF_THRESHOLD = 0.90

# 분류기 입력 크기
CLASSIFIER_IMGSZ = 224

# 검출 ROI: 프레임 오른쪽 절반만 추론합니다. 표지판이 주행 방향 오른쪽에
# 배치돼 있어 입력 크기를 절반으로 줄일 수 있었습니다.
DETECTION_ROI_LEFT_RATIO = 0.5

# 인식 순간 이미지를 저장하려던 설정. object_detector 가 폴더를 만들기만 하고
# 저장하는 코드는 없습니다(데이터 수집은 tools/collect_object_data.py 가 따로 합니다).
CAPTURE_FOLDER = "/home/keonha/AI_CAR/captured_images"
MAX_CAPTURES_PER_OBJECT = 1


# ============================================================
# 루프 주기
# ============================================================
# [원위치] lane_tracer.py 948, 699
# 차선 추적 루프의 프레임 간 대기입니다. 실제 FPS 는 Picamera2 캡처 속도가
# 결정하고 이 값은 최소 대기 시간입니다.

LANE_LOOP_DELAY_SEC = 0.02

# 후진 복구 중에는 더 긴 간격으로 돕니다.
REVERSE_LOOP_DELAY_SEC = 0.1

# 프레임을 object_detector 에 넘기는 주기 (N 프레임마다 1회).
# 프레임 복사와 락 획득 비용을 줄이려는 것으로 보입니다.
FRAME_SHARE_INTERVAL = 2

# 객체 검출 루프 주기. 실제 FPS 는 YOLO 추론 시간이 결정하고 이 값은 최소
# 대기 시간입니다.
# [원위치] object_detector.py 188
DETECTION_LOOP_DELAY_SEC = 0.10

# 아직 공유된 프레임이 없을 때 짧게 대기 [원위치] object_detector.py 83
DETECTION_NO_FRAME_DELAY_SEC = 0.05

# 검출 결과가 비었을 때 대기 [원위치] object_detector.py 97
DETECTION_EMPTY_RESULT_DELAY_SEC = 0.1

# 메인 스레드 상태 출력 주기 [원위치] main.py 61
MONITOR_INTERVAL_SEC = 1.0
