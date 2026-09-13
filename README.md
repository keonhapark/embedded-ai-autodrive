# AI_CAR

라즈베리파이 기반 소형 자율주행 차량입니다. 실제 주행 트랙에서 차선 유지와
표지판·신호등·교차로 대응을 수행합니다.

2025년 2학기 임베디드 AI 시스템 최적화 과목 팀 프로젝트 (2인).

## 구성

| 파일 | 역할 |
|------|------|
| `main.py` | 진입점. 하드웨어를 소유하고 두 스레드를 시작·종료합니다 |
| `lane_tracer.py` | 차선 유지 주행 루프, 표지판 대응 동작 |
| `control.py` | 주행 판단 (순수 함수). 갈래 선택, 조향, 교차로 상태, 동작 시퀀스 표, 속도 프로파일 |
| `vision.py` | Sense. 프레임 → 픽셀 수 |
| `hardware.py` | 하드웨어 경계. 모터·부저·카메라 드라이버와 테스트용 가짜 구현 |
| `config.py` | 주행 파라미터와 핀 번호 |
| `object_detector.py` | YOLOv8 2단계 추론(Detector → Classifier) |
| `shared_state.py` | 두 스레드 간 공유 상태 (Lock 기반) |
| `tools/collect_object_data.py` | 개발 도구. 표지판 학습 데이터 수집 (수동 주행 중 주기적 캡처) |
| `tools/key_remote.py` | 개발 도구. SSH 환경 수동 조종 |

## 역할 분담

| 담당 | 범위 |
|------|------|
| junsu | 차선 유지 주행 로직 (`lane_tracer.py` 주요 구현) |
| keonha | 표지판 인식 (`object_detector.py`, YOLO 모델 학습·데이터 수집), 파란색 정지선 2단계 종료 판정, 주행 파라미터 튜닝 |

## 실행

라즈베리파이(64-bit OS)에서만 동작합니다. GPIO와 Picamera2에 의존합니다.

```bash
pip install picamera2 opencv-python numpy gpiozero ultralytics
python main.py
```

`models/` 아래에 YOLO 가중치 두 개가 필요합니다. 용량 때문에 저장소에 포함하지
않았습니다.

```
models/sign_traffic_detectorV3.pt     # 표지판/신호등 영역 검출
models/sign_traffic_classifierV3.pt   # 8클래스 분류
```

Ctrl+C로 종료합니다. 종료 신호를 받으면 두 스레드에 알리고 기다린 뒤 모터를
정지시킵니다.

`tools/` 의 두 스크립트는 주행과 별개로 단독 실행하는 개발 도구입니다
(`python tools/key_remote.py`). 데이터 수집에는 과목에서 제공된 `mycamera`
모듈이 필요하며, 스크립트와 같은 `tools/` 폴더에 두어야 import 됩니다.

## 현재 상태

- **코드는 2025-12-05 시연에 사용된 실주행 버전입니다.** 라즈베리파이에서 회수한
  원본은 `salvage/pi-final-2025-12-13` 브랜치에 수정 없이 보존되어 있습니다.
- 이전 `main`(`3fef20f`)은 구조를 정리하는 과정에서 표지판 대응 로직이 누락되고
  구 버전 모델을 참조하는 상태였습니다. 동작하는 버전으로 되돌렸습니다.
- 961줄짜리 주행 루프 하나였던 코드를 Sense(`vision.py`) / Plan(`control.py`) /
  Act(`lane_tracer.py`) / 하드웨어 경계(`hardware.py`)로 분해했습니다. 동작 보존이
  규칙이었고, 트랙에서 튜닝한 임계값은 값을 바꾸지 않고 `config.py` 에 모았습니다.
  원본과 어긋나면 안 되는 계산은 원본 코드를 그대로 옮겨 놓고 비교합니다
  (`test_hardware_parity.py`, `test_vision.py`).
- 라즈베리파이가 없어 런타임 검증은 하지 못했습니다. 대신 테스트가 실제로 검증을
  하는지 코드를 일부러 망가뜨려(변이) 확인했습니다.

## 테스트

라즈베리파이 없이 테스트할 수 있습니다. `picamera2` 와 `gpiozero` 는 필요 없습니다.

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements-dev.txt
.venv/bin/pytest
```

| 파일 | 무엇을 확인하나 |
|------|----------------|
| `test_control.py` | 주행 판단 순수 함수의 현재 동작 |
| `test_hardware.py` | `FakePin` 으로 모터 출력과 부저 타이밍 |
| `test_hardware_parity.py` | 리팩토링 전 코드를 그대로 옮겨 놓고 핀 값 비교 |
| `test_loop.py` | `FakeCamera` 로 주행 루프를 실제로 돌려 판단이 모터까지 가는지 |
| `test_shutdown.py` | Ctrl+C 종료 경로. 정지 호출, join, 정리 실패 격리 |
| `test_vision.py` | 원본 측정 코드를 그대로 옮겨 놓고 픽셀 수 비교 |
| `test_key_remote.py` | 수동 조작 스크립트의 키 → 모터 방향 매핑 |
| `test_structure.py` | AST 검사. 도달 불가 분기와 고아가 된 판단 함수 |

`tests/scenes.py` 는 합성 프레임(차선, 교차로, 정지선, 좌우 비대칭 등)을 만들어
여러 테스트가 함께 씁니다.

`test_loop.py` 와 `test_structure.py` 는 판단 로직이 실제로 호출되는지 확인합니다.
순수 함수 테스트는 호출부를 실행하지 않아서, 분기 구조가 깨져 차가 주행하지
않아도 통과할 수 있습니다.

테스트는 대부분 "이렇게 동작해야 한다"가 아니라 **"지금 이렇게 동작한다"** 를
고정합니다. 구조를 바꾸는 동안 동작이 변하지 않았는지 확인하는 안전망입니다.

### 테스트 범위 밖

- `object_detector.py` 와 `tools/collect_object_data.py` 는 동작 테스트가 없습니다
  (`test_structure.py` 의 정적 검사만 적용됩니다).
- `tools/` 의 두 스크립트는 원본의 모터 코드와 핀 번호를
  복제해 갖고 있습니다. 리팩토링 범위 밖이라 정리하지 않았습니다.
