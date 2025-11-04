"""
object_detector.py
------------------

YOLOv8 탐지 + 분류 + 거리 기반 근접 이벤트

* 탐지 모델과 분류 모델을 모두 사용
* shared_state를 통해 lane_tracer와 병렬 통신
* 일정 거리(박스 면적 기준) 도달 시 객체 1회 인식 이벤트 발생
"""

import time
import cv2
import numpy as np
from ultralytics import YOLO
import shared_state
import os
from picamera2 import Picamera2

# 모델 경로 설정 (Colab 또는 라즈베리파이 환경에 맞게 수정)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DETECTOR_PATH = os.path.join(BASE_DIR, "models", "sign_traffic_detector.pt")
CLASSIFIER_PATH = os.path.join(BASE_DIR, "models", "sign_traffic_classifier.pt")


# 바운딩박스 면적 기준 (픽셀 단위)

MIN_AREA = 5000  # 너무 작은 객체 제외
NEAR_AREA = 20000  # 근접 판단 기준
CONF_THRESHOLD = 0.7  # 신뢰도 임계값
COOLDOWN = 3.0  # 초 단위 쿨다운


# 신호등 거리 및 비율 기준
TRAFFIC_RATIO_LIMIT = 1.7
TRAFFIC_AREA_LIMIT = 30000


def object_detect_loop():
    print("=" * 70)
    print(" YOLOv8 Object Detector Started")
    print("=" * 70)

    # 모델 로드
    detector = YOLO(DETECTOR_PATH)
    classifier = YOLO(CLASSIFIER_PATH)


    last_action_time = 0
    last_label = None

    try:
        while True:

            with shared_state.lock:
                frame = getattr(shared_state, "latest_frame", None)
            if frame is None:
                time.sleep(0.05)
                continue

            roi = frame[:, 320:640]  # 오른쪽 절반만 사용 (320x480)
            results = detector(roi, verbose=False)
            now = time.time()

            detected_label = None
            nearest_area = 0
            traffic_light_state = None
            traffic_light_visible = False
            traffic_light_area = 0

            # 탐지 결과 순회
            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                area = (x2 - x1) * (y2 - y1)
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cls_name = results[0].names[cls_id]

                if area < MIN_AREA or conf < CONF_THRESHOLD:
                    continue

                # crop 영역 분류
                crop = roi[y1:y2, x1:x2]
                cls_res = classifier.predict(crop, imgsz=224, verbose=False)
                sub_id = int(cls_res[0].probs.top1)
                sub_name = cls_res[0].names[sub_id]
                sub_conf = float(cls_res[0].probs.top1conf)

                if sub_conf < 0.8:
                    continue

                # 가장 근접한 객체 선택
                if sub_name not in ("traffic_red", "traffic_green"):
                    if area > nearest_area:
                        nearest_area = area
                        detected_label = sub_name
                    continue

                # ==========================
                # 신호등 처리
                # ==========================
                width = x2 - x1
                height = y2 - y1
                ratio = height / width if width > 0 else 0

                if sub_name == "traffic_red":
                    traffic_light_state = "red"
                    traffic_light_visible = True  # 빨간불은 항상 보이는 것으로
                    traffic_light_area = area

                elif sub_name == "traffic_green":
                    traffic_light_state = "green"
                    traffic_light_area = area

                    # 초록불이 실제로 프레임에 보이는지 판정
                    if ratio < TRAFFIC_RATIO_LIMIT or area > TRAFFIC_AREA_LIMIT:
                        traffic_light_visible = False
                    else:
                        traffic_light_visible = True
                    # ==========================
                    # 탐지 시각화 (디버깅용)
                    # ==========================
                    debug_frame = roi.copy()
                    for box in results[0].boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        conf = float(box.conf[0])
                        cls_id = int(box.cls[0])
                        cls_name = results[0].names[cls_id]
                        cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(debug_frame, f"{cls_name} {conf:.2f}",
                                    (x1, max(0, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5, (0, 255, 255), 1)

                    # 실시간 미리보기 창
                    cv2.imshow("Object Detection (ROI)", debug_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord('q')):  # ESC 또는 'q' 누르면 창 닫기
                        print("[INFO] Debug window closed by user.")
                        cv2.destroyAllWindows()


            # shared_state 갱신
            with shared_state.lock:
                shared_state.object_detected = detected_label
                shared_state.object_distance = nearest_area
                if traffic_light_state:
                    shared_state.traffic_light = traffic_light_state
                    shared_state.traffic_light_visible = traffic_light_visible
                    shared_state.traffic_light_area = traffic_light_area
                    shared_state.traffic_light_last_ts = now

            # 근접 조건 & 쿨다운 처리
            if (
                detected_label
                and nearest_area > NEAR_AREA
                and (now - last_action_time > COOLDOWN)
            ):
                print(f"[DETECTED] {detected_label} (area={nearest_area})")

                with shared_state.lock:
                    shared_state.last_trigger = detected_label

                last_label = detected_label
                last_action_time = now

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n[INFO] Object detector stopped by user.")
    finally:
        print("[✓] Detector cleanup complete")

