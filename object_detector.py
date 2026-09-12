# ============================================================
# object_detector.py (리팩토링 완료 버전)
# - 탐지: sign / traffic
# - 분류: go_straight, turn_left, ..., traffic_green, traffic_red
# - name_mapping 제거
# ============================================================

import time
import cv2
import numpy as np
from ultralytics import YOLO
import shared_state
import os
from datetime import datetime
from PIL import Image


# ============================================================
# 모델 경로
# ============================================================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(BASE_DIR)

DETECTOR_PATH = os.path.join(PARENT_DIR, "models", "sign_traffic_detectorV3.pt")   # sign / traffic
CLASSIFIER_PATH = os.path.join(PARENT_DIR, "models", "sign_traffic_classifierV3.pt")   # 8-class classifier

MIN_AREA = 10000
NEAR_AREA = 20000
CONF_THRESHOLD = 0.8
CLASSIFIER_CONF_THRESHOLD = 0.90


CAPTURE_FOLDER = "/home/keonha/AI_CAR/captured_images"
MAX_CAPTURES_PER_OBJECT = 1



# ============================================================
# 메인 루프
# ============================================================
def object_detect_loop():

    print("=" * 70)
    print(" YOLOv8 Object Detector (Detector → Classifier 구조)")
    print("=" * 70)

    # ---------------------------
    # 모델 로드
    # ---------------------------
    if not os.path.exists(DETECTOR_PATH):
        print(f"❌ 탐지모델 없음: {DETECTOR_PATH}")
        return

    detector = YOLO(DETECTOR_PATH)
    print("✓ 탐지 모델 로드완료")

    classifier = None
    if os.path.exists(CLASSIFIER_PATH):
        classifier = YOLO(CLASSIFIER_PATH)
        print("✓ 분류 모델 로드완료 (8-class)")
    else:
        print("⚠️ 분류 모델 없음 → 탐지만 수행")


    # ---------------------------
    # 폴더 준비
    # ---------------------------
    if not os.path.exists(CAPTURE_FOLDER):
        os.makedirs(CAPTURE_FOLDER)


    # ---------------------------
    # 실행 루프
    # ---------------------------
    try:
        while True:

            # 최신 프레임 가져오기
            with shared_state.lock:
                frame_rgb = getattr(shared_state, "latest_frame", None)

            if frame_rgb is None:
                time.sleep(0.05)
                continue

            _, width = frame_rgb.shape[:2]
            roi_rgb = frame_rgb[:, width // 2:]   # right half ROI


            # ------------------------------------------------
            # 1) 탐지 모델 실행 (sign / traffic)
            # ------------------------------------------------
            results = detector(roi_rgb, verbose=False)
            if not results or results[0].boxes is None:
                with shared_state.lock:
                    shared_state.object_detected = None
                time.sleep(0.1)
                continue


            detected_label = None
            detected_area = 0
            detected_conf = 0.0
            traffic_detected = False
            traffic_area = 0
            traffic_conf = 0


            # ------------------------------------------------
            # 2) 각 bounding box에 대해 classifier 실행
            # ------------------------------------------------
            for box in results[0].boxes:

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                area = (x2 - x1) * (y2 - y1)
                conf = float(box.conf[0])

                # 탐지 모델 신뢰도와 크기 조건
                if area < MIN_AREA or conf < CONF_THRESHOLD:
                    continue

                crop = roi_rgb[y1:y2, x1:x2]
                if crop.size == 0:
                    continue

                # -------------------------------
                # 분류 모델 실행
                # -------------------------------
                if classifier:
                    cls_res = classifier.predict(crop, imgsz=224, verbose=False)
                    if cls_res:
                        cls_id = int(cls_res[0].probs.top1)
                        cls_name = cls_res[0].names[cls_id]
                        cls_conf = float(cls_res[0].probs.top1conf)

                        if cls_conf < CLASSIFIER_CONF_THRESHOLD:
                            continue

                        # ----------------------
                        # traffic 신호등 별도 처리
                        # ----------------------
                        if "traffic" in cls_name:
                            traffic_detected = True
                            traffic_area = area
                            traffic_conf = cls_conf
                        else:
                            # 일반 표지판은 가장 큰 객체 선택
                            if area > detected_area:
                                detected_area = area
                                detected_label = cls_name
                                detected_conf = cls_conf

                        print(f"🔍 분류 결과: {cls_name} ({cls_conf:.0%}) area={area}")

                else:
                    # classifier 없으면 detector 결과만 사용
                    detected_label = results[0].names[int(box.cls[0])]
                    detected_area = area
                    detected_conf = conf


            # ------------------------------------------------
            # shared_state 업데이트
            # ------------------------------------------------
            with shared_state.lock:

                # 모든 객체 false 초기화
                for k in shared_state.object_state:
                    shared_state.object_state[k] = False

                # 신호등 처리
                if traffic_detected:
                    shared_state.object_state["traffic"] = True
                    shared_state.object_area["traffic"] = traffic_area
                    shared_state.confidence["traffic"] = traffic_conf
                    shared_state.object_last_seen["traffic"] = time.time()

                # 표지판 처리
                if detected_label:
                    shared_state.object_state[detected_label] = True
                    shared_state.object_detected = detected_label
                    shared_state.object_area[detected_label] = detected_area
                    shared_state.confidence[detected_label] = detected_conf
                    shared_state.object_last_seen[detected_label] = time.time()
                else:
                    shared_state.object_detected = None

            time.sleep(0.10)

    except KeyboardInterrupt:
        print("🛑 Object detector stopped")
    finally:
        cv2.destroyAllWindows()

