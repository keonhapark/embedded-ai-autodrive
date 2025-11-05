import time
import cv2
import numpy as np
from ultralytics import YOLO
import os
import time

# =============================
# 모델 경로
# =============================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DETECTOR_PATH = os.path.join(BASE_DIR, "models", "sign_traffic_detector.pt")
CLASSIFIER_PATH = os.path.join(BASE_DIR, "models", "sign_traffic_classifier.pt")

# =============================
# YOLO 모델 로드
# =============================
detector = YOLO(DETECTOR_PATH)
classifier = YOLO(CLASSIFIER_PATH)

# =============================
# 카메라 초기화
# =============================
from picamera2 import Picamera2
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"format": "RGB888", "size": (640, 480)}
)
picam2.configure(config)
picam2.start()
time.sleep(1)
print("[INFO] Camera ready. Press Ctrl+C to stop.")

# =============================
# 탐지 루프
# =============================
prev_time = time.time()

try:
    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        frame = cv2.flip(frame, -1)   # 상하좌우 반전 (카메라 뒤집힘 보정)
        
        now = time.time()
        fps = 1.0 / (now - prev_time)
        prev_time = now

        # 오른쪽 절반만 ROI로 설정
        roi = frame[:, 320:640]
        results = detector(roi, verbose=False)

        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            cls_name = results[0].names[cls_id]

            if conf < 0.5:
                continue

            crop = roi[y1:y2, x1:x2]
            cls_res = classifier.predict(crop, imgsz=224, verbose=False)
            sub_id = int(cls_res[0].probs.top1)
            sub_name = cls_res[0].names[sub_id]
            sub_conf = float(cls_res[0].probs.top1conf)

            if sub_conf < 0.7:
                continue

            cv2.rectangle(roi, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                roi, f"{sub_name} ({sub_conf:.2f})",
                (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 0), 2
            )
            print(f"[DETECTED] {sub_name} (conf={sub_conf:.2f})")

        # 카메라 뷰 표시
        cv2.imshow("Object Detection (Right Half)", roi)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\n[INFO] Stopped by user.")
finally:
    picam2.stop()
    cv2.destroyAllWindows()
