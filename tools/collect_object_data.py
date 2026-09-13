import mycamera
import cv2
import time
import os
from gpiozero import DigitalOutputDevice, PWMOutputDevice

# --- 모터 핀 설정 ---
PWMA = PWMOutputDevice(18)
AIN1 = DigitalOutputDevice(22)
AIN2 = DigitalOutputDevice(27)

PWMB = PWMOutputDevice(23)
BIN1 = DigitalOutputDevice(25)
BIN2 = DigitalOutputDevice(24)

# --- 모터 제어 함수 ---
def motor_go(speed):
    AIN1.value = 0; AIN2.value = 1; PWMA.value = speed
    BIN1.value = 0; BIN2.value = 1; PWMB.value = speed

def motor_back(speed):
    AIN1.value = 1; AIN2.value = 0; PWMA.value = speed
    BIN1.value = 1; BIN2.value = 0; PWMB.value = speed

def motor_left(speed):
    AIN1.value = 1; AIN2.value = 0; PWMA.value = 0.0
    BIN1.value = 0; BIN2.value = 1; PWMB.value = speed

def motor_right(speed):
    AIN1.value = 0; AIN2.value = 1; PWMA.value = speed
    BIN1.value = 1; BIN2.value = 0; PWMB.value = 0.0

def motor_stop():
    AIN1.value = 0; AIN2.value = 0; PWMA.value = 0.0
    BIN1.value = 0; BIN2.value = 0; PWMB.value = 0.0

# --- 기본 설정 ---
speedSet = 0.45  # 속도 (0.0~1.0)
save_dir = "/home/pi/AI_CAR/object_data/images"
os.makedirs(save_dir, exist_ok=True)

def main():
    camera = mycamera.MyPiCamera(640, 480)
    print("[INFO] Camera initialized. Press ↑ ↓ ← → to drive, 'q' to quit.")

    frame_interval = 0.3  # 초당 약 3장 (fps 조절용)
    last_time = time.time()
    frame_count = 0
    carState = "stop"

    while camera.isOpened():
        keyValue = cv2.waitKey(10)

        # --- 키 입력 처리 ---
        if keyValue == ord('q'):
            break
        elif keyValue == 82:  # ↑
            print("go"); carState = "go"; motor_go(speedSet)
        elif keyValue == 84:  # ↓
            print("stop"); carState = "stop"; motor_stop()
        elif keyValue == 81:  # ←
            print("left"); carState = "left"; motor_left(speedSet)
        elif keyValue == 83:  # →
            print("right"); carState = "right"; motor_right(speedSet)

        # --- 프레임 캡처 ---
        ret, frame = camera.read()
        if not ret:
            continue

        frame = cv2.flip(frame, -1)
        cv2.imshow("Camera View", frame)

        # 일정 시간 간격으로 저장
        now = time.time()
        if now - last_time >= frame_interval:
            filename = f"{save_dir}/frame_{frame_count:05d}.jpg"
            cv2.imwrite(filename, frame)
            print(f"[SAVE] {filename}")
            frame_count += 1
            last_time = now

    camera.release()
    motor_stop()
    cv2.destroyAllWindows()
    print("[INFO] Data collection finished.")

if __name__ == "__main__":
    main()
