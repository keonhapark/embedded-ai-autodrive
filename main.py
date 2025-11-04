"""
main.py
-------

라즈베리파이 자동차 주행 메인 스크립트

* lane_tracer.py (차선 주행)
* object_detector.py (객체 탐지 및 근접 이벤트)
* shared_state.py (상태 공유)
"""

import threading
import time
import shared_state
from lane_tracer import lane_follow_loop
from object_detector import object_detect_loop
from gpiozero import PWMOutputDevice

# ============================================================
# 메인 실행 함수
# ============================================================


def main():
    print("=" * 70)
    print("🚗 Autonomous Car System: Line + Object Integration")
    print("=" * 70)

    # 두 스레드 정의
    lane_thread = threading.Thread(target=lane_follow_loop, daemon=True)
    detect_thread = threading.Thread(target=object_detect_loop, daemon=True)

    # 스레드 시작
    lane_thread.start()
    detect_thread.start()

    print("[✓] Threads started (Lane Follower + Object Detector)")
    print("[INFO] Press Ctrl+C to terminate\n")

    # 메인 루프: 상태 모니터링
    last_trigger_displayed = None

    try:
        while True:
            with shared_state.lock:
                obj = shared_state.object_detected
                dist = shared_state.object_distance
                trig = shared_state.last_trigger
                tl_state = getattr(shared_state, "traffic_light", None)
                tl_vis = getattr(shared_state, "traffic_light_visible", False)

            # 탐지 결과 주기적 출력
            if obj:
                print(f"[MONITOR] Detected: {obj:10s} | area={dist}")
            if tl_state:
                print(f"[TL] {tl_state.upper():<6s} | Visible: {tl_vis}")
            if trig and trig != last_trigger_displayed:
                print(f"\n🎯 Action Triggered: {trig}\n")
                last_trigger_displayed = trig

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\n[INFO] Program stopped by user.")

    finally:
        # 모터 정지 (비상 안전)
        try:
            PWMA = PWMOutputDevice(18)
            PWMB = PWMOutputDevice(23)
            PWMA.value = 0.0
            PWMB.value = 0.0
            print("[✓] Motors stopped.")
        except Exception:
            pass

        print("[✓] All threads stopped. Cleanup complete.")


if __name__ == "__main__":
    main()
