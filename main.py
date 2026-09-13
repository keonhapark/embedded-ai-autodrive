"""
main.py
-------
라즈베리파이 자율주행 자동차의 진입점입니다.

* lane_tracer.py     : 차선 주행 스레드
* object_detector.py : 객체 탐지 스레드 (YOLOv8)
* hardware.py        : 모터·부저·카메라
* shared_state.py    : 두 스레드 간 공유 상태

종료 처리
--------
하드웨어는 main 이 소유하고 두 스레드에 넘깁니다. Ctrl+C 가 오면 stop_event 를
세우고 스레드를 join 한 뒤 모터를 정지시킵니다.

main 이 핀을 소유하므로 정지할 때 핀을 새로 열 필요가 없습니다. 원본은 주행
스레드가 점유한 핀에 PWMOutputDevice(18) 을 새로 만들어 정지를 시도했고,
GPIOPinInUse 가 except 에 삼켜져 Ctrl+C 를 눌러도 모터가 멈추지 않았습니다.
"""

import threading
import time

import config
import hardware
import shared_state
from lane_tracer import lane_follow_loop

# 스레드가 정리를 마칠 때까지 기다리는 시간입니다.
# 주행 루프에는 stop_event 를 보지 않는 sleep 이 한 프레임에 최대 2.5초
# (STOP 정지 2초 + 저속 재출발 0.5초) 있어 그보다 넉넉히 줍니다.
SHUTDOWN_TIMEOUT_SEC = 6.0


def main():
    print("=" * 70)
    print(" Autonomous Car System: Line + Object Integration")
    print("=" * 70)

    # object_detector 는 ultralytics(YOLO)를 끌어옵니다. 함수 안에서 import 하면
    # 라즈베리파이가 아닌 곳에서도 main 모듈을 import 할 수 있어 종료 경로를
    # 테스트할 수 있습니다. hardware.py 가 gpiozero 를 다루는 방식과 같습니다.
    from object_detector import object_detect_loop

    stop_event = threading.Event()

    # --- 하드웨어는 main 이 소유합니다 ---
    # 스레드 안에서 만들면 종료 시 main 이 모터를 정지시킬 수 없습니다.
    motor = hardware.Motor(hardware.create_gpio_pins())
    buzzer = hardware.Buzzer(hardware.create_gpio_buzzer())
    camera = hardware.create_camera()
    if camera is None:
        print("[오류] 카메라를 열 수 없어 종료한다.")
        motor.close()
        buzzer.close()
        return

    # daemon=True 는 join 이 시간 안에 끝나지 않아도 프로세스가 매달리지 않게
    # 하는 최후 수단입니다. 정상 종료 경로는 stop_event 입니다.
    lane_thread = threading.Thread(
        target=lane_follow_loop, daemon=True,
        kwargs={"motor": motor, "buzzer": buzzer, "camera": camera,
                "stop_event": stop_event})
    detect_thread = threading.Thread(
        target=object_detect_loop, daemon=True,
        kwargs={"stop_event": stop_event})
    lane_thread.start()
    detect_thread.start()

    print("[✓] Threads started (Lane Follower + Object Detector)")
    print("[INFO] Press Ctrl+C to terminate\n")

    try:
        # 주행 스레드가 끝나면(정지선 감지 등) 메인도 끝냅니다.
        while lane_thread.is_alive():
            # 공유 상태 확인
            with shared_state.lock:
                obj_name = shared_state.object_detected
                obj_dist = shared_state.object_distance
                obj_state = shared_state.object_state.copy()

            # --- 모니터링 출력 ---
            if obj_name:
                print(f"[MONITOR] Detected: {obj_name:12s} | area={obj_dist}")

            # 활성 상태맵 출력 (디버깅용)
            active_objects = [k for k, v in obj_state.items() if v]
            if active_objects:
                print(f"[ACTIVE] {', '.join(active_objects)}")

            time.sleep(config.MONITOR_INTERVAL_SEC)

    except KeyboardInterrupt:
        print("\n[INFO] Program stopped by user.")

    finally:
        shutdown(stop_event, (lane_thread, detect_thread), motor, buzzer, camera)


def shutdown(stop_event, threads, motor, buzzer, camera):
    """종료 신호를 보내고 스레드를 기다린 뒤 하드웨어를 정리합니다."""
    stop_event.set()

    for thread in threads:
        thread.join(timeout=SHUTDOWN_TIMEOUT_SEC)
        if thread.is_alive():
            print(f"[경고] {thread.name} 이 {SHUTDOWN_TIMEOUT_SEC}초 안에 "
                  f"끝나지 않았다. 모터는 아래에서 강제로 정지시킨다.")

    # 스레드가 끝났든 안 끝났든 모터는 정지시킵니다. 핀을 main 이 소유하므로
    # 새로 열 필요가 없습니다.
    try:
        motor.stop()
        print("[✓] Motors stopped.")
    except Exception as e:
        print(f"[오류] 모터 정지 실패: {e}")

    for resource in (motor, buzzer, camera):
        closer = getattr(resource, "close", None) or resource.release
        try:
            closer()
        except Exception as e:
            print(f"[정리 실패] {type(resource).__name__}: {e}")

    print("[✓] Cleanup complete.")


if __name__ == "__main__":
    main()
