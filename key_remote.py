"""
Keyboard Control - 키보드로 로봇 제어 (기본 lgpio 백엔드)
W/A/S/D 키로 로봇을 움직입니다 (SSH 환경용)
"""
import time
import sys
import tty
import termios
from gpiozero import DigitalOutputDevice, PWMOutputDevice

# 모터 핀 설정 (gpiozero 기본 백엔드 사용)
PWMA = PWMOutputDevice(18)
AIN1 = DigitalOutputDevice(22)
AIN2 = DigitalOutputDevice(27)

PWMB = PWMOutputDevice(23)
BIN1 = DigitalOutputDevice(25)
BIN2 = DigitalOutputDevice(24)

# 속도 설정
SPEED = 0.5  # 0.0 ~ 1.0

def motor_forward(speed=SPEED):
    """전진"""
    AIN1.off()
    AIN2.on()
    PWMA.value = speed
    BIN1.off()
    BIN2.on()
    PWMB.value = speed
    print(f"[MOTOR] FORWARD (speed={speed})")

def motor_backward(speed=SPEED):
    """후진"""
    AIN1.on()
    AIN2.off()
    PWMA.value = speed
    BIN1.on()
    BIN2.off()
    PWMB.value = speed
    print(f"[MOTOR] BACKWARD (speed={speed})")

def motor_left(speed=SPEED):
    """좌회전"""
    AIN1.on()
    AIN2.off()
    PWMA.value = 0.0
    BIN1.off()
    BIN2.on()
    PWMB.value = speed
    print(f"[MOTOR] LEFT (speed={speed})")

def motor_right(speed=SPEED):
    """우회전"""
    AIN1.off()
    AIN2.on()
    PWMA.value = speed
    BIN1.on()
    BIN2.off()
    PWMB.value = 0.0
    print(f"[MOTOR] RIGHT (speed={speed})")

def motor_stop():
    """정지"""
    AIN1.off()
    AIN2.off()
    PWMA.value = 0.0
    BIN1.off()
    BIN2.off()
    PWMB.value = 0.0
    print("[MOTOR] STOP")

def get_key():
    """SSH 터미널 키 입력"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':  # ESC 시퀀스 (화살표키)
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                ch3 = sys.stdin.read(1)
                if ch3 == 'A': return 'UP'
                elif ch3 == 'B': return 'DOWN'
                elif ch3 == 'C': return 'RIGHT'
                elif ch3 == 'D': return 'LEFT'
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def main():
    print("=" * 60)
    print(" Keyboard Control (SSH + gpiozero 기본 백엔드)")
    print("=" * 60)
    print("Controls:")
    print("  W / ↑ : Forward")
    print("  S / ↓ : Backward")
    print("  A / ← : Left")
    print("  D / → : Right")
    print("  X     : Stop")
    print("  Q     : Quit")
    print()
    print("[INFO] Press keys to control robot...")
    print()

    try:
        while True:
            key = get_key()

            if key in ['q', 'Q']:
                print("\n[INFO] Quitting...")
                break
            elif key in ['w', 'W', 'UP']:
                motor_forward()
            elif key in ['s', 'S', 'DOWN']:
                motor_backward()
            elif key in ['a', 'A', 'LEFT']:
                motor_left()
            elif key in ['d', 'D', 'RIGsT']:
                motor_right()
            elif key in ['x', 'X', ' ']:
                motor_stop()

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user")
    finally:
        motor_stop()
        PWMA.close()
        PWMB.close()
        print("[✓] GPIO cleanup complete")

if __name__ == '__main__':
    main()