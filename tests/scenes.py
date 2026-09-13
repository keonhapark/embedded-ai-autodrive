"""
테스트용 합성 장면입니다. test_vision.py 와 test_loop.py 가 함께 씁니다.

## 좌표계 주의 — 카메라가 거꾸로 달려 있습니다

주행 루프는 프레임을 받으면 가장 먼저 vision.upright() 를 부릅니다. 그 안에서
config.CAMERA_FLIP 이 True 면 cv2.flip(frame, -1) 로 180° 돌립니다. 따라서

  - vision.measure() 에 직접 넣을 때는 장면을 그대로 씁니다.
  - lane_follow_loop() 에 넣을 때는 as_camera_frame() 으로 미리 뒤집어야
    루프의 flip 을 거쳐 의도한 장면이 됩니다.

이걸 빠뜨리면 화면 하단에 그린 차선이 상단으로 가서 ROI 박스가 비어 보입니다.

## 색 공간 주의

HSV 변환은 vision.measure() 만 하며 cv2.COLOR_RGB2HSV 를 씁니다. Picamera2 가
RGB 로 주기 때문입니다. OpenCV 기본인 BGR 로 만들면 색상이 어긋나 아무것도
잡히지 않습니다.
"""

import cv2
import numpy as np

import config

CYAN = (0, 255, 255)    # RGB. 차선
BLUE = (0, 0, 255)      # RGB. 정지선
BLACK = (0, 0, 0)


def blank(width=config.CAMERA_WIDTH, height=config.CAMERA_HEIGHT):
    return np.zeros((height, width, 3), dtype=np.uint8)


def fill(frame, rgb, y1, y2, x1, x2):
    frame[y1:y2, x1:x2] = rgb
    return frame


def as_camera_frame(scene):
    """루프에 넣을 형태로 미리 뒤집습니다. 루프의 flip 과 상쇄됩니다."""
    return cv2.flip(scene, -1)


# ============================================================
# 장면
# ============================================================

def both_lanes():
    """좌우 하단에 차선이 있고 중앙은 비어 있습니다.

    박스를 가득 채우므로 한쪽 픽셀이 19200 이 되어 HIGH_PIXEL_THRESHOLD(12000)
    를 넘습니다. 몇 프레임 이어지면 후진 복구로 넘어가므로, 정상 추종이 이어지는
    장면이 필요하면 moderate_lanes() 를 씁니다.
    """
    f = blank()
    h, w = f.shape[:2]
    bw = int(w * config.ROI_BOX_WIDTH_RATIO)
    bh = int(h * config.ROI_BOX_HEIGHT_RATIO)
    fill(f, CYAN, h - bh, h, 0, bw)
    fill(f, CYAN, h - bh, h, w - bw, w)
    return f


def right_lane_missing():
    f = blank()
    h, w = f.shape[:2]
    bw = int(w * config.ROI_BOX_WIDTH_RATIO)
    bh = int(h * config.ROI_BOX_HEIGHT_RATIO)
    fill(f, CYAN, h - bh, h, 0, bw)
    return f


def intersection():
    """전방 중앙만 차 있고 좌우는 비어 있음."""
    f = blank()
    h, w = f.shape[:2]
    cw = int(w * config.CENTER_BOX_WIDTH_RATIO)
    ch = int(h * config.CENTER_BOX_HEIGHT_RATIO)
    x1 = (w - cw) // 2
    y1 = int(h * config.CENTER_BOX_TOP_RATIO)
    fill(f, CYAN, y1, y1 + ch, x1, x1 + cw)
    return f


def stopline():
    """화면 하단 1/4 에 파란색."""
    f = blank()
    h, w = f.shape[:2]
    fill(f, BLUE, int(h * config.STOPLINE_ROI_TOP_RATIO), h, 0, w)
    return f


def moderate_lanes():
    """좌우 박스를 절반만 채웁니다. 정상 추종이 여러 프레임 이어지는 장면입니다.

    both_lanes() 는 픽셀이 HIGH_PIXEL_THRESHOLD 를 넘어 몇 프레임 뒤 후진 복구로
    넘어갑니다. 속도 프로파일처럼 여러 프레임에 걸친 동작을 보려면 급증으로
    잡히지 않는 밀도가 필요합니다.

    ALL 에는 넣지 않습니다. 좌우 대칭이고 ROI 내부 경계가 없어 동등성 테스트에
    변별력을 더하지 않습니다. uneven_lanes 가 ROI 내부 경계를, speckle 이 형태학
    연산을 이미 다룹니다. 루프 테스트 전용입니다.
    """
    f = blank()
    h, w = f.shape[:2]
    bw = int(w * config.ROI_BOX_WIDTH_RATIO)
    bh = int(h * config.ROI_BOX_HEIGHT_RATIO) // 2
    fill(f, CYAN, h - bh, h, 0, bw)
    fill(f, CYAN, h - bh, h, w - bw, w)
    return f


def uneven_lanes(right_fill=0.43):
    """왼쪽 박스는 가득, 오른쪽 박스는 일부만 채웁니다. 좌우 비대칭이라 회전 판단이 나옵니다.

    모든 장면이 좌우 대칭이면 diff 가 0 이라 항상 FORWARD 가 되고,
    motor.left / motor.right 로 가는 디스패치가 한 번도 실행되지 않습니다.
    right_fill 0.43 에서 편차 약 0.397 → 임계값(0.35) 초과, 강도 약 0.795 입니다.
    강도가 1.0 으로 포화되지 않아 강도 전달까지 검증됩니다.
    """
    f = blank()
    h, w = f.shape[:2]
    bw = int(w * config.ROI_BOX_WIDTH_RATIO)
    bh = int(h * config.ROI_BOX_HEIGHT_RATIO)
    fill(f, CYAN, h - bh, h, 0, bw)
    fill(f, CYAN, h - bh, h, w - int(bw * right_fill), w)
    return f


def uneven_lanes_mirrored(right_fill=0.43):
    """uneven_lanes 의 좌우 반전입니다. 반대 방향 회전이 나옵니다."""
    f = blank()
    h, w = f.shape[:2]
    bw = int(w * config.ROI_BOX_WIDTH_RATIO)
    bh = int(h * config.ROI_BOX_HEIGHT_RATIO)
    fill(f, CYAN, h - bh, h, w - bw, w)
    fill(f, CYAN, h - bh, h, 0, int(bw * right_fill))
    return f


def speckle(width=config.CAMERA_WIDTH, height=config.CAMERA_HEIGHT):
    """고정 시드 난수 블록 무늬입니다.

    다른 장면은 ROI 박스를 정확히 가득 채우거나 완전히 비워서, 박스를 1px 밀어도
    countNonZero 가 변하지 않고 형태 연산(팽창)도 검증되지 않습니다. 블록 크기
    7px 는 침식 2회를 통과하고 팽창 3회의 효과가 개수에 드러나는 중간 두께입니다.
    """
    rng = np.random.default_rng(20250913)
    cell = 7
    mask = rng.random((height // cell + 1, width // cell + 1)) < 0.45
    big = np.kron(mask, np.ones((cell, cell), dtype=bool))[:height, :width]
    f = blank(width, height)
    f[big] = CYAN
    return f


def thin_noise():
    """1px 선입니다. 침식으로 지워져야 합니다."""
    f = blank()
    h, w = f.shape[:2]
    fill(f, CYAN, h - 1, h, 0, w)
    return f


def all_lane_color():
    """화면 전체가 차선 색입니다. 180도 뒤집어도 같습니다."""
    return fill(blank(), CYAN, 0, config.CAMERA_HEIGHT, 0, config.CAMERA_WIDTH)


def empty():
    return blank()


# 동등성 테스트(test_vision.py)가 이 딕셔너리를 파라미터로 받습니다.
# 장면을 추가하면 리팩토링 전 계산과의 비교가 자동으로 늘어납니다.
ALL = {
    "양쪽 차선": both_lanes,
    "좌우 비대칭": uneven_lanes,
    "좌우 비대칭 반전": uneven_lanes_mirrored,
    "얼룩 무늬": speckle,
    "오른쪽 차선 없음": right_lane_missing,
    "교차로": intersection,
    "정지선": stopline,
    "빈 화면": empty,
    "얇은 노이즈": thin_noise,
    "전체 차선색": all_lane_color,
}
