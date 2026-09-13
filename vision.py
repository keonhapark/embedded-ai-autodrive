"""
vision.py
---------
Sense 계층입니다. 카메라 프레임 하나를 픽셀 수 몇 개로 줄입니다.

## 경계

입력은 프레임(numpy 배열)이고 출력은 정수 몇 개입니다. GPIO·시간·공유 상태에
의존하지 않으므로 합성 프레임으로 테스트할 수 있습니다.

cv2 와 numpy 에는 의존합니다. 두 패키지는 맥·리눅스에서도 설치되고, 격리하면
검증하려는 마스킹 계산이 테스트에서 빠집니다. 격리 대상은 그 기기에만 있는 것
(GPIO, Picamera2)이며 hardware.py 가 맡습니다.

판단 로직(control.py)과 분리돼 있어서 "이 프레임에서 무엇을 봤는가"와 "그래서
무엇을 할 것인가"를 따로 검증할 수 있습니다.

[원위치] salvage/pi-final-2025-12-13:lane_tracer.py 592-661
(665-674 의 좌우 비율 계산은 판단이라 control.balance_ratios 에 있습니다.)
"""

from dataclasses import dataclass

import cv2
import numpy as np

import config


@dataclass(frozen=True)
class Measurements:
    """한 프레임에서 읽어낸 값입니다.

    기본값이 전부 0 입니다. 교차로 대기 중에는 차선을 보지 않으므로
    Measurements() 를 그대로 씁니다. 원본도 그때 모든 값을 0 으로 두었습니다.
    """
    left_pixels: int = 0      # 좌하단 박스의 차선 픽셀
    right_pixels: int = 0     # 우하단 박스의 차선 픽셀
    center_pixels: int = 0    # 전방 중앙 박스 (교차로 감지용)
    blue_pixels: int = 0      # 화면 하단 1/4 의 파란색 (정지선)

    @property
    def total_pixels(self):
        return self.left_pixels + self.right_pixels


def upright(frame):
    """카메라가 거꾸로 장착돼 있어 프레임을 180도 돌립니다.

    픽셀 좌표를 해석하는 일이라 Sense 계층에 둡니다. 주행 루프에 cv2 호출이
    없어야 tests/test_structure.py 가 "Act 계층에는 픽셀 연산이 없다"를 검증할
    수 있습니다.

    호출은 루프가 합니다. 돌린 프레임을 객체 탐지 스레드도 공유하므로
    measure() 안에 두면 공유 프레임이 돌지 않은 상태가 됩니다.

    [원위치] lane_tracer.py 의 cv2.flip(frame, -1)
    """
    return cv2.flip(frame, -1) if config.CAMERA_FLIP else frame


def _clean(mask, kernel):
    """노이즈를 제거합니다. 침식한 뒤 팽창합니다.

    팽창을 침식보다 많이 합니다(2 대 3). 얇은 노이즈를 지운 뒤 차선을 원래보다
    굵게 되돌려 끊긴 선을 잇는 의도로 보입니다.
    """
    mask = cv2.erode(mask, kernel, iterations=config.MORPH_ERODE_ITERATIONS)
    return cv2.dilate(mask, kernel, iterations=config.MORPH_DILATE_ITERATIONS)


def measure(frame):
    """프레임에서 차선·정지선 픽셀 수를 셉니다.

    프레임은 RGB 입니다. Picamera2 가 RGB 로 주므로 COLOR_RGB2HSV 를 씁니다.
    OpenCV 기본인 BGR 로 넘기면 색상이 어긋나 아무것도 잡히지 않습니다.
    """
    height, width = frame.shape[:2]

    box_width = int(width * config.ROI_BOX_WIDTH_RATIO)
    box_height = int(height * config.ROI_BOX_HEIGHT_RATIO)

    # 전방 중앙 박스는 화면 위쪽에 둡니다. 교차로는 차가 도달하기 전에
    # 감지해야 하므로 발밑이 아니라 앞을 봅니다.
    center_width = int(width * config.CENTER_BOX_WIDTH_RATIO)
    center_height = int(height * config.CENTER_BOX_HEIGHT_RATIO)
    center_x1 = (width - center_width) // 2
    center_y1 = int(height * config.CENTER_BOX_TOP_RATIO)

    # HSV 변환은 전체 프레임에 한 번만 하고 박스별로 잘라 씁니다.
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    kernel = np.ones((config.MORPH_KERNEL_SIZE, config.MORPH_KERNEL_SIZE), np.uint8)

    lane_lo = np.array(config.LANE_HSV_LOWER)
    lane_hi = np.array(config.LANE_HSV_UPPER)

    def lane_pixels(region):
        return cv2.countNonZero(_clean(cv2.inRange(region, lane_lo, lane_hi), kernel))

    # 좌우 박스는 화면 하단 양 끝입니다. 차선 두 개를 따로 봅니다.
    left = lane_pixels(hsv[height - box_height:height, 0:box_width])
    right = lane_pixels(hsv[height - box_height:height, width - box_width:width])
    center = lane_pixels(hsv[center_y1:center_y1 + center_height,
                             center_x1:center_x1 + center_width])

    # 정지선은 화면 하단 1/4 의 전체 폭에서 봅니다.
    blue_top = int(height * config.STOPLINE_ROI_TOP_RATIO)
    blue = cv2.countNonZero(_clean(
        cv2.inRange(hsv[blue_top:height, 0:width],
                    np.array(config.STOPLINE_HSV_LOWER),
                    np.array(config.STOPLINE_HSV_UPPER)),
        kernel))

    return Measurements(left_pixels=left, right_pixels=right,
                        center_pixels=center, blue_pixels=blue)
