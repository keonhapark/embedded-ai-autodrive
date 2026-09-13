"""
Sense 계층 테스트입니다.

legacy_measure() 는 리팩토링 전 코드(salvage/pi-final-2025-12-13:lane_tracer.py
592-661)를 옮긴 것입니다. 중앙 박스 비율(0.6 / 0.15 / 0.3)만 원본 리터럴이고
나머지 상수는 config 를 읽습니다. vision.measure() 가 같은 값을 내는지 합성
프레임으로 비교합니다.

라즈베리파이가 없어 실제 트랙 영상으로 검증할 수 없으므로, 원본 코드와의
동등성이 이 계층에서 확인할 수 있는 최대한입니다.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import numpy as np
import pytest

import config
import scenes
import vision

CYAN = scenes.CYAN


# ============================================================
# 리팩토링 전 코드 (그대로 옮김)
# ============================================================

def legacy_measure(frame):
    """리팩토링 전 계산을 그대로 옮긴 것입니다.

    [원위치] salvage/pi-final-2025-12-13:lane_tracer.py 592-661

    중앙 박스 비율 0.6 / 0.15 / 0.3 은 **의도적으로 리터럴**입니다. 원본 값을
    그대로 둬야 config.CENTER_BOX_* 로 옮긴 값이 맞는지 검증됩니다. config 를
    읽게 바꾸면 양쪽이 함께 변해서, 동등성은 유지되면서 동작만 바뀝니다.

    나머지 상수(ROI 비율, MORPH, HSV)는 config 를 공유하므로 여기서는 검증되지
    않습니다. test_control.py 의 test_원본_상수값을_그대로_옮겼다 가 그 값들을
    못 박습니다.
    """
    height, width = frame.shape[:2]
    BOX_WIDTH_RATIO = config.ROI_BOX_WIDTH_RATIO
    BOX_HEIGHT_RATIO = config.ROI_BOX_HEIGHT_RATIO
    lower_cyan = np.array(config.LANE_HSV_LOWER)
    upper_cyan = np.array(config.LANE_HSV_UPPER)
    lower_blue = np.array(config.STOPLINE_HSV_LOWER)
    upper_blue = np.array(config.STOPLINE_HSV_UPPER)

    BOX_WIDTH = int(width * BOX_WIDTH_RATIO)
    BOX_HEIGHT = int(height * BOX_HEIGHT_RATIO)

    left_box_x1 = 0
    left_box_y1 = height - BOX_HEIGHT
    left_box_x2 = BOX_WIDTH
    left_box_y2 = height

    right_box_x1 = width - BOX_WIDTH
    right_box_y1 = height - BOX_HEIGHT
    right_box_x2 = width
    right_box_y2 = height

    center_box_width = int(width * 0.6)
    center_box_height = int(height * 0.15)
    center_box_x1 = (width - center_box_width) // 2
    center_box_y1 = int(height * 0.3)
    center_box_x2 = center_box_x1 + center_box_width
    center_box_y2 = center_box_y1 + center_box_height

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    hsv_left = hsv_frame[left_box_y1:left_box_y2, left_box_x1:left_box_x2]
    mask_left = cv2.inRange(hsv_left, lower_cyan, upper_cyan)
    kernel = np.ones((config.MORPH_KERNEL_SIZE, config.MORPH_KERNEL_SIZE), np.uint8)
    mask_left = cv2.erode(mask_left, kernel, iterations=config.MORPH_ERODE_ITERATIONS)
    mask_left = cv2.dilate(mask_left, kernel, iterations=config.MORPH_DILATE_ITERATIONS)

    hsv_right = hsv_frame[right_box_y1:right_box_y2, right_box_x1:right_box_x2]
    mask_right = cv2.inRange(hsv_right, lower_cyan, upper_cyan)
    mask_right = cv2.erode(mask_right, kernel, iterations=config.MORPH_ERODE_ITERATIONS)
    mask_right = cv2.dilate(mask_right, kernel, iterations=config.MORPH_DILATE_ITERATIONS)

    hsv_center = hsv_frame[center_box_y1:center_box_y2, center_box_x1:center_box_x2]
    mask_center = cv2.inRange(hsv_center, lower_cyan, upper_cyan)
    mask_center = cv2.erode(mask_center, kernel, iterations=config.MORPH_ERODE_ITERATIONS)
    mask_center = cv2.dilate(mask_center, kernel, iterations=config.MORPH_DILATE_ITERATIONS)

    blue_box_x1 = 0
    blue_box_y1 = int(height * config.STOPLINE_ROI_TOP_RATIO)
    blue_box_x2 = width
    blue_box_y2 = height

    hsv_blue_area = hsv_frame[blue_box_y1:blue_box_y2, blue_box_x1:blue_box_x2]
    mask_blue = cv2.inRange(hsv_blue_area, lower_blue, upper_blue)
    mask_blue = cv2.erode(mask_blue, kernel, iterations=config.MORPH_ERODE_ITERATIONS)
    mask_blue = cv2.dilate(mask_blue, kernel, iterations=config.MORPH_DILATE_ITERATIONS)
    blue_pixels = cv2.countNonZero(mask_blue)

    left_pixels = cv2.countNonZero(mask_left)
    right_pixels = cv2.countNonZero(mask_right)
    center_pixels = cv2.countNonZero(mask_center)

    return left_pixels, right_pixels, center_pixels, blue_pixels


# ============================================================
# 장면은 tests/scenes.py 에 모여 있습니다 (test_loop.py 와 공용)
# vision.measure() 에는 뒤집지 않은 장면을 그대로 넣습니다.
# ============================================================

SCENES = scenes.ALL
blank = scenes.blank
fill = scenes.fill
scene_both_lanes = scenes.both_lanes
scene_right_lane_missing = scenes.right_lane_missing
scene_intersection = scenes.intersection
scene_stopline = scenes.stopline
scene_thin_noise = scenes.thin_noise


# ============================================================
# 동등성
# ============================================================

@pytest.mark.parametrize("name", sorted(SCENES))
def test_리팩토링_전_계산과_같다(name):
    frame = SCENES[name]()
    m = vision.measure(frame)
    assert (m.left_pixels, m.right_pixels, m.center_pixels, m.blue_pixels) \
        == legacy_measure(frame), name


@pytest.mark.parametrize("size", [(640, 480), (320, 240), (1280, 720)])
def test_해상도가_달라도_원본과_같다(size):
    """ROI 를 비율로 계산하므로 해상도를 따라가야 합니다."""
    width, height = size
    frame = fill(blank(width, height), CYAN, 0, height, 0, width)
    m = vision.measure(frame)
    assert (m.left_pixels, m.right_pixels, m.center_pixels, m.blue_pixels) \
        == legacy_measure(frame)


# ============================================================
# 동작 서술
# ============================================================

def test_양쪽_차선이_보이면_좌우가_비슷하다():
    m = vision.measure(scene_both_lanes())
    assert m.left_pixels > 0 and m.right_pixels > 0
    assert m.left_pixels == m.right_pixels          # 대칭 장면
    assert m.total_pixels == m.left_pixels + m.right_pixels


def test_한쪽_차선이_없으면_그쪽이_0():
    m = vision.measure(scene_right_lane_missing())
    assert m.left_pixels > 0
    assert m.right_pixels == 0


def test_교차로_장면은_중앙만_차_있다():
    m = vision.measure(scene_intersection())
    assert m.center_pixels > 0
    assert m.total_pixels == 0


def test_정지선_장면은_파란색만_잡힌다():
    m = vision.measure(scene_stopline())
    assert m.blue_pixels > 0
    assert m.total_pixels == 0, "파란색이 차선으로도 잡히면 안 된다"


def test_빈_화면은_전부_0():
    assert vision.measure(blank()) == vision.Measurements()


def test_얇은_노이즈는_침식으로_지워진다():
    """1px 선은 MORPH_ERODE_ITERATIONS(2회) 침식에서 살아남지 못합니다."""
    assert vision.measure(scene_thin_noise()).total_pixels == 0


def test_기본_Measurements_는_전부_0():
    """교차로 대기 중 차선을 보지 않는 원본 동작을 이 기본값으로 대신합니다."""
    m = vision.Measurements()
    assert (m.left_pixels, m.right_pixels, m.center_pixels,
            m.blue_pixels, m.total_pixels) == (0, 0, 0, 0, 0)


# ============================================================
# as_camera_frame 전제 — 이걸 빠뜨리면 루프 테스트가 조용히 무의미해집니다
# ============================================================

def test_as_camera_frame_은_루프의_flip_과_상쇄된다():
    """이 단정이 깨지면 tests/scenes.py 의 전제가 무너진 것입니다."""
    for name, make in scenes.ALL.items():
        original = make()
        assert np.array_equal(
            vision.upright(scenes.as_camera_frame(original)), original), name


def test_비대칭_장면은_헬퍼를_빠뜨리면_값이_달라진다():
    """헬퍼 누락을 드러내는 장면이 최소 하나 있어야 합니다.

    모든 장면이 상하좌우 대칭이면 as_camera_frame 을 빠뜨려도 측정값이 같아서
    루프 테스트가 통과하면서 아무것도 검증하지 않게 됩니다.
    """
    scene = scenes.uneven_lanes()
    assert vision.measure(scene) != vision.measure(scenes.as_camera_frame(scene))


def test_얼룩_장면은_상하좌우_모두_비대칭이다():
    """flip 의 수평 성분과 수직 성분을 각각 검증할 수 있어야 합니다."""
    scene = scenes.speckle()
    assert vision.measure(scene) != vision.measure(cv2.flip(scene, 0))   # 수직만
    assert vision.measure(scene) != vision.measure(cv2.flip(scene, 1))   # 수평만
