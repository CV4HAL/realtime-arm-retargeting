from types import SimpleNamespace

import numpy as np
import pytest

from holistic_tracking import hand_openness

WIDTH, HEIGHT = 640, 480
FINGER_OFFSETS_PX = {8: -30, 12: 0, 16: 25, 20: 45}


def make_hand(finger_length_px, angle_deg):
    points = np.zeros((21, 2))
    points[9] = [0, -100]
    for index, offset in FINGER_OFFSETS_PX.items():
        points[index] = [offset, -finger_length_px]
    c, s = np.cos(np.radians(angle_deg)), np.sin(np.radians(angle_deg))
    points = points @ np.array([[c, -s], [s, c]]).T + [WIDTH / 2, HEIGHT / 2]
    return SimpleNamespace(landmark=[
        SimpleNamespace(x=x / WIDTH, y=y / HEIGHT, z=0.0) for x, y in points])


@pytest.mark.parametrize("finger_length_px", [110, 150, 200])
def test_openness_is_stable_under_in_plane_rotation(finger_length_px):
    values = [hand_openness(make_hand(finger_length_px, angle), WIDTH, HEIGHT)
              for angle in range(0, 360, 15)]
    assert max(values) - min(values) < 0.01


def test_open_hand_reads_higher_than_fist():
    fist = hand_openness(make_hand(110, 30), WIDTH, HEIGHT)
    open_hand = hand_openness(make_hand(200, 30), WIDTH, HEIGHT)
    assert open_hand > fist
