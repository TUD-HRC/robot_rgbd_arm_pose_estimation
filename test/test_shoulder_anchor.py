"""The shoulder frame the swivel solver broadcasts: live during the init window, then fixed."""

from arm_pose_estimation.arm_solver_swivel import ShoulderAnchor
import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R


IDENTITY = np.array([0.0, 0.0, 0.0, 1.0])


def test_takes_median_after_window():
    anchor = ShoulderAnchor(init_window_sec=1.0)
    assert anchor.pose is None
    for i, x in enumerate((1.0, 1.1, 5.0, 0.9, 1.0)):  # 5.0: a depth outlier
        assert not anchor.frozen
        anchor.add_sample(np.array([x, 0.0, 0.0]), IDENTITY, 0.25 * i)
        if i < 4:
            assert anchor.pose[0][0] == x  # follows the samples during the window
    assert anchor.frozen
    assert anchor.sample_count == 5
    assert anchor.pose[0][0] == pytest.approx(1.0)

    # Later samples are ignored.
    anchor.add_sample(np.array([9.0, 0.0, 0.0]), IDENTITY, 9.0)
    assert anchor.pose[0][0] == pytest.approx(1.0)


def test_orientation_is_the_mean():
    anchor = ShoulderAnchor(init_window_sec=1.0)
    for i, angle in enumerate((-0.1, 0.1)):
        # q and -q are the same rotation; the mean must not cancel them out.
        quaternion = R.from_rotvec([0.0, 0.0, angle]).as_quat() * (-1.0) ** i
        anchor.add_sample(np.zeros(3), quaternion, float(i))
    assert anchor.frozen
    assert R.from_quat(anchor.pose[1]).magnitude() == pytest.approx(0.0, abs=1e-9)
