"""Hardware feedback must reflect measured motion and sensor validity."""
import math

import pytest

from amr_bringup.hardware_feedback import EncoderTicks, nearest_range


def test_encoder_quantizes_measured_motion_with_named_wheels():
    counter = EncoderTicks(100)
    names = ['right_wheel_joint', 'left_wheel_joint']
    assert counter.update(names, [1.0, 2.0], 1) == (0, 0)
    assert counter.update(names, [1.0-math.pi, 2.0+2*math.pi], 2) == (100, -50)
    # Sub-tick movement stays at zero until crossing half a tick.
    assert counter.update(names, [1.0, 2.0+0.02], 3) == (0, 0)


def test_encoder_rejects_bad_samples_and_resets_on_sim_restart():
    counter = EncoderTicks(100)
    names = ['left_wheel_joint', 'right_wheel_joint']
    assert counter.update(names, [0.0, 0.0], 10) == (0, 0)
    for positions in ([math.nan, 0.0], [0.0], [0.0, math.inf]):
        assert counter.update(names, positions, 11) is None
    assert counter.update(names[:1], [0.0], 11) is None
    assert counter.update(names, [2*math.pi, 0.0], 12) == (100, 0)
    assert counter.update(names, [2*math.pi, 0.0], 12) is None
    assert counter.update(names, [0.0, 0.0], 0) == (0, 0)


@pytest.mark.parametrize('cpr', [0, -1, 1.5, math.nan, math.inf])
def test_encoder_resolution_must_be_a_positive_integer(cpr):
    with pytest.raises(ValueError):
        EncoderTicks(cpr)


def test_range_selects_nearest_obstacle_and_uses_infinity_conventions():
    assert nearest_range([math.inf, 1.0, 0.4, math.nan], 0.03, 2.0) == 0.4
    assert nearest_range([2.1, math.inf, math.nan], 0.03, 2.0) == math.inf
    assert nearest_range([0.02, 0.5], 0.03, 2.0) == -math.inf
    assert nearest_range([-math.inf], 0.03, 2.0) == -math.inf
    assert nearest_range([], 0.03, 2.0) == math.inf
