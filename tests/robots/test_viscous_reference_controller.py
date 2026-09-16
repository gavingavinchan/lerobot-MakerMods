"""Offline checks for the experimental example; no hardware is constructed or connected."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location(
    "viscous_reference_controller",
    Path(__file__).parents[2] / "examples/viscous/reference_controller.py",
)
controller = importlib.util.module_from_spec(spec)
spec.loader.exec_module(controller)


def test_clipped_endpoint_can_finish_without_accepting_an_unfinished_ramp():
    lower, upper = np.array([-30.0, 2.9]), np.array([-2.4, 80.0])
    desired = np.array([-0.5, 0.0])
    assert controller.target_reached(np.array([-2.4, 2.9]), desired, lower, upper)
    assert not controller.target_reached(np.array([-4.0, 2.9]), desired, lower, upper)


def test_pan_can_adapt_beyond_old_limit_in_small_steps_without_mutating_previous():
    names = ["shoulder_pan.pos", "elbow_flex.pos"]
    previous = np.array([9.0, 0.0])
    updated = controller.corrected_offsets(names, previous, {"shoulder_pan.pos": 12.0})
    np.testing.assert_array_equal(updated, [12.0, 0.0])
    np.testing.assert_array_equal(previous, [9.0, 0.0])


@pytest.mark.parametrize(
    "previous,changes",
    [
        ([9.0, 0.0], {"shoulder_pan.pos": 13.0}),
        ([29.0, 0.0], {"shoulder_pan.pos": 31.0}),
        ([0.0, 9.0], {"elbow_flex.pos": 12.0}),
        ([0.0, 0.0], {"shoulder_pan.pos": float("nan")}),
        ([0.0, 0.0], {"shoulder_pan.pos": float("inf")}),
    ],
)
def test_corrections_retain_increment_and_total_bounds(previous, changes):
    with pytest.raises(ValueError):
        controller.corrected_offsets(["shoulder_pan.pos", "elbow_flex.pos"], np.array(previous), changes)
