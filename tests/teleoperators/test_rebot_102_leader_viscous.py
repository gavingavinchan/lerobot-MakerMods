#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
from unittest.mock import MagicMock, patch

import pytest

from lerobot.robots.viscous_follower.config_viscous_follower import (
    ViscousFollowerConfig,
    ViscousFollowerConfigBase,
)
from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.rebot_102_leader import (
    RebotArm102Leader,
    RebotArm102LeaderConfig,
    RebotArm102LeaderMakerConfig,
    RebotArm102LeaderViscousConfig,
    RebotArm102LeaderViscousTeleopConfig,
)
from lerobot.teleoperators.utils import make_teleoperator_from_config

_MODULE = "lerobot.teleoperators.rebot_102_leader.rebot_102_leader"

# Star leader travel per joint, degrees. From the maker-arm SDK `capture_star_ranges.py` run of
# 2026-08-29 (purple Star Arm 102), servo 0..6 in joint order.
_STAR_TRAVEL_DEG = {
    "shoulder_pan": 217.4,
    "shoulder_lift": 200.3,
    "elbow_flex": 271.0,
    "wrist_flex": 196.5,
    "wrist_yaw": 139.1,
    "wrist_roll": 322.5,
    "gripper": 65.9,
}


def _make_bus_mock(raw_deg: float) -> MagicMock:
    bus = MagicMock(name="FashionStarServoMock")
    bus.ping.return_value = True

    def _sync_monitor(ids):
        return {servo_id: MagicMock(angle_deg=raw_deg) for servo_id in ids}

    bus.sync_monitor.side_effect = _sync_monitor
    return bus


def test_viscous_ranges_match_follower_limits():
    """The preset may never emit a target the follower would have to clip.

    `joint_ranges` are ints derived from the follower's float `joint_limits` by rounding
    inward, so the check is strict containment.
    """
    ranges = RebotArm102LeaderViscousConfig(port="/dev/null").joint_ranges
    limits = ViscousFollowerConfig(port="/dev/null").joint_limits
    assert set(ranges) == set(limits)
    for joint, (lo, hi) in ranges.items():
        lim_lo, lim_hi = limits[joint]
        assert lim_lo <= lo < hi <= lim_hi, joint


def test_viscous_directions_are_the_captured_travel_ratios():
    """Each factor is `sign * (follower travel / Star travel)` for this arm.

    Magnitude is recomputed from the follower's own captured limits and the Star capture above,
    so drifting either one without recapturing the mapping fails here. Signs are carried from the
    Maker preset -- the two arms share a topology and joint map, and neither is mirrored.
    """
    directions = RebotArm102LeaderViscousConfig(port="/dev/null").joint_directions
    limits = ViscousFollowerConfigBase().joint_limits
    maker = RebotArm102LeaderMakerConfig(port="/dev/null").joint_directions
    assert set(directions) == set(_STAR_TRAVEL_DEG) == set(limits)
    for joint, factor in directions.items():
        lo, hi = limits[joint]
        assert abs(factor) == pytest.approx((hi - lo) / _STAR_TRAVEL_DEG[joint], abs=1e-5), joint
        assert math.copysign(1.0, factor) == math.copysign(1.0, maker[joint]), joint


def test_viscous_preset_shares_hardware_fields_with_parent():
    base = RebotArm102LeaderConfig(port="/dev/null")
    viscous = RebotArm102LeaderViscousConfig(port="/dev/null")
    assert viscous.joint_ids == base.joint_ids
    assert viscous.baudrate == base.baudrate
    assert set(viscous.joint_directions) == set(base.joint_directions)
    assert all(isinstance(d, float) and d != 0.0 for d in viscous.joint_directions.values())


def test_viscous_teleop_config_is_registered():
    assert (
        TeleoperatorConfig.get_choice_class("rebot_102_leader_viscous")
        is RebotArm102LeaderViscousTeleopConfig
    )


def test_factory_builds_the_stock_leader_for_the_viscous_type():
    cfg = RebotArm102LeaderViscousTeleopConfig(port="/dev/null")
    with patch(f"{_MODULE}.require_package", lambda *a, **kw: None):
        teleop = make_teleoperator_from_config(cfg)
    assert isinstance(teleop, RebotArm102Leader)
    assert teleop.config.joint_ranges == cfg.joint_ranges


@pytest.mark.parametrize("raw_deg", [5.0, -5.0])
def test_action_applies_scale_and_stays_inside_range(raw_deg):
    cfg = RebotArm102LeaderViscousTeleopConfig(port="/dev/null")
    with (
        patch(f"{_MODULE}.require_package", lambda *a, **kw: None),
        patch(f"{_MODULE}.FashionStarServo", return_value=_make_bus_mock(raw_deg)),
    ):
        teleop = RebotArm102Leader(cfg)
        teleop.connect(calibrate=False)
        action = teleop.get_action()
        teleop.disconnect()
    for joint, (lo, hi) in cfg.joint_ranges.items():
        value = action[f"{joint}.pos"]
        assert lo <= value <= hi, joint


# --- start-pose check ----------------------------------------------------------------------
# On connect the follower drives to the leader's pose, so a leader left extended would become the
# follower's first motion. The leader refuses to start before the follower ever connects.


def _bus_at(raw_by_joint: dict[str, float]) -> MagicMock:
    """A bus reporting `raw_by_joint` raw degrees, and 5 deg for every joint not listed."""
    names = {
        servo_id: joint for joint, servo_id in RebotArm102LeaderViscousConfig(port="x").joint_ids.items()
    }
    bus = MagicMock(name="FashionStarServoMock")
    bus.ping.return_value = True
    bus.sync_monitor.side_effect = lambda ids: {
        i: MagicMock(angle_deg=raw_by_joint.get(names[i], 5.0)) for i in ids
    }
    return bus


def _connect_at(raw_by_joint: dict[str, float], calibrate: bool = True, bus: MagicMock | None = None):
    cfg = RebotArm102LeaderViscousTeleopConfig(port="/dev/null")
    bus = _bus_at(raw_by_joint) if bus is None else bus
    with (
        patch(f"{_MODULE}.require_package", lambda *a, **kw: None),
        patch(f"{_MODULE}.FashionStarServo", return_value=bus),
    ):
        teleop = RebotArm102Leader(cfg)
        # Count as calibrated so connect(calibrate=True) does not prompt.
        teleop.calibration = dict.fromkeys(teleop.motor_names, object())
        teleop.connect(calibrate=calibrate)
    return teleop, bus


def test_start_offset_defaults():
    assert RebotArm102LeaderConfig(port="/dev/null").max_start_offset_deg is None
    assert RebotArm102LeaderViscousConfig(port="/dev/null").max_start_offset_deg == 30.0


def test_a_folded_leader_starts():
    teleop, _ = _connect_at({})
    assert teleop.is_connected
    teleop.disconnect()


def test_an_extended_joint_refuses_to_start_and_frees_the_bus():
    bus = _bus_at({"wrist_flex": 40.0})
    with pytest.raises(RuntimeError, match="leader is not at its zero pose") as excinfo:
        _connect_at({}, bus=bus)
    message = str(excinfo.value)
    assert "wrist_flex at +40.0 deg" in message and "30 deg" in message
    assert "shoulder_pan" not in message
    assert "--teleop.max_start_offset_deg=null" in message
    # The port is not held after a refused start.
    bus.close.assert_called_once()


def test_every_offending_joint_is_named():
    with pytest.raises(RuntimeError) as excinfo:
        _connect_at({"shoulder_lift": -45.0, "gripper": 90.0})
    assert "shoulder_lift at -45.0 deg" in str(excinfo.value)
    assert "gripper at +90.0 deg" in str(excinfo.value)


def test_a_wrapped_reading_counts_from_zero():
    # 358 deg is -2 deg from zero, not 358.
    teleop, _ = _connect_at({"wrist_roll": 358.0})
    assert teleop.is_connected
    teleop.disconnect()


def test_calibration_connect_skips_the_check():
    # `lerobot-calibrate` connects with calibrate=False, which is when the arm may be away from zero.
    teleop, _ = _connect_at(dict.fromkeys(("shoulder_pan", "elbow_flex"), 90.0), calibrate=False)
    assert teleop.is_connected
    teleop.disconnect()
