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

from unittest.mock import MagicMock, patch

import pytest

from lerobot.robots.metal_follower.config_metal_follower import MetalFollowerConfig
from lerobot.teleoperators.bi_rebot_102_leader import BiRebot102Leader, BiRebot102LeaderConfig
from lerobot.teleoperators.rebot_102_leader import (
    RebotArm102Leader,
    RebotArm102LeaderConfig,
    RebotArm102LeaderMakerTeleopConfig,
    RebotArm102LeaderMetalConfig,
    RebotArm102LeaderMetalTeleopConfig,
    RebotArm102LeaderTeleopConfig,
    RebotArm102LeaderViscousTeleopConfig,
)
from lerobot.teleoperators.utils import make_teleoperator_from_config

_MODULE = "lerobot.teleoperators.rebot_102_leader.rebot_102_leader"


# Mutable button state shared with the bus mock, so a test can press the button mid-loop.
# "angle" is what `read_angle` reports; None makes the read raise, like a silent board.
def _make_bus_mock(joint_ids: dict[str, int], button: dict | None = None) -> MagicMock:
    bus = MagicMock(name="FashionStarServoMock")
    bus.ping.return_value = True

    def _sync_monitor(ids):
        # Report each servo at 5 degrees raw.
        monitors = {}
        for servo_id in ids:
            monitor = MagicMock()
            monitor.angle_deg = 5.0
            monitors[servo_id] = monitor
        return monitors

    def _read_angle(servo_id, multi_turn=True):
        angle = None if button is None else button.get("angle")
        if angle is None:
            raise RuntimeError(f"servo {servo_id} lost 20 consecutive responses")
        sample = MagicMock()
        sample.raw_deg = angle
        return sample

    bus.sync_monitor.side_effect = _sync_monitor
    bus.read_angle.side_effect = _read_angle
    return bus


def _connect(cfg, bus_mock) -> RebotArm102Leader:
    with (
        patch(f"{_MODULE}.require_package", lambda *a, **kw: None),
        patch(f"{_MODULE}.FashionStarServo", return_value=bus_mock),
    ):
        teleop = RebotArm102Leader(cfg)
        teleop.connect(calibrate=False)
    return teleop


def _stop_mode_calls(bus_mock, mode: int) -> list[int]:
    """Servo ids that got `set_stop_mode` with `mode`, in call order."""
    return [call.args[0] for call in bus_mock.set_stop_mode.call_args_list if call.kwargs.get("mode") == mode]


@pytest.fixture
def leader():
    cfg = RebotArm102LeaderTeleopConfig(port="/dev/null")
    bus_mock = _make_bus_mock(cfg.joint_ids)
    with (
        patch(f"{_MODULE}.require_package", lambda *a, **kw: None),
        patch(f"{_MODULE}.FashionStarServo", return_value=bus_mock),
    ):
        teleop = RebotArm102Leader(cfg)
        teleop.connect(calibrate=False)
        yield teleop
        if teleop.is_connected:
            teleop.disconnect()


def test_action_features_match_joints():
    with patch(f"{_MODULE}.require_package", lambda *a, **kw: None):
        teleop = RebotArm102Leader(RebotArm102LeaderTeleopConfig(port="/dev/null"))
    assert set(teleop.action_features) == {f"{m}.pos" for m in teleop.motor_names}
    assert teleop.feedback_features == {}


def test_connect_disconnect(leader):
    assert leader.is_connected
    leader.disconnect()
    assert not leader.is_connected


def test_get_action_applies_direction_and_clamp(leader):
    action = leader.get_action()
    assert set(action) == {f"{m}.pos" for m in leader.motor_names}
    # shoulder_pan has direction -1, so a +5deg raw reading flips to -5deg.
    assert action["shoulder_pan.pos"] == pytest.approx(-5.0)
    # Every joint stays within its configured range.
    for motor, value in action.items():
        lo, hi = leader.config.joint_ranges[motor.removesuffix(".pos")]
        assert lo <= value <= hi


def test_get_action_applies_joint_offsets(leader):
    # Offsets are additive after the direction scale and before the clamp; unlisted joints get 0.
    leader.config.joint_offsets = {"shoulder_pan": -20.0}
    action = leader.get_action()
    # raw +5deg * direction -1 = -5deg, then -20deg offset.
    assert action["shoulder_pan.pos"] == pytest.approx(-25.0)
    lo, hi = leader.config.joint_ranges["shoulder_pan"]
    assert lo <= action["shoulder_pan.pos"] <= hi
    # An offset past the range still clamps.
    leader.config.joint_offsets = {"shoulder_pan": -10_000.0}
    assert leader.get_action()["shoulder_pan.pos"] == pytest.approx(lo)


def test_send_feedback_not_implemented(leader):
    with pytest.raises(NotImplementedError):
        leader.send_feedback({})


def test_metal_ranges_match_follower_limits():
    """The Metal preset's joint_ranges must stay identical to the follower's soft limits.

    This is the invariant the preset exists to hold: the leader must not be able to emit a
    target outside the follower's mechanical envelope. If MetalFollowerConfig.joint_limits
    changes, this fails and the preset has to be updated with it.
    """
    leader_ranges = RebotArm102LeaderMetalConfig(port="/dev/null").joint_ranges
    follower_limits = MetalFollowerConfig().joint_limits

    assert set(leader_ranges) == set(follower_limits)
    for joint, (lo, hi) in follower_limits.items():
        if joint == "gripper":
            continue
        assert leader_ranges[joint] == [lo, hi], f"{joint} drifted from the follower's limits"


def test_metal_gripper_range_is_deliberately_narrower():
    """The gripper is the one joint that intentionally stops short of the follower's limit.

    The follower allows the vendor's 0..137.5 deg, but its stroke table only documents jaw
    opening up to 116.4 deg, so the leader stops at 115 rather than commanding into the
    undocumented stretch.
    """
    leader_lo, leader_hi = RebotArm102LeaderMetalConfig(port="/dev/null").joint_ranges["gripper"]
    follower_lo, follower_hi = MetalFollowerConfig().joint_limits["gripper"]

    assert leader_lo == follower_lo
    assert leader_hi < follower_hi
    assert leader_hi <= 116.4


def test_metal_preset_differs_from_b601_where_expected():
    """Guard the three joints the Metal mapping deliberately flips or rescales.

    Without these, elbow_flex and gripper saturate against the follower's limits and stop
    moving entirely, while wrist_flex tracks backwards.
    """
    b601 = RebotArm102LeaderConfig(port="/dev/null")
    metal = RebotArm102LeaderMetalConfig(port="/dev/null")

    assert metal.joint_ids == b601.joint_ids, "same servo bus layout, ids must not diverge"

    differing = {
        joint
        for joint, direction in metal.joint_directions.items()
        if direction != b601.joint_directions[joint]
    }
    assert differing == {"elbow_flex", "wrist_flex", "gripper"}


def test_metal_preset_is_registered_and_builds_the_shared_driver():
    cfg = RebotArm102LeaderMetalTeleopConfig(port="/dev/null")
    assert cfg.type == "rebot_102_leader_metal"

    with patch(f"{_MODULE}.require_package", lambda *a, **kw: None):
        teleop = make_teleoperator_from_config(cfg)
    assert isinstance(teleop, RebotArm102Leader)
    assert teleop.config.joint_ranges["gripper"] == [0, 115]


def test_bimanual_prefixes_features():
    with patch(f"{_MODULE}.require_package", lambda *a, **kw: None):
        cfg = BiRebot102LeaderConfig(
            left_arm_config=RebotArm102LeaderConfig(port="/dev/null0"),
            right_arm_config=RebotArm102LeaderConfig(port="/dev/null1"),
        )
        teleop = BiRebot102Leader(cfg)
    assert any(k.startswith("left_") for k in teleop.action_features)
    assert any(k.startswith("right_") for k in teleop.action_features)
    assert "left_gripper.pos" in teleop.action_features
    assert "right_gripper.pos" in teleop.action_features


# --- lock button -------------------------------------------------------------------------


@pytest.fixture
def button_state() -> dict:
    return {"angle": 0.0}


@pytest.fixture
def button_leader(button_state):
    """A leader with the bus lock button enabled, starting unlocked."""
    cfg = RebotArm102LeaderTeleopConfig(port="/dev/null")
    cfg.button.enabled = True
    bus_mock = _make_bus_mock(cfg.joint_ids, button_state)
    teleop = _connect(cfg, bus_mock)
    yield teleop
    if teleop.is_connected:
        teleop.disconnect()


def test_button_disabled_by_default_is_inert(leader):
    """The default config must not touch stop modes or poll an extra id at all."""
    assert leader.config.button.enabled is False
    assert leader.is_locked is False
    leader.get_action()
    # configure() unlocks through `bus.unlock`, so nothing else may reach set_stop_mode.
    leader.bus.set_stop_mode.assert_not_called()
    leader.bus.read_angle.assert_not_called()
    # The button id is never folded into the joints' sync read: an id that does not answer
    # fails the whole packet, taking the joint positions down with it.
    for call in leader.bus.sync_monitor.call_args_list:
        assert 7 not in call.args[0]


def test_presets_inherit_button_defaults():
    for cfg in (
        RebotArm102LeaderMakerTeleopConfig(port="/dev/null"),
        RebotArm102LeaderViscousTeleopConfig(port="/dev/null"),
    ):
        assert cfg.button.enabled is False
        assert cfg.button.id == 7
        assert cfg.button.excluded_lock_joints == []


def test_connect_fails_without_a_button_board():
    cfg = RebotArm102LeaderTeleopConfig(port="/dev/null")
    cfg.button.enabled = True
    bus_mock = _make_bus_mock(cfg.joint_ids, {"angle": 0.0})
    bus_mock.ping.side_effect = lambda servo_id: servo_id != cfg.button.id
    with pytest.raises(RuntimeError, match="Lock button not found"):
        _connect(cfg, bus_mock)
    bus_mock.close.assert_called_once()


def test_button_press_holds_every_joint_once_and_freezes_the_action(button_leader, button_state):
    free_action = button_leader.get_action()
    assert button_leader.is_locked is False

    button_state["angle"] = 180.0
    locked_action = button_leader.get_action()

    assert button_leader.is_locked is True
    held = _stop_mode_calls(button_leader.bus, 0x11)
    assert held == list(button_leader.config.joint_ids.values())
    for call in button_leader.bus.set_stop_mode.call_args_list:
        assert call.kwargs["power"] == button_leader.config.button.hold_power_mw
    # Frozen: the follower keeps being sent the last action from before the lock.
    assert locked_action == free_action

    # Held, not re-sent: the lock is edge-triggered.
    button_leader.get_action()
    assert _stop_mode_calls(button_leader.bus, 0x11) == held


def test_button_release_frees_every_joint(button_leader, button_state):
    button_state["angle"] = 180.0
    button_leader.get_action()
    button_state["angle"] = 0.0
    button_leader.get_action()

    assert button_leader.is_locked is False
    assert _stop_mode_calls(button_leader.bus, 0x10) == list(button_leader.config.joint_ids.values())
    # Live again once released.
    assert button_leader.get_action()["shoulder_pan.pos"] == pytest.approx(-5.0)


def test_excluded_joints_are_never_locked(button_state):
    cfg = RebotArm102LeaderTeleopConfig(port="/dev/null")
    cfg.button.enabled = True
    cfg.button.excluded_lock_joints = ["gripper"]
    bus_mock = _make_bus_mock(cfg.joint_ids, button_state)
    teleop = _connect(cfg, bus_mock)
    try:
        teleop.get_action()
        button_state["angle"] = 180.0
        action = teleop.get_action()

        gripper_id = cfg.joint_ids["gripper"]
        assert gripper_id not in _stop_mode_calls(bus_mock, 0x11)
        assert len(_stop_mode_calls(bus_mock, 0x11)) == len(cfg.joint_ids) - 1
        # An unlocked joint still moves, so its action is not frozen.
        assert "gripper.pos" in action
    finally:
        teleop.disconnect()


def test_silent_button_does_not_raise_and_keeps_the_state(button_leader, button_state):
    button_state["angle"] = 180.0
    button_leader.get_action()
    assert button_leader.is_locked is True

    button_state["angle"] = None  # the board stops answering
    for _ in range(3):
        action = button_leader.get_action()
    assert set(action) == {f"{m}.pos" for m in button_leader.motor_names}
    assert button_leader.is_locked is True


def test_button_polling_stops_after_repeated_failures(button_state):
    cfg = RebotArm102LeaderTeleopConfig(port="/dev/null")
    cfg.button.enabled = True
    cfg.button.max_consecutive_read_errors = 3
    button_state["angle"] = None
    bus_mock = _make_bus_mock(cfg.joint_ids, button_state)
    teleop = _connect(cfg, bus_mock)
    try:
        for _ in range(6):
            teleop.get_action()
        # Polling gives up rather than paying a bus timeout on every cycle forever.
        assert bus_mock.read_angle.call_count == 3
    finally:
        teleop.disconnect()


def test_disconnect_releases_every_joint_even_when_locked(button_leader, button_state):
    button_state["angle"] = 180.0
    button_leader.get_action()
    assert button_leader.is_locked is True

    bus = button_leader.bus  # disconnect() drops the reference
    button_leader.disconnect()
    assert button_leader.is_locked is False
    # Every joint, not only the lockable ones, and a plain torque-off.
    assert _stop_mode_calls(bus, 0x10) == list(button_leader.config.joint_ids.values())
    bus.close.assert_called_once()


def test_disconnect_releases_even_if_the_bus_is_gone(button_leader):
    button_leader.bus.set_stop_mode.side_effect = RuntimeError("USB gone")
    button_leader.disconnect()
    assert not button_leader.is_connected


def test_damping_release_mode_uses_the_damping_stop_mode(button_state):
    cfg = RebotArm102LeaderTeleopConfig(port="/dev/null")
    cfg.button.enabled = True
    cfg.button.release_mode = "damping"
    bus_mock = _make_bus_mock(cfg.joint_ids, button_state)
    teleop = _connect(cfg, bus_mock)
    try:
        button_state["angle"] = 180.0
        teleop.get_action()
        button_state["angle"] = 0.0
        teleop.get_action()

        assert _stop_mode_calls(bus_mock, 0x12) == list(cfg.joint_ids.values())
        for call in bus_mock.set_stop_mode.call_args_list:
            if call.kwargs["mode"] == 0x12:
                assert call.kwargs["power"] == cfg.button.damping_power_mw
    finally:
        teleop.disconnect()


def test_lock_on_connect(button_state):
    cfg = RebotArm102LeaderTeleopConfig(port="/dev/null")
    cfg.button.enabled = True
    cfg.button.lock_on_connect = True
    bus_mock = _make_bus_mock(cfg.joint_ids, button_state)
    teleop = _connect(cfg, bus_mock)
    try:
        assert teleop.is_locked is True
        assert _stop_mode_calls(bus_mock, 0x11) == list(cfg.joint_ids.values())
    finally:
        teleop.disconnect()


def test_keyboard_style_manual_lock_and_release(leader):
    """The standalone script's path: lock()/release() called directly, button disabled."""
    leader.lock()
    assert leader.is_locked is True
    assert _stop_mode_calls(leader.bus, 0x11) == list(leader.config.joint_ids.values())
    leader.release()
    assert leader.is_locked is False
    assert _stop_mode_calls(leader.bus, 0x10) == list(leader.config.joint_ids.values())


@pytest.mark.parametrize(
    ("field_name", "value", "match"),
    [
        ("excluded_lock_joints", ["nope"], "unknown joints"),
        ("release_mode", "hold", "release_mode"),
        ("id", 3, "collides"),
    ],
)
def test_invalid_button_config_is_rejected(field_name, value, match):
    cfg = RebotArm102LeaderTeleopConfig(port="/dev/null")
    cfg.button.enabled = True
    setattr(cfg.button, field_name, value)
    with (
        patch(f"{_MODULE}.require_package", lambda *a, **kw: None),
        pytest.raises(ValueError, match=match),
    ):
        RebotArm102Leader(cfg)
