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

import logging
import time
from typing import TYPE_CHECKING

from lerobot.lerobot_types import RobotAction
from lerobot.motors import MotorCalibration
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from lerobot.utils.import_utils import _motorbridge_smart_servo_available, require_package

from ..teleoperator import Teleoperator
from .config_rebot_102_leader import RebotArm102LeaderTeleopConfig

if TYPE_CHECKING or _motorbridge_smart_servo_available:
    from motorbridge_smart_servo import FashionStarServo, ServoMonitor
else:
    FashionStarServo = None
    ServoMonitor = None

logger = logging.getLogger(__name__)

_SETTLE_SEC = 0.01

# Servo stop-on-control modes (UART protocol command 24).
_STOP_RELEASE = 0x10  # torque off: the joint is free and backdrivable
_STOP_HOLD = 0x11  # hold the current position
_STOP_DAMPING = 0x12  # free but resisting, at the command's power cap

# Seconds between two "the button did not answer" warnings, so a silent button board cannot
# flood a 60 Hz teleop log.
_BUTTON_WARN_PERIOD_SEC = 5.0


class RebotArm102Leader(Teleoperator):
    """Seeed Studio StarArm102 / reBot Arm 102 leader arm.

    A 7-joint (incl. gripper) leader built on FashionStar UART smart servos. Servo
    communication is handled by the ``motorbridge-smart-servo`` package; this class
    only reads joint angles, so it produces actions but accepts no feedback.
    """

    config_class = RebotArm102LeaderTeleopConfig
    name = "rebot_102_leader"

    def __init__(self, config: RebotArm102LeaderTeleopConfig):
        require_package("motorbridge-smart-servo", extra="rebot", import_name="motorbridge_smart_servo")
        super().__init__(config)
        self.config = config
        self.bus: FashionStarServo | None = None
        self.motor_names = list(config.joint_ids.keys())
        self._last_raw_positions: dict[str, float] = {}

        button = config.button
        unknown = sorted(set(button.excluded_lock_joints) - set(self.motor_names))
        if unknown:
            raise ValueError(
                f"button.excluded_lock_joints contains unknown joints {unknown}; "
                f"known joints: {sorted(self.motor_names)}."
            )
        if button.release_mode not in ("release", "damping"):
            raise ValueError(
                f"button.release_mode must be 'release' or 'damping', got {button.release_mode!r}."
            )
        if button.enabled and button.id in config.joint_ids.values():
            raise ValueError(
                f"button.id={button.id} collides with a joint servo id; the button board is a "
                "separate device on the bus."
            )

        self._is_locked = False
        # Action last emitted while the arm was free, replayed for the locked joints while the
        # lock holds so a follower freezes with the leader.
        self._last_free_action: RobotAction = {}
        self._button_read_errors = 0
        self._button_polling = True
        self._button_warned_at = 0.0

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.motor_names}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus is not None

    @property
    def is_locked(self) -> bool:
        """True while the lock button is holding the arm's pose (stop mode 0x11)."""
        return self._is_locked

    @property
    def lockable_motor_names(self) -> list[str]:
        """Joints the lock applies to: everything except `button.excluded_lock_joints`."""
        excluded = set(self.config.button.excluded_lock_joints)
        return [name for name in self.motor_names if name not in excluded]

    def _write_stop_mode(self, motor_names: list[str], mode: int, power: int) -> None:
        for motor_name in motor_names:
            self.bus.set_stop_mode(self.config.joint_ids[motor_name], mode=mode, power=power)
            time.sleep(_SETTLE_SEC)

    @check_if_not_connected
    def lock(self) -> None:
        """Hold every lockable joint at its current pose (stop mode 0x11).

        Idempotent from the caller's point of view but not a no-op: the command is re-sent
        every call, so a servo that has been power-cycled or released behind our back is
        brought back under the lock.
        """
        self._write_stop_mode(self.lockable_motor_names, _STOP_HOLD, self.config.button.hold_power_mw)
        if not self._is_locked:
            logger.info(
                f"{self} locked: holding {len(self.lockable_motor_names)} joints "
                f"(power {self.config.button.hold_power_mw} mW)."
            )
        self._is_locked = True

    @check_if_not_connected
    def release(self) -> None:
        """Free every lockable joint again, per `button.release_mode`.

        "release" is torque off (0x10), "damping" leaves the joint free but resisting at
        `button.damping_power_mw` (0x12).
        """
        if self.config.button.release_mode == "damping":
            mode, power = _STOP_DAMPING, self.config.button.damping_power_mw
        else:
            mode, power = _STOP_RELEASE, 0
        self._write_stop_mode(self.lockable_motor_names, mode, power)
        if self._is_locked:
            logger.info(f"{self} released ({self.config.button.release_mode}).")
        self._is_locked = False

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        logger.info(f"Connecting {self} on {self.config.port}...")
        bus = FashionStarServo(self.config.port, baudrate=self.config.baudrate)
        try:
            for motor_name, motor_id in self.config.joint_ids.items():
                if not bus.ping(motor_id):
                    raise RuntimeError(f"Servo not found for {motor_name} (id={motor_id}).")
                self._last_raw_positions[motor_name] = 0.0

            if self.config.button.enabled and not bus.ping(self.config.button.id):
                raise RuntimeError(
                    f"Lock button not found at id={self.config.button.id}. Only the 102-HD "
                    "(purple) arm has a button board, and it must be wired to the servo bus in "
                    "slave mode. Pass --teleop.button.enabled=false to run without it."
                )
            self.bus = bus

            if not self.is_calibrated and calibrate:
                logger.info(
                    "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
                )
                self.calibrate()

            self.configure()

            if self.config.button.enabled and self.config.button.lock_on_connect:
                self.lock()
        except Exception:
            bus.close()
            self.bus = None
            raise

        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return bool(self.calibration) and set(self.calibration) == set(self.motor_names)

    def calibrate(self) -> None:
        if self.calibration:
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, "
                "or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Using calibration file associated with the id {self.id}")
                return

        logger.info(f"\nRunning calibration of {self}")
        input(
            "\nCalibration: set zero position.\n"
            "Manually move the reBot Arm 102 to its zero pose and close the gripper.\n"
            "Press ENTER when ready..."
        )

        self.calibration = {}
        for motor_name, motor_id in self.config.joint_ids.items():
            self.bus.unlock(motor_id)
            time.sleep(_SETTLE_SEC)
            self.bus.set_origin_point(motor_id)
            range_min, range_max = self.config.joint_ranges[motor_name]
            self.calibration[motor_name] = MotorCalibration(
                id=motor_id,
                drive_mode=0,
                homing_offset=0,
                range_min=int(range_min),
                range_max=int(range_max),
            )

        self._save_calibration()
        logger.info(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        # Every joint starts free, whatever stop mode the servos were left in by a previous
        # session, so `_is_locked` is authoritative again from here on.
        for motor_id in self.config.joint_ids.values():
            self.bus.unlock(motor_id)
            time.sleep(_SETTLE_SEC)
        self._is_locked = False
        # Reset the multi-turn counter of each servo individually.
        for motor_id in self.config.joint_ids.values():
            self.bus.reset_multi_turn(motor_id)

    def _read_raw_positions(self) -> dict[str, float]:
        result: dict[int, ServoMonitor | None] = self.bus.sync_monitor(list(self.config.joint_ids.values()))
        id_to_name = {v: k for k, v in self.config.joint_ids.items()}
        raw_positions: dict[str, float] = {}
        for motor_id, monitor in result.items():
            motor_name = id_to_name[motor_id]
            if monitor is None:
                raise RuntimeError(f"Servo {motor_name} (id={motor_id}) has never responded.")
            raw_positions[motor_name] = monitor.angle_deg
        return raw_positions

    @staticmethod
    def _round_to_valid_range(value: float, min_value: float, max_value: float) -> tuple[float, int]:
        """Unwrap a multi-turn angle into the ±180° window centred on (min+max)/2.

        The servo may report an angle that has accumulated extra full rotations
        (value = true_angle + N*360). Subtract the nearest whole number of turns
        to bring it back into [center-180, center+180]. Returns the unwrapped
        angle and the number of turns removed.
        """
        center = (min_value + max_value) / 2.0
        turns = round((value - center) / 360.0)
        return value - turns * 360.0, abs(turns)

    def _read_button_angle(self) -> float | None:
        """Return the button board's angle in degrees, or None when it did not answer.

        Read on its own, never folded into the joints' `sync_monitor` call: an id that does
        not answer trips the package's consecutive-loss check and fails the *whole* sync read,
        so a missing or half-wired button board would take the joint positions down with it
        (observed on the bench: `ServoBusError: servo 7 lost 20 consecutive responses`, and no
        data for ids 0-6 either).
        """
        if not self._button_polling:
            return None
        button = self.config.button
        try:
            sample = self.bus.read_angle(button.id, multi_turn=False)
            # `raw_deg` is the protocol value; `filtered_deg` suppresses A->0->B glitches,
            # which is exactly the 180->0 edge this button is for.
            angle = getattr(sample, "raw_deg", None)
            if angle is None:
                angle = getattr(sample, "angle_deg", None)
            if angle is None:
                raise RuntimeError("no angle in the button's reply")
        except Exception as e:
            self._button_read_errors += 1
            now = time.perf_counter()
            if now - self._button_warned_at > _BUTTON_WARN_PERIOD_SEC:
                self._button_warned_at = now
                logger.warning(
                    f"Lock button (id={button.id}) did not answer "
                    f"({self._button_read_errors} consecutive): {e}. "
                    f"Keeping the arm {'locked' if self._is_locked else 'released'}."
                )
            if (
                button.max_consecutive_read_errors
                and self._button_read_errors >= button.max_consecutive_read_errors
            ):
                self._button_polling = False
                logger.error(
                    f"Lock button (id={button.id}) has been silent for "
                    f"{self._button_read_errors} reads; polling it every cycle costs a full bus "
                    "timeout, so the button is now ignored for the rest of this session. The arm "
                    f"stays {'locked' if self._is_locked else 'released'}. Check the board's "
                    "wiring, then reconnect."
                )
            return None
        self._button_read_errors = 0
        return float(angle)

    def _poll_button(self) -> None:
        """Read the button and apply the lock on a state change only (edge-triggered)."""
        angle = self._read_button_angle()
        if angle is None:
            return
        locked = angle >= self.config.button.trigger_threshold
        if locked == self._is_locked:
            return
        logger.info(
            f"Lock button (id={self.config.button.id}) at {angle:.1f} deg: "
            f"{'locking' if locked else 'releasing'} the leader."
        )
        if locked:
            self.lock()
        else:
            self.release()

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        start = time.perf_counter()
        try:
            raw_positions = self._read_raw_positions()
            self._last_raw_positions = raw_positions
        except Exception as e:
            logger.error(f"Failed to read raw positions: {e}")
            logger.warning("[EMERGENCY STOP] Hold the follower arm and cut off the main power to the arms.")
            logger.warning(
                "[EMERGENCY STOP] Break the teleoperation session and check the leader USB connection or power."
            )
            raw_positions = self._last_raw_positions

        action_dict: dict[str, float] = {}
        for motor_name in self.motor_names:
            range_min, range_max = self.config.joint_ranges[motor_name]
            direction = self.config.joint_directions[motor_name]
            sign = 1.0 if direction >= 0 else -1.0
            unwrapped, k = self._round_to_valid_range(
                raw_positions[motor_name], range_min * sign, range_max * sign
            )
            position = unwrapped * direction + self.config.joint_offsets.get(motor_name, 0.0)
            if k > 0:
                logger.debug(
                    f"Servo {motor_name} (id={self.config.joint_ids[motor_name]}) wrapped {k} * 360°. "
                    f"Unwrapped pos: {unwrapped:.1f}° (raw: {raw_positions[motor_name]:.1f}°)"
                )
            action_dict[f"{motor_name}.pos"] = max(float(range_min), min(float(range_max), position))

        action: RobotAction = action_dict
        if self.config.button.enabled:
            if not self._last_free_action:
                self._last_free_action = dict(action)
            self._poll_button()
            if self._is_locked and self.config.button.freeze_action_on_lock:
                # Only the joints actually held are frozen; an excluded joint is still free,
                # so freezing it would stop the operator moving it.
                action = dict(action)
                for motor_name in self.lockable_motor_names:
                    key = f"{motor_name}.pos"
                    action[key] = self._last_free_action[key]
            else:
                self._last_free_action = dict(action)
        else:
            self._last_free_action = dict(action)

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError("Feedback is not implemented for the reBot Arm 102 leader.")

    @check_if_not_connected
    def disconnect(self) -> None:
        # Always leave the arm free, whatever the lock state was: a leader left holding after
        # the port closes keeps drawing current with nothing watching it, and nothing could
        # release it short of a power cycle. This is a plain torque-off (0x10) even under
        # `release_mode="damping"`, for the same reason -- damping keeps the servos energised.
        try:
            for motor_id in self.config.joint_ids.values():
                self.bus.set_stop_mode(motor_id, mode=_STOP_RELEASE, power=0)
        except Exception:
            logger.warning(
                f"{self} failed to release the joints before disconnecting; they may still be "
                "holding. Power-cycle the leader if it stays stiff.",
                exc_info=True,
            )
        finally:
            self._is_locked = False

        # Drop the bus even if closing it fails: after a USB dropout `close()` raises, which
        # would leave `self.bus` set, keep `is_connected` True, and make the destructor try to
        # disconnect a second time during interpreter shutdown.
        try:
            self.bus.close()
        except Exception:
            logger.warning(f"{self} failed to close the serial port cleanly.", exc_info=True)
        finally:
            self.bus = None
        logger.info(f"{self} disconnected.")
