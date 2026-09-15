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
from functools import cached_property
from typing import Any

from lerobot.cameras import make_cameras_from_configs
from lerobot.lerobot_types import RobotAction, RobotObservation
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.robstride import RobstrideMotorsBus
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_maker_follower import MakerFollowerConfig

logger = logging.getLogger(__name__)

# RobStride model per joint, as named in `motors/robstride/tables.py` (MIT-protocol parameter
# ranges). RS00 maps to "O0" (+-12.57 rad, +-33 rad/s, +-14 Nm) and RS02 to "O1" (+-44 rad/s,
# +-17 Nm). Fixed by the hardware build: shoulder_lift and elbow_flex carry the RS02 motors.
# Exposed as `MakerFollower.motor_models` so a same-topology arm with a different motor fit
# (see `viscous_follower`, all RS00) only has to override that class attribute.
MOTOR_MODELS = {
    "shoulder_pan": "O0",
    "shoulder_lift": "O1",
    "elbow_flex": "O1",
    "wrist_flex": "O0",
    "wrist_yaw": "O0",
    "wrist_roll": "O0",
    "gripper": "O0",
}

# Startup-sync stall release: a joint counts as making progress when it moved more than
# _SYNC_PROGRESS_EPS_DEG since its last progress mark; after _SYNC_STALL_RELEASE_SEC without
# progress it is released from the slow sync (it physically cannot close the gap at the capped
# step size, e.g. parked past a soft limit or stiction above kp * step).
_SYNC_PROGRESS_EPS_DEG = 0.2
_SYNC_STALL_RELEASE_SEC = 1.0

# A RobStride motor can come back from a power cycle reporting its angle a whole turn off (the
# multi-turn count resets; seen on this arm's shoulder_lift). Joint travel is under one turn, so
# a reading that lands inside the soft limits only after a +-360 deg shift is unambiguous. The
# grace widens the test so a joint parked just past a limit is not mistaken for a wrapped one.
_FULL_TURN_DEG = 360.0
_WRAP_GRACE_DEG = 20.0

# Thermal monitoring cadences. The over-temperature WARNING repeats at most this often per motor,
# so a hot joint stays visible in a 60 Hz log without drowning it; the health line prints every
# joint's temperature on a slower beat so a session log carries a thermal trace even when nothing
# crosses a threshold.
_TEMP_WARN_INTERVAL_S = 10.0
_TEMP_CRITICAL_INTERVAL_S = 2.0
_SILENCE_WARN_INTERVAL_S = 2.0
_HEALTH_LOG_INTERVAL_S = 30.0

# Consecutive observation cycles a joint must read at or over `temp_stop_c` before the thermal stop
# fires. The stop drops the arm, so one corrupted feedback frame must not be enough.
_TEMP_STOP_CYCLES = 3


class MakerFollower(Robot):
    """
    Maker Arm v1 follower: 6 joints + a permanent gripper, all RobStride motors over classic CAN
    via the stock `RobstrideMotorsBus` in MIT position control.

    All 7 motors are normalized in degrees. The gripper is passed through in raw motor degrees,
    same as the joints.
    """

    config_class = MakerFollowerConfig
    name = "maker_follower"
    # RobStride model string per joint; overridden by same-topology arms with a different fit.
    motor_models: dict[str, str] = MOTOR_MODELS
    # Used in operator-facing calibration prompts.
    display_name = "Maker"

    def __init__(self, config: MakerFollowerConfig):
        super().__init__(config)
        self.config = config

        if not config.port:
            raise ValueError(
                f"{self.name} requires `port`. With can_interface='slcan' (the default) it is "
                "the USB-CAN adapter's serial port, '/dev/ttyACM0' on Linux, "
                "'/dev/cu.usbmodem1101' on macOS, 'COM5' on Windows. With "
                "can_interface='socketcan' it is the interface name, e.g. 'can0'."
            )

        if config.can_interface not in ("socketcan", "slcan"):
            raise ValueError(
                f"{self.name} supports can_interface='socketcan' or 'slcan', got "
                f"'{config.can_interface}'. On Linux, bring the USB-CAN adapter up as a socketcan "
                "interface (`sudo slcand -o -f -s8 /dev/ttyACM0 can0 && sudo ip link set up can0`) "
                "and set port='can0'. On macOS/Windows, where SocketCAN does not exist, use "
                "can_interface='slcan' with the adapter's serial port as `port`."
            )

        if config.temp_stop_c is not None and config.temp_stop_c <= config.temp_critical_c:
            raise ValueError(
                f"{self.name} temp_stop_c ({config.temp_stop_c} C) must be above temp_critical_c "
                f"({config.temp_critical_c} C): the stop drops the arm, so the operator needs the "
                "critical warning first."
            )

        # RobStride MIT feedback carries the motor id in payload byte 0, so recv_id == id.
        motors: dict[str, Motor] = {}
        for motor_name, can_id in config.motor_can_ids.items():
            motor_type_str = self.motor_models[motor_name]
            motor = Motor(can_id, motor_type_str, MotorNormMode.DEGREES)
            motor.recv_id = can_id
            motor.motor_type_str = motor_type_str
            motors[motor_name] = motor

        self._joint_motor_names = list(motors)

        self.bus = RobstrideMotorsBus(
            port=self.config.port,
            motors=motors,
            calibration=self.calibration,
            can_interface=self.config.can_interface,
            use_can_fd=False,
            bitrate=self.config.can_bitrate,
            data_bitrate=None,
        )

        self.cameras = make_cameras_from_configs(config.cameras)

        # False until the follower has caught up to the leader (slow initial sync), then full speed.
        self._synced = False
        self._synced_motors: set[str] = set()
        self._sync_progress: dict[str, tuple[float, float]] = {}
        self._resolved_gains: dict[str, tuple[float, float]] = {}
        # Per-session +-360 deg correction per joint, see `_detect_full_turn_offsets`.
        self._turn_offset: dict[str, float] = dict.fromkeys(self._joint_motor_names, 0.0)
        # Set when a joint sits far outside its limits and no whole-turn shift explains it. The
        # arm then refuses to move until it is recalibrated; connect() itself still succeeds so
        # `lerobot-calibrate` (which connects first) can perform that recalibration.
        self._stale_zero: dict[str, float] = {}
        # Last good reading per joint, used when a motor misses a reply.
        self._last_positions: dict[str, float] = {}
        # Winding temperature (deg C) and torque (Nm) from the same feedback frame as the
        # position, see `_read_raw_positions`.
        self._last_temperatures: dict[str, float] = {}
        self._last_torques: dict[str, float] = {}
        # Consecutive read cycles in which a motor answered nothing at all.
        self._silent_cycles: dict[str, int] = dict.fromkeys(self._joint_motor_names, 0)
        # Consecutive observation cycles a motor has read at or over `temp_stop_c`.
        self._hot_cycles: dict[str, int] = dict.fromkeys(self._joint_motor_names, 0)
        # Rate-limit marks for the thermal logs.
        self._last_temp_warn_t: dict[str, float] = {}
        self._last_silence_warn_t: float | None = None
        self._last_health_log_t: float | None = None

    def _detect_full_turn_offsets(self) -> None:
        """Pick a +-360 deg correction for joints that report a whole turn outside their limits.

        A joint far outside its limits that no single-turn shift explains has a stale zero.
        Driving the arm from there would be unsafe, so it is recorded in `_stale_zero` and
        `send_action` refuses until `calibrate()` runs.
        """
        raw = self._read_raw_positions()
        self._stale_zero = {}
        for motor, position in raw.items():
            self._turn_offset[motor] = 0.0
            if motor not in self.config.joint_limits:
                continue
            low, high = self.config.joint_limits[motor]
            low, high = low - _WRAP_GRACE_DEG, high + _WRAP_GRACE_DEG
            if low <= position <= high:
                continue
            for shift in (-_FULL_TURN_DEG, _FULL_TURN_DEG):
                if low <= position + shift <= high:
                    self._turn_offset[motor] = shift
                    logger.warning(
                        f"{self} {motor} reads {position:+.1f} deg, a full turn outside its limits "
                        f"{self.config.joint_limits[motor]}; correcting by {shift:+.0f} deg for this session."
                    )
                    break
            else:
                self._stale_zero[motor] = position
                logger.error(
                    f"{self} {motor} reads {position:+.1f} deg, outside its soft limits "
                    f"{self.config.joint_limits[motor]} by more than {_WRAP_GRACE_DEG:.0f} deg and "
                    "not by a whole turn. The motor zero no longer matches the calibration pose; "
                    "the arm will not move until it is recalibrated."
                )

    def _read_raw_positions(self) -> dict[str, float]:
        """Read every joint in motor coordinates, one request in flight at a time.

        `sync_read` bursts all seven requests and collects the replies afterwards. USB-CAN
        adapters with a shallow receive FIFO drop much of that burst: on a CANable at 30 Hz a
        third of the frames came back stale, which lands in a recorded dataset as a repeated
        observation. `read` sends one request and waits for that motor's reply, so each frame
        has the bus to itself, at a cost of about a millisecond per joint. A motor that still
        misses its reply keeps its previous value rather than failing the tick.

        Temperature and torque ride along for free. One MIT feedback frame carries position,
        velocity, torque and temperature together, and `RobstrideMotorsBus` decodes the whole
        frame into a per-motor cache that `read` serves from without touching the bus while the
        entry is younger than `STATE_CACHE_TTL_S` (20 ms). The position read above has just
        refreshed that entry, so the two reads below are cache hits and put no extra frames on
        the wire. Either the whole frame lands or the motor counts as silent for this cycle.
        """
        positions: dict[str, float] = {}
        for motor in self._joint_motor_names:
            try:
                position = float(self.bus.read("Present_Position", motor))
                temperature = float(self.bus.read("Temperature_MOS", motor))
                torque = float(self.bus.read("Present_Torque", motor))
            except Exception:
                self._silent_cycles[motor] += 1
                previous = self._last_positions.get(motor)
                if previous is None:
                    raise
                logger.debug("%s no reply from %s, keeping the previous position", self, motor)
                positions[motor] = previous
            else:
                self._silent_cycles[motor] = 0
                positions[motor] = position
                self._last_temperatures[motor] = temperature
                self._last_torques[motor] = torque
        self._last_positions = positions
        self._monitor_temperatures()
        return positions

    def _monitor_temperatures(self) -> None:
        """Log the thermal state: rate-limited warnings per hot joint, plus a periodic health line.

        Logging only, deliberately: every way a session ends (an exception, a disconnect, cutting
        motor power) releases torque, and the arm has no brakes, so a stop triggered mid-motion
        drops it. Over `temp_critical_c` the warning becomes an ERROR-level line every 2 s until the
        operator parks the arm and ends the session themselves. The only automatic stop is
        `temp_stop_c`, checked separately in `_check_thermal_stop`.
        """
        if not self._last_temperatures:
            return

        now = time.perf_counter()
        for motor, temperature in self._last_temperatures.items():
            if temperature < self.config.temp_warn_c:
                continue
            critical = temperature >= self.config.temp_critical_c
            interval = _TEMP_CRITICAL_INTERVAL_S if critical else _TEMP_WARN_INTERVAL_S
            last_warned = self._last_temp_warn_t.get(motor)
            if last_warned is not None and now - last_warned < interval:
                continue
            self._last_temp_warn_t[motor] = now
            if critical:
                stop_note = (
                    " The follower keeps driving because stopping on its own would release torque "
                    "and drop the arm."
                    if self.config.temp_stop_c is None
                    else f" The session ends itself at {self.config.temp_stop_c:.0f} C, and the arm "
                    "drops when it does."
                )
                logger.error(
                    f"{self} {motor} is at {temperature:.1f} C, over the "
                    f"{self.config.temp_critical_c:.0f} C critical limit. Park the arm in its rest "
                    f"pose NOW and end the session once it is resting.{stop_note}"
                )
            else:
                logger.warning(
                    f"{self} {motor} is at {temperature:.1f} C, over the {self.config.temp_warn_c:.0f} C "
                    "warning threshold. Bring the arm to its rest pose now, before it reaches "
                    f"{self.config.temp_critical_c:.0f} C. The arm has no brakes, so end the session "
                    "only once it is resting."
                )

        if self._last_health_log_t is not None and now - self._last_health_log_t < _HEALTH_LOG_INTERVAL_S:
            return
        self._last_health_log_t = now
        hottest = max(self._last_temperatures, key=lambda m: self._last_temperatures[m])
        readings = ", ".join(f"{m} {t:.1f}" for m, t in self._last_temperatures.items())
        logger.info(
            f"{self} temperatures (C): {readings} | max {hottest} {self._last_temperatures[hottest]:.1f}"
        )

    def _check_thermal_stop(self) -> None:
        """End the session once a joint has held `temp_stop_c` for `_TEMP_STOP_CYCLES` readings.

        Raising releases torque and drops the arm, so this is the last line of defence, not a
        way to park. A reading under the limit resets that joint's count, so a single corrupted
        frame cannot fire it. A silent motor keeps its last known temperature and keeps counting.
        """
        if self.config.temp_stop_c is None:
            return
        for motor, temperature in self._last_temperatures.items():
            if temperature < self.config.temp_stop_c:
                self._hot_cycles[motor] = 0
                continue
            self._hot_cycles[motor] = self._hot_cycles.get(motor, 0) + 1
            if self._hot_cycles[motor] >= _TEMP_STOP_CYCLES:
                raise RuntimeError(
                    f"{self} thermal stop: {motor} at {temperature:.1f} C for "
                    f"{self._hot_cycles[motor]} consecutive readings "
                    f"(temp_stop_c={self.config.temp_stop_c:.0f} C). Ending the session; torque is "
                    "released and the arm has no brakes, so it drops."
                )

    def _check_bus_silence(self) -> None:
        """React to motors that have answered nothing for `max_silent_cycles` read cycles.

        A single silent motor only warns (rate-limited): the other joints are still holding, and
        raising here would end the session and drop them. Only when every joint is silent — a
        tripped supply, a power cut, a dead adapter — is there nothing left holding the arm, and
        the tick raises so the loop stops instead of spinning forever on last-known values with
        nothing louder than a per-loop warning from the bus.
        """
        if self.config.max_silent_cycles <= 0:
            return
        silent = {m: n for m, n in self._silent_cycles.items() if n >= self.config.max_silent_cycles}
        if not silent:
            return
        named = ", ".join(f"{m} ({n} cycles)" for m, n in sorted(silent.items()))
        if len(silent) >= len(self._joint_motor_names):
            raise RuntimeError(
                f"{self} has no CAN feedback from any motor: {named} — check arm power / CAN "
                "adapter. A tripped supply, an over-temperature shutdown and a dead CAN adapter all "
                "look like this. Support the arm: it has no brakes."
            )
        now = time.perf_counter()
        if (
            self._last_silence_warn_t is not None
            and now - self._last_silence_warn_t < _SILENCE_WARN_INTERVAL_S
        ):
            return
        self._last_silence_warn_t = now
        logger.warning(
            f"{self} has no CAN feedback for {named}; serving its last known position. The session "
            "keeps running because the other joints are still holding — check the wiring, and park "
            "the arm if it persists."
        )

    def _read_present_positions(self) -> dict[str, float]:
        raw = self._read_raw_positions()
        return {motor: raw[motor] + self._turn_offset[motor] for motor in self._joint_motor_names}

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self._joint_motor_names}

    @property
    def _telemetry_ft(self) -> dict[str, type]:
        """Observation-only telemetry decoded from the same MIT feedback frame as the position.

        `{motor}.temp` is the motor's **winding** temperature in degrees Celsius. The RobStride
        MIT feedback frame carries it big-endian in bytes 6-7 as tenths of a degree, and
        `RobstrideMotorsBus` decodes it as `((data[6] << 8) | data[7]) / 10` (RS00 manual,
        "MIT protocol feedback frame": *winding temperature, Temp(Celsius) x 10*). It is not the
        MOS/driver temperature, despite the bus naming the field `Temperature_MOS` after the
        Damiao frame it was modelled on — RobStride's MIT frame has no separate MOS byte. On this
        arm at rest the joints read roughly 14-34 C.

        `{motor}.torque` is the estimated joint torque in Nm from the same frame, decoded against
        the motor model's torque range (+-14 Nm on an RS00, +-17 Nm on an RS02).

        Both are observation-only: they are not commandable, so they are absent from
        `action_features`.
        """
        return {
            **{f"{motor}.temp": float for motor in self._joint_motor_names},
            **{f"{motor}.torque": float for motor in self._joint_motor_names},
        }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {cam_key: (cam.height, cam.width, 3) for cam_key, cam in self.cameras.items()}

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._telemetry_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        logger.info(f"Connecting arm on {self.config.port}...")
        self.bus.connect()
        try:
            for cam in self.cameras.values():
                cam.connect()

            if not self.is_calibrated and calibrate:
                logger.info(
                    "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
                )
                self.calibrate()

            # Gains ride along in every MIT frame, so they must be in the bus before the first
            # command. The bus default kp=10 is far too soft to hold the arm against gravity.
            # Entries for motors this arm does not have are dropped rather than written to a
            # missing id.
            self._resolved_gains = {
                m: (kp, kd) for m, (kp, kd) in self.config.gains.items() if m in self._joint_motor_names
            }
            self.bus.sync_write("Kp", {m: kp for m, (kp, kd) in self._resolved_gains.items()})
            self.bus.sync_write("Kd", {m: kd for m, (kp, kd) in self._resolved_gains.items()})
            self._detect_full_turn_offsets()
            self.bus.enable_torque()
        except Exception:
            # Do not hold the serial port (or a half-enabled arm) after a failed connect.
            self.bus.disconnect(self.config.disable_torque_on_disconnect)
            raise

        # Re-arm the slow initial sync on every connect.
        self._synced = False
        self._synced_motors = set()
        self._sync_progress = {}
        # A new session starts with a clean bus-silence count and prints its own health line.
        self._silent_cycles = dict.fromkeys(self._joint_motor_names, 0)
        self._hot_cycles = dict.fromkeys(self._joint_motor_names, 0)
        self._last_temp_warn_t = {}
        self._last_silence_warn_t = None
        self._last_health_log_t = None

        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        # RobStride motors hold their zero across power cycles; calibration here means the user
        # has confirmed the zero pose once and a calibration file exists on disk.
        return bool(self.calibration)

    def calibrate(self) -> None:
        """Interactive zero-pose calibration for the follower arm.

        No range-of-motion recording is needed. This procedure confirms the arm is in its zero
        pose and sets that reference on each motor, matching the reBot B601 and Metal followers.
        """
        if self.calibration:
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, "
                "or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Using calibration file associated with the id {self.id}")
                return

        logger.info(f"\nRunning calibration of {self}")
        self.bus.disable_torque()
        print(
            "\nCalibration: set zero position.\n"
            f"Manually move the {self.display_name} arm to its ZERO POSE: the folded resting pose it "
            "ships in,\n"
            "arm tucked against the base, gripper fully open.\n"
            "The soft limits in `joint_limits` are measured from this pose.\n"
        )
        input("Press ENTER when ready...")

        # Log the pre-zero readings so an offset against an earlier zero stays recoverable.
        positions = self._read_raw_positions()
        for motor in self._joint_motor_names:
            logger.info(f"Pre-zero position of {motor}: {positions[motor]:.2f} deg")
        self.bus.set_zero_position()
        self._turn_offset = dict.fromkeys(self._joint_motor_names, 0.0)
        self._stale_zero = {}
        self._last_positions = {}
        logger.info("Arm zero position set.")

        self.calibration = {}
        for motor_name, can_id in self.config.motor_can_ids.items():
            range_min, range_max = self.config.joint_limits.get(motor_name, (-360.0, 360.0))
            self.calibration[motor_name] = MotorCalibration(
                id=can_id,
                drive_mode=0,
                homing_offset=0,
                range_min=int(range_min),
                range_max=int(range_max),
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        """No-op: follow gains are set in connect() from config.gains."""
        pass

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        start = time.perf_counter()

        obs_dict: dict[str, Any] = {}

        positions = self._read_present_positions()
        self._check_bus_silence()
        self._check_thermal_stop()
        for motor in self._joint_motor_names:
            obs_dict[f"{motor}.pos"] = positions[motor]
        # A motor that missed this cycle's frame reports its last known telemetry, the same way
        # its position does; 0.0 until it has ever answered.
        for motor in self._joint_motor_names:
            obs_dict[f"{motor}.temp"] = self._last_temperatures.get(motor, 0.0)
        for motor in self._joint_motor_names:
            obs_dict[f"{motor}.torque"] = self._last_torques.get(motor, 0.0)

        for cam_key, cam in self.cameras.items():
            cam_start = time.perf_counter()
            obs_dict[cam_key] = cam.read_latest()
            dt_ms = (time.perf_counter() - cam_start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} get_observation took: {dt_ms:.1f}ms")

        return obs_dict

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        if self._stale_zero:
            stale = ", ".join(f"{m} at {p:+.1f} deg" for m, p in self._stale_zero.items())
            raise RuntimeError(
                f"{self} refuses to move: {stale} is outside the soft limits and not by a whole "
                "turn, so the motor zero no longer matches the calibration pose. Move the arm to "
                "its zero pose and run `lerobot-calibrate` again."
            )
        # Deliberately no thermal stop here: temperature is logged by `_monitor_temperatures`, the
        # optional `temp_stop_c` stop lives in `get_observation`, and anything short of it is the
        # operator's call, because ending the session releases torque.
        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        # Clamp every motor, gripper included, to its soft limit.
        for motor_name, position in goal_pos.items():
            if motor_name in self.config.joint_limits:
                min_limit, max_limit = self.config.joint_limits[motor_name]
                clipped_position = max(min_limit, min(max_limit, position))
                if clipped_position != position:
                    logger.debug(
                        f"Clipped {motor_name} from {position:.2f} deg to {clipped_position:.2f} deg"
                    )
                goal_pos[motor_name] = clipped_position

        # The startup sync and the relative-target cap both need the arm's present state. Read
        # it at most once per tick and share it; the bus serves it from its cache when the
        # observation read of this tick is still fresh.
        syncing = self.config.startup_sync_speed_deg is not None and not self._synced
        present_pos: dict[str, float] = {}
        if syncing or self.config.max_relative_target is not None:
            present_pos = self._read_present_positions()

        # Slow initial sync: at teleop start cap each joint's per-step motion until the follower
        # has caught up to the leader, so firm follow gains don't snap it across a large gap.
        # Sync is per joint: a joint that cannot converge (parked past a soft limit, or stiction
        # above what kp * step can overcome) is released after a stall instead of capping the
        # whole arm forever; max_relative_target still bounds its speed after release.
        if syncing:
            step = self.config.startup_sync_speed_deg
            now = time.perf_counter()
            for motor_name, position in goal_pos.items():
                if motor_name in self._synced_motors:
                    continue
                present = present_pos[motor_name]
                err = position - present
                if abs(err) <= self.config.startup_sync_tolerance_deg:
                    self._synced_motors.add(motor_name)
                    continue
                last_pos, last_progress_t = self._sync_progress.get(motor_name, (present, now))
                if abs(present - last_pos) > _SYNC_PROGRESS_EPS_DEG:
                    self._sync_progress[motor_name] = (present, now)
                elif now - last_progress_t > _SYNC_STALL_RELEASE_SEC:
                    logger.warning(
                        f"{self} startup sync stalled on {motor_name} ({abs(err):.1f} deg from goal); "
                        "releasing it to full speed."
                    )
                    self._synced_motors.add(motor_name)
                    continue
                else:
                    self._sync_progress.setdefault(motor_name, (present, now))
                goal_pos[motor_name] = present + max(-step, min(step, err))
            if len(self._synced_motors) >= len(goal_pos):
                self._synced = True
                logger.info(f"{self} synced to leader; tracking at full speed.")

        # Cap goal position when too far away from present position.
        if self.config.max_relative_target is not None:
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        # Gains were stored in the bus at connect and ride along in every MIT frame. Written one
        # motor at a time for the same reason reads are, see `_read_raw_positions`.
        for motor, position in goal_pos.items():
            self.bus.write("Goal_Position", motor, position - self._turn_offset[motor])

        return {f"{motor}.pos": val for motor, val in goal_pos.items()}

    @check_if_not_connected
    def disconnect(self) -> None:
        self.bus.disconnect(self.config.disable_torque_on_disconnect)

        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")
