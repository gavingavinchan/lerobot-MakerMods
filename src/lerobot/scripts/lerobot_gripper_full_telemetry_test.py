#!/usr/bin/env python

"""Phased SO101 gripper cube test that records and plots EVERY readable servo value.

Unlike lerobot_gripper_cube_release_test.py (which plots a handful of fields), this
script polls the full readable Feetech register set for the gripper servo each loop,
decodes the direction-encoded Present_Load into a signed value, and decodes the Status
byte (addr 65) into its six individual protection/error bits:

    bit0 voltage, bit1 sensor, bit2 temperature, bit3 current, bit4 angle, bit5 overload

Every continuous signal gets its own panel; the binary signals (Moving + the six error
bits) are drawn as step traces in a stacked "flags" panel so they are easy to read.

Status bit map confirmed against the Feetech STS3215 e-manual / python-st3215.
"""

import argparse
import csv
import json
import math
import time
from datetime import datetime
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw, ImageFont

from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.motors.feetech.calibration_defaults import SO_FOLLOWER_MOTORS
from lerobot.scripts.lerobot_gripper_telemetry_test import safe_write, write_csv

MOTOR = "gripper"

# Registers polled every loop, read DIRECTLY off the servo via the raw packet
# handler -- not via lerobot's bus.read(). lerobot raises on a non-zero error byte
# and discards the data sitting in the same packet; reading raw keeps the servo's
# actual values even while it is faulted (e.g. in overload protection).
# name -> (address, num_bytes, sign_magnitude_bit | None)
RAW_REGISTERS = {
    "pos": (56, 2, None),
    "velocity": (58, 2, 15),  # sign-magnitude, bit 15
    "load_raw": (60, 2, None),  # direction-encoded, decoded separately into load_signed
    "voltage": (62, 1, None),
    "temperature": (63, 1, None),
    "moving": (66, 1, None),
    "current": (69, 2, None),
    "goal_pos": (42, 2, None),
}

# Register 65 holds the hardware-error/protection byte (mirrors the packet error byte).
STATUS_ADDR = 65

# Status byte (addr 65) bit -> flag name, matching scservo_sdk ERRBIT_* used by lerobot.
# 1 = error/protection active. (bit4 is unused by this SDK.)
STATUS_BITS = [
    ("err_voltage", 0),  # ERRBIT_VOLTAGE  = 1
    ("err_sensor", 1),  # ERRBIT_ANGLE    = 2  (angle/sensor)
    ("err_temperature", 2),  # ERRBIT_OVERHEAT = 4
    ("err_current", 3),  # ERRBIT_OVERELE  = 8  (over-current)
    ("err_overload", 5),  # ERRBIT_OVERLOAD = 32
]

# Plausibility bounds to drop corrupted single-sample reads (raw register units).
SANE_RANGE = {
    "voltage": (40.0, 160.0),  # 0.1 V units -> 4.0 .. 16.0 V
    "temperature": (0.0, 100.0),  # deg C
}

# Panels of continuous signals, drawn one per row.
CONTINUOUS_FIELDS = [
    ("pos", "position (raw ticks)"),
    ("goal_pos", "goal position (raw ticks)"),
    ("velocity", "velocity (signed)"),
    ("load_signed", "load decoded (signed, duty)"),
    ("load_raw", "load raw (direction-encoded)"),
    ("current", "current (raw, ~6.5 mA/unit)"),
    ("voltage", "voltage (0.1 V units)"),
    ("temperature", "temperature (deg C)"),
]

# Binary lanes, drawn stacked in one panel.
BINARY_FIELDS = [
    ("moving", "moving"),
    ("err_overload", "overload"),
    ("err_current", "overcurrent"),
    ("err_temperature", "overheat"),
    ("err_voltage", "voltage"),
    ("err_sensor", "sensor"),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("logs"))
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--grip-dwell-s", type=float, default=6.0, help="Hold after a close completes/stalls.")
    parser.add_argument("--place-dwell-s", type=float, default=10.0, help="Hold open to place/remove the cube.")
    parser.add_argument("--open-dwell-s", type=float, default=2.0, help="Hold after the final open.")
    parser.add_argument("--move-timeout-s", type=float, default=8.0, help="Max time to wait per move.")
    parser.add_argument("--move-grace-s", type=float, default=0.6, help="Delay before stall detection arms.")
    parser.add_argument("--pos-tol", type=int, default=30, help="Ticks within target counted as reached.")
    parser.add_argument("--stall-velocity", type=float, default=8.0, help="|velocity| at/below this is stopped.")
    parser.add_argument("--stall-samples", type=int, default=8, help="Consecutive stopped samples = stalled.")
    parser.add_argument("--no-final-open", action="store_true")
    parser.add_argument("--no-packet-log", action="store_true", help="Disable raw SDK TX/RX byte logging.")
    parser.add_argument("--retries", type=int, default=2)
    return parser.parse_args()


class PacketLogger:
    """JSONL logger for every byte chunk passed through the SDK serial port."""

    def __init__(self, path: Path):
        self.path = path
        self.f = path.open("w")
        self.t0 = time.perf_counter()
        self.index = 0

    def close(self) -> None:
        self.f.close()

    def log(self, direction: str, data: bytes, *, requested: int | None = None) -> None:
        row = {
            "index": self.index,
            "elapsed_s": time.perf_counter() - self.t0,
            "direction": direction,
            "requested": requested,
            "n_bytes": len(data),
            "hex": data.hex(" "),
        }
        self.f.write(json.dumps(row) + "\n")
        self.f.flush()
        self.index += 1


def install_packet_logger(bus: FeetechMotorsBus, packet_logger: PacketLogger) -> None:
    original_write_port = bus.port_handler.writePort
    original_read_port = bus.port_handler.readPort

    def write_port(packet: Any):
        data = bytes(packet)
        packet_logger.log("tx", data, requested=len(data))
        return original_write_port(packet)

    def read_port(length: int):
        data = original_read_port(length)
        packet_logger.log("rx", bytes(data), requested=length)
        return data

    bus.port_handler.writePort = write_port
    bus.port_handler.readPort = read_port


def write_packet_summary(path: Path, packet_path: Path, rows: list[dict[str, float]]) -> None:
    tx_chunks = rx_chunks = tx_bytes = rx_bytes = 0
    nonempty_rx_chunks = 0
    with packet_path.open() as f:
        for line in f:
            row = json.loads(line)
            if row["direction"] == "tx":
                tx_chunks += 1
                tx_bytes += row["n_bytes"]
            else:
                rx_chunks += 1
                rx_bytes += row["n_bytes"]
                if row["n_bytes"]:
                    nonempty_rx_chunks += 1

    summary = {
        "packet_log": str(packet_path),
        "telemetry_rows": len(rows),
        "tx_chunks": tx_chunks,
        "rx_chunks": rx_chunks,
        "nonempty_rx_chunks": nonempty_rx_chunks,
        "tx_bytes": tx_bytes,
        "rx_bytes": rx_bytes,
        "note": "Packet log records every byte chunk passed through scservo_sdk PortHandler.writePort/readPort.",
    }
    path.write_text(json.dumps(summary, indent=2) + "\n")


def decode_load(raw: float) -> float:
    """Decode direction-encoded Present_Load. Bit 10 is sign, low 10 bits magnitude."""
    if not math.isfinite(raw):
        return math.nan
    value = int(raw)
    magnitude = value & 0x3FF
    return -magnitude if value & 0x400 else magnitude


def decode_sign_magnitude(value: int, sign_bit: int) -> int:
    """Decode a sign-magnitude register (e.g. Present_Velocity, sign bit 15)."""
    mask = (1 << sign_bit) - 1
    magnitude = value & mask
    return -magnitude if value & (1 << sign_bit) else magnitude


def raw_read(bus: FeetechMotorsBus, motor_id: int, addr: int, nbytes: int, retries: int) -> tuple[float, int]:
    """Read a register straight off the servo. Returns (value, error_byte).

    Ignores the servo error byte for the value itself -- the data is present in the
    packet regardless. Only a comm-layer failure (bad/missing/corrupt packet, i.e. a
    transport problem) yields NaN, and is retried first. Never raises.
    """
    reader = {1: bus.packet_handler.read1ByteTxRx,
              2: bus.packet_handler.read2ByteTxRx,
              4: bus.packet_handler.read4ByteTxRx}[nbytes]
    last_error = 0
    for _ in range(1 + max(0, retries)):
        try:
            data, comm, error = reader(bus.port_handler, motor_id, addr)
        except Exception:  # noqa: BLE001
            continue
        last_error = int(error) if error else last_error
        if comm == bus._comm_success:
            return float(data), int(error)
    return math.nan, last_error  # transport failure after retries


def read_all(bus: FeetechMotorsBus, motor_id: int, retries: int) -> dict[str, float]:
    sample: dict[str, float] = {}
    error_acc = 0  # OR of every packet error byte seen this sample

    for field, (addr, nbytes, sign_bit) in RAW_REGISTERS.items():
        value, error = raw_read(bus, motor_id, addr, nbytes, retries)
        error_acc |= error
        if math.isfinite(value):
            if sign_bit is not None:
                value = float(decode_sign_magnitude(int(value), sign_bit))
            lo, hi = SANE_RANGE.get(field, (-math.inf, math.inf))
            if not (lo <= value <= hi):
                value = math.nan
        sample[field] = value

    sample["load_signed"] = decode_load(sample.get("load_raw", math.nan))

    # Status/protection byte: register 65 value OR'd with every packet error byte seen.
    status_val, status_err = raw_read(bus, motor_id, STATUS_ADDR, 1, retries)
    error_acc |= status_err
    if math.isfinite(status_val):
        status = float((int(status_val) | error_acc) & 0xFF)
    elif error_acc:
        status = float(error_acc & 0xFF)
    else:
        status = math.nan
    sample["status_raw"] = status
    for name, bit in STATUS_BITS:
        sample[name] = float((int(status) >> bit) & 1) if math.isfinite(status) else math.nan
    return sample


def scale(value: float, src_min: float, src_max: float, dst_min: float, dst_max: float) -> float:
    if src_max == src_min:
        return (dst_min + dst_max) / 2
    return dst_min + ((value - src_min) / (src_max - src_min)) * (dst_max - dst_min)


def draw_plot(
    path: Path,
    rows: list[dict[str, float]],
    marks: list[tuple[float, str]],
    *,
    min_pos: int,
    max_pos: int,
) -> None:
    width = 1700
    panel_h = 200
    binary_h = 40
    binary_panel_h = binary_h * len(BINARY_FIELDS) + 40
    top = 100
    left = 230
    right = 45
    gap = 26
    bottom = 50
    height = (
        top
        + len(CONTINUOUS_FIELDS) * (panel_h + gap)
        + binary_panel_h
        + bottom
    )
    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()
    title_font = ImageFont.load_default(18)

    draw.text(
        (left, 22),
        "SO101 gripper full telemetry: place cube, grip, remove, close empty",
        fill="black",
        font=title_font,
    )
    draw.text(
        (left, 52),
        f"calibrated gripper range: min={min_pos}, max={max_pos}  |  status bits decoded per Feetech STS3215",
        fill="#444444",
        font=font,
    )

    min_t = rows[0]["elapsed_s"]
    max_t = rows[-1]["elapsed_s"]
    plot_w = width - left - right

    def x_of(t: float) -> float:
        return scale(t, min_t, max_t, left, left + plot_w)

    # Continuous panels.
    for index, (field, label) in enumerate(CONTINUOUS_FIELDS):
        y0 = top + index * (panel_h + gap)
        plot_top = y0 + 22
        plot_bottom = y0 + panel_h - 30
        values = [row[field] for row in rows if math.isfinite(row.get(field, math.nan))]
        if field in ("pos", "goal_pos"):
            vmin, vmax = float(min_pos), float(max_pos)
        elif values:
            vmin, vmax = min(values), max(values)
            if vmin == vmax:
                pad = max(1.0, abs(vmin) * 0.05)
            else:
                pad = (vmax - vmin) * 0.08
            vmin -= pad
            vmax += pad
        else:
            vmin, vmax = 0.0, 1.0

        draw.text((20, y0 + 4), label, fill="black", font=font)
        draw.text((150, plot_top - 2), f"{vmax:.1f}", fill="#777777", font=font)
        draw.text((150, plot_bottom - 8), f"{vmin:.1f}", fill="#777777", font=font)
        draw.rectangle((left, plot_top, width - right, plot_bottom), outline="#dddddd")

        for mark_t, mark_label in marks:
            x = x_of(mark_t)
            draw.line((x, plot_top, x, plot_bottom), fill="#cccccc", width=1)
            if index == 0:
                draw.text((x + 3, plot_top + 2), mark_label, fill="#333333", font=font)

        points = []
        for row in rows:
            value = row.get(field, math.nan)
            if not math.isfinite(value):
                continue
            points.append((x_of(row["elapsed_s"]), scale(value, vmin, vmax, plot_bottom, plot_top)))
        if len(points) > 1:
            draw.line(points, fill="#d62728", width=3)

    # Binary flags panel (stacked lanes).
    by0 = top + len(CONTINUOUS_FIELDS) * (panel_h + gap)
    draw.text((20, by0), "binary flags (high = active)", fill="black", font=title_font)
    lane_top = by0 + 28
    for li, (field, label) in enumerate(BINARY_FIELDS):
        lane_y = lane_top + li * binary_h
        low_y = lane_y + binary_h - 12
        high_y = lane_y + 6
        color = "#d62728" if field.startswith("err") else "#1f77b4"
        draw.text((20, lane_y + 6), label, fill=color, font=font)
        draw.line((left, low_y, width - right, low_y), fill="#eeeeee", width=1)
        points = []
        prev = None
        for row in rows:
            value = row.get(field, math.nan)
            if not math.isfinite(value):
                continue
            x = x_of(row["elapsed_s"])
            y = high_y if value >= 0.5 else low_y
            if prev is not None and prev[1] != y:
                points.append((x, prev[1]))  # vertical edge
            points.append((x, y))
            prev = (x, y)
        if len(points) > 1:
            draw.line(points, fill=color, width=2)

    for mark_t, _ in marks:
        x = x_of(mark_t)
        draw.line((x, lane_top, x, lane_top + len(BINARY_FIELDS) * binary_h), fill="#cccccc", width=1)

    draw.text((left, height - 30), f"{min_t:.1f}s", fill="#555555", font=font)
    draw.text((width - right - 50, height - 30), f"{max_t:.1f}s", fill="#555555", font=font)
    image.save(path)


def run_phase(
    bus: FeetechMotorsBus,
    rows: list[dict[str, float]],
    marks: list[tuple[float, str]],
    start_t: float,
    args: argparse.Namespace,
    motor_id: int,
    label: str,
    target: int,
    dwell_s: float,
) -> None:
    """Command a move, sample until reached/stalled/timeout, then dwell while sampling."""
    elapsed0 = time.perf_counter() - start_t
    ok = safe_write(bus, "Goal_Position", MOTOR, target, retries=args.retries)
    marks.append((elapsed0, f"{label}:{target}" + ("" if ok else "!")))
    print(f"{elapsed0:.2f}s command {label} -> {target} ok={ok}", flush=True)

    period = 1 / args.fps
    phase_start = time.perf_counter()
    stall_run = 0
    settled_at: float | None = None

    while True:
        loop_start = time.perf_counter()
        sample = read_all(bus, motor_id, args.retries)
        sample["elapsed_s"] = loop_start - start_t
        rows.append(sample)

        pos = sample.get("pos", math.nan)
        vel = sample.get("velocity", math.nan)
        t_in_phase = loop_start - phase_start

        reached = math.isfinite(pos) and abs(pos - target) <= args.pos_tol
        stopped = (not math.isfinite(vel)) or abs(vel) <= args.stall_velocity
        if t_in_phase >= args.move_grace_s and stopped and not reached:
            stall_run += 1
        else:
            stall_run = 0
        stalled = stall_run >= args.stall_samples
        timed_out = t_in_phase >= args.move_timeout_s

        if settled_at is None and (reached or stalled or timed_out):
            why = "reached" if reached else "stalled" if stalled else "timeout"
            settled_at = loop_start
            posr = f"{pos:.0f}" if math.isfinite(pos) else "nan"
            print(f"{sample['elapsed_s']:.2f}s {label} {why} pos={posr}", flush=True)
        if settled_at is not None and loop_start - settled_at >= dwell_s:
            break

        sleep_s = period - (time.perf_counter() - loop_start)
        if sleep_s > 0:
            time.sleep(sleep_s)


def main() -> None:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = args.out_dir / f"gripper_full_telemetry_{stamp}.csv"
    png_path = args.out_dir / f"gripper_full_telemetry_{stamp}.png"
    packet_path = args.out_dir / f"gripper_full_telemetry_{stamp}.packets.jsonl"
    packet_summary_path = args.out_dir / f"gripper_full_telemetry_{stamp}.packets.summary.json"

    bus = FeetechMotorsBus(port=args.port, motors=SO_FOLLOWER_MOTORS)
    packet_logger = None if args.no_packet_log else PacketLogger(packet_path)
    if packet_logger is not None:
        install_packet_logger(bus, packet_logger)

    rows: list[dict[str, float]] = []
    marks: list[tuple[float, str]] = []
    min_pos, max_pos = 0, 4095

    try:
        bus.connect()
        motor_id = bus.motors[MOTOR].id
        start_t = time.perf_counter()

        min_pos = int(bus.read("Min_Position_Limit", MOTOR, normalize=False, num_retry=args.retries))
        max_pos = int(bus.read("Max_Position_Limit", MOTOR, normalize=False, num_retry=args.retries))
        print(f"gripper_min={min_pos} gripper_max={max_pos}", flush=True)

        safe_write(bus, "Lock", MOTOR, 0, retries=args.retries)
        safe_write(bus, "Operating_Mode", MOTOR, OperatingMode.POSITION.value, retries=args.retries)
        safe_write(bus, "Acceleration", MOTOR, 50, retries=args.retries)
        safe_write(bus, "Goal_Velocity", MOTOR, 350, retries=args.retries)
        safe_write(bus, "Torque_Enable", MOTOR, 1, retries=args.retries)

        # Each phase runs until the gripper reaches its target or stalls (e.g. on the
        # cube), then dwells so the hold can be observed -- no fixed-time cutoff.
        phases = [
            ("open/place_cube", max_pos, args.place_dwell_s),
            ("close_on_cube", min_pos, args.grip_dwell_s),
            ("open/remove_cube", max_pos, args.place_dwell_s),
            ("close_empty", min_pos, args.grip_dwell_s),
        ]
        if not args.no_final_open:
            phases.append(("final_open", max_pos, args.open_dwell_s))

        for label, target, dwell_s in phases:
            run_phase(bus, rows, marks, start_t, args, motor_id, label, target, dwell_s)

        safe_write(bus, "Torque_Enable", MOTOR, 0, retries=args.retries)
    finally:
        if rows:
            ordered = []
            for row in rows:
                ordered.append({"elapsed_s": row["elapsed_s"], **{k: row[k] for k in row if k != "elapsed_s"}})
            write_csv(csv_path, ordered)
            draw_plot(png_path, rows, marks, min_pos=min_pos, max_pos=max_pos)
            print(f"csv={csv_path}", flush=True)
            print(f"plot={png_path}", flush=True)
        if bus.is_connected:
            bus.disconnect(disable_torque=False)
        if packet_logger is not None:
            packet_logger.close()
            write_packet_summary(packet_summary_path, packet_path, rows)
            print(f"packet_log={packet_path}", flush=True)
            print(f"packet_summary={packet_summary_path}", flush=True)


if __name__ == "__main__":
    main()
