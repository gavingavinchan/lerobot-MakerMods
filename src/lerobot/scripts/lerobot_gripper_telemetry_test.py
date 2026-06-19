#!/usr/bin/env python

import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw, ImageFont

from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.motors.feetech.calibration_defaults import SO_FOLLOWER_MOTORS


TELEMETRY_REGISTERS = {
    "pos": "Present_Position",
    "velocity": "Present_Velocity",
    "load": "Present_Load",
    "voltage": "Present_Voltage",
    "temperature": "Present_Temperature",
    "status": "Status",
    "moving": "Moving",
    "current": "Present_Current",
}

MOTOR_COLORS = {
    "shoulder_pan": "#7f7f7f",
    "shoulder_lift": "#1f77b4",
    "elbow_flex": "#2ca02c",
    "wrist_flex": "#9467bd",
    "wrist_roll": "#8c564b",
    "gripper": "#d62728",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Move only the SO101 gripper while plotting telemetry.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("logs"))
    parser.add_argument("--delta", type=int, default=300, help="Raw ticks to move on either side of start.")
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--hold-s", type=float, default=1.25)
    parser.add_argument("--start-hold-s", type=float, default=None)
    parser.add_argument("--open-hold-s", type=float, default=None)
    parser.add_argument("--settle-s", type=float, default=0.75)
    parser.add_argument("--retries", type=int, default=1)
    parser.add_argument("--skip-return", action="store_true", help="Do not command the gripper back to start.")
    parser.add_argument("--release-torque-at-end", action="store_true")
    return parser.parse_args()


def read_all(bus: FeetechMotorsBus, retries: int) -> dict[str, dict[str, float]]:
    sample = {motor: {} for motor in bus.motors}
    for field, register in TELEMETRY_REGISTERS.items():
        try:
            values = bus.sync_read(register, normalize=False, num_retry=retries)
            for motor, value in values.items():
                sample[motor][field] = float(value)
        except Exception as exc:
            print(f"read {register} failed: {exc}", flush=True)
            for motor in bus.motors:
                sample[motor][field] = math.nan
    return sample


def flatten_sample(elapsed_s: float, sample: dict[str, dict[str, float]]) -> dict[str, float]:
    row = {"elapsed_s": elapsed_s}
    for motor, fields in sample.items():
        for field, value in fields.items():
            row[f"{motor}.{field}"] = value
    return row


def write_csv(path: Path, rows: list[dict[str, float]]) -> None:
    fieldnames = list(rows[0])
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def safe_write(
    bus: FeetechMotorsBus,
    register: str,
    motor: str,
    value: int,
    *,
    retries: int,
) -> bool:
    try:
        bus.write(register, motor, value, normalize=False, num_retry=retries)
    except Exception as exc:
        print(f"write {register} {motor}={value} failed: {exc}", flush=True)
        return False
    return True


def scale(value: float, src_min: float, src_max: float, dst_min: float, dst_max: float) -> float:
    if src_max == src_min:
        return (dst_min + dst_max) / 2
    return dst_min + ((value - src_min) / (src_max - src_min)) * (dst_max - dst_min)


def draw_plot(
    path: Path,
    rows: list[dict[str, float]],
    command_marks: list[tuple[float, str]],
    start_pos: int,
    targets: list[int],
) -> None:
    motors = list(SO_FOLLOWER_MOTORS)
    fields = list(TELEMETRY_REGISTERS)
    width = 1600
    panel_h = 210
    top = 92
    left = 105
    right = 35
    bottom = 46
    gap = 28
    height = top + len(fields) * panel_h + (len(fields) - 1) * gap + bottom
    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()
    title_font = ImageFont.load_default(18)

    draw.text(
        (left, 20),
        f"SO101 gripper-only telemetry | start={start_pos} | targets={targets}",
        fill="black",
        font=title_font,
    )

    legend_x = left
    for motor in motors:
        color = MOTOR_COLORS[motor]
        draw.line((legend_x, 61, legend_x + 26, 61), fill=color, width=4 if motor == "gripper" else 2)
        draw.text((legend_x + 32, 53), motor, fill="black", font=font)
        legend_x += 180

    min_t = rows[0]["elapsed_s"]
    max_t = rows[-1]["elapsed_s"]
    plot_w = width - left - right

    for index, field in enumerate(fields):
        y0 = top + index * (panel_h + gap)
        y1 = y0 + panel_h
        plot_top = y0 + 24
        plot_bottom = y1 - 28
        values = [
            row[f"{motor}.{field}"]
            for row in rows
            for motor in motors
            if math.isfinite(row[f"{motor}.{field}"])
        ]
        if not values:
            values = [0.0]
        vmin = min(values)
        vmax = max(values)
        if vmin == vmax:
            pad = max(1.0, abs(vmin) * 0.05)
            vmin -= pad
            vmax += pad
        else:
            pad = (vmax - vmin) * 0.08
            vmin -= pad
            vmax += pad

        draw.text((20, y0 + 4), field, fill="black", font=title_font)
        draw.text((20, plot_top), f"{vmax:.1f}", fill="#555555", font=font)
        draw.text((20, plot_bottom - 10), f"{vmin:.1f}", fill="#555555", font=font)
        draw.rectangle((left, plot_top, width - right, plot_bottom), outline="#dddddd")

        for mark_t, label in command_marks:
            x = scale(mark_t, min_t, max_t, left, left + plot_w)
            draw.line((x, plot_top, x, plot_bottom), fill="#cccccc", width=1)
            if index == 0:
                draw.text((x + 3, plot_top + 2), label, fill="#555555", font=font)

        for motor in motors:
            points = []
            for row in rows:
                if not math.isfinite(row[f"{motor}.{field}"]):
                    continue
                x = scale(row["elapsed_s"], min_t, max_t, left, left + plot_w)
                y = scale(row[f"{motor}.{field}"], vmin, vmax, plot_bottom, plot_top)
                points.append((x, y))
            if len(points) > 1:
                draw.line(points, fill=MOTOR_COLORS[motor], width=4 if motor == "gripper" else 2)

        draw.text((left, y1 - 20), f"{min_t:.1f}s", fill="#555555", font=font)
        draw.text((width - right - 45, y1 - 20), f"{max_t:.1f}s", fill="#555555", font=font)

    image.save(path)


def main() -> None:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = args.out_dir / f"gripper_telemetry_{stamp}.csv"
    png_path = args.out_dir / f"gripper_telemetry_{stamp}.png"

    bus = FeetechMotorsBus(port=args.port, motors=SO_FOLLOWER_MOTORS)
    bus.connect()
    rows: list[dict[str, float]] = []
    command_marks: list[tuple[float, str]] = []
    start_pos = -1
    targets: list[int] = []
    start_t = time.perf_counter()

    try:
        start_pos = int(bus.read("Present_Position", "gripper", normalize=False, num_retry=args.retries))
        min_pos = int(bus.read("Min_Position_Limit", "gripper", normalize=False, num_retry=args.retries))
        max_pos = int(bus.read("Max_Position_Limit", "gripper", normalize=False, num_retry=args.retries))
        initial_torque = int(bus.read("Torque_Enable", "gripper", normalize=False, num_retry=args.retries))
        initial_lock = int(bus.read("Lock", "gripper", normalize=False, num_retry=args.retries))
        initial_velocity = int(bus.read("Goal_Velocity", "gripper", normalize=False, num_retry=args.retries))
        initial_acceleration = int(bus.read("Acceleration", "gripper", normalize=False, num_retry=args.retries))

        lower = max(min_pos, start_pos - args.delta)
        upper = min(max_pos, start_pos + args.delta)
        targets = [upper, lower, start_pos]

        safe_write(bus, "Lock", "gripper", 0, retries=args.retries)
        safe_write(bus, "Operating_Mode", "gripper", OperatingMode.POSITION.value, retries=args.retries)
        safe_write(bus, "Acceleration", "gripper", 50, retries=args.retries)
        safe_write(bus, "Goal_Velocity", "gripper", 450, retries=args.retries)
        safe_write(bus, "Torque_Enable", "gripper", 1, retries=args.retries)

        start_hold_s = args.start_hold_s if args.start_hold_s is not None else args.hold_s
        open_hold_s = args.open_hold_s if args.open_hold_s is not None else args.hold_s
        schedule = [(args.settle_s, "start", start_pos)]
        t = args.settle_s + start_hold_s
        schedule.append((t, "out", upper))
        t += open_hold_s
        schedule.append((t, "in", lower))
        if not args.skip_return:
            t += args.hold_s
            schedule.append((t, "return", start_pos))
        end_at = schedule[-1][0] + (args.hold_s if args.skip_return else args.settle_s)

        next_command = 0
        period = 1 / args.fps
        while True:
            now = time.perf_counter()
            elapsed = now - start_t
            while next_command < len(schedule) and elapsed >= schedule[next_command][0]:
                _, label, target = schedule[next_command]
                ok = safe_write(bus, "Goal_Position", "gripper", target, retries=args.retries)
                suffix = "" if ok else "!"
                command_marks.append((elapsed, f"{label}:{target}{suffix}"))
                print(f"{elapsed:.3f}s command gripper -> {target} ({label}) ok={ok}", flush=True)
                next_command += 1

            rows.append(flatten_sample(elapsed, read_all(bus, args.retries)))
            if elapsed >= end_at:
                break
            sleep_s = period - (time.perf_counter() - now)
            if sleep_s > 0:
                time.sleep(sleep_s)

        if not args.skip_return:
            safe_write(bus, "Goal_Position", "gripper", start_pos, retries=args.retries)
            time.sleep(args.settle_s)
            rows.append(flatten_sample(time.perf_counter() - start_t, read_all(bus, args.retries)))

        safe_write(bus, "Goal_Velocity", "gripper", initial_velocity, retries=args.retries)
        safe_write(bus, "Acceleration", "gripper", initial_acceleration, retries=args.retries)
        torque_at_end = 0 if args.release_torque_at_end else initial_torque
        safe_write(bus, "Torque_Enable", "gripper", torque_at_end, retries=args.retries)
        safe_write(bus, "Lock", "gripper", initial_lock, retries=args.retries)

    finally:
        if rows:
            write_csv(csv_path, rows)
            draw_plot(png_path, rows, command_marks, start_pos, targets)
            print(f"start_pos={start_pos}")
            print(f"targets={targets}")
            print(f"csv={csv_path}")
            print(f"plot={png_path}")
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
