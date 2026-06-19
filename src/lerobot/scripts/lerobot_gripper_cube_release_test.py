#!/usr/bin/env python

import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.motors.feetech.calibration_defaults import SO_FOLLOWER_MOTORS
from lerobot.scripts.lerobot_gripper_telemetry_test import (
    TELEMETRY_REGISTERS,
    flatten_sample,
    read_all,
    safe_write,
    write_csv,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Open, grip cube, release cube, then close empty.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("logs"))
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--place-window-s", type=float, default=10.0)
    parser.add_argument("--grip-hold-s", type=float, default=6.0)
    parser.add_argument("--remove-window-s", type=float, default=10.0)
    parser.add_argument("--empty-hold-s", type=float, default=4.0)
    parser.add_argument("--final-open", action="store_true")
    parser.add_argument("--retries", type=int, default=2)
    return parser.parse_args()


def scale(value: float, src_min: float, src_max: float, dst_min: float, dst_max: float) -> float:
    if src_max == src_min:
        return (dst_min + dst_max) / 2
    return dst_min + ((value - src_min) / (src_max - src_min)) * (dst_max - dst_min)


def draw_gripper_plot(
    path: Path,
    rows: list[dict[str, float]],
    marks: list[tuple[float, str]],
    *,
    min_pos: int,
    max_pos: int,
) -> None:
    fields = ["gripper.pos", "gripper.load", "gripper.current", "gripper.velocity", "gripper.moving"]
    labels = ["position", "load", "current", "velocity", "moving"]
    width = 1600
    panel_h = 260
    top = 95
    left = 110
    right = 45
    gap = 34
    bottom = 55
    height = top + len(fields) * panel_h + (len(fields) - 1) * gap + bottom
    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()
    title_font = ImageFont.load_default(18)

    draw.text(
        (left, 22),
        "SO101 gripper cube test: close on cube, release/remove, close empty",
        fill="black",
        font=title_font,
    )
    draw.text(
        (left, 55),
        f"calibrated gripper range: min={min_pos}, max={max_pos}",
        fill="#444444",
        font=font,
    )

    min_t = rows[0]["elapsed_s"]
    max_t = rows[-1]["elapsed_s"]
    plot_w = width - left - right

    for index, (field, label) in enumerate(zip(fields, labels, strict=True)):
        y0 = top + index * (panel_h + gap)
        y1 = y0 + panel_h
        plot_top = y0 + 25
        plot_bottom = y1 - 34
        values = [row[field] for row in rows if math.isfinite(row[field])]
        if field == "gripper.pos":
            vmin, vmax = min_pos, max_pos
        elif values:
            vmin, vmax = min(values), max(values)
            if vmin == vmax:
                pad = max(1.0, abs(vmin) * 0.05)
                vmin -= pad
                vmax += pad
            else:
                pad = (vmax - vmin) * 0.08
                vmin -= pad
                vmax += pad
        else:
            vmin, vmax = 0.0, 1.0

        draw.text((20, y0 + 5), label, fill="black", font=title_font)
        draw.text((20, plot_top), f"{vmax:.1f}", fill="#555555", font=font)
        draw.text((20, plot_bottom - 10), f"{vmin:.1f}", fill="#555555", font=font)
        draw.rectangle((left, plot_top, width - right, plot_bottom), outline="#dddddd")

        if field == "gripper.pos":
            for value, text in [(min_pos, "min"), (max_pos, "max")]:
                y = scale(value, vmin, vmax, plot_bottom, plot_top)
                draw.line((left, y, width - right, y), fill="#dddddd", width=1)
                draw.text((width - right - 55, y - 10), text, fill="#777777", font=font)

        for mark_t, mark_label in marks:
            x = scale(mark_t, min_t, max_t, left, left + plot_w)
            draw.line((x, plot_top, x, plot_bottom), fill="#c9c9c9", width=1)
            if index == 0:
                draw.text((x + 3, plot_top + 2), mark_label, fill="#333333", font=font)

        points = []
        for row in rows:
            value = row[field]
            if not math.isfinite(value):
                continue
            x = scale(row["elapsed_s"], min_t, max_t, left, left + plot_w)
            y = scale(value, vmin, vmax, plot_bottom, plot_top)
            points.append((x, y))
        if len(points) > 1:
            draw.line(points, fill="#d62728", width=4)

        draw.text((left, y1 - 25), f"{min_t:.1f}s", fill="#555555", font=font)
        draw.text((width - right - 50, y1 - 25), f"{max_t:.1f}s", fill="#555555", font=font)

    image.save(path)


def main() -> None:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = args.out_dir / f"gripper_cube_release_{stamp}.csv"
    png_path = args.out_dir / f"gripper_cube_release_{stamp}.png"

    bus = FeetechMotorsBus(port=args.port, motors=SO_FOLLOWER_MOTORS)
    bus.connect()
    rows: list[dict[str, float]] = []
    marks: list[tuple[float, str]] = []
    start_t = time.perf_counter()
    min_pos = 0
    max_pos = 4095

    try:
        min_pos = int(bus.read("Min_Position_Limit", "gripper", normalize=False, num_retry=args.retries))
        max_pos = int(bus.read("Max_Position_Limit", "gripper", normalize=False, num_retry=args.retries))
        print(f"gripper_min={min_pos} gripper_max={max_pos}", flush=True)

        safe_write(bus, "Lock", "gripper", 0, retries=args.retries)
        safe_write(bus, "Operating_Mode", "gripper", OperatingMode.POSITION.value, retries=args.retries)
        safe_write(bus, "Acceleration", "gripper", 50, retries=args.retries)
        safe_write(bus, "Goal_Velocity", "gripper", 350, retries=args.retries)
        safe_write(bus, "Torque_Enable", "gripper", 1, retries=args.retries)

        schedule = [
            (0.75, "open/place_cube", max_pos),
            (0.75 + args.place_window_s, "close_on_cube", min_pos),
            (0.75 + args.place_window_s + args.grip_hold_s, "open/remove_cube", max_pos),
            (
                0.75 + args.place_window_s + args.grip_hold_s + args.remove_window_s,
                "close_empty",
                min_pos,
            ),
        ]
        end_at = schedule[-1][0] + args.empty_hold_s
        if args.final_open:
            schedule.append((end_at, "final_open", max_pos))
            end_at += 2.0

        next_command = 0
        period = 1 / args.fps
        while True:
            loop_start = time.perf_counter()
            elapsed = loop_start - start_t
            while next_command < len(schedule) and elapsed >= schedule[next_command][0]:
                _, label, target = schedule[next_command]
                ok = safe_write(bus, "Goal_Position", "gripper", target, retries=args.retries)
                marks.append((elapsed, f"{label}:{target}" + ("" if ok else "!")))
                print(f"{elapsed:.3f}s command {label} -> {target} ok={ok}", flush=True)
                next_command += 1

            rows.append(flatten_sample(elapsed, read_all(bus, args.retries)))
            if elapsed >= end_at:
                break

            sleep_s = period - (time.perf_counter() - loop_start)
            if sleep_s > 0:
                time.sleep(sleep_s)

        safe_write(bus, "Torque_Enable", "gripper", 0, retries=args.retries)
    finally:
        if rows:
            write_csv(csv_path, rows)
            draw_gripper_plot(png_path, rows, marks, min_pos=min_pos, max_pos=max_pos)
            print(f"csv={csv_path}", flush=True)
            print(f"plot={png_path}", flush=True)
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
