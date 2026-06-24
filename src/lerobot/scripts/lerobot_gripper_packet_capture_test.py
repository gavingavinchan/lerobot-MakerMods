#!/usr/bin/env python

"""Run a short SO101 gripper test while logging every SDK serial TX/RX byte chunk."""

import argparse
import csv
import json
import time
from datetime import datetime
from pathlib import Path
from typing import Any

from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.motors.feetech.calibration_defaults import SO_FOLLOWER_MOTORS
from lerobot.scripts.lerobot_gripper_full_telemetry_test import read_all
from lerobot.scripts.lerobot_gripper_telemetry_test import safe_write, write_csv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("logs"))
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--open-s", type=float, default=2.0)
    parser.add_argument("--close-s", type=float, default=2.0)
    parser.add_argument("--final-open-s", type=float, default=1.5)
    parser.add_argument("--retries", type=int, default=2)
    return parser.parse_args()


class PacketLogger:
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


def flatten_gripper_sample(elapsed_s: float, sample: dict[str, float]) -> dict[str, float]:
    row = {"elapsed_s": elapsed_s}
    for key, value in sample.items():
        row[f"gripper.{key}"] = value
    return row


def write_summary(path: Path, packet_path: Path, rows: list[dict[str, float]]) -> None:
    tx_chunks = rx_chunks = tx_bytes = rx_bytes = 0
    with packet_path.open() as f:
        for line in f:
            row = json.loads(line)
            if row["direction"] == "tx":
                tx_chunks += 1
                tx_bytes += row["n_bytes"]
            else:
                rx_chunks += 1
                rx_bytes += row["n_bytes"]

    summary = {
        "packet_log": str(packet_path),
        "telemetry_rows": len(rows),
        "tx_chunks": tx_chunks,
        "rx_chunks": rx_chunks,
        "tx_bytes": tx_bytes,
        "rx_bytes": rx_bytes,
        "note": "Packet log records every byte chunk passed through scservo_sdk PortHandler.writePort/readPort.",
    }
    path.write_text(json.dumps(summary, indent=2) + "\n")


def main() -> None:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    packet_path = args.out_dir / f"gripper_packet_capture_{stamp}.jsonl"
    telemetry_path = args.out_dir / f"gripper_packet_capture_{stamp}.csv"
    summary_path = args.out_dir / f"gripper_packet_capture_{stamp}.summary.json"

    packet_logger = PacketLogger(packet_path)
    bus = FeetechMotorsBus(port=args.port, motors=SO_FOLLOWER_MOTORS)
    install_packet_logger(bus, packet_logger)

    rows: list[dict[str, float]] = []
    start_t = time.perf_counter()
    period = 1 / args.fps
    motor_id = SO_FOLLOWER_MOTORS["gripper"].id

    try:
        bus.connect()
        min_pos = int(bus.read("Min_Position_Limit", "gripper", normalize=False, num_retry=args.retries))
        max_pos = int(bus.read("Max_Position_Limit", "gripper", normalize=False, num_retry=args.retries))
        start_pos = int(bus.read("Present_Position", "gripper", normalize=False, num_retry=args.retries))
        print(f"start={start_pos} min={min_pos} max={max_pos}", flush=True)

        safe_write(bus, "Lock", "gripper", 0, retries=args.retries)
        safe_write(bus, "Operating_Mode", "gripper", OperatingMode.POSITION.value, retries=args.retries)
        safe_write(bus, "Acceleration", "gripper", 50, retries=args.retries)
        safe_write(bus, "Goal_Velocity", "gripper", 300, retries=args.retries)
        safe_write(bus, "Torque_Enable", "gripper", 1, retries=args.retries)

        schedule = [
            (0.5, "open", max_pos),
            (0.5 + args.open_s, "close", min_pos),
            (0.5 + args.open_s + args.close_s, "final_open", max_pos),
        ]
        end_at = schedule[-1][0] + args.final_open_s
        next_command = 0

        while True:
            loop_start = time.perf_counter()
            elapsed = loop_start - start_t
            while next_command < len(schedule) and elapsed >= schedule[next_command][0]:
                _, label, target = schedule[next_command]
                ok = safe_write(bus, "Goal_Position", "gripper", target, retries=args.retries)
                print(f"{elapsed:.3f}s command {label} -> {target} ok={ok}", flush=True)
                next_command += 1

            rows.append(flatten_gripper_sample(elapsed, read_all(bus, motor_id, args.retries)))
            if elapsed >= end_at:
                break
            sleep_s = period - (time.perf_counter() - loop_start)
            if sleep_s > 0:
                time.sleep(sleep_s)

        safe_write(bus, "Torque_Enable", "gripper", 0, retries=args.retries)
    finally:
        if rows:
            write_csv(telemetry_path, rows)
        if bus.is_connected:
            bus.disconnect(disable_torque=False)
        packet_logger.close()
        write_summary(summary_path, packet_path, rows)
        print(f"packet_log={packet_path}", flush=True)
        print(f"telemetry_csv={telemetry_path}", flush=True)
        print(f"summary={summary_path}", flush=True)


if __name__ == "__main__":
    main()
