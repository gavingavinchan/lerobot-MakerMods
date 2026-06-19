#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
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

import argparse
import csv
import json
import time
from pathlib import Path
from typing import Any

from lerobot.motors.feetech import FeetechMotorsBus
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Poll raw Feetech SO101 servo telemetry.")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/cu.usbmodem5B140324321")
    parser.add_argument(
        "--fields",
        default="all",
        help="Comma-separated fields to poll, or 'all'. Supported: " + ", ".join(TELEMETRY_REGISTERS),
    )
    parser.add_argument("--fps", type=float, default=10.0, help="Target polling rate.")
    parser.add_argument("--samples", type=int, default=0, help="Number of samples to read. 0 means forever.")
    parser.add_argument("--csv", type=Path, default=None, help="Optional CSV output path.")
    parser.add_argument("--retries", type=int, default=1, help="Read retries per register.")
    parser.add_argument(
        "--no-handshake", action="store_true", help="Skip the full motor handshake on connect."
    )
    return parser.parse_args()


def resolve_fields(fields_arg: str) -> list[str]:
    if fields_arg == "all":
        return list(TELEMETRY_REGISTERS)

    fields = [field.strip() for field in fields_arg.split(",") if field.strip()]
    unknown = set(fields) - set(TELEMETRY_REGISTERS)
    if unknown:
        raise ValueError(f"Unsupported telemetry fields: {sorted(unknown)}")
    return fields


def flatten_sample(sample: dict[str, Any], motors: list[str], fields: list[str]) -> dict[str, Any]:
    row = {"timestamp": sample["timestamp"]}
    for motor in motors:
        for field in fields:
            row[f"{motor}.{field}"] = sample["motors"][motor][field]
    return row


def read_sample(bus: FeetechMotorsBus, fields: list[str], retries: int) -> dict[str, Any]:
    sample = {
        "timestamp": time.time(),
        "motors": {motor: {} for motor in bus.motors},
    }

    for field in fields:
        register = TELEMETRY_REGISTERS[field]
        values = bus.sync_read(register, normalize=False, num_retry=retries)
        for motor, value in values.items():
            sample["motors"][motor][field] = value

    return sample


def main() -> None:
    args = parse_args()
    fields = resolve_fields(args.fields)
    period_s = 1 / args.fps if args.fps > 0 else 0

    bus = FeetechMotorsBus(port=args.port, motors=SO_FOLLOWER_MOTORS)
    bus.connect(handshake=not args.no_handshake)

    csv_file = None
    csv_writer = None
    try:
        motors = list(bus.motors)
        if args.csv is not None:
            args.csv.parent.mkdir(parents=True, exist_ok=True)
            csv_file = args.csv.open("w", newline="")
            fieldnames = ["timestamp"] + [f"{motor}.{field}" for motor in motors for field in fields]
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            csv_writer.writeheader()

        count = 0
        while args.samples <= 0 or count < args.samples:
            loop_start = time.perf_counter()
            sample = read_sample(bus, fields, args.retries)
            print(json.dumps(sample), flush=True)

            if csv_writer is not None:
                csv_writer.writerow(flatten_sample(sample, motors, fields))
                csv_file.flush()

            count += 1
            elapsed_s = time.perf_counter() - loop_start
            if period_s > elapsed_s:
                time.sleep(period_s - elapsed_s)
    except KeyboardInterrupt:
        pass
    finally:
        if csv_file is not None:
            csv_file.close()
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
