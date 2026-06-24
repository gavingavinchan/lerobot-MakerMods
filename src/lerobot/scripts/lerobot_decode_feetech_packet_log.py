#!/usr/bin/env python

"""Decode Feetech packet JSONL logs into packet-level CSV rows.

Input logs are produced by the gripper telemetry scripts that monkeypatch
scservo_sdk PortHandler.writePort/readPort. RX packets can be split across
multiple readPort chunks; this utility reassembles them before decoding.
"""

import argparse
import csv
import importlib.util
import json
from collections import deque
from dataclasses import dataclass
from pathlib import Path


INSTRUCTIONS = {
    1: "ping",
    2: "read",
    3: "write",
    4: "reg_write",
    5: "action",
    6: "reset",
    131: "sync_write",
}

ERROR_BITS = [
    ("voltage", 0),
    ("sensor", 1),
    ("temperature", 2),
    ("current", 3),
    ("overload", 5),
]

FIELDNAMES = [
    "packet_index",
    "direction",
    "source_start_index",
    "source_end_index",
    "elapsed_start_s",
    "elapsed_end_s",
    "id",
    "length",
    "kind",
    "instruction",
    "instruction_name",
    "error",
    "error_flags",
    "address",
    "address_name",
    "size",
    "value",
    "value_hex",
    "params_hex",
    "checksum",
    "checksum_ok",
    "packet_hex",
]

MAX_PACKET_LEN = 256


def load_sts_control_table() -> dict[str, tuple[int, int]]:
    tables_path = Path(__file__).resolve().parents[1] / "motors" / "feetech" / "tables.py"
    spec = importlib.util.spec_from_file_location("feetech_tables", tables_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load Feetech tables from {tables_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.STS_SMS_SERIES_CONTROL_TABLE


@dataclass
class Chunk:
    index: int
    elapsed_s: float
    data: bytes


@dataclass
class Packet:
    direction: str
    data: bytes
    start_index: int
    end_index: int
    start_elapsed_s: float
    end_elapsed_s: float


ADDRESS_NAMES = {address: name for name, (address, _size) in load_sts_control_table().items()}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("packet_log", type=Path, help="Packet JSONL log from a gripper experiment.")
    parser.add_argument(
        "--out",
        type=Path,
        help="Decoded CSV path. Defaults to '<packet_log>.decoded.csv'.",
    )
    return parser.parse_args()


def checksum(packet: bytes) -> int:
    return (~sum(packet[2:-1])) & 0xFF


def discard_prefix(buffer: bytearray, chunks: deque[Chunk], nbytes: int) -> None:
    del buffer[:nbytes]
    while nbytes and chunks:
        chunk = chunks.popleft()
        if len(chunk.data) > nbytes:
            chunks.appendleft(Chunk(chunk.index, chunk.elapsed_s, chunk.data[nbytes:]))
            break
        nbytes -= len(chunk.data)


def int_from_little_endian(data: bytes) -> int | None:
    if not data:
        return None
    return int.from_bytes(data, "little")


def error_flags(error: int | None) -> str:
    if error is None:
        return ""
    return "|".join(name for name, bit in ERROR_BITS if error & (1 << bit))


def append_complete_packets(
    packets: list[Packet],
    *,
    direction: str,
    buffer: bytearray,
    chunks: deque[Chunk],
) -> None:
    while True:
        header_at = buffer.find(b"\xff\xff")
        if header_at < 0:
            buffer.clear()
            chunks.clear()
            return
        if header_at:
            discard_prefix(buffer, chunks, header_at)

        if len(buffer) < 4:
            return

        packet_len = buffer[3] + 4
        if packet_len > MAX_PACKET_LEN:
            discard_prefix(buffer, chunks, 1)
            continue
        if len(buffer) < packet_len:
            return

        packet = bytes(buffer[:packet_len])
        if checksum(packet) != packet[-1]:
            discard_prefix(buffer, chunks, 1)
            continue

        packet_chunks: list[Chunk] = []
        remaining = packet_len
        while remaining and chunks:
            chunk = chunks.popleft()
            if len(chunk.data) <= remaining:
                packet_chunks.append(chunk)
                remaining -= len(chunk.data)
            else:
                packet_chunks.append(Chunk(chunk.index, chunk.elapsed_s, chunk.data[:remaining]))
                chunks.appendleft(Chunk(chunk.index, chunk.elapsed_s, chunk.data[remaining:]))
                remaining = 0

        del buffer[:packet_len]
        packets.append(
            Packet(
                direction=direction,
                data=packet,
                start_index=packet_chunks[0].index,
                end_index=packet_chunks[-1].index,
                start_elapsed_s=packet_chunks[0].elapsed_s,
                end_elapsed_s=packet_chunks[-1].elapsed_s,
            )
        )


def read_packets(path: Path) -> list[Packet]:
    packets: list[Packet] = []
    buffers = {"tx": bytearray(), "rx": bytearray()}
    chunks = {"tx": deque(), "rx": deque()}

    with path.open() as f:
        for line in f:
            row = json.loads(line)
            data = bytes.fromhex(row["hex"]) if row["hex"] else b""
            if not data:
                continue
            direction = row["direction"]
            buffers[direction].extend(data)
            chunks[direction].append(Chunk(row["index"], row["elapsed_s"], data))
            append_complete_packets(
                packets,
                direction=direction,
                buffer=buffers[direction],
                chunks=chunks[direction],
            )

    leftovers = {direction: bytes(buffer) for direction, buffer in buffers.items() if buffer}
    if leftovers:
        details = ", ".join(f"{direction}={data.hex(' ')}" for direction, data in leftovers.items())
        raise ValueError(f"Unparsed trailing bytes: {details}")
    return packets


def decode_tx(packet: Packet) -> dict[str, object]:
    data = packet.data
    instruction = data[4]
    params = data[5:-1]
    row: dict[str, object] = {
        "kind": "instruction",
        "instruction": instruction,
        "instruction_name": INSTRUCTIONS.get(instruction, f"unknown_{instruction}"),
        "error": "",
        "error_flags": "",
        "address": "",
        "address_name": "",
        "size": "",
        "value": "",
        "value_hex": "",
        "params_hex": params.hex(" "),
    }

    if instruction == 2 and len(params) >= 2:
        address, size = params[0], params[1]
        row.update(
            {
                "address": address,
                "address_name": ADDRESS_NAMES.get(address, ""),
                "size": size,
            }
        )
    elif instruction in (3, 4) and params:
        address = params[0]
        value_bytes = params[1:]
        row.update(
            {
                "address": address,
                "address_name": ADDRESS_NAMES.get(address, ""),
                "size": len(value_bytes),
                "value": int_from_little_endian(value_bytes),
                "value_hex": value_bytes.hex(" "),
            }
        )
    elif instruction == 131 and len(params) >= 2:
        address, size = params[0], params[1]
        row.update(
            {
                "address": address,
                "address_name": ADDRESS_NAMES.get(address, ""),
                "size": size,
            }
        )
    return row


def decode_rx(packet: Packet, request: dict[str, object] | None) -> dict[str, object]:
    data = packet.data
    error = data[4]
    params = data[5:-1]
    row: dict[str, object] = {
        "kind": "status",
        "instruction": "",
        "instruction_name": "",
        "error": error,
        "error_flags": error_flags(error),
        "address": "",
        "address_name": "",
        "size": len(params),
        "value": "",
        "value_hex": params.hex(" "),
        "params_hex": params.hex(" "),
    }

    if request and request.get("instruction_name") == "read":
        row.update(
            {
                "address": request.get("address", ""),
                "address_name": request.get("address_name", ""),
                "size": request.get("size", len(params)),
                "value": int_from_little_endian(params),
                "value_hex": params.hex(" "),
            }
        )
    return row


def base_row(index: int, packet: Packet) -> dict[str, object]:
    data = packet.data
    return {
        "packet_index": index,
        "direction": packet.direction,
        "source_start_index": packet.start_index,
        "source_end_index": packet.end_index,
        "elapsed_start_s": packet.start_elapsed_s,
        "elapsed_end_s": packet.end_elapsed_s,
        "id": data[2],
        "length": data[3],
        "checksum": data[-1],
        "checksum_ok": checksum(data) == data[-1],
        "packet_hex": data.hex(" "),
    }


def decode_packets(packets: list[Packet]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    pending_requests: deque[dict[str, object]] = deque()

    for index, packet in enumerate(packets):
        row = base_row(index, packet)
        if packet.direction == "tx":
            decoded = decode_tx(packet)
            row.update(decoded)
            if row["id"] != 254:
                pending_requests.append(decoded)
        else:
            request = pending_requests.popleft() if pending_requests else None
            row.update(decode_rx(packet, request))
        rows.append(row)

    return rows


def main() -> None:
    args = parse_args()
    out_path = args.out or args.packet_log.with_suffix(args.packet_log.suffix + ".decoded.csv")
    rows = decode_packets(read_packets(args.packet_log))

    with out_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)

    print(f"decoded_packets={len(rows)}")
    print(f"out={out_path}")


if __name__ == "__main__":
    main()
