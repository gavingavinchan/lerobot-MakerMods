"""Experimental supervised reference controller; holds between commands and at episode end.

SIGINT/SIGTERM request hold, not torque release. release_parked is explicitly gated on
measured proximity to the folded initial pose. No calibration or leader connection.
Use only with an operator present. Shoulder-pan corrections may accumulate to 30 degrees,
in steps of at most 3 degrees; other joints retain a 10-degree cumulative correction limit.
All corrected targets must remain inside the commissioned joint limits.
"""

import argparse
import json
import logging
import math
import queue
import signal
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from lerobot.robots.viscous_follower import ViscousFollower, ViscousFollowerConfig

REFERENCE = Path("/home/gavin/Documents/robot-experiments/teleop-demo-20260915-182851")


def corrected_offsets(names, previous, changes):
    """Validate incremental corrections separately from cumulative workspace adaptation."""
    new = previous.copy()
    for name, value in changes.items():
        index = names.index(name)
        value = float(value)
        limit = 30.0 if name == "shoulder_pan.pos" else 10.0
        if not math.isfinite(value) or abs(value - new[index]) > 3.01 or abs(value) > limit:
            raise ValueError(f"{name}: change must be <=3 degrees, total <={limit:g}")
        new[index] = value
    return new


def target_reached(targets, desired, lower, upper):
    """Report command interpolation completion against the target actually allowed to be sent.

    This does not establish measured physical arrival or task success.
    """
    return bool(np.max(np.abs(targets - np.clip(desired, lower, upper))) < 0.01)


class Reference:
    def __init__(self):
        self.rows = [json.loads(line) for line in (REFERENCE / "samples.jsonl").read_text().splitlines()]
        origin = self.rows[0]["leader_monotonic_ns"]
        self.times = np.array([(row["leader_monotonic_ns"] - origin) / 1e9 for row in self.rows])
        self.names = list(self.rows[0]["sent_action"])
        self.values = np.array([[row["sent_action"][key] for key in self.names] for row in self.rows])

    def pose(self, sec):
        if not math.isfinite(sec) or not 8.19 <= sec <= 17.9:
            raise ValueError("Reference time outside inspected 8.19–17.9 second interval")
        return np.array([np.interp(sec, self.times, self.values[:, j]) for j in range(len(self.names))])


def main(out):
    logging.basicConfig(level=logging.INFO)
    out.mkdir(parents=True, exist_ok=False)
    ref = Reference()
    cfg = ViscousFollowerConfig(
        port=json.loads((REFERENCE / "config.json").read_text())["robot"]["port"], id="viscous_01"
    )
    follower = ViscousFollower(cfg)
    if not follower.is_calibrated:
        raise RuntimeError("Existing follower calibration required")
    freeze, done = threading.Event(), threading.Event()
    signal.signal(signal.SIGINT, lambda *_: freeze.set())
    signal.signal(signal.SIGTERM, lambda *_: freeze.set())
    camera_errors = []
    camera_times = {0: 0.0, 2: 0.0}
    ready = {i: threading.Event() for i in camera_times}
    commands, statuses = queue.Queue(), queue.Queue(maxsize=1000)

    def camera(index):
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        writer = None
        try:
            if not cap.isOpened():
                raise RuntimeError(f"camera {index} unavailable")
            last_snapshot = 0
            with (out / f"camera_{index}_timestamps.jsonl").open("w", buffering=1) as log:
                while not done.is_set():
                    ok, frame = cap.read()
                    if not ok:
                        raise RuntimeError(f"camera {index} failed")
                    now = time.monotonic()
                    if writer is None:
                        h, w = frame.shape[:2]
                        writer = cv2.VideoWriter(
                            str(out / f"camera_{index}.avi"),
                            cv2.VideoWriter_fourcc(*"MJPG"),
                            25 if index == 0 else 5,
                            (w, h),
                        )
                        if not writer.isOpened():
                            raise RuntimeError("video writer unavailable")
                    writer.write(frame)
                    log.write(
                        json.dumps({"monotonic_ns": time.monotonic_ns(), "time_ns": time.time_ns()}) + "\n"
                    )
                    camera_times[index] = now
                    if now - last_snapshot >= 0.2:
                        temp = out / f"camera_{index}_next.jpg"
                        if not cv2.imwrite(str(temp), frame):
                            raise RuntimeError("snapshot write failed")
                        temp.replace(out / f"camera_{index}_latest.jpg")
                        last_snapshot = now
                    ready[index].set()
        except Exception as exc:
            camera_errors.append(str(exc))
            freeze.set()
            logging.exception("Camera failure; requesting hold")
        finally:
            cap.release()
            if writer is not None:
                writer.release()

    def io_worker():
        previous = None
        try:
            with (out / "telemetry.jsonl").open("w", buffering=1) as log:
                while not done.is_set():
                    try:
                        s = statuses.get(timeout=0.04)
                        log.write(json.dumps(s) + "\n")
                        temp = out / "status.next.json"
                        temp.write_text(json.dumps(s, indent=2))
                        temp.replace(out / "status.json")
                    except queue.Empty:
                        pass
                    path = out / "command.json"
                    if path.exists():
                        try:
                            command = json.loads(path.read_text())
                        except (OSError, ValueError):
                            continue
                        if command.get("id") is not None and command["id"] != previous:
                            previous = command["id"]
                            commands.put(command)
        except Exception:
            freeze.set()
            camera_errors.append("Recording/control file worker failed")
            logging.exception("File worker failed")

    threads = [threading.Thread(target=camera, args=(i,), daemon=True) for i in (0, 2)]
    for thread in threads:
        thread.start()
    if not all(e.wait(10) for e in ready.values()) or camera_errors:
        done.set()
        raise RuntimeError(f"Cameras not ready: {camera_errors}")
    threading.Thread(target=io_worker, daemon=True).start()
    follower.connect(calibrate=False)
    obs = follower.get_observation()
    baseline = np.array([obs[n] for n in ref.names])
    lower = np.array([cfg.joint_limits[n.removesuffix(".pos")][0] for n in ref.names])
    upper = np.array([cfg.joint_limits[n.removesuffix(".pos")][1] for n in ref.names])
    targets = np.clip(baseline, lower, upper)
    (out / "baseline.json").write_text(
        json.dumps({"observation": obs, "names": ref.names, "target": targets.tolist()}, indent=2)
    )
    offset = np.zeros(len(ref.names))
    ref_time = None
    segment = None
    last_id = None
    result = "holding initial pose"
    last_status = 0
    last_tick = time.monotonic()
    print("READY: holding current folded pose; cameras recording; no automatic motion", flush=True)
    while True:
        tick = time.monotonic()
        dt = min(0.04, max(0.001, tick - last_tick))
        last_tick = tick
        if freeze.is_set() or camera_errors or any(tick - v > 1.0 for v in camera_times.values()):
            segment = None
            result = "hold requested or camera unavailable"
            freeze.clear()
        try:
            command = commands.get_nowait()
        except queue.Empty:
            command = None
        if command is not None:
            last_id = command["id"]
            try:
                if command.get("hold"):
                    segment = None
                    result = "holding"
                elif command.get("release_parked"):
                    deviation = float(np.max(np.abs(np.array([obs[n] for n in ref.names]) - baseline)))
                    if segment is not None or deviation > 3.5:
                        raise ValueError(f"Not at initial rest: {deviation:.2f} degrees")
                    follower.disconnect()
                    done.set()
                    for thread in threads:
                        thread.join(timeout=3)
                    print("Released at verified initial rest pose", flush=True)
                    return
                elif segment is not None:
                    raise ValueError("Wait until the previous movement finishes")
                elif camera_errors or any(tick - v > 1.0 for v in camera_times.values()):
                    raise ValueError("Camera recording unavailable")
                elif "pose_time" in command:
                    sec = float(command["pose_time"])
                    goal = ref.pose(sec) + offset
                    if np.max(np.abs(goal - targets)) > 30:
                        raise ValueError("Initial pose transition too large")
                    segment = {
                        "kind": "pose",
                        "start": tick,
                        "duration": max(2.0, float(command.get("duration", 2))),
                        "from": targets.copy(),
                        "to": goal,
                        "time": sec,
                    }
                    result = "moving to reference start"
                elif "until" in command:
                    if ref_time is None:
                        raise ValueError("Align with the reference first")
                    end = float(command["until"])
                    ref.pose(end)
                    speed = float(command.get("speed", 0.6))
                    if not 0.1 <= speed <= 0.8:
                        raise ValueError("Speed must be between 0.1 and 0.8 of demonstration")
                    segment = {"kind": "reference", "end": end, "speed": speed}
                    result = "following reference segment"
                elif "offset" in command:
                    if ref_time is None:
                        raise ValueError("Align with reference first")
                    new = corrected_offsets(ref.names, offset, command["offset"])
                    goal = ref.pose(ref_time) + new
                    if np.any(goal < lower) or np.any(goal > upper):
                        raise ValueError("Offset exceeds joint limits")
                    segment = {
                        "kind": "offset",
                        "start": tick,
                        "duration": max(1.0, float(command.get("duration", 1))),
                        "from": targets.copy(),
                        "to": goal,
                        "offset": new,
                    }
                    result = "applying bounded adjustment"
                else:
                    raise ValueError("Unknown command")
            except (ValueError, TypeError, KeyError) as exc:
                result = f"REJECTED: {exc}"
            print(last_id, result, flush=True)
        if segment is not None:
            if segment["kind"] == "reference":
                delta = segment["end"] - ref_time
                next_time = ref_time + np.clip(delta, -dt * segment["speed"], dt * segment["speed"])
                desired = ref.pose(next_time) + offset
            else:
                u = min(1.0, (tick - segment["start"]) / segment["duration"])
                u = u * u * (3 - 2 * u)
                desired = segment["from"] + (segment["to"] - segment["from"]) * u
            targets += np.clip(desired - targets, -60 * dt, 60 * dt)
            targets = np.clip(targets, lower, upper)
            if segment["kind"] == "reference":
                ref_time = float(next_time)
                completed = abs(ref_time - segment["end"]) < 1e-6 and target_reached(
                    targets, desired, lower, upper
                )
            else:
                completed = u >= 1 and target_reached(targets, desired, lower, upper)
            if completed:
                if segment["kind"] == "pose":
                    ref_time = segment["time"]
                if segment["kind"] == "offset":
                    offset = segment["offset"]
                segment = None
                result = "segment complete; holding"
                print(last_id, result, flush=True)
        sent = follower.send_action(dict(zip(ref.names, targets.tolist(), strict=True)))
        obs = follower.get_observation()
        if tick - last_status > 0.1:
            status = {
                "time_ns": time.time_ns(),
                "monotonic_ns": time.monotonic_ns(),
                "reference_time_s": ref_time,
                "moving": segment is not None,
                "last_command": last_id,
                "result": result,
                "targets": dict(zip(ref.names, targets.tolist(), strict=True)),
                "offsets": dict(zip(ref.names, offset.tolist(), strict=True)),
                "sent_action": sent,
                "observation": obs,
                "camera_errors": camera_errors,
                "max_temperature": max(v for k, v in obs.items() if k.endswith(".temp")),
            }
            try:
                statuses.put_nowait(status)
            except queue.Full:
                freeze.set()
                camera_errors.append("Telemetry queue full")
            last_status = tick
        time.sleep(max(0, 1 / 60 - (time.monotonic() - tick)))


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    main(args.output)
