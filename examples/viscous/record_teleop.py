"""Record the standard teleop loop without giving recording control over arm shutdown.

Launch through teleop.sh with record_teleop.sh as VISCOUS_TELEOP_BIN. Touch FINISH
in VISCOUS_RECORD_DIR to finish capture while teleop continues. Raw camera frames
and telemetry share host monotonic timestamps; camera timestamps are read-completion
times, not hardware exposure timestamps. Crop/export offline after capture.
"""

import json
import logging
import os
import queue
import threading
import time
from dataclasses import asdict
from pathlib import Path

import cv2

from lerobot.scripts import lerobot_teleoperate as teleop_script


class Capture:
    def __init__(self, root):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=False)
        self.stop = threading.Event()
        self.samples = queue.Queue(maxsize=6000)
        self.errors = []
        self.count = 0
        self.camera_counts = {0: 0, 2: 0}
        self.ready = {i: threading.Event() for i in self.camera_counts}
        self.threads = []

    def error(self, message):
        self.errors.append(message)
        logging.error("RECORDING ERROR (teleop continues): %s", message)

    def camera(self, index):
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        writer = None
        try:
            if not cap.isOpened():
                raise RuntimeError(f"camera {index} did not open")
            with (self.root / f"camera_{index}_timestamps.jsonl").open("w", buffering=1) as stamps:
                while not self.stop.is_set():
                    ok, frame = cap.read()
                    mono_ns, wall_ns = time.monotonic_ns(), time.time_ns()
                    if not ok:
                        raise RuntimeError(f"camera {index} read failed")
                    if writer is None:
                        h, w = frame.shape[:2]
                        writer = cv2.VideoWriter(
                            str(self.root / f"camera_{index}.avi"),
                            cv2.VideoWriter_fourcc(*"MJPG"),
                            25 if index == 0 else 5,
                            (w, h),
                        )
                        if not writer.isOpened():
                            raise RuntimeError(f"camera {index} video writer failed")
                    writer.write(frame)
                    stamps.write(
                        json.dumps(
                            {"frame": self.camera_counts[index], "monotonic_ns": mono_ns, "time_ns": wall_ns}
                        )
                        + "\n"
                    )
                    self.camera_counts[index] += 1
                    self.ready[index].set()
        except Exception as exc:
            self.error(str(exc))
        finally:
            cap.release()
            if writer is not None:
                writer.release()

    def status(self, state):
        temp = self.root / "status.tmp"
        temp.write_text(
            json.dumps(
                {
                    "state": state,
                    "samples": self.count,
                    "camera_frames": self.camera_counts,
                    "errors": self.errors,
                    "time_ns": time.time_ns(),
                },
                indent=2,
            )
        )
        temp.replace(self.root / "status.json")

    def write_samples(self):
        try:
            with (self.root / "samples.jsonl").open("w", buffering=1) as out:
                last_status = 0
                while not self.stop.is_set() or not self.samples.empty():
                    if (self.root / "FINISH").exists():
                        self.stop.set()
                    try:
                        sample = self.samples.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    out.write(json.dumps(sample) + "\n")
                    self.count += 1
                    if time.monotonic() - last_status > 1:
                        self.status("recording")
                        last_status = time.monotonic()
            for thread in self.threads[:2]:
                thread.join(timeout=3)
            if any(thread.is_alive() for thread in self.threads[:2]):
                self.error("A camera has not closed yet")
            self.status("finished" if not self.errors else "finished_with_errors")
        except Exception as exc:
            self.error(str(exc))
            self.stop.set()

    def start(self):
        for index in self.camera_counts:
            thread = threading.Thread(target=self.camera, args=(index,), daemon=True)
            self.threads.append(thread)
            thread.start()
        if not all(event.wait(10) for event in self.ready.values()) or self.errors:
            self.close()
            raise RuntimeError(f"Recording cameras not ready: {self.errors}")
        thread = threading.Thread(target=self.write_samples, daemon=True)
        self.threads.append(thread)
        thread.start()

    def enqueue(self, sample):
        if self.stop.is_set():
            return
        try:
            self.samples.put_nowait(sample)
        except queue.Full:
            self.error("Telemetry queue full; capture stopped to avoid blocking control")
            self.stop.set()

    def close(self):
        self.stop.set()
        for thread in self.threads:
            thread.join(timeout=4)


def instrument_loop(original_loop, capture, **kwargs):
    robot, leader = kwargs["robot"], kwargs["teleop"]
    capture.root.joinpath("config.json").write_text(
        json.dumps(
            {
                "robot": asdict(robot.config),
                "teleop": asdict(leader.config),
                "fps": kwargs["fps"],
                "camera_names": {"0": "wrist", "2": "external"},
            },
            default=str,
            indent=2,
        )
    )
    observe, action, send = robot.get_observation, leader.get_action, robot.send_action
    current = {}

    def observed():
        result = observe()
        current["observation"] = dict(result)
        current["observation_monotonic_ns"] = time.monotonic_ns()
        return result

    def acted():
        result = action()
        current["leader_action"] = dict(result)
        current["leader_raw"] = dict(leader._last_raw_positions)
        current["leader_monotonic_ns"] = time.monotonic_ns()
        return result

    def sent(command):
        result = send(command)
        capture.enqueue(
            {
                **current,
                "command": dict(command),
                "sent_action": dict(result),
                "monotonic_ns": time.monotonic_ns(),
                "time_ns": time.time_ns(),
                "synced": robot._synced,
            }
        )
        return result

    robot.get_observation, leader.get_action, robot.send_action = observed, acted, sent
    try:
        return original_loop(**kwargs)
    finally:
        robot.get_observation, leader.get_action, robot.send_action = observe, action, send


def main():
    capture = Capture(os.environ["VISCOUS_RECORD_DIR"])
    original_loop = teleop_script.teleop_loop
    try:
        capture.start()
        teleop_script.teleop_loop = lambda **kwargs: instrument_loop(original_loop, capture, **kwargs)
        teleop_script.main()
    finally:
        teleop_script.teleop_loop = original_loop
        capture.close()


if __name__ == "__main__":
    main()
