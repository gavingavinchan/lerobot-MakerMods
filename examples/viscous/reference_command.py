"""Send one staged command to an existing controller, without connecting to hardware."""

import argparse
import json
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("command", help="JSON object with a unique command id")
    args = parser.parse_args()
    command = json.loads(args.command)
    if not isinstance(command.get("id"), str) or not command["id"]:
        raise ValueError("A nonempty unique command id is required")
    status = json.loads((args.run / "status.json").read_text())
    if time.time_ns() - status["time_ns"] > 2_000_000_000 or status["camera_errors"]:
        raise RuntimeError("Controller status or cameras unavailable")
    if status["moving"] and not command.get("hold"):
        raise RuntimeError("Previous movement is still active")
    if status["last_command"] == command["id"]:
        raise ValueError("Use a new command id")
    temp = args.run / "command.next.json"
    temp.write_text(json.dumps(command))
    temp.replace(args.run / "command.json")
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        status = json.loads((args.run / "status.json").read_text())
        if time.time_ns() - status["time_ns"] > 2_000_000_000:
            raise RuntimeError("Stale controller status; inspect controller, do not force termination")
        if status["last_command"] == command["id"] and not status["moving"]:
            if status["result"] not in {"segment complete; holding", "holding"}:
                raise RuntimeError(status["result"])
            time.sleep(0.25)
            print(
                json.dumps(
                    {
                        "reference_time_s": status["reference_time_s"],
                        "result": status["result"],
                        "max_temperature": status["max_temperature"],
                        "offsets": status["offsets"],
                    }
                )
            )
            return
        time.sleep(0.05)
    raise RuntimeError("Command still active; inspect controller status")


if __name__ == "__main__":
    main()
