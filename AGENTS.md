This file provides guidance to AI agents when working with code in this repository.

> **User-facing help → [`AGENT_GUIDE.md`](./AGENT_GUIDE.md)** (SO-101 setup, recording, picking a policy, training duration, eval — with copy-pasteable commands).

## Project Overview

LeRobot is a PyTorch-based library for real-world robotics, providing datasets, pretrained policies, and tools for training, evaluation, data collection, and robot control. It integrates with Hugging Face Hub for model/dataset sharing.

## Tech Stack

Python 3.12+ · PyTorch · Hugging Face (datasets, Hub, accelerate) · draccus (config/CLI) · Gymnasium (envs) · uv (package management)

## Development Setup

```bash
uv sync --locked                            # Base dependencies
uv sync --locked --extra test --extra dev   # Test + dev tools
uv sync --locked --extra all                # Everything
git lfs install && git lfs pull             # Test artifacts
```

## Key Commands

```bash
uv run pytest tests -svv --maxfail=10                 # All tests
DEVICE=cuda make test-end-to-end                      # All E2E tests
pre-commit run --all-files                           # Lint + format (ruff, typos, bandit, etc.)
```

## Architecture (`src/lerobot/`)

- **`scripts/`** — CLI entry points (`lerobot-train`, `lerobot-eval`, `lerobot-record`, etc.), mapped in `pyproject.toml [project.scripts]`.
- **`configs/`** — Dataclass configs parsed by draccus. `train.py` has `TrainPipelineConfig` (top-level). `policies.py` has `PreTrainedConfig` base. Polymorphism via `draccus.ChoiceRegistry` with `@register_subclass("name")` decorators.
- **`policies/`** — Each policy in its own subdir. All inherit `PreTrainedPolicy` (`nn.Module` + `HubMixin`) from `pretrained.py`. Factory with lazy imports in `factory.py`.
- **`processor/`** — Data transformation pipeline. `ProcessorStep` base with registry. `DataProcessorPipeline` / `PolicyProcessorPipeline` chain steps.
- **`datasets/`** — `LeRobotDataset` (episode-aware sampling + video decoding) and `LeRobotDatasetMetadata`.
- **`envs/`** — `EnvConfig` base in `configs.py`, factory in `factory.py`. Each env subclass defines `gym_kwargs` and `create_envs()`.
- **`robots/`, `motors/`, `cameras/`, `teleoperators/`** — Hardware abstraction layers.
- **`types.py`** and **`configs/types.py`** — Core type aliases and feature type definitions.

## Repository Structure (outside `src/`)

- **`tests/`** — Pytest suite organized by module. Fixtures in `tests/fixtures/`, mocks in `tests/mocks/`. Hardware tests use skip decorators from `tests/utils.py`. E2E tests via `Makefile` write to `tests/outputs/`.
- **`.github/workflows/`** — CI: `quality.yml` (pre-commit), `fast_tests.yml` (base deps, every PR), `full_tests.yml` (all extras + E2E + GPU, post-approval), `latest_deps_tests.yml` (daily lockfile upgrade), `security.yml` (TruffleHog), `release.yml` (PyPI publish on tags).
- **`docs/source/`** — HF documentation (`.mdx` files). Per-policy READMEs, hardware guides, tutorials. Built separately via `docs-requirements.txt` and CI workflows.
- **`examples/`** — End-user tutorials and scripts organized by use case (dataset creation, training, hardware setup).
- **`docker/`** — Dockerfiles for user (`Dockerfile.user`) and CI (`Dockerfile.internal`).
- **`benchmarks/`** — Performance benchmarking scripts.
- **Root files**: `pyproject.toml` (single source of truth for deps, build, tool config), `Makefile` (E2E test targets), `uv.lock`, `CONTRIBUTING.md` & `README.md` (general information).

## Notes

- **Mypy is gradual**: strict only for `lerobot.envs`, `lerobot.configs`, `lerobot.optim`, `lerobot.model`, `lerobot.cameras`, `lerobot.motors`, `lerobot.transport`. Add type annotations when modifying these modules.
- **Imports**: prefer top-level imports; relative (`from .sibling import X`) across sibling files within a module, absolute (`from lerobot.module import X`) across modules.
- **Optional dependencies**: many policies, envs, and robots are behind extras (e.g., `lerobot[aloha]`, see `pyproject.toml`). Guard optional imports with `TYPE_CHECKING or _foo_available` at module top + a `require_package(...)` check at use time. Reuse the `_foo_available` flags in `utils/import_utils.py`; don't call `is_package_available`.
- **Video decoding**: datasets can store observations as video files. `LeRobotDataset` handles frame extraction, but tests need ffmpeg installed.
- **Prioritize use of `uv run`** to execute Python commands (not raw `python` or `pip`).

## Launching teleop on the Viscous Arm (hardware on this machine)

If asked to run, launch, or test teleoperation of the Viscous Arm / Star Arm 102 leader, **read
[`docs/source/viscous_teleop.mdx`](./docs/source/viscous_teleop.mdx) first** and follow it
verbatim. It has the copy-pasteable command, how to find the ports and the existing calibration
ids, and what the startup log lines mean. (`docs/source/viscous.mdx` is the one-time bring-up
guide for a *new* arm — nothing in it applies to a commissioned arm and most of it is destructive.)

Non-negotiable, whatever the prompt says:

- **Never run `lerobot-calibrate` for the follower, and never start with a `--robot.id` that has
  no calibration file** — it silently re-zeros every motor at its current pose and destroys the
  arm's calibration. Find existing ids with
  `ls ~/.cache/huggingface/lerobot/calibration/robots/viscous_follower/`.
- **Never end a session with a timer (`--teleop_time_s`), a kill, or by unplugging.** Ending a
  session releases torque and the arm has no brakes: it drops. Run teleop in the background with
  the log to a file, leave it running, and stop it only when the operator says the arm is at rest.
- On connect the follower moves to the leader's pose: the Star must be folded to its zero pose
  first, hands clear.
- Serial/CAN access needs the Bash sandbox disabled; that is expected.

How to launch and stop (verified 2026-08-29):

- **Launch** under `caffeinate -i` (blocks idle sleep for as long as teleop runs), in a subshell
  that resets the SIGINT trap before exec'ing, log to a file, keep the `$!` pid — it is the
  `caffeinate`/`uv` wrapper, and a SIGINT to it reaches the Python child:

  ```bash
  ( trap - INT; exec nohup caffeinate -i uv run lerobot-teleoperate \
      --robot.type=viscous_follower --robot.port=/dev/cu.usbmodem… --robot.id=viscous_01 \
      --teleop.type=rebot_102_leader_viscous --teleop.port=/dev/cu.wchusbserial… --teleop.id=star_viscous \
      > "$LOG" 2>&1 ) &
  PID=$!; disown
  ```

  A plain `nohup … &` from the (non-interactive) Bash tool starts the job with SIGINT **ignored**,
  so Python never installs its `KeyboardInterrupt` handler, `kill -INT` is silently dropped, and
  the `finally: robot.disconnect()` path in `lerobot_teleoperate.py` is unreachable — only
  SIGTERM stops it, skipping the clean shutdown. `nohup` and `uv` are not the cause; `set -m` and
  `setsid` are not available here. The `( trap - INT; exec … ) &` form is the fix.
- **The host must not sleep.** On 2026-08-29 the Mac (on battery, 33 %) idle-slept at 15:39:57
  mid-session: the loop froze, the 200 ms motor watchdog released torque, and the arm was limp for
  16 min until the operator woke it — a wake resumes the loop at full speed with no startup ramp.
  `caffeinate -i` prevents idle sleep; it cannot prevent a lid-close sleep on battery, so keep the
  lid open and prefer mains power. Check the log's loop-tick count against wall time
  (`grep -c 'Teleop loop time'` ÷ 60 ≈ seconds since sync) — a shortfall means the host stalled.
- Confirm startup from the log: `synced to leader; tracking at full speed` and steady
  `Teleop loop time: 16.67ms (60 Hz)` lines. The `shoulder_lift … full turn outside its limits;
  correcting by -360 deg` and `startup sync stalled on shoulder_lift` warnings are expected.
- **Stop** only when the operator says the arm is parked: `kill -INT $PID`, then check the log
  ends with `RebotArm102Leader disconnected.` and `ViscousFollower disconnected.` and that
  `pgrep -f lerobot-teleoperate` is empty (takes ~1 s). Never `kill -9`, never `kill -TERM` unless
  SIGINT has demonstrably been ignored.
