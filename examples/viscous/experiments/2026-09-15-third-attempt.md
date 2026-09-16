# 2026-09-15 — third requested orange-ball pick and place

**Outcome: unsuccessful.** Two empty closures across the initial attempt and continuation;
no verified pickup, lift, or deposit. The operator stopped the attempts, then requested
immediate torque release. The controller was terminated; watchdog release is expected but
was not independently measured. No calibration occurred. The initial portion below ended
in a powered folded hold; the continuation and shutdown are recorded afterward.

## Records and sequence

Evidence: `/home/gavin/Documents/robot-experiments/reference-pick-place/adaptive_run_01/third_run_06/`.
Contains requested commands, timestamped telemetry snapshots, paired wrist/external JPEGs,
`episode_telemetry.jsonl`, and `result.json`. Continuous camera AVIs and timestamp sidecars
remain in the parent session. The existing `reference_controller.py` remained the sole
hardware owner, using `viscous_01` and its existing calibration/camera configuration.

Initial state: folded at raw reference 17.9 s, pan offset +10.5 degrees, other offsets
zero, maximum temperature 41 C. In the external 640x480 image the ball was approximately
(354, 298), with the foam to its right and rotated relative to the previous success.

The first approach reached reference 12.0 s at pan +10.5. The ball was behind the fingers
and absent from the wrist image. Reversing to 11.0 s and local corrections did not establish
a grasp. At pan +10.5, shoulder +6, elbow -6, wrist -6, part of the ball appeared at the
wrist image's right edge. The elbow reached 60 C; return peaked at 64 C. This initial
approach/return lasted 85.41 s, approximately 17.65 s sampled moving.

After cooling, a second approach started at 44 C. Pan increased while folded in <=3-degree
steps to +30. At reference 12.0 s with shoulder +6, elbow -6, wrist -6, the ball bounding
box was 170x89 pixels, clipped at the bottom edge, still too far back between the fingers.
Pan +27 / elbow -9 preceded closure to 12.5 s. The gripper measured -75.53 degrees against
a -75.49-degree target, with -0.01 Nm torque; the external view showed no capture.
The ball moved closer to the arm.

Reopened at 12.0 s and corrected locally: shoulder/wrist offsets reached +9/-9, and elbow
stepped from -9 to 0. The ball remained under the gripper, out of the wrist view. Subsequent
reference 10.5 s telemetry reported 67 C, prompting return. Offset removal and folding
peaked at 68 C. Final hold: reference 17.9 s, pan +27, other offsets zero. A later result
snapshot was 56 C and cooling. The final external image shows the ball farther down the
table, outside the foam; these checkpoints do not establish every displacement's cause.

Saved controller wall-time interval: 1789525359219503416–1789525820228510703 ns,
461.01 s including cooling, approximately 51.56 s sampled moving, peak 68 C. Moving flags
describe interpolation, not physical arrival; other time includes cooling, settling,
tool/decision gaps, and holding. Camera recordings continued after this saved interval.

## Lessons and limits

- The previous two successful scenes do not establish success for this closer ball and
  relocated deposit target. No hardware reach limit was established.
- A ball clipped by the wrist-image boundary does not supply a full-ball size comparison.
  Resolve whether it is behind the fingers using the external view before closing. This
  attempt closed too early and missed.
- Image-coordinate changes alone did not identify the required correction direction.
  Simultaneous corrections and contact prevent an isolated joint-to-distance conclusion;
  the attempted elbow corrections did not solve the near-ball geometry.
- Checkpoint-only thermal monitoring was inadequate. Later temperatures rose from 51 C
  to 67 C between checkpoints. The retry did not unload promptly at the warning threshold.
  An active monitor with a validated return path is needed before repeating this procedure.
- Independent closure at an inspected arm pose remains unimplemented in the live controller.
  Reference closure changes other joints; limited gripper offsets do not provide general
  open/close control at arbitrary phases.

See [TODO.md](../TODO.md). This records failure evidence, not a third successful episode
or a validated improved controller.

## Continuation after the operator reset the ball; stopped by operator

Further evidence is in sibling `reset_trial_07/`, including `result.json`, extracted
`episode_telemetry.jsonl`, paired checkpoints, and `user_stop.json`. The operator reiterated
that misses must be retried locally and asked whether the experience guide had been read.
The operator then explicitly requested stopping. No successful pickup or placement occurred.

An external command watcher (`reference-pick-place/guarded_trial.py`) serialized commands
and monitored idle checkpoints at 0.1 s intervals. It triggered returns at 55 C, resetting
non-pan offsets in <=3-degree steps, reversing to 8.2 s, then aligning folded at 17.9 s.
The first observed return completed in 10.03 s and that interval peaked at 55 C. This is
limited evidence for these empty-gripper poses, not validation of arbitrary parking,
return with an object, exception handling, or torque release. A shell-session watcher later
exited (143); its stale heartbeat blocked a new approach before motion. A detached watcher
was then launched. It was explicitly stopped after the user's stop request.

At pan +30, shoulder -10, elbow +6, wrist flex +10, wrist yaw -9 and reference 12.0 s,
the reset ball appeared at approximately (295, 352), full box 145x141 px. One approach
returned on temperature before closure. Applying these offsets while folded reduced the
next pregrasp temperature from 55 C to 50 C in these two nonidentical trials; this is not
a controlled performance comparison. A scene-specific image gate then allowed closure.
The gripper measured about -75.44 degrees with -0.06 Nm against -75.49 commanded: empty.
The paired external images show the ball moving outward during closure. Matching apparent
ball position/size did not establish capture in this changed gripper orientation. A later
live image showed the operator's hand by the foam; no further task motion was requested.
The thermal watcher returned folded again.

On the user's stop request, a fresh completed folded state was verified, the watcher was
stopped, and the hardware controller acknowledged an explicit hold command. Final reference
17.9 s, pan offset +30 degrees, all other offsets zero; maximum temperature 46 C and cooling.
Torque and the original controller/camera recording remain enabled. The final external
image shows the ball outside the foam. No further attempts are authorized by this request.

### Subsequent explicit torque-release request

The operator then requested torque release and reiterated immediate release. A fresh external
image showed a hand supporting the arm. The controller's initial-rest release gate could not
match the current folded pose (about 27 degrees pan and 22 degrees shoulder discrepancy).
After the reiterated request, controller PID 90072 was terminated with SIGUSR1, after verifying
its identity and that this signal had the default, unhandled disposition. INT/TERM in this
controller only request hold. Process exit was verified and more than the documented 200 ms
motor-watchdog interval elapsed. Torque release is expected from that watchdog; no independent
motor telemetry confirmed it afterward. This was not a normal `release_parked` disconnect.
The controller and camera recording are no longer running; normal video finalization did not
occur. `reset_trial_07/operator_release.json` records the action and its limits.
