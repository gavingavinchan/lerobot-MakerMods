# 2026-09-15 — review of reference-guided orange-ball pick and place

**Outcome: one supervised pickup and deposit into the black foam opening.** Reviewed the
saved result, controller source, telemetry, and timestamp-aligned wrist/external frames.
This review performed no hardware actions. The original report also records operator
confirmation. Model identity was supplied by the operator; the inspected artifacts do not
independently establish which model ran the attempt.

## Sources and review boundary

- [Original report](/home/gavin/Documents/robot-experiments/reference-pick-place/run_02/README.md)
- [Telemetry](/home/gavin/Documents/robot-experiments/reference-pick-place/run_02/telemetry.jsonl)
- [Saved result](/home/gavin/Documents/robot-experiments/reference-pick-place/run_02/result.json)
- [Demonstration](/home/gavin/Documents/robot-experiments/teleop-demo-20260915-182851/README.md)
- [Final external image](/home/gavin/Documents/robot-experiments/reference-pick-place/run_02/camera_2_success.jpg)
- [Controller source](/home/gavin/Documents/robot-experiments/reference-pick-place/controller.py)

Statistics use the first 3,186 telemetry rows through `result.json.time_ns =
1789520607033862671`, spanning 319.54 s. Later recording is excluded. Elapsed times below
are relative to first telemetry `monotonic_ns = 69446798290445`; reference times are relative
to the original demonstration's first leader sample. Telemetry is sampled at about 10 Hz,
so these figures do not capture every control-cycle transient.

## Reconstructed sequence

| Elapsed time (s) | Raw reference time (s) | Recorded event |
| --- | --- | --- |
| 18.76 | 8.2 | Alignment segment completed |
| 34.01 | 10.3 | Approach/open segment completed |
| 51.97 | 11.6 | Pregrasp segment completed |
| 107.35 | 11.6 | First correction completed: total pan +1.5°, shoulder lift +1° |
| 135.64 | 11.6 | Second correction completed: total pan −1.5°, shoulder lift +2.5° |
| 154.71 | 12.65 | Grasp segment completed |
| 173.67 | 13.8 | Lift/transfer segment completed |
| 191.52 | 14.6 | Next segment completed; external image at 200 s shows deposited ball |
| 209.18 | 14.61 | Return segment started |
| 257.94 | 17.9 | Explicit hold ended the return's stuck moving state |

Wrist frames at approximately 100, 120, and 145 s show the original alignment, its worsening
after the first correction, and improvement after the second. External frames at approximately
160, 180, and 200 s show ball capture, transfer toward the foam, and deposit. The foam changes
position during the sequence. Segment names alone were not used to establish success.

To reproduce the visual checks, select the nearest sidecar `monotonic_ns` to first telemetry
plus each elapsed time, then decode that numbered frame from the matching AVI. Frames used
were wrist 2520/3020/3645 and external 801/901/1001 (zero-based). Sequential decoding was used
because random seeking in the unfinalized external AVI returned an incorrect image.

## Findings beyond the original summary

- **Waiting dominated pregrasp.** From pregrasp completion to starting grasp was 101.33 s;
  approximately 2.4 s applied the two corrections. The remaining roughly 99 s was holding
  between commands. This is a concrete opportunity to improve execution time and thermal duty.
- **The elbow was the hottest joint.** Sampled peak temperatures: pan 32 °C, shoulder lift
  50 °C, elbow 80 °C, wrist flex 41 °C, wrist yaw/roll 36 °C, gripper 42 °C. Elbow temperature
  started at 35 °C and was 47 °C in the completion snapshot. It rose from about 70 to 80 °C
  during the roughly 17 s hold after the lift/transfer segment. This supports reducing loaded
  pauses; it does not establish a universal safe hold duration.
- **Position differences need contact context.** Maximum sampled absolute position differences
  from `sent_action` were 9.19° at shoulder lift and 9.99° at elbow. These are peak sampled
  errors across the reviewed window, not steady-state accuracy or a measured Cartesian error.
  In the subsequent review, the operator noted that tabletop contact could explain these
  differences and can help align the gripper. Table contact is accepted in this task; the
  reviewed peaks have not been attributed to contact versus sag or other causes. Their
  magnitude alone does not establish faulty tracking or task failure.
- **The return completion bug is visible in source.** Targets were clipped to joint limits,
  but completion compared them with the unclipped desired target. The carried shoulder offset
  made that comparison unreachable at the return endpoint. Offsets useful near grasp must also
  be checked over transfer and return; they are not automatically valid for the full trajectory.
- **Successful placement did not mean clean shutdown.** The saved result says folded and
  holding with torque on. `release_parked` checks against an initial rest with a substantially
  different shoulder angle. No validated transition to torque release was demonstrated.

The retained operating lessons and future episode-recording procedure are in
[EXPERIENCE.md](../EXPERIENCE.md). Implementation work remains in [TODO.md](../TODO.md).
