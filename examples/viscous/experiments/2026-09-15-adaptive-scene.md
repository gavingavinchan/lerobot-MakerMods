# 2026-09-15 — adaptation to a changed ball and foam position

**Current outcome: no successful capture or deposit.** The operator intentionally moved
the ball and rotated/repositioned the foam to test adaptation. Matching the original scene
was not the objective; the operator explicitly corrected the suggestion to reset it.

## Records

- Initial bounded attempt:
  `/home/gavin/Documents/robot-experiments/reference-pick-place/repeat_20260915/`.
  Its baseline/end markers refer into the old `run_02` continuous camera/telemetry recording.
  Pan corrections reached +9°; the ball remained outside the grasp area and rolled slightly
  from contact. Returned along the inspected reference to folded, without capture.
- Replacement controller session:
  `/home/gavin/Documents/robot-experiments/reference-pick-place/adaptive_run_01/`.
  `first_capture_result.json` records the failed capture and final state. Camera/telemetry
  recording continues; this result is historical, not live status.
- `reach_probe/` within that session contains before/after images and status snapshots for
  brief single-joint probes, followed by a return to folded before image analysis.

## Controller handoff

The old controller's cumulative 10° offset limit was a software restriction, not a measured
workspace boundary. The replacement example permits cumulative shoulder-pan corrections
up to 30° while retaining <=3° changes per command and commissioned joint-limit checks.
Other joints retain the original 10° cumulative correction bound.

The old process could not reload code, and SIGINT/SIGTERM only requested hold. The operator
supported the folded arm during a measured transition near raw reference 16.3 s with bounded
offsets toward the original baseline. Early readings failed the release check; no release
was issued on those readings. After refinement to reference 16.26 s and 1.5 s of stable
measured proximity, the controller logged `Released at verified initial rest pose` and exited.
Only then was the replacement connected. This is an operator-assisted handoff, not a proven
automatic parking routine. The initial shell-background launch did not persist; the successful
replacement launch used a detached process and verified fresh status before operation.

The replacement also compares segment completion with the clipped command target. Its return
to reference 8.2 s with a +3° shoulder offset completed instead of getting stuck against the
upper shoulder limit. This verifies command bookkeeping, not physical task success.

## Measured local response

At pregrasp reference 11.6 s with +15° pan, the wrist-image orange-ball centroid was estimated
by HSV segmentation. Each probe commanded +3° on one joint, restored it, and saved a status
snapshot and both views. Values below are approximate pixels and measured joint angles:

| Probe | Ball centroid (x, y) | Relevant measured joint angle |
| --- | --- | --- |
| Baseline | (370.3, 254.9) | Elbow 26.17°, wrist flex 10.19°, shoulder lift −68.72° |
| Elbow +3° target | (367.3, 299.9) | Elbow 28.28° |
| Wrist flex +3° target | (369.2, 286.1) | Wrist flex 12.72° |
| Shoulder lift +3° target | (368.8, 281.8) | Shoulder lift −66.08° |

These are local responses in this pose and scene, not a Cartesian calibration. Actual joint
changes differed from targets; baseline was not recaptured before every probe, so settling
and residual motion limit isolated-effect estimates. Probe images remain available for review.

## Failed capture and lessons

The next approach used +18° pan and +9° elbow relative to the reference. The ball appeared
between the jaws in the wrist image, but following the grasp/transfer segments did not lift
or deposit it. The ball remained outside the foam. The operator clarified that the fingers
started below the ball, then rose before securing it. Their earlier shorthand that the fingers
were above the ball described the later condition, not an initially too-high approach.

Telemetry supports a sequencing failure: while the elbow correction increased to +9°, the
gripper stayed open at a sent target of −106.70° (measured −106.92° at correction completion).
The subsequent grasp segment did command closure to −72.84°, but that did not capture the
ball. At its first completed sample, the gripper measured −72.85° with reported torque
0.04 Nm. In the successful run, the same sent closure target produced measured −79.77° and
2.42 Nm at the corresponding completed sample, consistent with object resistance.
These individual readings are useful comparative evidence, not general grasp-detection
thresholds or proof of contact by themselves.

The correction is to secure the ball before raising/transferring, rather than merely lower
the next starting pose. Once jaws surround the ball at the appropriate height, hold the arm
pose, close, check visual retention together with gripper position/torque response, then lift.
The current reference-based tool couples multiple joints during a grasp segment; an explicit
independent close-in-place operation would make this sequence easier to enforce.

The arm returned folded; the extra elbow offset was removed in three small steps. Peak
sampled temperature through the saved failed-capture result was 73 °C. Loaded delays and
continuing past warning remain unresolved shortcomings. Probe-and-return sequences reduced
time spent analyzing while extended, but later image/decision pauses still ran too long.

Retain these lessons:

- Test adaptation against the changed scene; do not silently replace that objective with
  repeatability at the original arrangement.
- Reaching a software offset limit does not establish a hardware reach limit.
- Single-joint probes plus paired images can establish local motion response without a URDF.
- A centered ball in a wrist image does not prove fingers are at grasp height or have captured
  the ball. Confirm retention/lift before treating transfer as successful.
- Do not keep raising open fingers to improve the image alignment. Preserve the operator's
  full observation of the motion sequence; the eventual high pose was not the whole failure.
- Preserve unsuccessful attempts and operator interventions alongside successful ones.

See [TODO.md](../TODO.md) for remaining controller and thermal work.
