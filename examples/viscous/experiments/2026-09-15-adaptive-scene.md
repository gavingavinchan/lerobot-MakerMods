# 2026-09-15 — adaptation to a changed ball and foam position

**Final outcome: successful pickup and deposit in `scale_05`, confirmed by the operator.**
Earlier retries failed. The operator intentionally moved
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

## Successful correction using the demonstrated ball size

The operator identified the remaining mismatch as the gripper being too far back from the
ball, and directed comparison of its apparent size in the wrist camera with the demonstration.
This resolved an error in interpreting image position as approach progress. At raw reference
12.0 s the demonstrated ball bounding box was 153×145 px, centroid (300.2, 404.1). In local
retry `elbow6_grip0`, it was only 120×118 px, centroid (310.3, 353.4). Raising the elbow through
offsets +5° to +9° moved the ball downward in the image but reduced segmented area from
11,140 to 9,666 px². It did not achieve the required proximity.

Measurements use HSV segmentation of the orange ball in 640×480 wrist frames. Framing,
partial occlusion, rotation, and contact affect these values; they are a local comparison,
not calibrated depth. The successful capture began at about 145 px width, so exact equality
with the demonstrated 153 px is not required.

`adaptive_run_01/scale_05/` records the following completed sequence. Offsets are absolute
corrections from the indicated raw reference phase; each offset command changed a joint by
at most 3°. Omitted yaw/roll/gripper offsets were zero until deposit preparation.

| Checkpoint | Raw reference (s) | Pan / shoulder / elbow / wrist-flex offsets (°) | Ball width / centroid (px) |
| --- | --- | --- | --- |
| Baseline approach | 12.0 | +18 / 0 / 0 / 0 | 133 / (293.6, 245.2) |
| Forward correction 1 | 12.0 | +18 / −3 / 0 / +3 | 135 / (287.2, 266.4) |
| Forward correction 2 | 12.0 | +18 / −6 / 0 / +6 | 138 / (282.0, 289.9) |
| Forward correction 3, lateral refinement | 12.0 | +16.5 / −9 / 0 / +9 | 142 / (298.8, 314.5) |
| Approach refinement | 12.0 | +16.5 / −10 / +3 / +10 | 146 / (301.9, 335.8) |
| Before closure | 12.0 | +16.5 / −10 / +6 / +10 | 145 / (301.3, 357.6) |
| Capture | 12.5 | +16.5 / −10 / +6 / +10 | 144 / (309.3, 400.5) |
| Small lift, gripper target held | 12.5 | +16.5 / −7 / +6 / +10 | 145 / (309.2, 400.7) |
| Retained transfer | 13.8 | +16.5 / −7 / +6 / +10 | 144 / (309.4, 399.8) |

At capture the gripper measured −80.37° with 1.69 Nm reported torque against a −75.49° target.
This differed from the nearly unloaded empty closures. The ball remained in essentially the
same wrist-image position through the small lift and transfer while the external scene moved.
The external view then showed it held over the foam opening. These observations, followed
by the final deposited ball and operator confirmation, establish success for this attempt.

The capture segment from 12.0 to 12.5 s also changes pan, elbow, and wrist yaw slightly; this
was not fully independent gripper closure. The subsequent +3° shoulder lift occurred only
after capture evidence. Do not attribute the entire successful correction to one joint.

For deposit, pan offset changed +16.5 → +13.5 → +10.5° and gripper offset 0 → −3 → −6° at
reference 13.8 s. Following to 14.6 s opened the gripper to a measured −90.10° with about
−0.01 Nm torque. The ball was released into the opening. Following to 17.9 s folded the arm;
shoulder, elbow, wrist-flex and gripper offsets were removed in bounded steps. Pan offset
remained +10.5°. The final external image shows the ball inside the foam after the arm cleared
it. The foam shifted slightly; this is placement into its final opening, not a fixed-target
precision measurement. Torque remains enabled; no shutdown or hardware handoff occurred.

The user explicitly confirmed: “success!. once again, log down this episode and record down
what you learnt”. This is the second overall successful attempt and the first in the changed
scene, following multiple misses. It does not establish a general success rate.

### Frozen episode and remaining limitations

- Live-source checkpoints: `/home/gavin/Documents/robot-experiments/reference-pick-place/adaptive_run_01/scale_05/`.
- Frozen export: `/home/gavin/Documents/robot-experiments/hf-viscous-pick-place-2026-09-15/adaptive_success/`.
- Export boundary: wall time 1789524779140297712–1789524877189617542 ns; approximately 98 s.
- 977 telemetry rows, 2,451 wrist frames, 490 external frames; original compressed MJPEG
  packets verified after AVI remux; timestamp-aligned MP4 frame counts verified by decoding.
- Commands, paired checkpoint images, operator-confirmed result, exporter and command helper
  are preserved. Earlier retry checkpoints remain alongside the successful episode.
- Peak temperature was 68 °C, including the return; the final recorded powered hold was
  64 °C and subsequently cooled. Decision delays and timely thermal unloading still need work.
  Integrating the sampled `moving` flag gives approximately 24.49 s moving and 73.48 s
  holding across the interval. Holding includes observation, reasoning/tool gaps and settling;
  this telemetry does not separately measure model reasoning time.
  Camera capture ran continuously; this success does not demonstrate improved camera latency.
- This export is synchronized evidence, not a newly trained model or native LeRobot training
  episode. Learning here means retained operating notes and data for future controllers.

Apply the lesson as a procedure: compare size and position with a compatible wrist reference,
make small coordinated approach corrections, close and check retention before lifting, then
adapt deposit alignment to the actual foam. After an empty closure, reopen and retry locally;
do not restart the entire trajectory solely because of a miss.
