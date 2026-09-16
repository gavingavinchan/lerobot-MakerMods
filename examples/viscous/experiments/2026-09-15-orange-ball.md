# 2026-09-15 — follower-only orange-ball pick-and-place trial

**Outcome: no successful grasp, lift, or placement.** The follower opened its gripper and made
small camera-guided joint adjustments. The operator ended the experiment to leave for lunch.
The follower was returned to its initial resting pose (all measured joints within 0.73°),
the empty gripper was closed, torque was released, and both camera recordings ended cleanly
at approximately 12:28:11 CDT. The leader was never connected or commanded. Calibration and
repository gain defaults were unchanged.

## Durable experiment archive

[Full report and artifact index](/home/gavin/Documents/robot-experiments/2026-09-15-orange-ball/README.md)

Local archive: `/home/gavin/Documents/robot-experiments/2026-09-15-orange-ball/`.
It contains original videos, MP4 review copies, per-frame timestamps, 9,753 telemetry samples,
command CSV/JSON, baseline and final states, session scripts, calibration/configuration
snapshots, temperature/torque plots, and raw-file checksums. It has not been uploaded.

## Findings to retain

- Both cameras work: wrist `/dev/video0`, about 25 fps; external `/dev/video2`, about 5 fps;
  both 640×480. The external camera moved during the experiment, so it is not a fixed
  calibrated frame for the whole recording.
- More-negative gripper motor angles open the fingers. A recorded 6° opening-and-return test
  had less than 1° tracking error and stayed at 29 °C.
- Small base-pan adjustments brought the ball near the wrist-image centerline. This did not
  establish a grasp pose or camera-to-arm calibration.
- The operator confirmed finger contact with the tabletop and said light contact was okay.
- Target interpolation was limited to 1°/s, with new jogs at most 5°. This did not bound actual
  joint velocity or holding torque. The elbow reached 8.05° tracking error, 4.17 Nm peak absolute
  reported torque, and **63 °C** peak temperature. The heat warning prompted unloading.
- The main recording lasted 32 min 37 s, including long holds between commands and replies.
  About 297 s involved changing targets, estimated from telemetry. Slower command changes
  did not eliminate heating while holding under load.
- No URDF was necessary for small joint commands, but no reliable Cartesian reach/lift mapping
  was established. A public Maker model was found but was not verified or used for Viscous.
- The operator reports already accomplishing this task in teleop. This trial does not show
  that the hardware is incapable of it.

## Deferred work

Review recorded tip motion against command timestamps; establish measured approach/lift
motions and consistent camera framing; address sustained target error and long holding time.
Do not re-zero the follower. The archived controller is experimental: its exception path can
release torque, and its general automatic park mechanism is unvalidated. This run ended using
explicit staged return commands plus a measured resting-pose check.
