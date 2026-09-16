# Viscous Arm — open items

Checked-in list of known work and open questions for the Viscous Arm teleop setup. Add an item
when something is deferred; delete it (in the same commit as the fix) when it is done. Newest
context first within each item; dates are absolute.

## Next

- **Changed-scene adaptation remains incomplete (2026-09-15).**
  [Trial and local response measurements](experiments/2026-09-15-adaptive-scene.md).
  The operator wants adaptation, not objects reset to the demonstration. +18° pan/+9° elbow
  aligned the ball in the wrist view but did not capture it. The operator clarified that open
  fingers rose from below the ball before securing it; later closure was effectively empty.
  Enforce close-and-verify before lift, with independent gripper control at a held arm pose.
  Peak sampled temperature reached 73 °C; reduce loaded decision delays and enforce a timely
  return on warning. The replacement controller is holding folded in `adaptive_run_01`;
  inspect its fresh status before any new hardware owner. The old `run_02` process was cleanly
  released with operator support and is no longer the live controller.
- **Reduce observation-to-command latency.** Camera capture in the successful run remained
  at roughly 40 ms/200 ms intervals while command gaps lasted tens of seconds. Batch views
  and telemetry, prepare narrow checkpoint decisions, and instrument capture/inspection/
  decision/command timing. Validate continuous monitored reference segments after controller
  fixes; a local visual correction loop is a further option. See [EXPERIENCE.md](EXPERIENCE.md).
- **Measure improvement across supervised repeats.** The [experience guide](EXPERIENCE.md)
  records lessons from both 2026-09-15 trials. After the controller fixes below, measure local
  joint-to-image responses near pregrasp and repeatability with a consistent scene; compare
  outcomes, correction count, loaded hold time, and peak temperature. These measurements have
  not yet been performed.
- **Reference-guided pick-and-place succeeded once (2026-09-15); harden the controller.**
  [Successful attempt and live-state notes](/home/gavin/Documents/robot-experiments/reference-pick-place/run_02/README.md).
  A 7.91-second teleop reference, staged execution, and camera-guided corrections (-1.5° pan,
  +2.5° shoulder lift) achieved pickup and deposit. Inspection pauses brought a motor to 80 °C;
  shorten loaded holds. The experimental controller needs completion checks against clipped
  targets (fixed in the replacement `reference_controller.py`, not the archived source),
  a tested automatic transition from powered folded pose to its initial-rest release gate, and
  independent recording finalization. At task completion it was holding folded, not disconnected;
  consult the linked live status before starting another controller. Do not reuse unattended.
- **Follower-only pick-and-place experiment remains incomplete (2026-09-15).** The gripper and
  camera recording worked, but no grasp was achieved; the elbow reached 63 °C while holding
  with significant target error. The arm was returned to rest and disconnected cleanly.
  [Experiment log](./experiments/2026-09-15-orange-ball.md) links the durable video/telemetry
  archive and records the operator's acceptance of light tabletop contact. Review measured
  approach/lift motion and sustained holding loads before repeating; the archived controller's
  general parking and exception paths are not validated for unattended use.
- **Follower `wrist_flex` presses its folded-end stop.** The folded Star leader reads `wrist_flex`
  ~+10.6° raw, which maps to a follower target of about -11.5°. That is inside the follower's soft
  limits `(-120.8, 92.0)` but past where the wrist physically stops, so startup sync stalls ~10°
  short on every start (10.7°, 10.3°, 9.4° on 2026-09-15) and the joint holds against the stop at
  kp 30, running hotter than the other idle joints (36–38 °C vs 28–31 °C). **Decision
  (operator, 2026-09-15): fix with a software rotation limit, not by re-zeroing either arm.**
  Measure the real stop, narrow that end of `wrist_flex` in
  `ViscousFollowerConfigBase.joint_limits`; the leader's `joint_ranges` derive from it, and
  `test_viscous_directions_are_the_captured_travel_ratios` pins the leader scale factors to the
  follower's travel, so expect to revisit that mapping/test.
- **Gradual return to zero on stop.** Ending a session releases torque and the arm drops. The
  operator believes a slow return-to-zero exists in the Maker Labs code (not on the Linux host);
  port it so `teleop.sh stop` (and ideally the thermal stop) parks the arm before disconnecting.

## To verify

- **Desktop notifications on real events.** `teleop.sh`'s watcher (heat, thermal stop, power loss,
  unexpected exit) was only exercised against a fake teleop with pop-ups off; no real session has
  triggered one yet.
- **Motor power drop at 10:26:16 on 2026-09-15**, 24 min into a session: all 7 motors silent at
  once, not thermal (max `elbow_flex` 44 °C), no USB event. Likely a supply over-current trip under
  `elbow_flex` load (31→44 °C in the last 2 min). Check the supply's trip/fuse and the replaced
  J1→J2 cable seating.

## Known limitations

- **A `stop` during the ~5 s startup is not a clean disconnect.** `lerobot_teleoperate.py` connects
  both arms outside its `try/finally`, so SIGINT before the loop starts skips `disconnect()`. Low
  risk (the arm is folded then), but `teleop.sh stop` will report an unclean log.
- **`shoulder_lift` / `elbow_flex` gains are untuned placeholders** (kp 30, kd 2). See
  [Tuning gains](../../docs/source/viscous.mdx) before changing them.
- `docs/source/maker.mdx` "Thermal duty" still says the follower "never stops the session itself"
  right before the paragraph introducing `temp_stop_c`; true for Maker's default, but reword.
