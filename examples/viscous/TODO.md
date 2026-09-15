# Viscous Arm — open items

Checked-in list of known work and open questions for the Viscous Arm teleop setup. Add an item
when something is deferred; delete it (in the same commit as the fix) when it is done. Newest
context first within each item; dates are absolute.

## Next

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
