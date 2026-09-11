# Star Arm 102 — hardware notes

Working notes on the FashionStar / Seeed **Star Arm 102** family, written 2026-08-28 while bringing
up the purple arm. This is a reference for the `rebot_102_leader` work on this branch, not a
user guide; see [reBot B601-DM](./rebot_b601) for the existing leader setup.

## Variants

The "Star Arm 102" is a family of three SKUs. Seeed resells only the first and never uses the
suffixes, which is why LeRobot's `rebot_102_leader` is read-only.

| Variant    | Colour         | Vendor role                                   | Servos                                                                                             | Power              |
| ---------- | -------------- | --------------------------------------------- | -------------------------------------------------------------------------------------------------- | ------------------ |
| **102-LD** | green          | passive leader, "drops when released"         | RA8-U01H-M ×4 (ids 0–3), RA8-U02H-M (id 4), RA8-U03H-M ×2 (ids 5–6). No public datasheet.          | 12 V 2–3 A, DC5521 |
| **102-HD** | **purple**     | "pose-holding leader": one-press lock / hover | **RP8-U45H-M ×7** (ids 0–6; -C028/-C029 sub-variants on 4–6) + lock-button board at **servo id 7** | 12 V 10 A, XT30    |
| 102-FL     | green, gripper | follower, 500 g payload                       | RA8-U35H-M / RX8-U50H-M (vendor docs disagree)                                                     | 12 V 10 A, XT30    |

**Our arms:** the green one is a 102-LD. The purple one is a **102-HD** (confirmed 2026-08-28).
The LD's servos are treated as encoder-only in this repo; whether they contain a usable motor is
unverified (the vendor lists 12-bit encoders on all seven LD joints, but no datasheet exists).

### Purple arm as found

- Port on macOS: `/dev/cu.wchusbserial10` (CH340 UART bridge), 1 Mbaud.
- `scan(20)` returns ids **0–6 only**. Nothing answers at id 7, so either this unit has no
  lock-button board or it is not on the bus. Lock/hover must be software-triggered.
- Idle telemetry via `sync_monitor`: ~12.1 V, ~30 mA and ~365 mW per servo, status 0.

## RP8-U45H-M servo

From the [official datasheet](https://fashionstar.com.hk/wiki/servo/uart/datasheet/pdf/Fashion-Star-RP8-U45H-M.pdf)
(RP = dual shaft, H = 12 V, M = magnetic encoder):

|                 |                                                                                  |
| --------------- | -------------------------------------------------------------------------------- |
| Voltage         | 9.0–12.6 V                                                                       |
| Motor / gearing | coreless, 273:1                                                                  |
| Torque          | **4.41 N·m stall**, 1.67 N·m max dynamic, **0.54 N·m rated**                     |
| Speed           | 64 rpm rated, 90 rpm no-load; no-load current < 300 mA                           |
| Encoder         | 12-bit non-contact magnetic absolute, **4096 counts / 360° (0.088°)**            |
| Range           | ±180° single-turn, ±368,640° multi-turn (±1024 turns), power-off angle memory    |
| Modes           | single-turn, multi-turn, wheel, **damping**; stop modes hold / release / damping |
| Feedback        | voltage, current, power, temperature, angle, turns                               |
| Protection      | temperature, voltage, stall, power, current                                      |
| Size / mass     | 40×20×40 mm, 73 g                                                                |

Plan follower loads against the **rated** 0.54 N·m, not the stall figure. Compare RX8-U50H-M
(FL shoulder): brushless, 4.90 N·m stall.

## UART protocol

Fully public, 20 commands
([spec v1.0.25](https://fashionstar.com.hk/wiki/servo/uart/protocols/uart-rs485-protocol/) — returns 403
to non-browser user agents). Small enough to implement natively over pyserial.

- Packet: `header(2) cmd(1) len(1) payload checksum(1)`, little-endian. Request header
  `0x12 0x4C`, response `0x05 0x1C`, checksum = Σ bytes mod 256. 8N1 half-duplex, 9600–1 Mbps,
  ids 0–254, keep 5–10 ms between commands. A new command pre-empts the current one.
  Responses to write commands are off by default (config id 33).

|         Code | Command                                                                                          | Payload                                   |
| -----------: | ------------------------------------------------------------------------------------------------ | ----------------------------------------- |
|            1 | ping                                                                                             | id                                        |
|        3 / 4 | read data / write config                                                                         | id, data_id(, value)                      |
|            8 | set angle                                                                                        | id, i16 0.1°, u16 ms, u16 mW              |
|            9 | damping                                                                                          | id, u16 mW (only when released / damping) |
|      10 / 16 | read angle / multi-turn angle (+ i16 turns)                                                      | id                                        |
|      11 / 12 | angle by time / by speed                                                                         | + u16 t_acc, u16 t_dec (≥ 20 ms), u16 mW  |
| 13 / 14 / 15 | multi-turn set / by time / by speed                                                              | i32 0.1°, u32 ms, …                       |
|           17 | reset turn counter (only when released)                                                          | id                                        |
|      18 / 19 | async write / activate                                                                           |                                           |
|           22 | data monitor → V mV, I mA, P mW, temp ADC, status u8, pos i32 0.1°, turns i16                    | id                                        |
|           23 | set origin (only when released)                                                                  | id, 0x00                                  |
|       **24** | **stop-on-control**: `0x10` release (torque off), `0x11` hold, `0x12` damping; then u16 power mW | id, mode, power                           |
|           25 | sync wrapper for 8/11/12/13/14/15/22, no response                                                |                                           |

Useful config ids (cmd 3/4): 1 V, 2 I, 3 P, 4 temp, 5 status; 33 response switch, 34 id,
36 baud, 37 stall-protection switch, **38 stall power limit**, 39/40 voltage limits, 41 temp
limit, **42 power-protection threshold**, **43 current protection**, **46 power-on hold torque**,
48–52 angle limits.

**There is no torque or current command.** Effort is bounded only by the per-command `power`
cap and the protection thresholds above. Bilateral force feedback is therefore not achievable;
position–position coupling or damping is the ceiling.

## Existing software

| Resource                                                                                                         | What it gives                                                                                                                                                                                                                                                                                              | Drives the HD?       |
| ---------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------- |
| [servodevelop/Star-Arm-102](https://github.com/servodevelop/Star-Arm-102)                                        | Official monorepo. LeRobot plugin registering `stararm102_hd` (teleop: lock/unlock via stop `0x11`/`0x10` when button id 7 ≥ 90°) and `stararm102_fl` (robot: `sync_write("Goal_Position")` in degrees). `stararm102_ro_hover.py`: hover = command own angle after 1 s stillness, release on > 5° push.    | HD as leader only    |
| [fashionstar-uart-sdk](https://github.com/servodevelop/fashionstar-uart-servo-python) (MIT, PyPI 1.3.12)         | Official Python SDK: `set_servo_angle`, `set_damping`, `stop_on_control_mode(id, 0x10/0x11/0x12, power)`, sync moves, sync monitor, `read_data`/`write_data`.                                                                                                                                              | yes, full write      |
| [lerobot-motor-starai](https://github.com/servodevelop/fashionstar-lerobot-motor-starai) (PyPI 0.0.7)            | Vendor `StaraiMotorsBus(MotorsBus)` over the SDK: `sync_read("Present_Position")`, `sync_write("Goal_Position")`, `enable_torque()` → hold, `disable_torque(mode="damping", power=900)`. Reference for a control table.                                                                                    | yes                  |
| [motorbridge-smart-servo](https://github.com/motorbridge/motorbridge-smart-servo) (what `rebot_102_leader` uses) | Reads: `ping/scan/read_angle/sync_monitor`. Writes: `set_angle(id, deg, multi_turn, interval_ms)`, `set_stop_mode(id, mode, power)`, `unlock()` = `set_stop_mode(id, 0x10, 0)`, `set_origin_point`, `reset_multi_turn`. No damping (9), no sync write, no read/write data. README disclaims write support. | partial, unsupported |
| `huggingface/lerobot` `rebot_102_leader`                                                                         | Read-only leader; `feedback_features = {}`, `send_feedback` raises.                                                                                                                                                                                                                                        | no                   |

Nobody has published a follower class for the HD, and there are no upstream issues or PRs about
driving the 102 or force feedback.

## Incorporation plan

Tiers, cheapest first. The HD is the same 7-servo bus, ids and joint mapping as the LD, so this is
new _capabilities_ on the existing leader rather than a new robot type.

1. **HD-aware leader.** _Lock half landed:_ `RebotArm102LeaderConfig.button` (off by default)
   polls the button board at id 7 and puts every non-excluded joint into hold (0x11) / release
   (0x10) / damping (0x12) on the press edge, `connect` pings the board, `disconnect` always
   releases, and `examples/rebot_102/star_hd_lock_toggle.py` drives the same toggle from the
   keyboard on a unit whose board is silent. The button is read on its own `read_angle`, never
   folded into the joints' `sync_monitor`: an id that does not answer trips the package's
   consecutive-loss check and fails the whole packet, so ids 0-6 come back empty too. Still to
   do: add `motorized: bool = False` to `RebotArm102LeaderConfig`. When set:
   `feedback_features` becomes the seven `.pos` floats and `send_feedback` writes positions
   through the inverse of `joint_directions` / `joint_ranges` (enables the DAgger/HIL smooth
   handover path). Add `lock()` / `release()` / `damp(power_mw)` on stop command 24. Configure
   and disconnect must always release torque. Set hold/damping power from config ids 38/42/46
   rather than the vendor's `65535/16` default.
2. **Native bus.** Implement the protocol in `src/lerobot/motors/fashionstar/` as a `MotorsBus`
   subclass (pattern of `feetech/`), which drops both third-party packages and gives
   `lerobot-setup-motors` / `lerobot-calibrate` for free. Preferred over depending on
   `fashionstar-uart-sdk`; `StaraiMotorsBus` is the control-table reference.
3. **`rebot_102_follower`.** Mirror of `rebot_b601_follower`: `ensure_safe_goal_position` with
   `max_relative_target`, `sync_write("Goal_Position")`, current and temperature as extra
   observations, current ceiling enforced via config id 43. Light duty only (0.54 N·m rated, no
   gripper); a `bi_rebot_102_follower` follows the `bi_maker_follower` pattern.
4. **Bilateral.** Only position–position coupling or damping scaled by follower current. Treat as
   an experiment after 1–3.

No motion has been commanded on the HD yet; a supervised single-joint test comes before any of
the above lands.

## Bench log and third-party findings (2026-09-08)

- **Hold / release verified on the purple HD** through `RebotArm102Leader.lock()` / `release()`
  (stop mode `0x11` power 0, then `0x10`). Locked, shoulder_lift and elbow_flex drew 34–45 mA
  against 30 mA idle and dropped back to 30 mA on release; angles held within 1°. Still no
  _motion_ commanded on the HD.
- **Our button board is electrically absent.** No reply at ids 7–11 at 14 baud rates
  (9600–2 Mbaud), no reply to ping (1), query angle (10/16), monitor (22) or read data (3) at
  id 7, and a 45 s raw sniff at 1 Mbaud saw zero unsolicited bytes while the operator pressed
  the button. The bridge is a bare CH340 (`1a86:7523`), so the main board's two connectors are
  one electrical bus: a powered, working board would have been seen. Suspect wiring/power on
  the button connector. `motorbridge_smart_servo.sync_monitor` raises
  `ServoBusError: servo 7 lost 20 consecutive responses` (and returns nothing for ids 0–6) if
  the absent id is included — read the button separately.
- **Another 102-HD owner's notes**
  ([cijerezg/lerobot `pi07_wiki/leader_102hd_actuation.md`](https://github.com/cijerezg/lerobot/blob/main/pi07_wiki/leader_102hd_actuation.md),
  measured 2026-09-02) fill in what our board could not tell us:
  - Their button answers at id 7. `sync_monitor` misparses it (turn 255, 22136 mA …) because it
    is not a servo; read only its angle. States from the vendor's `stararm102_ro.py`
    `Button.key_state_*`: **0 = unlocked, 180 = lock requested, 90 = unlock requested** — hence
    our `trigger_threshold` default of 135, not the vendor plugin's 90.
  - **Single-servo `set angle` is silently ignored unless stop mode `0x11` was set first**; the
    sync path (command 25 wrapping 14, `<BiIHHH` id, angle×10, interval, t_acc, t_dec, power)
    drives from any stop mode. Use the sync path for feedback/shadowing.
  - `motorbridge_smart_servo` cannot drive the arm: `set_angle` has no `power` field and there
    is no sync-with-power call or raw-packet escape. They drive through `fashionstar-uart-sdk`
    1.3.12. Long intervals (15° over 3000 ms) stick-slip audibly; the vendor re-sends with
    `interval=100` every loop.
  - They report current telemetry flat at 30 mA even while holding weight or moving. Ours
    rose to 45 mA under hold, so treat current as a weak indicator and verify by angle or hand.
