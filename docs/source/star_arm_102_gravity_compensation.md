# Star Arm 102-HD — gravity compensation decision record

Working notes written 2026-09-08 on the purple **Star Arm 102-HD** leader. Companion to
[Star Arm 102 — hardware notes](./star_arm_102), which has the variant table, the datasheet
summary and the full UART protocol table; this page repeats only what the decision turns on.
Not a user guide.

## Summary

- The RP8-U45H-M servos offer **position, velocity (wheel) and three stop states** — nothing else.
- There is **no torque, current, PWM or gain-access channel**, so gravity torque cannot be
  commanded as feedforward. True gravity compensation is not achievable on this arm.
- An "active hover" loop (re-command own angle under a low power cap) was analysed and rejected:
  it is a damper, not a support, and drifts.
- LeRobot's SO-101 leader holds its pose with **gearing and friction, not software**; the same
  principle applies here.
- **Decision:** drop software gravity compensation; ship a **lock toggle** (hold / release) on stop
  command 24, then position-following for DAgger; springs and a power-cap bench test are optional
  follow-ups.

## Hardware & bus

|                             |                                                                                                                     |
| --------------------------- | ------------------------------------------------------------------------------------------------------------------- |
| Arm                         | Star Arm 102-HD, purple, 7× FashionStar **RP8-U45H-M**, ids 0–6                                                     |
| Bus                         | UART 1 Mbaud, 12 V, macOS `/dev/cu.wchusbserial110` (CH340)                                                         |
| Idle telemetry (2026-09-08) | 12.2 V, ~30 mA and ~365 mW per servo, status 0                                                                      |
| Motor / gearing             | coreless, **273:1**                                                                                                 |
| Torque                      | 0.54 N·m rated, 1.67 N·m max dynamic, 4.41 N·m stall                                                                |
| Encoder                     | 12-bit magnetic (0.088°), ±1024 turns multi-turn                                                                    |
| Datasheet                   | [Fashion-Star-RP8-U45H-M.pdf](https://fashionstar.com.hk/wiki/servo/uart/datasheet/pdf/Fashion-Star-RP8-U45H-M.pdf) |

**Lock-button board.** The vendor ships it at servo id 7 with two modes: _host_ (the board itself
sends lock/unlock to the joints) and _slave_ (polled as a simulated servo; 0.0° = unlocked,
180.0° = locked). On our unit nothing answers at id 7 at 1 M, 500 k, 250 k or 115200 baud (ids
0–253 scanned). A 45 s current poll while the operator pressed the button showed no change —
inconclusive, since a folded arm at rest draws no extra current in hold. The board's firmware and
docs live in the vendor's non-public "Star-Arm-102-dev-main" project. Practical note: including
id 7 in `motorbridge_smart_servo.sync_monitor` raises
`ServoBusError: servo 7 lost 20 consecutive responses`, so the driver must not poll it blindly.

## Control modes

FashionStar UART spec v1.0.25 (public Arduino header:
[FashionStar_UartServoProtocol.h](https://github.com/mushroom-x/fashionstar-uart-servo-arduino/blob/main/FashionStar_UartServoProtocol.h)).
Every motion command carries a u16 **power** field in mW.

| Mode                             |          Command(s) | Setpoint                                                | Notes                                                         |
| -------------------------------- | ------------------: | ------------------------------------------------------- | ------------------------------------------------------------- |
| Single-turn position             |                   8 | i16 angle 0.1°, u16 time ms                             | Firmware position loop, fixed gains                           |
| Single-turn position, profiled   |             11 / 12 | + u16 t_acc, u16 t_dec (≥ 20 ms)                        | By interval or by velocity                                    |
| Multi-turn position (+ profiled) |        13 / 14 / 15 | i32 angle 0.1°, u32 ms                                  | Same variants; 16 reads back, 17 resets turns (released only) |
| Wheel (velocity)                 |                   7 | speed, direction; indefinitely / for time / for N turns | Config 47: brake or coast on stop                             |
| Stop: **release**                |          24, `0x10` | u16 power                                               | Torque off; arm drops                                         |
| Stop: **hold**                   |          24, `0x11` | u16 power                                               | Holds current position                                        |
| Stop: **damping**                | 24, `0x12` (also 9) | u16 power                                               | Viscous resistance, no position target                        |

Supporting commands: 1 ping, 2 reset user data, 3/4 read/write config, 5/6 batch read/write,
10 query angle, 18/19 async write / activate, 22 data monitor (V, I, P, temp, status, pos,
turns), 23 set origin (released only), 25 sync wrapper for 8/11/12/13/14/15/22.

Config ids relevant to effort: 37 stall-protect switch, 38 stall power limit, 42 over-power
threshold, 43 over-current threshold, 44 accel switch, 46 power-on hold switch, 49/50 soft-start
switch/time, 48/51/52 angle-limit switch/high/low, 53 mid offset.

**What is missing:** no torque command, no current command, no PWM/voltage mode, and no access
to the position-loop gains (stiffness is fixed in firmware). The per-command `power` is an
**effort ceiling**, not a stiffness or torque setpoint; whether it acts as a proportional limit or
a stall-style cutoff is _unverified_ on the bench (see open questions).

## What gravity compensation needs vs what the protocol offers

Gravity compensation means computing the gravity torque at each joint from a URDF or geometric
model, then commanding that torque as **feedforward** so the arm feels weightless while the human
moves it. Every published implementation sits on hardware with a torque, current or PWM channel:

| Implementation                                                                                                                              | Hardware                 | Channel used                                                          |
| ------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------ | --------------------------------------------------------------------- |
| MakerMods Metal leader ([lerobot#4369](https://github.com/huggingface/lerobot/pull/4369))                                                   | Damiao QDD               | MIT mode: feedforward torque, kp = 0, friction comp; Pinocchio + URDF |
| OpenArm leader ([lerobot#2352](https://github.com/huggingface/lerobot/pull/2352), `examples/openarms/teleop_with_gravity_compensation.py`)  | Torque-controlled motors | Torque; Pinocchio                                                     |
| GELLO / [FACTR](https://github.com/RaindragonD/FACTR_Teleop)                                                                                | Dynamixel                | Current mode                                                          |
| norma-core ElRobot ([#83](https://github.com/norma-core/norma-core/pull/83), [robomotic#3](https://github.com/robomotic/norma-core/pull/3)) | Feetech ST3215           | Position offset; open-loop PWM                                        |
| [pollen-robotics/so100_gravity_compensation](https://github.com/pollen-robotics/so100_gravity_compensation)                                 | —                        | Math only                                                             |

The RP8-U45H-M has none of these channels. Nobody has published gravity compensation for the
Star Arm 102; the vendor's `Python_SDK/stararm102_ro_hover.py` in
[servodevelop/Star-Arm-102](https://github.com/servodevelop/Star-Arm-102) is a **pose lock**
(re-commands its own angle after 1 s of stillness, releases on a > 5° push), not compensation.

## Alternatives considered

| Approach                          | What it would do                                                                                                                                                                         | Outcome                                                                                               |
| --------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| **Active hover**                  | Each tick: sync-read all angles, sync-write each servo its own current angle under a low per-joint power cap, 1–2° deadband/hysteresis, small anti-gravity goal bias from a 2-link model | **Rejected** (see below)                                                                              |
| Damping mode (`0x12`) as "assist" | Viscous resistance while the human moves                                                                                                                                                 | Not a support; slows the operator, does nothing against sag. Kept only as an optional release flavour |
| Servo swap: Feetech STS3215       | ~$24 each (~$170/7); PWM mode                                                                                                                                                            | PWM is voltage control, not torque control; 345:1 gearing no more backdrivable. **Rejected**          |
| Servo swap: Dynamixel XM430       | ~$310 each (~$2,170/7); true current control                                                                                                                                             | Would work, but ~$2,170 in servos alone before mechanical integration. **Rejected on cost**           |
| Passive counterbalance            | Springs or cable counterweight on shoulder_lift / elbow_flex                                                                                                                             | Reviewer's preferred solution. **Deferred** as optional mechanical work                               |
| **Lock toggle**                   | Button/keyboard → hold all joints (`0x11`, power cap); again → release (`0x10` / `0x12`)                                                                                                 | **Adopted**                                                                                           |

### Why active hover was rejected

Independent review (OpenAI Codex, gpt-6-astra, high reasoning, 2026-09-08) rejected the premise
rather than the implementation:

- Commanding the last measured angle gives τ ≈ Kp·(q(t−T) − q(t)) ≈ −Kp·T·q̇: a **damper**, with
  no static gravity support. Whatever sags becomes the next target ("staircase" drift); a deadband
  only delays it.
- Angle motion alone cannot distinguish a human push from insufficient holding torque, so the loop
  cannot decide when to yield.
- An effort cap is not a stiffness: the servo may stay fully stiff up to the cap and then yield
  abruptly.
- A 60 Hz read does not prove 60 Hz read + write; the protocol asks for 5–10 ms between non-sync
  commands, and at 180°/s one 16.7 ms tick is already 3° of error.
- 0.54 N·m "rated" is not a guarantee of indefinite stationary hold — heat at zero mechanical
  output.
- The current driver silently substitutes cached positions after read failures; tolerable for a
  read-only leader, unacceptable once those positions become goals.

## Precedent

**SO-101 leader.** LeRobot's SO-101 leader has _no_ software gravity compensation: the driver
disables torque and only reads positions (`enable_torque` / `disable_torque` exist solely for the
DAgger handover). It stays put mechanically through per-joint gear ratios
([so101.mdx](./so101)): shoulder pan 1:191, **shoulder lift 1:345** (the gravity-loaded joint
keeps the high ratio so gearbox friction holds the arm), elbow 1:191, wrist flex / roll / gripper
1:147 for low-effort backdriving.

**MakerMods Metal leader.** The one in-tree example of real gravity compensation, and it exists
because Damiao QDD motors expose MIT-mode torque.

The HD's 273:1 servos fall on the SO-101 side of this line: "stays where placed" comes from gearing
and friction (optionally springs), "feels light" from lower ratios or counterbalance — not from a
control loop.

## Decision & next steps (2026-09-08)

Software gravity compensation is dropped for the Star Arm 102-HD.

1. **Lock toggle — adopted.** Button press → hold all seven joints (stop mode `0x11` with an
   explicit power cap); press again → release (`0x10`, optionally `0x12` damping). Torque is
   always released on disconnect. Keyboard fallback while the button board is silent. Implemented
   in `rebot_102_leader` behind a config flag so the read-only green 102-LD is unaffected.
2. **Position-following — next.** `send_feedback` plus `enable_torque` / `disable_torque` so the
   arm satisfies the actuated-teleop contract of `lerobot-rollout --strategy.type=dagger`
   (`src/lerobot/rollout/strategies/dagger.py`; `teleop_supports_feedback` and
   `teleop_smooth_move_to` in `src/lerobot/common/control_utils.py`).
3. **Optional, later.** Springs on shoulder_lift / elbow_flex. A single-joint bench test of
   power-cap semantics — fixture, lever, force gauge; sweep imposed error at several caps; measure
   torque-vs-error, breakaway, creep, hysteresis, and whether power 0 means default or unlimited —
   before any software assist is reconsidered.

## Open questions

- **Button board.** Is it absent, unpowered, in host mode (never answering polls), or on a baud
  we did not try? A host-mode board would lock the joints itself on press; a current poll on a
  folded arm cannot detect that. Repeat the poll with the arm extended, or watch for an
  unsolicited stop-mode change via `sync_monitor` status.
- **Power-cap semantics.** Proportional limit vs stall-style cutoff; what `power = 0` selects.
  Blocks any future software assist and sets the hold cap for the lock toggle.
- **Thermal soak.** How long can a joint hold under load at a given cap before the temperature
  or over-power protection trips, and what does it do when it trips (release? damping?).

## Sources

- RP8-U45H-M datasheet: <https://fashionstar.com.hk/wiki/servo/uart/datasheet/pdf/Fashion-Star-RP8-U45H-M.pdf>
- FashionStar UART protocol, Arduino header: <https://github.com/mushroom-x/fashionstar-uart-servo-arduino/blob/main/FashionStar_UartServoProtocol.h>
- Vendor monorepo incl. `stararm102_ro_hover.py`: <https://github.com/servodevelop/Star-Arm-102>
- MakerMods Metal leader gravity comp: <https://github.com/huggingface/lerobot/pull/4369>
- OpenArm leader gravity comp: <https://github.com/huggingface/lerobot/pull/2352>
- FACTR teleop (Dynamixel current mode): <https://github.com/RaindragonD/FACTR_Teleop>
- norma-core ElRobot: <https://github.com/norma-core/norma-core/pull/83>, <https://github.com/robomotic/norma-core/pull/3>
- SO-100 gravity compensation math: <https://github.com/pollen-robotics/so100_gravity_compensation>
- SO-101 gear ratios: `docs/source/so101.mdx` in this repo
- Independent review: OpenAI Codex (gpt-6-astra, high reasoning), 2026-09-08, session notes
