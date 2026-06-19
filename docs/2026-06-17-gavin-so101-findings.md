# Gavin SO101 Findings - 2026-06-17

## Summary

This note captures the SO101 connection, calibration, servo-test, and data-stream findings from the current `lerobot-MakerMods` workspace.

## Current SO101 Connection

- Working SO101 Feetech bus port: `/dev/cu.usbmodem5B140324321`
- Other detected USB modem port: `/dev/cu.usbmodem5AB01799231`
- The other port did not report Feetech servos during the last check.
- The working bus reported all expected SO101 servo IDs: `1, 2, 3, 4, 5, 6`.
- Full handshake against the SO101 motor map succeeded.

## Auto-Calibration

The project already includes an auto-calibration script:

```bash
conda run -n lerobot python src/lerobot/scripts/lerobot_auto_calibrate_feetech.py \
  --port /dev/cu.usbmodem5B140324321 \
  --save \
  --robot-id default
```

Important notes:

- The script is hardware-active: it unfolds/moves the arm and probes joint limits.
- It expects a full SO101-style Feetech bus with servo IDs `1-6`.
- It saves calibration with `--save` to the LeRobot calibration cache path.
- Earlier failures happened because USB was connected but servo power was not connected, so the bus opened but no servo IDs responded.

## Single-Servo Test

A single powered Feetech servo was tested successfully on:

- Port: `/dev/cu.usbmodem5B140316961`
- Servo ID: `1`
- Model number reported: `777`

Successful movement tests:

- Small nudge from raw position `15` to about `165`, then returned near `17`.
- Three-second oscillation between raw positions about `17` and `267`, then returned near `19`.
- Torque was released after each test.

## UI / App Location

There is no active web UI inside this repo anymore.

Verified repo history:

- Commit `726b89d9`: `remove webui - moved to makermods-lerobot-ui repo`
- `src/lerobot/webui` is absent from this working tree.

Separate app location found on this machine:

- `~/MakerMods-App`

Claude reported launching it there with:

- Backend: `http://localhost:8000`
- Frontend: `http://localhost:3000`

## Servo Data Read By LeRobot

The SO101 servos do not continuously stream telemetry by themselves. LeRobot polls the Feetech bus over serial.

Default data read during normal SO101 operation:

- Register: `Present_Position`
- One value per servo ID `1-6`
- Raw servo values are encoder ticks in the `0-4095` range.
- LeRobot can normalize these into degrees or normalized ranges depending on config.

Follower arm observation fields:

- `shoulder_pan.pos`
- `shoulder_lift.pos`
- `elbow_flex.pos`
- `wrist_flex.pos`
- `wrist_roll.pos`
- `gripper.pos`

Relevant code path:

```python
self.bus.sync_read("Present_Position")
```

This is used by the SO101 follower in `get_observation()` and by the SO101 leader in `get_action()`.

Other Feetech registers, such as torque/current/temperature/voltage/status, may be readable explicitly, but they are not read by default in the normal SO101 teleop/recording path found here.

## Servo Telemetry Polling

Feetech SO101 servos do not push a continuous telemetry stream. To get "streamed" data, poll the readable registers repeatedly.

This workspace now has a read-only telemetry command:

```bash
conda run -n lerobot python src/lerobot/scripts/lerobot_stream_feetech_telemetry.py \
  --port /dev/cu.usbmodem5B140324321 \
  --fields all \
  --fps 10 \
  --csv logs/so101_telemetry.csv
```

Supported telemetry fields:

- `pos`: `Present_Position`
- `velocity`: `Present_Velocity`
- `load`: `Present_Load`
- `voltage`: `Present_Voltage`
- `temperature`: `Present_Temperature`
- `status`: `Status`
- `moving`: `Moving`
- `current`: `Present_Current`

A one-sample live read from `/dev/cu.usbmodem5B140324321` succeeded on 2026-06-17. All six servos returned these fields. While idle, velocity/load/current were `0`, voltage was about `122-124`, temperature was about `34-37`, and status/moving were `0`.

To record the same telemetry into `lerobot-record`, enable it on the robot config:

```bash
lerobot-record \
  --robot.type=so101_follower \
  --robot.port=/dev/cu.usbmodem5B140324321 \
  --robot.id=left_follower \
  --robot.include_servo_telemetry=true
```

For a smaller dataset state vector, use only selected fields:

```bash
lerobot-record \
  --robot.type=so101_follower \
  --robot.port=/dev/cu.usbmodem5B140324321 \
  --robot.id=left_follower \
  --robot.servo_telemetry='["velocity","current","temperature"]'
```

For bimanual SO101 recording, the same flags apply to both follower arms:

```bash
lerobot-record \
  --robot.type=bi_so101_follower \
  --robot.left_arm_port={LEFT_FOLLOWER_PORT} \
  --robot.right_arm_port={RIGHT_FOLLOWER_PORT} \
  --robot.include_servo_telemetry=true
```

## Gripper Telemetry Notes

Follow-up tests on 2026-06-18 focused on reading SO101 gripper telemetry while moving the gripper alone and while interacting with a cube. These notes are observations from early probing, not final conclusions.

### Auto-Calibration Result

The original project auto-calibration script was run successfully:

```bash
conda run -n lerobot python src/lerobot/scripts/lerobot_auto_calibrate_feetech.py \
  --port /dev/cu.usbmodem5B140324321 \
  --save \
  --robot-id default
```

Calibration was saved to:

```text
/Users/gavin/.cache/huggingface/lerobot/calibration/robots/so_follower/default.json
```

Gripper calibration result:

- `range_min`: `1234`
- `range_max`: `2860`
- `mid`: `203`
- `homing_offset`: `-1844`

### Telemetry Register Notes

`Present_Load` should not be treated as a direct force sensor. Feetech describes it as current control output duty cycle / torque output, with `1000` corresponding to 100 percent torque output. In practice, this appears closer to a controller effort signal than to fingertip force.

Raw `Present_Load` is direction-encoded. Values above `1024` are not simply larger positive values. A rough decode used during notes:

```python
def decode_load(raw):
    return 1024 - raw if raw >= 1024 else raw
```

> **KEY OBSERVATION: `Present_Current` was the clearest telemetry signal for increased gripper effort in the cube tests.**
>
> **It changed more clearly than raw `Present_Load`, and much more clearly than gripper position.**

Feetech documents current feedback with a scale of roughly:

```text
1 raw current unit ~= 6.5 mA
```

Servo position values were recorded because they are part of the default SO101 observation stream, but they were not useful for the gripper torque/contact question in these early tests. The position traces mostly showed commanded motion and mechanical settling, not grip force.

### Gripper / Cube Test Observations

Several plots were generated in `logs/`.

Initial calibrated full-range sweep:

- CSV: `logs/gripper_telemetry_20260617_234530.csv`
- Plot: `logs/gripper_telemetry_20260617_234530.png`

Close-on-cube test:

- CSV: `logs/gripper_telemetry_20260617_235643.csv`
- Plot: `logs/gripper_telemetry_20260617_235643.png`
- The gripper was commanded toward the calibrated closed end while a cube was between the fingers.
- `Present_Current` rose visibly during the close/hold period.
- Raw `Present_Load` also changed, but raw load values are direction-encoded and should not be interpreted directly as force.

Cleaner phased cube test:

- CSV: `logs/gripper_cube_release_20260618_000325.csv`
- Plot: `logs/gripper_cube_release_20260618_000325.png`
- Sequence: open/place cube, close on cube, open/remove cube, close empty, final open.
- Position was not useful for interpreting torque/contact in this run.
- `Present_Current` was the clearest signal associated with increased gripper effort.
- Raw load values need decoded/renamed before they are useful in plots; the label `load` is misleading.

Working hypotheses for further investigation:

- `Present_Current` may be the best built-in proxy for gripper torque/contact available from these servos.
- Decoded `Present_Load` may still be useful as a controller-effort signal, but it should be plotted as signed/decoded effort, not raw `load`.
- Position alone is likely not useful for estimating torque or grip contact in this setup.
- A better test should use controlled object sizes/materials and repeated trials, and should compare empty close, soft object, hard object, and intentional stall.

Useful scripts added during these tests:

- `src/lerobot/scripts/lerobot_stream_feetech_telemetry.py`
- `src/lerobot/scripts/lerobot_gripper_telemetry_test.py`
- `src/lerobot/scripts/lerobot_gripper_cube_release_test.py`

## Full-Telemetry Cube Experiment And Overload Findings (2026-06-19)

A new experiment was added to plot **every** readable gripper value, including the
binary protection/error flags, and to capture them reliably even while the servo is
faulted.

Script:

- `src/lerobot/scripts/lerobot_gripper_full_telemetry_test.py`

It runs a motion-gated phased cube sequence (open/place cube -> close on cube ->
open/remove cube -> close empty -> final open). Each phase runs until the gripper
reaches its target or stalls, then dwells, so motions are never cut off mid-travel.

### Why These Experiments (Intent)

Gavin's intent behind this line of experiments, in his own framing:

- **Understand grip effort / contact / force sensing from the servo itself.** Position
  alone says nothing about grip force, and `Present_Load` turned out to be poor at
  sensing actual external torque. The goal is to find which built-in telemetry signal
  best indicates that the gripper is pressing on something and how hard.
- **Get the data that comes directly out of the servo.** The priority is the servo's
  own registers, packet contents, and status/error byte. Behavior introduced by
  lerobot or other host software -- e.g. raising an exception on a non-zero error byte
  and producing `NaN` gaps -- is **secondary** and treated as a software quirk to work
  around, not as signal. See the raw-read path below.
- **Plot every value, including the binary protection flags.** Gavin explicitly wanted
  all values graphed, including bit flags like the overload-protection warning, to see
  the full servo state during a grip rather than a curated subset.
- **Characterize protection behavior on purpose.** The cube grip vs empty close lets us
  observe what the servo reports when it stalls on an object, including deliberately
  driving it into overload protection to see how it reacts.
- **Attribute errors to the right layer.** Distinguish what the servo reports
  (`[RxPacketError]`, servo firmware) from transport/comm problems (`[TxRxResult]`,
  which would implicate the USB->TTL driver board, cabling, or baud rate). In these
  tests the driver board was a transparent, correctly-working bridge; every fault came
  from the servo's own protection logic.

### Status Byte (Register 65) Bit Map

The status/hardware-error byte at address 65 is bit-packed. The bit meanings match
`scservo_sdk`'s `ERRBIT_*` constants (the SDK lerobot actually uses):

| bit | mask | meaning |
|-----|------|---------|
| 0 | 1 | input voltage error |
| 1 | 2 | angle / sensor error |
| 2 | 4 | overheat (temperature) |
| 3 | 8 | over-current (OverEle) |
| 5 | 32 | overload |

Bit 4 (16) is unused by this SDK. Note an earlier guess had bit 1 = sensor and
bit 4 = angle; the SDK collapses angle/sensor into bit 1.

### Capturing The Error Byte Without Losing Data

KEY METHOD NOTE: when any protection bit is set, the servo returns a **packet error**
instead of data, so lerobot's normal `read()` raises and the continuous channels
(position, voltage, current, etc.) read as `NaN` for exactly the faulted samples.

Two pitfalls were found and fixed:

1. Catching the raised exception and string-matching `getRxPacketError()` is unreliable,
   because that function reports only the **highest-priority** bit it finds, and voltage
   (bit 0) is checked before overload (bit 5). This produced a spurious `voltage` flag
   that was really an overload event being shadowed.
2. The fix is to read register 65 **low-level** every sample and never raise:

   ```python
   data, comm, error = bus.packet_handler.read1ByteTxRx(bus.port_handler, motor_id, 65)
   status = (data | error) & 0xFF   # both carry the protection bits
   ```

   The script ORs the register value with the packet error byte, so all set bits are
   recorded per sample even during a fault.

### Gripper Protection-Config Registers (read live)

| Register | Value | Meaning |
|----------|-------|---------|
| `Overload_Torque` | 25 | effort threshold = 25% of output |
| `Protection_Time` | 200 | time above threshold before tripping (ms-scale) |
| `Protective_Torque` | 20 | torque cut to 20% once tripped |
| `Protection_Current` | 250 | overcurrent threshold (~250 x 6.5 mA ~= 1.6 A) |
| `Max_Torque_Limit` / `Torque_Limit` | 500 / 500 | output capped at 50% (0-1000 scale) |
| `Min_Voltage_Limit` / `Max_Voltage_Limit` | 40 / 140 | valid supply = 4.0 V .. 14.0 V |
| `Present_Voltage` | 122 | 12.2 V |
| `Max_Temperature_Limit` | 70 | 70 deg C |

### What Causes The Overload Warning

The overload bit is **not** a force/torque sensor reading. It trips when the servo's
own commanded output effort stays above `Overload_Torque` (25%) for longer than
`Protection_Time`, after which torque is cut to `Protective_Torque` (20%) to protect
the motor at stall.

`Present_Load` **is** that effort signal: it is the open-loop PWM duty cycle
(0-1000 = 0-100%), not a measured external force. That is why `load` is poor at
sensing fingertip/external torque, yet is exactly the quantity the overload logic
watches. `Present_Current` is the cleaner observable because it is measured motor
current, and motor torque is proportional to current.

### Confirmed Run (logs/gripper_full_telemetry_20260618_235255)

- `close_on_cube` commanded at 11.0 s; gripper stalled on the cube at 16.1 s (held
  ~1298, could not reach goal 1234).
- Decoded `load` climbed to ~357 (~36%), above the 25% `Overload_Torque` threshold.
- **Overload (byte = 32, bit 5) tripped at 17.58 s** and stayed latched until 25.15 s,
  which is why the subsequent `open/remove_cube` write returned `ok=False`.
- The **only** nonzero status byte observed across the whole run was `32` (overload),
  for 142 samples. `voltage`, `sensor`, `temperature`, and `current` bits were never
  set. This definitively confirms the earlier `voltage` flag was a measurement artifact,
  not a real supply event. Supply held ~12.2 V throughout.

Also fixed in this run: the empty close now reaches the calibrated closed end (~1264,
near `min`=1234), instead of being cut off mid-travel by a fixed-time schedule (an
earlier run reversed at ~1445 before reaching closed).

### Run Command

```bash
conda run -n lerobot python src/lerobot/scripts/lerobot_gripper_full_telemetry_test.py \
  --port /dev/cu.usbmodem5B140324321 \
  --grip-dwell-s 9
```

## Practical Commands

Quickly probe current `/dev/cu.usbmodem*` ports for Feetech servos:

```bash
conda run -n lerobot python -c "import glob; from lerobot.motors import Motor, MotorNormMode; from lerobot.motors.feetech import FeetechMotorsBus
ports=sorted(glob.glob('/dev/cu.usbmodem*'))
for port in ports:
    bus=FeetechMotorsBus(port=port, motors={'probe': Motor(1,'sts3215',MotorNormMode.RANGE_M100_100)})
    try:
        bus.connect(handshake=False)
        bus.set_timeout(100)
        bus.set_baudrate(1000000)
        found=bus.broadcast_ping(num_retry=0) or {}
        print(f'{port}: {found}', flush=True)
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)"
```

Run auto-calibration on the currently detected SO101 bus:

```bash
conda run -n lerobot python src/lerobot/scripts/lerobot_auto_calibrate_feetech.py \
  --port /dev/cu.usbmodem5B140324321 \
  --save \
  --robot-id default
```
