# FD Servo Comparison Data — 2026-06-24

This folder contains the curated raw exports from the Windows FD Debug V1.9.8.3
comparison tests of STS3215 servos A, B, and C.

The full experiment history, including the earlier Python/LeRobot comparisons and
the command-method investigation, is recorded in
[Gavin SO101 Findings](../../docs/2026-06-17-gavin-so101-findings.md).

## Hardware Status

- **SB-C** is the retained driver board and the controlled baseline for the latest
  A/B/C comparison.
- **SB-A** and **SB-B** were deemed broken after connection testing and are being
  discarded.
- Data collected through SB-A is retained for historical context only and should
  not be used as the primary acceptance baseline.
- No successful servo telemetry dataset was collected through SB-B.

## Test Protocol

Unless a filename says otherwise:

- Servo connected alone to the driver board's left port.
- Serial baud rate: `1,000,000`.
- Position mode through FD Debug's combined position/speed/acceleration control.
- Acceleration: `8`.
- Travel: `512` encoder ticks in each direction.
- Three outward-and-return cycles.
- Full FD telemetry enabled and exported.
- Torque released after the test.

The `speed1000` A/B/C tests were all run through SB-C with the same procedure and
are the directly comparable acceptance dataset.

## Files

| File | Servo | Board | Speed | Rows | Peak velocity | Median moving velocity | Load range | Max current | Temperature | Voltage | Errors |
|------|-------|-------|------:|-----:|--------------:|-----------------------:|-----------:|------------:|------------:|--------:|-------:|
| `fd_servo_a_sbc_left_speed250.txt` | A | SB-C | 250 | 811 | 300 | 250 | -96..100 | 2 | 29–66 °C | 11.9–12.0 V | 0 |
| `fd_servo_a_sbc_left_speed1000.txt` | A | SB-C | 1000 | 640 | 650 | 350 | -204..204 | 4 | 31–46 °C | 12.0 V | 0 |
| `fd_servo_b_sba_left_speed250.txt` | B | SB-A | 250 | 2075 | 300 | 250 | -92..92 | 2 | 30–43 °C | 12.3–12.5 V | 0 |
| `fd_servo_b_sbc_left_speed1000.txt` | B | SB-C | 1000 | 705 | 650 | 350 | -204..204 | 4 | 26–48 °C | 11.9–12.2 V | 0 |
| `fd_servo_c_sbc_left_speed1000.txt` | C | SB-C | 1000 | 814 | 650 | 350 | -212..212 | 4 | 26–41 °C | 12.2–12.3 V | 0 |

The A speed-250 file contains isolated implausible temperature samples, including
66 °C, while almost all samples remained at 29 °C. Similar isolated temperature
spikes appeared in other FD captures. Treat these as likely read/export artifacts,
not established physical heating.

## FD Export Columns

| Column | Meaning |
|--------|---------|
| `No` | FD sample sequence number; not a calibrated timestamp |
| `Pos` | Present position |
| `Gol` | Goal position |
| `Ft` | Signed torque/load or controller-effort value reported by FD |
| `V` | Present velocity |
| `C` | Present current |
| `T` | Temperature in degrees Celsius |
| `Vol` | Voltage in tenths of a volt |
| `Err` | Servo error/status value |

## Current Conclusion

The controlled SB-C speed-1000 results for A, B, and C are effectively identical:

- Peak reported velocity: `650` for all three.
- Median moving velocity: `350` for all three.
- Maximum current: `4` for all three.
- Signed load range: approximately `±204`.
- No status errors.
- Final position error: one or two ticks.

Within this test's scope, servo A shows no mechanical or electrical abnormality
relative to fresh servos B and C. This is evidence of normal operation, not proof
that any servo is perfect under every load, duration, or environmental condition.
