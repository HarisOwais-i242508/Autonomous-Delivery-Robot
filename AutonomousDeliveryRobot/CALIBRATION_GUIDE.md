# Grid-Car Calibration Guide

Use this after flashing `draft2/draft2.ino`.

## 1) Safety Setup

- Put the car on blocks first (wheels off ground).
- Confirm motor directions:
  - `FORWARD` command rotates both wheels forward.
  - `TURN_LEFT` / `TURN_RIGHT` rotate opposite directions.
- If right-turn yaw sign is reversed, set `YAW_RIGHT_SIGN = -1` in firmware.

## 2) IMU Stability

- Reset board while car is still.
- Wait for calibration + straight reference capture.
- Check telemetry on `car/telemetry`:
  - `yaw_rel_deg` should stay near 0 while stationary.

## 3) Turn Calibration (Near 90 Degrees)

Tune these in firmware:
- `TURN_TOL_DEG`
- `TURN_KP`
- `BASE_TURN_PWM`
- `TURN_TIMEOUT_MS`

Procedure:
1. Command `TURN_RIGHT` once.
2. Measure actual heading change (floor markings/protractor).
3. Repeat 10 times and compute average and spread.
4. Target: average within `90 +/- 8` degrees and low variance.

If overshoot:
- Lower `BASE_TURN_PWM` or `TURN_KP`, or increase settle time.

If undershoot:
- Increase `BASE_TURN_PWM` or `TURN_KP`, or increase timeout.

## 4) Forward-One-Cell Calibration (12 inch)

Tune:
- `FORWARD_CELL_MS`
- `BASE_FWD_PWM`
- `FWD_HEADING_KP`

Procedure:
1. Place on test lane with 12-inch marks.
2. Send `FORWARD` 10 times from same start.
3. Measure final stop positions.
4. Adjust `FORWARD_CELL_MS` until mean travel is ~12 inches.

If drifting sideways:
- Increase `FWD_HEADING_KP` slightly.

## 5) Acceptance Target

For mission runs, use this minimum acceptance:
- Right/left turns: mostly inside `90 +/- 10` degrees.
- Forward move: cell center landing error less than ~3 inches.
- MQTT command ack reliability: no dropped commands across 30 steps.