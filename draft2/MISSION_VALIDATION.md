# Mission Validation Checklist

Mission: 10x5 grid, 12x12 inch cells, reach 3 goals, no reverse actions.

## Stage A: Connectivity

- [ ] Laptop can subscribe to `car/status` and `car/telemetry`.
- [ ] Car receives command on `car/command`.
- [ ] For each command, car returns matching `action_id`.
- [ ] Timeout/retry path works (disconnect broker briefly and recover).

## Stage B: Motion Primitive Quality

- [ ] `TURN_LEFT` and `TURN_RIGHT` complete near 90 degrees.
- [ ] `FORWARD` advances approximately one cell.
- [ ] No spontaneous movement while idle.

## Stage C: Perception

- [ ] Front-cell red detection marks blocked path.
- [ ] Front-cell green detection marks passable path.
- [ ] ArUco IDs are detected at intended anchor cells.
- [ ] Marker ID -> anchor cell mapping matches `config.py`.

## Stage D: Closed-Loop Decision

- [ ] Laptop sends one action at a time and waits for `DONE_*`.
- [ ] Belief heading updates after turn acknowledgements.
- [ ] Belief position updates after successful `FORWARD`.
- [ ] Safety override prevents known blocked `FORWARD`.

## Stage E: Full Mission

- [ ] Run with no dynamic obstacles and reach all 3 goals.
- [ ] Run with obstacles and still complete all 3 goals.
- [ ] No `STOP` abort due to repeated command timeout.
- [ ] No reverse actions issued during entire run.

## Logging Recommendations

Capture these for each step:
- `action_id`, `action`, `status`, `ok`
- `belief_cell`, `belief_heading`, `goal_idx`
- `front_blocked`, `marker_ids`, `confidence`

The `run_real_car.py` step log already prints most of this.