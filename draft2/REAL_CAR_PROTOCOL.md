# Real Car MQTT Protocol

This document defines the command/ack contract between laptop runtime and car ESP32 firmware.

## Topics

- `car/command` (laptop -> car)
- `car/status` (car -> laptop)
- `car/telemetry` (car -> laptop)

## Command Payload

JSON object:

```json
{
  "v": 1,
  "action_id": 42,
  "action": "FORWARD",
  "ts_ms": 1714833000123
}
```

Notes:
- `v` is protocol version (`1` currently).
- `action_id` must be unique per command transaction.
- `action` is one of:
  - `FORWARD`
  - `TURN_LEFT`
  - `TURN_RIGHT`
  - `STOP`

## Status Payload

JSON object:

```json
{
  "v": 1,
  "action_id": 42,
  "status": "DONE_FORWARD",
  "ok": true,
  "ts_ms": 1714833000960,
  "detail": "optional text"
}
```

`status` values used by the runtime:
- `DONE_FORWARD`
- `DONE_LEFT`
- `DONE_RIGHT`
- `IDLE`
- `ERROR`

## Telemetry Payload

JSON object:

```json
{
  "v": 1,
  "yaw_rel_deg": 3.4,
  "heading_idx": 0,
  "busy": false,
  "ts_ms": 1714833001000
}
```

## Transaction Rules

1. Laptop publishes one command with a new `action_id`.
2. Car executes exactly one primitive for that command.
3. Car publishes one terminal status (`DONE_*` or `ERROR`) with the same `action_id`.
4. Laptop waits up to `COMMAND_ACK_TIMEOUT_S`.
5. If timeout occurs, laptop retries up to `COMMAND_RETRY_LIMIT`, then sends `STOP` and aborts mission.

## Idempotency / Stale Messages

- `action_id` correlation prevents stale `status` packets from advancing mission state.
- Laptop must ignore statuses with mismatched `action_id`.