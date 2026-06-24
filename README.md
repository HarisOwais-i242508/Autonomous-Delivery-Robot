# Autonomous-Delivery-Robot

Everything for the real-car pipeline lives in this folder. (Refer to `System Flow Diagram.pdf` and `Circuit Diagram.pdf` for basic comprehension of the system)

## Firmware (flash before anything else)

| Board | Sketch |
|--------|--------|
| **RC car ESP32** (Wi‑Fi + web UI, `/cmd`, `/status`, etc.) | `rc_esp32/rc_esp32.ino` |
| **ESP32‑CAM** (MJPEG stream for OpenCV) | `esp32cam_stream/esp32cam_stream.ino` |

There is also `AutonomousDeliveryRobot.ino` — same car, **MQTT** command/ack for `run_real_car.py` (RL loop). Use that path if you are not using the dashboard web car.

Set Wi‑Fi credentials in each sketch to match your router. After flashing, note each device’s IP (serial monitor or router DHCP list).

## Before you run Python — are both devices online?

1. **RC car ESP32** — ping it or open `http://<car-ip>/` in a browser. You should get the control page (or a simple response). If it does not load, fix Wi‑Fi or power before continuing.
2. **ESP32‑CAM** — open `http://<cam-ip>/` or hit the stream URL you use (often `http://<cam-ip>/stream`). If the stream does not load, fix the camera module / sketch / network first.

Replace the example IPs below with your real addresses.

## Dashboard (live car + camera + planner UI)

Install deps once:

```bash
pip install -r requirements.txt
```

Example run (adjust IPs):

```bash
python real_run_dashboard.py --car-url http://192.168.100.26 --camera-url http://192.168.100.20/stream
```

- `--car-url` — base URL of the **RC car** web server (no trailing slash required on most setups).
- `--camera-url` — ESP32‑CAM **MJPEG stream** URL (must match what your `esp32cam_stream.ino` serves, often `/stream`).

Use `python real_run_dashboard.py --help` for more options.

## Other useful scripts

- **Camera / CV only:**  
  `python cv_stream_test.py --camera-url http://<cam-ip>/stream`
- **RL + MQTT car** (needs `AutonomousDeliveryRobot.ino` + broker + `dqn_model.pth`):  
  `python run_real_car.py --camera-url http://<cam-ip>/stream --goals …`  
  See `config.py` for MQTT and map settings.

## ArUco markers (printable targets)

Generate marker images and (optional) a laid‑out PDF:

```bash
python create_6in_aruco_markers.py
python make_6in_aruco_pdf.py
```

`generate_aruco_markers.py` is an alternate/simple generator if you prefer a different layout.

## Files (reference)

- `real_run_dashboard.py` — Tk dashboard (car HTTP API + camera stream + grid sim hooks)
- `run_real_car.py` — CV + DQN + MQTT loop for the MQTT car firmware
- `cv_runtime_core.py` — shared stream client + detection helpers (used by the dashboard)
- `cv_stream_test.py` — standalone stream test
- `config.py`, `rl_shared.py` — shared constants / model
- `requirements.txt` — Python dependencies
- `REAL_CAR_PROTOCOL.md`, `CALIBRATION_GUIDE.md`, `MISSION_VALIDATION.md` — extra docs

## Notes

- Put trained **`dqn_model.pth`** in this `AutonomousDeliveryRobot` directory when using `run_real_car.py` (or set `MODEL_PATH` in `config.py`).
- Stream port/path can differ (e.g. `:81/stream` vs `/stream`); always match whatever your flashed `esp32cam_stream.ino` exposes.
