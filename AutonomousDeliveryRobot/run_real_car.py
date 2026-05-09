"""
Real-car runtime loop:
ESP32-CAM stream -> CV -> RL action -> MQTT command -> ESP32 car ack.
"""

from __future__ import annotations

import argparse
import json
import threading
import time
import urllib.request
from collections import Counter, deque
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np
import paho.mqtt.client as mqtt
import torch
import torch.nn as nn

import config as C
from rl_shared import DQN, build_observation


EXPECTED_DONE = {
    C.ACTION_FORWARD: C.STATUS_DONE_FORWARD,
    C.ACTION_TURN_LEFT: C.STATUS_DONE_LEFT,
    C.ACTION_TURN_RIGHT: C.STATUS_DONE_RIGHT,
}


def parse_cell(text: str) -> Tuple[int, int]:
    x_str, y_str = text.split(",", 1)
    x, y = int(x_str), int(y_str)
    if not (0 <= x < C.GRID_COLS and 0 <= y < C.GRID_ROWS):
        raise ValueError(f"Cell out of range: {text}")
    return x, y


def plan_to_targets(
    start: Tuple[int, int],
    heading: int,
    blocked: set,
    targets: set,
    walls: Optional[set] = None,
) -> Optional[str]:
    if walls is None:
        walls = C.WALLS
    if start in targets:
        return None

    q = deque([start])
    parents: dict = {start: (None, None)}
    found: Optional[Tuple[int, int]] = None

    while q:
        cell = q.popleft()
        if cell in targets and cell != start:
            found = cell
            break
        for dh in (C.HEADING_N, C.HEADING_E, C.HEADING_S, C.HEADING_W):
            dx, dy = C.HEADING_DELTA[dh]
            nb = (cell[0] + dx, cell[1] + dy)
            if not (0 <= nb[0] < C.GRID_COLS and 0 <= nb[1] < C.GRID_ROWS):
                continue
            if nb in walls or nb in blocked:
                continue
            if nb in parents:
                continue
            parents[nb] = (cell, dh)
            q.append(nb)

    if found is None:
        return None

    cur = found
    first_heading: Optional[int] = None
    while parents[cur][0] is not None:
        prev, taken_heading = parents[cur]
        if prev == start:
            first_heading = taken_heading
            break
        cur = prev

    if first_heading is None:
        return None

    diff = (first_heading - heading) % 4
    if diff == 0:
        return C.ACTION_FORWARD
    if diff == 1:
        return C.ACTION_TURN_RIGHT
    if diff == 3:
        return C.ACTION_TURN_LEFT
    return C.ACTION_TURN_RIGHT


class DQNPolicy:
    def __init__(self, model_path: str = C.MODEL_PATH) -> None:
        ckpt = torch.load(model_path, map_location="cpu")
        obs_dim = ckpt["obs_dim"]
        num_actions = ckpt["num_actions"]
        state_dict = ckpt["state_dict"]

        # Prefer current model first.
        self.net = DQN(obs_dim, num_actions)
        try:
            self.net.load_state_dict(state_dict)
        except RuntimeError:
            # Backward-compatible loader for older checkpoints:
            # net: Linear(obs,256)->ReLU->Linear(256,256)->ReLU->Linear(256,num_actions)
            if "net.4.weight" in state_dict and "net.6.weight" not in state_dict:
                legacy = nn.Sequential(
                    nn.Linear(obs_dim, 256),
                    nn.ReLU(),
                    nn.Linear(256, 256),
                    nn.ReLU(),
                    nn.Linear(256, num_actions),
                )
                wrapped = nn.Module()
                wrapped.add_module("net", legacy)
                wrapped.load_state_dict(state_dict)
                self.net = wrapped
                print("[INFO] Loaded legacy DQN checkpoint architecture.")
            else:
                raise
        self.net.eval()

    def act(self, obs: np.ndarray) -> int:
        with torch.no_grad():
            q = self.net(torch.from_numpy(obs).unsqueeze(0))
            return int(q.argmax(dim=1).item())


class MqttBridge:
    def __init__(self, host: str, port: int, username: str, password: str) -> None:
        self.client = mqtt.Client(client_id="laptop-rl-runtime", clean_session=True)
        if username:
            self.client.username_pw_set(username, password)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self._status_by_action_id: dict[int, dict] = {}
        self._cv = threading.Condition()
        self.client.connect(host, port, C.MQTT_KEEPALIVE)
        self.client.loop_start()

    def close(self) -> None:
        self.client.loop_stop()
        self.client.disconnect()

    def _on_connect(self, client, _userdata, _flags, rc):
        if rc == 0:
            client.subscribe(C.TOPIC_STATUS)
            client.subscribe(C.TOPIC_TELEMETRY)

    def _on_message(self, _client, _userdata, msg):
        if msg.topic != C.TOPIC_STATUS:
            return
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except Exception:
            payload = {C.KEY_STATUS: msg.payload.decode("utf-8", errors="replace")}
        action_id = int(payload.get(C.KEY_ACTION_ID, -1))
        with self._cv:
            self._status_by_action_id[action_id] = payload
            self._cv.notify_all()

    def send_action(self, action_id: int, action: str) -> None:
        payload = {
            C.KEY_VERSION: C.PROTO_VERSION,
            C.KEY_ACTION_ID: action_id,
            C.KEY_ACTION: action,
            C.KEY_TS_MS: int(time.time() * 1000),
        }
        self.client.publish(C.TOPIC_COMMAND, json.dumps(payload), qos=1)

    def send_stop(self) -> None:
        payload = {
            C.KEY_VERSION: C.PROTO_VERSION,
            C.KEY_ACTION_ID: -1,
            C.KEY_ACTION: C.ACTION_STOP,
            C.KEY_TS_MS: int(time.time() * 1000),
        }
        self.client.publish(C.TOPIC_COMMAND, json.dumps(payload), qos=1)

    def wait_status(self, action_id: int, timeout_s: float) -> Optional[dict]:
        deadline = time.time() + timeout_s
        with self._cv:
            while time.time() < deadline:
                status = self._status_by_action_id.get(action_id)
                if status is not None:
                    return status
                self._cv.wait(timeout=0.05)
        return None


@dataclass
class Perception:
    front_blocked: bool
    red_ratio: float
    green_ratio: float
    marker_ids: list[int]


class Vision:
    def __init__(self) -> None:
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def detect(self, frame: np.ndarray) -> Perception:
        h, w = frame.shape[:2]
        y0, y1 = int(0.05 * h), int(0.38 * h)
        x0, x1 = int(0.18 * w), int(0.82 * w)
        roi = frame[y0:y1, x0:x1]

        # Gray-world normalization reduces green tint bias before thresholding.
        roi_f = roi.astype(np.float32) + 1.0
        b_ch, g_ch, r_ch = cv2.split(roi_f)
        b_m, g_m, r_m = np.mean(b_ch), np.mean(g_ch), np.mean(r_ch)
        gray_m = (b_m + g_m + r_m) / 3.0
        b_ch *= gray_m / b_m
        g_ch *= gray_m / g_m
        r_ch *= gray_m / r_m
        roi_norm = np.clip(cv2.merge((b_ch, g_ch, r_ch)), 0, 255).astype(np.uint8)

        hsv = cv2.cvtColor(roi_norm, cv2.COLOR_BGR2HSV)
        sat = hsv[:, :, 1]
        val = hsv[:, :, 2]
        chroma_mask = (sat > 50) & (val > 45)
        chroma_count = max(1, int(np.count_nonzero(chroma_mask)))

        red1 = cv2.inRange(hsv, (0, 65, 50), (12, 255, 255))
        red2 = cv2.inRange(hsv, (158, 65, 50), (179, 255, 255))
        red_hsv = cv2.bitwise_or(red1, red2)
        green_hsv = cv2.inRange(hsv, (36, 45, 45), (96, 255, 255))

        red_hue_ratio = float(np.count_nonzero((red_hsv > 0) & chroma_mask)) / chroma_count
        green_hue_ratio = float(np.count_nonzero((green_hsv > 0) & chroma_mask)) / chroma_count

        b_u8, g_u8, r_u8 = cv2.split(roi_norm)
        sum_rgb = r_u8.astype(np.float32) + g_u8.astype(np.float32) + b_u8.astype(np.float32) + 1.0
        red_idx = np.mean(np.clip((r_u8.astype(np.float32) - g_u8.astype(np.float32)) / sum_rgb, 0.0, 1.0))
        green_idx = np.mean(np.clip((g_u8.astype(np.float32) - r_u8.astype(np.float32)) / sum_rgb, 0.0, 1.0))

        red_dom = ((r_u8 > (g_u8 * 1.22)) & (r_u8 > (b_u8 * 1.22)) & (r_u8 > 70))
        green_dom = ((g_u8 > (r_u8 * 1.18)) & (g_u8 > (b_u8 * 1.15)) & (g_u8 > 70))

        total = float(roi.shape[0] * roi.shape[1])
        red_dom_ratio = float(np.count_nonzero(red_dom)) / total
        green_dom_ratio = float(np.count_nonzero(green_dom)) / total

        red_ratio = (0.55 * red_hue_ratio) + (0.35 * red_dom_ratio) + (0.10 * red_idx)
        green_ratio = (0.55 * green_hue_ratio) + (0.35 * green_dom_ratio) + (0.10 * green_idx)
        front_blocked = (red_ratio >= 0.20) or (
            (red_ratio > 0.045) and
            (red_ratio > green_ratio * 1.15) and
            ((red_ratio - green_ratio) > 0.012)
        )

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _rej = self.aruco_detector.detectMarkers(gray)
        marker_ids = []
        if ids is not None and len(corners) > 0:
            marker_ids = [int(v) for v in ids.flatten().tolist()]

        return Perception(front_blocked, red_ratio, green_ratio, marker_ids)


def fetch_snapshot_frame(url: str, timeout_s: float = 2.0):
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0", "Cache-Control": "no-cache"})
    with urllib.request.urlopen(req, timeout=timeout_s) as resp:
        data = resp.read()
    arr = np.frombuffer(data, dtype=np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)


class RealNavigator:
    def __init__(self, goals: list[Tuple[int, int]], policy: DQNPolicy) -> None:
        self.goals = goals
        self.goal_idx = 0
        self.policy = policy
        self.localized = False
        self.belief_cell: Optional[Tuple[int, int]] = None
        self.belief_heading = C.HEADING_N
        self.force_anchor_seek = False
        self.confidence = C.CONFIDENCE_INIT
        self.known_blocked_cells: set = set()
        self.recent_cells: deque = deque(maxlen=10)
        self.cell_visits: Counter = Counter()
        self.last_anchor_cell: Optional[Tuple[int, int]] = None
        self.steps_since_anchor = 0

    @property
    def current_goal(self) -> Tuple[int, int]:
        return self.goals[self.goal_idx]

    def done(self) -> bool:
        return self.goal_idx >= len(self.goals)

    def _explore_action(self, front_blocked: bool) -> str:
        return C.ACTION_TURN_RIGHT if front_blocked else C.ACTION_FORWARD

    def _blocked_dirs_sensor(self) -> Tuple[int, int, int, int]:
        if self.belief_cell is None:
            return (0, 0, 0, 0)
        cx, cy = self.belief_cell
        out = []
        for h in (C.HEADING_N, C.HEADING_E, C.HEADING_S, C.HEADING_W):
            dx, dy = C.HEADING_DELTA[h]
            nb = (cx + dx, cy + dy)
            if not (0 <= nb[0] < C.GRID_COLS and 0 <= nb[1] < C.GRID_ROWS):
                out.append(1)
            elif nb in C.WALLS or nb in self.known_blocked_cells:
                out.append(1)
            else:
                out.append(0)
        return tuple(out)  # type: ignore[return-value]

    def _check_anchor(self, cell: Tuple[int, int]) -> None:
        if cell not in C.ANCHORS:
            return
        self._apply_anchor_observation(cell, cell)

    def _apply_anchor_observation(self, anchor_cell: Tuple[int, int], current_cell: Tuple[int, int]) -> None:
        self.localized = True
        self.belief_cell = current_cell
        self.confidence = C.CONFIDENCE_RESET_ON_ANCHOR
        self.force_anchor_seek = False
        is_new = (self.last_anchor_cell != anchor_cell) or (self.steps_since_anchor > 0)
        self.last_anchor_cell = anchor_cell
        self.steps_since_anchor = 0
        if is_new:
            self.recent_cells.clear()
            self.recent_cells.append(current_cell)
            self.cell_visits[current_cell] = 1

    def update_from_perception(self, p: Perception) -> None:
        marker_to_cell = {v: k for k, v in C.ANCHORS.items()}
        for marker_id in p.marker_ids:
            marker_cell = marker_to_cell.get(marker_id)
            if marker_cell is not None:
                dx, dy = C.HEADING_DELTA[self.belief_heading]
                inferred_current = (marker_cell[0] - dx, marker_cell[1] - dy)
                if (
                    0 <= inferred_current[0] < C.GRID_COLS
                    and 0 <= inferred_current[1] < C.GRID_ROWS
                    and inferred_current not in C.WALLS
                ):
                    self._apply_anchor_observation(marker_cell, inferred_current)
                break

        if self.localized and self.belief_cell is not None and p.front_blocked:
            dx, dy = C.HEADING_DELTA[self.belief_heading]
            front = (self.belief_cell[0] + dx, self.belief_cell[1] + dy)
            if 0 <= front[0] < C.GRID_COLS and 0 <= front[1] < C.GRID_ROWS:
                self.known_blocked_cells.add(front)

    def pick_action(self, p: Perception) -> Tuple[str, str]:
        if self.confidence < C.CONFIDENCE_LOW_THRESHOLD:
            self.force_anchor_seek = True

        if self.force_anchor_seek:
            if self.belief_cell is None:
                return "anchor-seek-explore", self._explore_action(p.front_blocked)
            action = plan_to_targets(self.belief_cell, self.belief_heading,
                                     set(self.known_blocked_cells), set(C.ANCHORS.keys()), C.WALLS)
            if action is not None:
                return "anchor-seek", action
            return "anchor-seek-fallback", self._explore_action(p.front_blocked)

        if self.belief_cell is None:
            return "explore", self._explore_action(p.front_blocked)

        obs = build_observation(
            cell=self.belief_cell,
            heading=self.belief_heading,
            goal=self.current_goal,
            blocked_cells=self.known_blocked_cells,
            confidence=self.confidence,
            walls=C.WALLS,
            anchors=C.ANCHORS,
            grid_cols=C.GRID_COLS,
            grid_rows=C.GRID_ROWS,
        )
        action = C.ACTIONS[self.policy.act(obs)]

        blocked_dirs = self._blocked_dirs_sensor()
        if action == C.ACTION_FORWARD and blocked_dirs[self.belief_heading] == 1:
            detour = plan_to_targets(self.belief_cell, self.belief_heading,
                                     set(self.known_blocked_cells), {self.current_goal}, C.WALLS)
            if detour is not None:
                return "rl-reroute", detour
            return "rl-reroute-fallback", self._explore_action(True)
        return "rl", action

    def apply_action_result(self, action: str, ok: bool) -> None:
        if action == C.ACTION_TURN_LEFT and ok:
            self.belief_heading = (self.belief_heading - 1) % 4
        elif action == C.ACTION_TURN_RIGHT and ok:
            self.belief_heading = (self.belief_heading + 1) % 4
        elif action == C.ACTION_FORWARD and ok and self.localized and self.belief_cell is not None:
            dx, dy = C.HEADING_DELTA[self.belief_heading]
            self.belief_cell = (self.belief_cell[0] + dx, self.belief_cell[1] + dy)
            self.steps_since_anchor += 1
            self.confidence = max(0.0, self.confidence - C.CONFIDENCE_DECAY_PER_STEP)

        if self.belief_cell is not None:
            self.recent_cells.append(self.belief_cell)
            self.cell_visits[self.belief_cell] += 1
            self._check_anchor(self.belief_cell)
            while self.goal_idx < len(self.goals) and self.belief_cell == self.goals[self.goal_idx]:
                self.goal_idx += 1


def wait_ack_with_retry(bridge: MqttBridge, action_id: int, action: str) -> Optional[dict]:
    for attempt in range(C.COMMAND_RETRY_LIMIT + 1):
        bridge.send_action(action_id, action)
        status = bridge.wait_status(action_id, C.COMMAND_ACK_TIMEOUT_S)
        if status is not None:
            return status
        if attempt < C.COMMAND_RETRY_LIMIT:
            print(f"[WARN] action_id={action_id} timed out, retrying...")
    return None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera-url", required=True, help="ESP32-CAM MJPEG URL")
    ap.add_argument("--camera-mode", choices=["stream", "snapshot"], default="snapshot")
    ap.add_argument("--snapshot-url", default="", help="Optional explicit /jpg URL")
    ap.add_argument("--goals", nargs=3, required=True, metavar=("G1", "G2", "G3"))
    ap.add_argument("--broker-host", default=C.MQTT_BROKER_HOST)
    ap.add_argument("--broker-port", type=int, default=C.MQTT_BROKER_PORT)
    ap.add_argument("--mqtt-user", default=C.MQTT_USERNAME)
    ap.add_argument("--mqtt-pass", default=C.MQTT_PASSWORD)
    ap.add_argument("--model-path", default=C.MODEL_PATH)
    args = ap.parse_args()

    goals = [parse_cell(g) for g in args.goals]
    cap = None
    snapshot_url = args.snapshot_url.strip() or args.camera_url.replace("/stream", "/jpg")
    if args.camera_mode == "stream":
        cap = cv2.VideoCapture(args.camera_url)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open camera stream: {args.camera_url}")
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    policy = DQNPolicy(model_path=args.model_path)
    nav = RealNavigator(goals, policy)
    vision = Vision()
    bridge = MqttBridge(args.broker_host, args.broker_port, args.mqtt_user, args.mqtt_pass)

    action_id = 1
    try:
        while not nav.done():
            if args.camera_mode == "snapshot":
                try:
                    frame = fetch_snapshot_frame(snapshot_url, timeout_s=2.0)
                except Exception as e:
                    print(f"[WARN] Snapshot fetch failed: {e}")
                    time.sleep(0.05)
                    continue
                if frame is None:
                    time.sleep(0.03)
                    continue
            else:
                # Drain stale buffered frames to reduce end-to-end latency.
                for _ in range(2):
                    cap.grab()
                ok, frame = cap.read()
                if not ok:
                    time.sleep(0.05)
                    continue

            p = vision.detect(frame)
            nav.update_from_perception(p)
            mode, action = nav.pick_action(p)

            status_msg = wait_ack_with_retry(bridge, action_id, action)
            if status_msg is None:
                print(f"[ERROR] No ACK for action_id={action_id}, sending STOP.")
                bridge.send_stop()
                break

            status = str(status_msg.get(C.KEY_STATUS, ""))
            expected = EXPECTED_DONE.get(action, "")
            ok_action = status == expected and bool(status_msg.get(C.KEY_OK, True))
            nav.apply_action_result(action, ok_action)

            print(
                f"[STEP] id={action_id} mode={mode} action={action} status={status} "
                f"ok={ok_action} cell={nav.belief_cell} heading={nav.belief_heading} "
                f"goal_idx={nav.goal_idx}/{len(goals)} blocked={p.front_blocked} "
                f"red={p.red_ratio:.3f} green={p.green_ratio:.3f} markers={p.marker_ids}"
            )
            action_id += 1
            time.sleep(0.05)
    finally:
        bridge.close()
        if cap is not None:
            cap.release()


if __name__ == "__main__":
    main()
