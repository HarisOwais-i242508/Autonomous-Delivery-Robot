"""
Standalone ESP32-CAM stream CV test.

Shows:
- front-cell blocked/open classification (red vs green)
- ArUco marker IDs in frame

Usage:
  python cv_stream_test.py --camera-url http://192.168.1.50:81/stream
"""

from __future__ import annotations

import argparse
import socket
import time
import urllib.error
import urllib.request

import cv2
import numpy as np


def open_mjpeg_stream(url: str, timeout_s: float = 12.0):
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    return urllib.request.urlopen(req, timeout=timeout_s)

def fetch_snapshot_frame(url: str, timeout_s: float = 2.0):
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0", "Cache-Control": "no-cache"})
    with urllib.request.urlopen(req, timeout=timeout_s) as resp:
        data = resp.read()
    arr = np.frombuffer(data, dtype=np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera-url", required=True)
    ap.add_argument("--mode", choices=["auto", "opencv", "mjpeg", "snapshot"], default="snapshot")
    ap.add_argument("--snapshot-url", default="", help="Optional explicit snapshot URL, e.g. http://ip:81/jpg")
    ap.add_argument("--display-scale", type=float, default=2.0, help="Scale factor for preview window")
    args = ap.parse_args()
    print(f"[INFO] Starting mode={args.mode} camera={args.camera_url}", flush=True)

    cap = None
    use_snapshot = args.mode == "snapshot"
    use_mjpeg_fallback = args.mode == "mjpeg"

    snapshot_url = args.snapshot_url.strip()
    if not snapshot_url:
        # Default mapping: /stream -> /jpg
        snapshot_url = args.camera_url.replace("/stream", "/jpg")

    if args.mode in ("auto", "opencv"):
        cap = cv2.VideoCapture(args.camera_url)
        if cap.isOpened():
            ok, _frame = cap.read()
            if ok:
                print("[INFO] Using OpenCV VideoCapture backend.")
            elif args.mode == "auto":
                print("[WARN] VideoCapture opened but first frame failed; switching to MJPEG fallback.")
                cap.release()
                cap = None
                use_mjpeg_fallback = True
            else:
                raise RuntimeError("VideoCapture opened but failed to read first frame.")
        elif args.mode == "opencv":
            raise RuntimeError(f"Cannot open stream with VideoCapture: {args.camera_url}")
        else:
            print("[WARN] VideoCapture open failed; switching to MJPEG fallback.")
            use_mjpeg_fallback = True

    stream = None
    byte_buf = b""
    snapshot_fail_count = 0
    if use_snapshot:
        print(f"[INFO] Using snapshot mode: {snapshot_url}")
    elif use_mjpeg_fallback:
        print("[INFO] Using MJPEG fallback parser.")
        stream = open_mjpeg_stream(args.camera_url, timeout_s=5.0)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    last_ok_frame_ts = time.time()
    last_warn_ts = 0.0
    while True:
        if use_snapshot:
            try:
                frame = fetch_snapshot_frame(snapshot_url, timeout_s=2.0)
                snapshot_fail_count = 0
            except (TimeoutError, socket.timeout, OSError, urllib.error.URLError) as e:
                snapshot_fail_count += 1
                now = time.time()
                if now - last_warn_ts > 1.0:
                    print(f"[WARN] Snapshot fetch failed: {e}")
                    last_warn_ts = now
                if snapshot_fail_count >= 5:
                    print("[WARN] Snapshot repeatedly failed; switching to MJPEG stream mode.")
                    use_snapshot = False
                    use_mjpeg_fallback = True
                    stream = open_mjpeg_stream(args.camera_url, timeout_s=12.0)
                    byte_buf = b""
                continue
            if frame is None:
                now = time.time()
                if now - last_warn_ts > 1.0:
                    print("[WARN] Snapshot decode failed.")
                    last_warn_ts = now
                continue
        elif cap is not None:
            ok, frame = cap.read()
            if not ok:
                if time.time() - last_ok_frame_ts > 2.0:
                    print("[WARN] No frames from VideoCapture for 2s.")
                    last_ok_frame_ts = time.time()
                continue
        else:
            try:
                chunk = stream.read(4096)
            except (TimeoutError, socket.timeout, OSError) as e:
                print(f"[WARN] Stream read timeout/error: {e}. Reconnecting...")
                try:
                    stream.close()
                except Exception:
                    pass
                stream = open_mjpeg_stream(args.camera_url, timeout_s=5.0)
                byte_buf = b""
                continue
            if not chunk:
                now = time.time()
                if now - last_warn_ts > 1.0:
                    print("[WARN] Empty stream chunk. Reconnecting...")
                    last_warn_ts = now
                try:
                    stream.close()
                except Exception:
                    pass
                stream = open_mjpeg_stream(args.camera_url, timeout_s=5.0)
                byte_buf = b""
                continue
            byte_buf += chunk
            a = byte_buf.find(b"\xff\xd8")  # JPEG SOI
            if a == -1:
                # Keep a small rolling tail so buffer doesn't grow forever.
                if len(byte_buf) > 32768:
                    byte_buf = byte_buf[-4096:]
                if time.time() - last_ok_frame_ts > 4.0:
                    print("[WARN] JPEG SOI not found for 4s. Reconnecting...")
                    try:
                        stream.close()
                    except Exception:
                        pass
                    stream = open_mjpeg_stream(args.camera_url, timeout_s=12.0)
                    byte_buf = b""
                continue

            # Drop multipart headers/preamble bytes before SOI.
            if a > 0:
                byte_buf = byte_buf[a:]

            # Look for EOI AFTER SOI (critical for continuous MJPEG parsing).
            b = byte_buf.find(b"\xff\xd9", 2)
            if b == -1:
                if time.time() - last_ok_frame_ts > 4.0:
                    print("[WARN] JPEG EOI not found for 4s. Reconnecting...")
                    try:
                        stream.close()
                    except Exception:
                        pass
                    stream = open_mjpeg_stream(args.camera_url, timeout_s=12.0)
                    byte_buf = b""
                continue

            # If buffer has multiple complete JPEGs, keep only the newest one
            # to avoid seconds of lag from queued old frames.
            latest_start = 0
            search_pos = 2
            while True:
                na = byte_buf.find(b"\xff\xd8", search_pos)
                if na == -1 or na > b:
                    break
                latest_start = na
                search_pos = na + 2

            jpg = byte_buf[latest_start:b + 2]
            byte_buf = byte_buf[b + 2:]
            arr = np.frombuffer(jpg, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                continue
        last_ok_frame_ts = time.time()

        h, w = frame.shape[:2]
        y0, y1 = int(0.35 * h), int(0.9 * h)
        x0, x1 = int(0.30 * w), int(0.70 * w)
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

        red_score = (0.55 * red_hue_ratio) + (0.35 * red_dom_ratio) + (0.10 * red_idx)
        green_score = (0.55 * green_hue_ratio) + (0.35 * green_dom_ratio) + (0.10 * green_idx)

        red_ratio = red_score
        green_ratio = green_score
        blocked = (red_score >= 0.20) or (
            (red_score > 0.045) and
            (red_score > green_score * 1.15) and
            ((red_score - green_score) > 0.012)
        )

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _rej = aruco_detector.detectMarkers(gray)
        marker_ids = []
        if ids is not None:
            marker_ids = [int(v) for v in ids.flatten().tolist()]
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        cv2.rectangle(frame, (x0, y0), (x1, y1), (255, 200, 0), 2)
        status = "BLOCKED" if blocked else "OPEN"
        color = (0, 0, 255) if blocked else (0, 255, 0)
        cv2.putText(frame, f"Front: {status}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(frame, f"red={red_ratio:.3f} green={green_ratio:.3f}",
                    (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(frame, f"delta={(red_ratio - green_ratio):.3f}",
                    (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(frame, f"markers={marker_ids}",
                    (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        if args.display_scale != 1.0:
            frame_show = cv2.resize(
                frame,
                None,
                fx=args.display_scale,
                fy=args.display_scale,
                interpolation=cv2.INTER_LINEAR,
            )
        else:
            frame_show = frame

        cv2.imshow("ESP32-CAM CV Test", frame_show)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    if cap is not None:
        cap.release()
    if stream is not None:
        stream.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()