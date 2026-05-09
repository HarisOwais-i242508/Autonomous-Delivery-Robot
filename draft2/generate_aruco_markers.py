from __future__ import annotations

from pathlib import Path

import cv2


def main() -> None:
    # Match the runtime dictionary used by your CV code.
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    # 6-inch marker with 300 DPI print target.
    dpi = 300
    marker_inches = 6.0
    marker_px = int(marker_inches * dpi)  # 1800 px

    # Add a white border around each marker for safer detection.
    border_px = 220
    canvas_px = marker_px + (2 * border_px)

    out_dir = Path(__file__).resolve().parent / "aruco_markers_6in"
    out_dir.mkdir(parents=True, exist_ok=True)

    # Current anchor IDs in your project map.
    marker_ids = [0, 1, 2]
    for marker_id in marker_ids:
        marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_px)
        canvas = 255 * (marker_img * 0 + 1)
        canvas = cv2.copyMakeBorder(
            marker_img,
            border_px,
            border_px,
            border_px,
            border_px,
            cv2.BORDER_CONSTANT,
            value=255,
        )
        out_path = out_dir / f"aruco_id_{marker_id}_6in_300dpi.png"
        cv2.imwrite(str(out_path), canvas)
        print(f"Generated: {out_path}")


if __name__ == "__main__":
    main()
