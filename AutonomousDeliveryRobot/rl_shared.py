from __future__ import annotations

import numpy as np
import torch.nn as nn

import config as C


BASE_FEATURES = 17


def build_observation(
    cell,
    heading,
    goal,
    blocked_cells,
    confidence=1.0,
    walls=C.WALLS,
    anchors=C.ANCHORS,
    grid_cols=C.GRID_COLS,
    grid_rows=C.GRID_ROWS,
) -> np.ndarray:
    cx, cy = cell
    gx, gy = goal

    heading_oh = [0.0] * 4
    heading_oh[heading] = 1.0

    blocked_dirs = []
    for h in (C.HEADING_N, C.HEADING_E, C.HEADING_S, C.HEADING_W):
        dxh, dyh = C.HEADING_DELTA[h]
        nb = (cx + dxh, cy + dyh)
        if (not (0 <= nb[0] < grid_cols and 0 <= nb[1] < grid_rows)
                or nb in walls or nb in blocked_cells):
            blocked_dirs.append(1.0)
        else:
            blocked_dirs.append(0.0)

    if anchors:
        nearest = min(anchors.keys(), key=lambda a: abs(a[0] - cx) + abs(a[1] - cy))
        adx = (nearest[0] - cx) / grid_cols
        ady = (nearest[1] - cy) / grid_rows
    else:
        adx = ady = 0.0

    base = np.asarray([
        cx / grid_cols, cy / grid_rows,
        gx / grid_cols, gy / grid_rows,
        (gx - cx) / grid_cols, (gy - cy) / grid_rows,
        *heading_oh,
        *blocked_dirs,
        float(confidence),
        adx, ady,
    ], dtype=np.float32)

    occupancy = np.zeros(grid_cols * grid_rows, dtype=np.float32)
    for (wx, wy) in walls:
        occupancy[wy * grid_cols + wx] = 1.0
    for (bx, by) in blocked_cells:
        occupancy[by * grid_cols + bx] = 1.0

    anchor_mask = np.zeros(grid_cols * grid_rows, dtype=np.float32)
    for (ax, ay) in anchors.keys():
        anchor_mask[ay * grid_cols + ax] = 1.0

    return np.concatenate([base, occupancy, anchor_mask])


class DQN(nn.Module):
    def __init__(self, obs_dim: int, num_actions: int) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, num_actions),
        )

    def forward(self, x):
        return self.net(x)
