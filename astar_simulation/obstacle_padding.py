from __future__ import annotations

import math
from typing import Iterable, Set, Tuple

Voxel = Tuple[int, int, int]
GridSize = Tuple[int, int, int]


def padding_radius(speed: float, gain: float) -> int:
    if not math.isfinite(speed) or speed < 0.0:
        raise ValueError("speed must be finite and non-negative")
    if not math.isfinite(gain) or gain < 0.0:
        raise ValueError("gain must be finite and non-negative")
    return int(math.ceil(speed * gain))


def cubic_padding(center: Voxel, radius: int, grid_size: GridSize) -> Set[Voxel]:
    if radius < 0:
        raise ValueError("radius must be non-negative")
    size_x, size_y, size_z = grid_size
    cx, cy, cz = center
    return {
        (x, y, z)
        for x in range(max(0, cx - radius), min(size_x - 1, cx + radius) + 1)
        for y in range(max(0, cy - radius), min(size_y - 1, cy + radius) + 1)
        for z in range(max(0, cz - radius), min(size_z - 1, cz + radius) + 1)
    }


def padded_obstacle_union(
    obstacles: Iterable[Tuple[Voxel, float]],
    gain: float,
    grid_size: GridSize,
) -> Set[Voxel]:
    padded: Set[Voxel] = set()
    for voxel, speed in obstacles:
        padded.update(cubic_padding(voxel, padding_radius(speed, gain), grid_size))
    return padded
