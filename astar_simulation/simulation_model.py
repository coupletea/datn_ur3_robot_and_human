from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Set, Tuple

from astar_simulation.obstacle_padding import padded_obstacle_union
from astar_simulation.obstacle_padding import cubic_padding, padding_radius

Voxel = Tuple[int, int, int]
GridSize = Tuple[int, int, int]


class SceneValidationError(ValueError):
    pass


@dataclass(frozen=True)
class Obstacle:
    obstacle_id: int
    voxel: Voxel
    speed: float = 0.0


@dataclass(frozen=True)
class ScanEntry:
    obstacle_id: int
    base_voxel: Voxel
    padded_voxels: Set[Voxel]


class SimulationModel:
    def __init__(self, grid_size: GridSize = (10, 10, 10), padding_gain: float = 1.0):
        self.grid_size = grid_size
        self.start: Optional[Voxel] = None
        self.goal: Optional[Voxel] = None
        self.current_robot: Optional[Voxel] = None
        self.obstacles: Dict[int, Obstacle] = {}
        self.padding_gain = self._valid_non_negative(padding_gain, "padding gain")
        self._next_obstacle_id = 1

    def _validate_voxel(self, voxel: Voxel) -> Voxel:
        if not isinstance(voxel, (tuple, list)) or len(voxel) != 3:
            raise SceneValidationError("voxel must contain three integer coordinates")
        result = tuple(voxel)
        if any(not isinstance(value, int) or isinstance(value, bool) for value in result):
            raise SceneValidationError("voxel coordinates must be integers")
        if any(value < 0 or value >= size for value, size in zip(result, self.grid_size)):
            raise SceneValidationError("voxel is outside the grid")
        return result  # type: ignore[return-value]

    @staticmethod
    def _valid_non_negative(value: float, label: str) -> float:
        value = float(value)
        if not math.isfinite(value) or value < 0.0:
            raise SceneValidationError(f"{label} must be finite and non-negative")
        return value

    def _base_voxels(self) -> Set[Voxel]:
        return {obstacle.voxel for obstacle in self.obstacles.values()}

    def _validate_endpoints(self, padded: Optional[Set[Voxel]] = None) -> None:
        if self.start is not None and self.goal is not None and self.start == self.goal:
            raise SceneValidationError("start and goal must be different")
        blocked = self.padded_obstacles() if padded is None else padded
        if self.start in blocked:
            raise SceneValidationError("padding would cover start")
        if self.goal in blocked:
            raise SceneValidationError("padding would cover goal")

    def set_start(self, voxel: Voxel) -> None:
        previous = self.start
        self.start = self._validate_voxel(voxel)
        try:
            self._validate_endpoints()
        except SceneValidationError:
            self.start = previous
            raise

    def initialize_robot(self) -> Voxel:
        if self.start is None:
            raise SceneValidationError("start is required")
        if self.current_robot is None:
            self.current_robot = self.start
        return self.current_robot

    def move_robot(self, voxel: Voxel) -> None:
        self.current_robot = self._validate_voxel(voxel)

    def set_goal(self, voxel: Voxel) -> None:
        previous = self.goal
        self.goal = self._validate_voxel(voxel)
        try:
            self._validate_endpoints()
        except SceneValidationError:
            self.goal = previous
            raise

    def add_obstacle(self, voxel: Voxel, speed: float = 0.0) -> int:
        voxel = self._validate_voxel(voxel)
        speed = self._valid_non_negative(speed, "speed")
        if voxel in self._base_voxels():
            raise SceneValidationError("an obstacle already occupies that voxel")
        obstacle_id = self._next_obstacle_id
        candidate = Obstacle(obstacle_id, voxel, speed)
        self.obstacles[obstacle_id] = candidate
        try:
            self._validate_endpoints()
        except SceneValidationError:
            del self.obstacles[obstacle_id]
            raise
        self._next_obstacle_id += 1
        return obstacle_id

    def set_obstacle_speed(self, obstacle_id: int, speed: float) -> None:
        if obstacle_id not in self.obstacles:
            raise SceneValidationError("unknown obstacle")
        speed = self._valid_non_negative(speed, "speed")
        previous = self.obstacles[obstacle_id]
        self.obstacles[obstacle_id] = Obstacle(obstacle_id, previous.voxel, speed)
        try:
            self._validate_endpoints()
        except SceneValidationError:
            self.obstacles[obstacle_id] = previous
            raise

    def set_padding_gain(self, gain: float) -> None:
        gain = self._valid_non_negative(gain, "padding gain")
        previous = self.padding_gain
        self.padding_gain = gain
        try:
            self._validate_endpoints()
        except SceneValidationError:
            self.padding_gain = previous
            raise

    def delete_obstacle(self, obstacle_id: int) -> None:
        if obstacle_id not in self.obstacles:
            raise SceneValidationError("unknown obstacle")
        del self.obstacles[obstacle_id]

    def clear_obstacles(self) -> None:
        self.obstacles.clear()

    def padded_obstacles(self) -> Set[Voxel]:
        return padded_obstacle_union(
            ((obstacle.voxel, obstacle.speed) for obstacle in self.obstacles.values()),
            gain=self.padding_gain,
            grid_size=self.grid_size,
        )

    def scan_entries(self):
        return [
            ScanEntry(
                obstacle_id=obstacle.obstacle_id,
                base_voxel=obstacle.voxel,
                padded_voxels=cubic_padding(
                    obstacle.voxel,
                    padding_radius(obstacle.speed, self.padding_gain),
                    self.grid_size,
                ),
            )
            for obstacle in sorted(self.obstacles.values(), key=lambda item: item.obstacle_id)
        ]
