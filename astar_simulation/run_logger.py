from __future__ import annotations

import csv
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict

from astar_simulation.planner_adapter import PlanningOutcome
from astar_simulation.simulation_model import SimulationModel

CSV_FIELDS = [
    "timestamp", "scenario_id", "map_size_x", "map_size_y", "map_size_z",
    "start_x", "start_y", "start_z", "goal_x", "goal_y", "goal_z",
    "obstacle_count", "obstacle_density", "success", "reason",
    "planning_time_ms", "first_solution_time_ms", "expanded_nodes", "iterations",
    "path_length", "path_cost", "path_distance_m", "epsilon_start",
    "epsilon_final", "epsilon_satisfied", "is_replan", "changed_obstacle_count",
]


class RunLogger:
    def __init__(self, path: Path):
        self.path = Path(path)

    def log_plan(
        self,
        scenario_id: str,
        model: SimulationModel,
        outcome: PlanningOutcome,
        voxel_size_m: float,
    ) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        write_header = not self.path.exists() or self.path.stat().st_size == 0
        row = self._build_row(scenario_id, model, outcome, voxel_size_m)
        with self.path.open("a", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
            if write_header:
                writer.writeheader()
            writer.writerow(row)

    @staticmethod
    def _build_row(
        scenario_id: str,
        model: SimulationModel,
        outcome: PlanningOutcome,
        voxel_size_m: float,
    ) -> Dict[str, object]:
        result = outcome.result
        metrics = result.metrics if result is not None else {}
        plan_start = model.current_robot if model.current_robot is not None else model.start
        goal = model.goal
        blocked_count = len(model.padded_obstacles())
        map_volume = model.grid_size[0] * model.grid_size[1] * model.grid_size[2]
        path_cost = metrics.get("path_cost", "")
        distance = float(path_cost) * voxel_size_m if isinstance(path_cost, (int, float)) else ""
        return {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "scenario_id": scenario_id,
            "map_size_x": model.grid_size[0],
            "map_size_y": model.grid_size[1],
            "map_size_z": model.grid_size[2],
            "start_x": plan_start[0] if plan_start else "",
            "start_y": plan_start[1] if plan_start else "",
            "start_z": plan_start[2] if plan_start else "",
            "goal_x": goal[0] if goal else "",
            "goal_y": goal[1] if goal else "",
            "goal_z": goal[2] if goal else "",
            "obstacle_count": blocked_count,
            "obstacle_density": blocked_count / map_volume,
            "success": str(bool(result and result.success)).lower(),
            "reason": result.reason if result else outcome.status,
            "planning_time_ms": metrics.get("planning_time_ms", ""),
            "first_solution_time_ms": metrics.get("first_solution_time_ms", ""),
            "expanded_nodes": metrics.get("expanded_steps", ""),
            "iterations": metrics.get("iterations", ""),
            "path_length": metrics.get("path_length", ""),
            "path_cost": path_cost,
            "path_distance_m": distance,
            "epsilon_start": metrics.get("epsilon_start", ""),
            "epsilon_final": metrics.get("epsilon_final", ""),
            "epsilon_satisfied": metrics.get("epsilon_satisfied", ""),
            "is_replan": str(bool(plan_start and model.start and plan_start != model.start)).lower(),
            "changed_obstacle_count": metrics.get("changed_obstacle_count", ""),
        }
