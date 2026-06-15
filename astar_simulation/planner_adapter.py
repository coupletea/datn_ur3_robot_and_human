from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from scripts.astar_improved_3d import AStarImproved3D, PlanResult

from astar_simulation.simulation_model import SimulationModel

WAITING_FOR_START_GOAL = "WAITING_FOR_START_GOAL"


@dataclass(frozen=True)
class PlanningOutcome:
    status: str
    result: Optional[PlanResult]


class PlannerAdapter:
    def __init__(
        self,
        max_time_ms: float = 200.0,
        max_steps: int = 150000,
        epsilon_start: float = 1.5,
        epsilon_final: float = 1.0,
        epsilon_decay: float = 0.2,
    ):
        self.max_time_ms = max_time_ms
        self.max_steps = max_steps
        self.epsilon_start = epsilon_start
        self.epsilon_final = epsilon_final
        self.epsilon_decay = epsilon_decay

    def plan(self, model: SimulationModel) -> PlanningOutcome:
        if model.start is None or model.goal is None:
            return PlanningOutcome(WAITING_FOR_START_GOAL, None)
        planner = AStarImproved3D(
            *model.grid_size,
            diagonal=False,
            epsilon_start=self.epsilon_start,
            epsilon_final=self.epsilon_final,
            epsilon_decay=self.epsilon_decay,
            max_time_ms=self.max_time_ms,
            max_steps=self.max_steps,
            smooth=False,
        )
        plan_start = model.current_robot if model.current_robot is not None else model.start
        result = planner.plan_with_info(plan_start, model.goal, model.padded_obstacles())
        return PlanningOutcome(result.reason, result)
