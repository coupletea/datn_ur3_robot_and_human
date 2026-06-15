import csv
import tempfile
import unittest
from pathlib import Path

from astar_simulation.planner_adapter import PlannerAdapter
from astar_simulation.run_logger import CSV_FIELDS, RunLogger
from astar_simulation.simulation_model import SimulationModel


class RunLoggerTest(unittest.TestCase):
    def test_appends_complete_row_with_derived_metrics(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "astar_runs.csv"
            model = SimulationModel()
            model.set_start((0, 0, 0))
            model.set_goal((2, 0, 0))
            model.add_obstacle((8, 8, 8))
            outcome = PlannerAdapter().plan(model)
            logger = RunLogger(path)
            logger.log_plan("scenario_000001", model, outcome, voxel_size_m=0.05)

            with path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(list(rows[0].keys()), CSV_FIELDS)
            self.assertEqual(rows[0]["scenario_id"], "scenario_000001")
            self.assertEqual(rows[0]["obstacle_count"], "1")
            self.assertEqual(float(rows[0]["obstacle_density"]), 0.001)
            self.assertEqual(float(rows[0]["path_distance_m"]), 0.1)
            self.assertEqual(rows[0]["is_replan"], "false")

    def test_marks_replan_from_current_robot(self):
        with tempfile.TemporaryDirectory() as tmp:
            model = SimulationModel()
            model.set_start((0, 0, 0))
            model.set_goal((3, 0, 0))
            model.initialize_robot()
            model.move_robot((1, 0, 0))
            outcome = PlannerAdapter().plan(model)
            path = Path(tmp) / "astar_runs.csv"
            RunLogger(path).log_plan("scenario_000002", model, outcome, voxel_size_m=0.05)
            with path.open(newline="", encoding="utf-8") as handle:
                row = next(csv.DictReader(handle))
            self.assertEqual(row["start_x"], "1")
            self.assertEqual(row["is_replan"], "true")


if __name__ == "__main__":
    unittest.main()
