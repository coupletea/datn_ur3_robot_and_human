import unittest

from scripts.astar_improved_3d import PlanResult

from astar_simulation.planner_adapter import PlannerAdapter, WAITING_FOR_START_GOAL
from astar_simulation.simulation_model import SimulationModel


class PlannerAdapterTest(unittest.TestCase):
    def test_waits_until_start_and_goal_exist(self):
        result = PlannerAdapter().plan(SimulationModel())
        self.assertIsNone(result.result)
        self.assertEqual(result.status, WAITING_FOR_START_GOAL)

    def test_empty_scene_returns_production_plan_result(self):
        model = SimulationModel()
        model.set_start((0, 0, 0))
        model.set_goal((19, 19, 19))
        outcome = PlannerAdapter(max_time_ms=500.0).plan(model)
        self.assertIsInstance(outcome.result, PlanResult)
        self.assertTrue(outcome.result.success)
        self.assertEqual(outcome.result.path[0], model.start)
        self.assertEqual(outcome.result.path[-1], model.goal)
        self.assertIn("expanded_steps", outcome.result.metrics)

    def test_padded_wall_forces_no_path(self):
        model = SimulationModel()
        model.set_start((1, 10, 10))
        model.set_goal((18, 10, 10))
        for y in range(20):
            for z in range(20):
                model.add_obstacle((10, y, z), speed=0.0)
        outcome = PlannerAdapter(max_time_ms=1000.0).plan(model)
        self.assertIsNotNone(outcome.result)
        self.assertFalse(outcome.result.success)


if __name__ == "__main__":
    unittest.main()
