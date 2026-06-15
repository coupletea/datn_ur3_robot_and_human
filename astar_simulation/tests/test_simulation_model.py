import unittest

from astar_simulation.simulation_model import SceneValidationError, SimulationModel


class SimulationModelTest(unittest.TestCase):
    def setUp(self):
        self.model = SimulationModel()
        self.model.set_start((1, 1, 1))
        self.model.set_goal((18, 18, 18))

    def test_start_and_goal_must_be_distinct_and_in_bounds(self):
        with self.assertRaises(SceneValidationError):
            self.model.set_goal((1, 1, 1))
        with self.assertRaises(SceneValidationError):
            self.model.set_start((20, 0, 0))

    def test_obstacles_have_stable_ids_and_independent_speeds(self):
        first = self.model.add_obstacle((5, 5, 5), speed=1.0)
        second = self.model.add_obstacle((10, 10, 10), speed=2.0)
        self.model.set_obstacle_speed(first, 3.0)
        self.assertEqual(self.model.obstacles[first].speed, 3.0)
        self.assertEqual(self.model.obstacles[second].speed, 2.0)

    def test_rejects_duplicate_and_endpoint_obstacles(self):
        self.model.add_obstacle((5, 5, 5))
        with self.assertRaises(SceneValidationError):
            self.model.add_obstacle((5, 5, 5))
        with self.assertRaises(SceneValidationError):
            self.model.add_obstacle((1, 1, 1))

    def test_rejects_speed_change_when_padding_would_cover_start(self):
        obstacle_id = self.model.add_obstacle((3, 1, 1), speed=0.0)
        with self.assertRaises(SceneValidationError):
            self.model.set_obstacle_speed(obstacle_id, 2.0)
        self.assertEqual(self.model.obstacles[obstacle_id].speed, 0.0)

    def test_delete_and_clear_update_padded_obstacles(self):
        first = self.model.add_obstacle((5, 5, 5), speed=0.0)
        self.model.add_obstacle((10, 10, 10), speed=0.0)
        self.model.delete_obstacle(first)
        self.assertNotIn((5, 5, 5), self.model.padded_obstacles())
        self.model.clear_obstacles()
        self.assertEqual(self.model.padded_obstacles(), set())


if __name__ == "__main__":
    unittest.main()
