import unittest

from astar_simulation.simulation_model import SceneValidationError, SimulationModel


class SimulationModelTest(unittest.TestCase):
    def setUp(self):
        self.model = SimulationModel()
        self.model.set_start((1, 1, 1))
        self.model.set_goal((8, 8, 8))

    def test_default_grid_is_ten_and_robot_initializes_from_start(self):
        self.assertEqual(self.model.grid_size, (10, 10, 10))
        self.assertIsNone(self.model.current_robot)
        self.model.initialize_robot()
        self.assertEqual(self.model.current_robot, self.model.start)

    def test_start_and_goal_must_be_distinct_and_in_bounds(self):
        with self.assertRaises(SceneValidationError):
            self.model.set_goal((1, 1, 1))
        with self.assertRaises(SceneValidationError):
            self.model.set_start((10, 0, 0))

    def test_obstacles_have_stable_ids_and_independent_speeds(self):
        first = self.model.add_obstacle((5, 5, 5), speed=1.0)
        second = self.model.add_obstacle((7, 7, 7), speed=0.0)
        self.model.set_obstacle_speed(first, 1.0)
        self.assertEqual(self.model.obstacles[first].speed, 1.0)
        self.assertEqual(self.model.obstacles[second].speed, 0.0)

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
        self.model.add_obstacle((7, 7, 7), speed=0.0)
        self.model.delete_obstacle(first)
        self.assertNotIn((5, 5, 5), self.model.padded_obstacles())
        self.model.clear_obstacles()
        self.assertEqual(self.model.padded_obstacles(), set())

    def test_scan_entries_are_stable_by_obstacle_id(self):
        first = self.model.add_obstacle((5, 5, 5), speed=0.0)
        second = self.model.add_obstacle((7, 7, 7), speed=0.0)
        entries = self.model.scan_entries()
        self.assertEqual([entry.obstacle_id for entry in entries], [first, second])
        self.assertEqual(entries[0].padded_voxels, {(5, 5, 5)})

    def test_robot_moves_only_to_in_bounds_voxel(self):
        self.model.initialize_robot()
        self.model.move_robot((2, 1, 1))
        self.assertEqual(self.model.current_robot, (2, 1, 1))
        with self.assertRaises(SceneValidationError):
            self.model.move_robot((10, 1, 1))

    def test_reset_robot_returns_to_original_start(self):
        self.model.initialize_robot()
        self.model.move_robot((2, 1, 1))
        self.model.reset_robot_to_start()
        self.assertEqual(self.model.current_robot, self.model.start)


if __name__ == "__main__":
    unittest.main()
