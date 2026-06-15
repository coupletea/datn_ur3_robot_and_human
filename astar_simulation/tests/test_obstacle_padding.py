import unittest

from astar_simulation.obstacle_padding import (
    cubic_padding,
    padded_obstacle_union,
    padding_radius,
)


class ObstaclePaddingTest(unittest.TestCase):
    def test_padding_radius_is_linear_and_rounded_up(self):
        self.assertEqual(padding_radius(0.0, 1.5), 0)
        self.assertEqual(padding_radius(0.1, 1.5), 1)
        self.assertEqual(padding_radius(2.0, 1.5), 3)

    def test_cubic_padding_fills_complete_cube(self):
        padded = cubic_padding((5, 5, 5), radius=1, grid_size=(20, 20, 20))
        self.assertEqual(len(padded), 27)
        self.assertIn((4, 4, 4), padded)
        self.assertIn((6, 6, 6), padded)

    def test_cubic_padding_clips_to_grid(self):
        padded = cubic_padding((0, 0, 0), radius=1, grid_size=(20, 20, 20))
        self.assertEqual(padded, {
            (0, 0, 0), (0, 0, 1), (0, 1, 0), (0, 1, 1),
            (1, 0, 0), (1, 0, 1), (1, 1, 0), (1, 1, 1),
        })

    def test_union_deduplicates_overlapping_padding(self):
        obstacles = [((5, 5, 5), 1.0), ((6, 5, 5), 1.0)]
        padded = padded_obstacle_union(obstacles, gain=1.0, grid_size=(20, 20, 20))
        self.assertEqual(len(padded), 36)


if __name__ == "__main__":
    unittest.main()
