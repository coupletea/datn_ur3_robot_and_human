import unittest

from astar_simulation.app_window import snap_coordinate


class AppWindowHelperTest(unittest.TestCase):
    def test_half_voxel_surface_snaps_to_adjacent_cell(self):
        self.assertEqual(snap_coordinate(4.5, grid_max=9), 5)

    def test_snap_coordinate_clamps_to_grid(self):
        self.assertEqual(snap_coordinate(-2.0, grid_max=9), 0)
        self.assertEqual(snap_coordinate(12.0, grid_max=9), 9)


if __name__ == "__main__":
    unittest.main()
