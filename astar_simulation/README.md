# ARA* Improved 3D Simulator

Standalone Windows desktop app for testing `scripts/astar_improved_3d.py` on a
rotatable `10 x 10 x 10` voxel grid.

## Install

```powershell
python -m pip install -r astar_simulation/requirements.txt
```

## Run

```powershell
python astar_simulation/main.py
```

## Controls

- Select an edit mode, choose the Z layer, then click the XY pick plane.
- Use numeric controls to set start and goal exactly.
- In `Select` mode, click a red base obstacle to edit its speed or delete it.
- Cubic padding radius is `ceil(speed * gain)`.
- Press `Start` to scan each obstacle/padding region, run ARA*, draw the full
  planned path, then move the robot marker one voxel at a time.
- Planning uses six directions only and disables path smoothing.
- Press `Stop` to pause at the current robot voxel, edit obstacles, then press
  `Start` to scan and plan again from that voxel.
- Planned path is blue. Traveled path is green.
- Robot and scan intervals have separate controls.
- When movement reaches the goal, the robot marker resets to the original start
  while planned/traveled paths remain visible.
- Delete one obstacle using the obstacle list and `Delete Selected`, or
  right-click the red obstacle in the 3D view.
- Every ARA* plan appends experiment metrics to
  `astar_simulation/logs/astar_runs.csv`.
- `voxel_size_m` controls conversion from path cost to `path_distance_m`.

## Tests

```powershell
python -m unittest discover -s astar_simulation/tests -v
```
