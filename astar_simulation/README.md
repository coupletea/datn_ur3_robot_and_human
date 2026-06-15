# ARA* Improved 3D Simulator

Standalone Windows desktop app for testing `scripts/astar_improved_3d.py` on a
rotatable `20 x 20 x 20` voxel grid.

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
- Every valid change automatically replans with the production ARA* module.

## Tests

```powershell
python -m unittest discover -s astar_simulation/tests -v
```
