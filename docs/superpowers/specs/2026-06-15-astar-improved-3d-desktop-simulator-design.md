# Design Spec: ARA* Improved 3D Desktop Simulator

**Date:** 2026-06-15  
**Status:** Approved  
**Target platform:** Windows desktop  
**Launch command:** `python astar_simulation/main.py`

## 1. Goal

Create a standalone desktop application for experimenting with the existing
`scripts/astar_improved_3d.py` implementation.

The simulator provides a rotatable 3D voxel grid. The user places the start,
goal, and obstacles; assigns an independent speed to each obstacle; and observes
how speed-dependent cubic padding changes the ARA* path.

The application runs without ROS and does not modify or duplicate the ARA*
implementation.

## 2. Scope

### Included

- Fixed `20 x 20 x 20` voxel planning grid.
- PyVista/PyVistaQt desktop UI with rotate, zoom, and pan.
- Start and goal placement by mouse or numeric coordinate input.
- Obstacle placement by mouse.
- One cubic voxel per base obstacle.
- Independent speed value for every obstacle.
- Cubic padding derived linearly from obstacle speed.
- Automatic replanning after every relevant scene change.
- Visualization of base obstacles, padding, start, goal, path, and planner metrics.
- Focused tests for model, padding, and planner adapter logic.

### Excluded

- ROS integration.
- Robot or MoveIt integration.
- Moving obstacles or velocity vectors.
- Executable packaging with PyInstaller.
- Alternative grid sizes.
- Editing `scripts/astar_improved_3d.py`.

## 3. Repository Structure

The simulator is isolated from production ROS nodes:

```text
astar_simulation/
├── main.py
├── app_window.py
├── simulation_model.py
├── obstacle_padding.py
├── planner_adapter.py
├── requirements.txt
└── tests/
    ├── test_obstacle_padding.py
    ├── test_simulation_model.py
    └── test_planner_adapter.py
```

`planner_adapter.py` resolves the repository root from its own location, adds
that root to the Python import path, then imports the production algorithm
directly:

```python
from scripts.astar_improved_3d import AStarImproved3D, PlanResult
```

The simulator must not copy or alter the algorithm source.

## 4. Architecture

### `main.py`

- Validates desktop dependencies.
- Creates the Qt application and main window.
- Starts the event loop.
- Produces a clear error if required packages are missing.

### `simulation_model.py`

Owns all editable simulation state without PyVista or Qt dependencies:

- Grid size, fixed at `(20, 20, 20)`.
- Optional start voxel.
- Optional goal voxel.
- Obstacles keyed by stable integer ID.
- Each obstacle stores its voxel coordinate and speed.
- Global padding gain.

It validates coordinates, prevents duplicate base obstacles, and exposes the
complete padded obstacle set used by planning.

### `obstacle_padding.py`

Contains pure padding logic:

```text
padding_radius = ceil(max(0, speed) * padding_gain)
```

For a base obstacle at `(x, y, z)`, padding fills the clipped cubic region:

```text
[x-p, x+p] x [y-p, y+p] x [z-p, z+p]
```

where `p` is `padding_radius`. The base voxel is part of this cube. Voxels
outside the `20 x 20 x 20` grid are discarded.

### `planner_adapter.py`

- Imports `AStarImproved3D` directly from `scripts.astar_improved_3d`.
- Adds only the repository root to the runtime import path; it does not change
  `scripts/`.
- Owns an `AStarImproved3D` instance configured for the fixed grid.
- Converts model state into the algorithm input contract.
- Calls `plan_with_info(start, goal, padded_obstacles)`.
- Returns the original `PlanResult` for visualization.

The initial configuration uses diagonal movement and path smoothing. Existing
algorithm defaults remain unchanged unless surfaced explicitly in the UI later.

### `app_window.py`

Owns PyVistaQt rendering and user interaction. It does not implement planning or
padding rules.

It:

- Renders the grid, pick plane, scene objects, path, and metrics.
- Translates UI actions into model updates.
- Calls the planner adapter after valid changes.
- Refreshes only affected visual actors where practical.

## 5. Desktop UI

### 3D View

The main view displays a real 3D planning grid:

- Grid extent: `0..19` on X, Y, and Z.
- Whole grid rendered as lightweight wireframe/axes.
- Empty voxels are not rendered as 8,000 solid cubes.
- Left-drag rotates the camera.
- Mouse wheel zooms.
- Middle/right-drag pans according to PyVista controls.

Colors:

| Element | Color |
|---|---|
| Start | Green |
| Goal | Purple |
| Base obstacle voxel | Red |
| Padding-only voxels | Orange with transparency |
| Planned path | Blue |
| Active XY pick plane | Yellow with transparency |

### Side Control Panel

Controls:

- Edit mode: `Add Obstacle`, `Set Start`, `Set Goal`, `Select`.
- Z slider selecting the active XY pick plane from `0` to `19`.
- Numeric `(x, y, z)` inputs and apply buttons for start and goal.
- Selected-obstacle speed input/slider.
- Global padding gain input.
- `Delete Selected` and `Clear Obstacles`.
- Planner result panel showing:
  - success and reason;
  - planning time;
  - path cost;
  - path node count;
  - expanded steps;
  - padded obstacle count.

### Mouse Placement

A screen click alone cannot identify depth in empty 3D space. Placement therefore
uses the active XY pick plane:

1. User chooses an edit mode.
2. User selects Z with the slider.
3. User clicks the visible XY plane.
4. The hit point is snapped to the nearest valid `(x, y, Z)` voxel.

In `Select` mode, clicking a rendered red base-obstacle voxel selects that
obstacle for speed editing or deletion.

## 6. Interaction and Replanning

Every valid change automatically replans:

- start changed;
- goal changed;
- obstacle added or deleted;
- selected obstacle speed changed;
- global padding gain changed.

Replanning flow:

```text
UI action
  -> SimulationModel validates and updates state
  -> obstacle_padding builds padded obstacle union
  -> PlannerAdapter calls AStarImproved3D.plan_with_info()
  -> AppWindow refreshes obstacle actors, path, and metrics
```

If start or goal is missing, the app updates visuals but reports
`WAITING_FOR_START_GOAL` without calling ARA*.

Speed is a static risk value only. Obstacles do not move.

## 7. Validation and Error Handling

- Coordinates must be integer voxels inside `[0, 19]`.
- Speeds and padding gain must be finite and non-negative.
- Duplicate base-obstacle placement is rejected.
- Start and goal cannot occupy the same voxel.
- A base obstacle cannot be placed directly on start or goal.
- A speed or padding-gain change that would make padding cover start or goal is
  rejected; the previous valid scene remains active and the UI shows the error.
- `NO_PATH`, timeout, and other `PlanResult.reason` values are shown without
  crashing or clearing the editable scene.
- Dependency/import failures produce a clear startup message.
- Unexpected planning exceptions are caught at the UI boundary and shown in the
  result panel while preserving current scene state.

## 8. Rendering Strategy

Rendering must remain responsive on the `20 x 20 x 20` grid:

- Render one lightweight grid/wireframe actor, not 8,000 empty cube actors.
- Group base obstacles into one mesh when possible.
- Group padding-only voxels into one mesh.
- Render the path as a polyline through voxel centers.
- Render start and goal as distinct markers.
- Keep base-obstacle actors pickable; padding actors are not selectable.

Automatic replanning happens immediately after edits. UI rendering may debounce
continuous speed/gain slider movement briefly so rapid slider events do not
queue excessive planner calls.

## 9. Dependencies

`astar_simulation/requirements.txt` contains desktop-only dependencies:

```text
pyvista
pyvistaqt
PySide6
```

The app uses PySide6 as the Qt binding. ROS packages are not required.

## 10. Testing

### Padding tests

- Zero speed produces only the base voxel when gain is positive.
- Linear formula uses `ceil(speed * gain)`.
- Padding is cubic.
- Padding is clipped at every grid boundary.
- Union of overlapping padded obstacles is deduplicated.

### Model tests

- Valid and invalid coordinate changes.
- Duplicate obstacle rejection.
- Independent speed updates by obstacle ID.
- Base obstacle cannot occupy start or goal.
- Start and goal cannot be identical.
- Deleting selected obstacles updates padded output.

### Planner adapter tests

- Direct import of `scripts.astar_improved_3d`.
- Empty scene returns a valid path.
- Padded wall causes detour or `NO_PATH`.
- Speed/gain changes that would pad over start or goal are rejected.
- Metrics are passed through unchanged.

### Manual UI checks

- Window launches on Windows with `python astar_simulation/main.py`.
- Grid rotates, zooms, and pans.
- Pick plane follows Z slider.
- Click placement snaps to correct voxel.
- Base obstacle selection edits only that obstacle's speed.
- Path and metrics refresh automatically.

## 11. Success Criteria

The feature is complete when a user can:

1. Run `python astar_simulation/main.py` on Windows.
2. Rotate and inspect a `20 x 20 x 20` 3D voxel grid.
3. Place or numerically enter start and goal.
4. Click to add single-voxel obstacles.
5. Select each obstacle and assign a separate static speed.
6. Observe cubic speed-derived padding.
7. See ARA* automatically find a new path around the padded obstacles.
8. Read the algorithm result and metrics without using ROS.
