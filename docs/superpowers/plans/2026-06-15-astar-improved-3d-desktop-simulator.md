# ARA* Improved 3D Desktop Simulator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a standalone Windows PyVistaQt desktop app for editing a `20 x 20 x 20` voxel scene and automatically testing `scripts/astar_improved_3d.py`.

**Architecture:** Pure Python model, padding, and planner-adapter modules hold all testable behavior. A PySide6/PyVistaQt window renders the 3D grid and translates mouse/control events into model updates.

**Tech Stack:** Python 3, unittest, PyVista, PyVistaQt, PySide6

---

## File Structure

- Create `astar_simulation/__init__.py`: simulator package marker.
- Create `astar_simulation/obstacle_padding.py`: pure cubic padding functions.
- Create `astar_simulation/simulation_model.py`: validated editable scene state.
- Create `astar_simulation/planner_adapter.py`: direct adapter to production ARA*.
- Create `astar_simulation/app_window.py`: PySide6/PyVistaQt UI and rendering.
- Create `astar_simulation/main.py`: Windows desktop entry point.
- Create `astar_simulation/requirements.txt`: desktop dependencies.
- Create `astar_simulation/tests/test_obstacle_padding.py`: padding tests.
- Create `astar_simulation/tests/test_simulation_model.py`: model tests.
- Create `astar_simulation/tests/test_planner_adapter.py`: adapter tests.

### Task 1: Cubic obstacle padding

- [ ] Write failing tests for radius formula, cube shape, clipping, and union.
- [ ] Run `python -m unittest astar_simulation.tests.test_obstacle_padding -v`; verify missing-module failure.
- [ ] Implement `padding_radius`, `cubic_padding`, and `padded_obstacle_union`.
- [ ] Re-run padding tests; expect all pass.

### Task 2: Simulation model

- [ ] Write failing tests for start/goal validation, obstacle IDs, independent speeds, deletion, and padding-over-endpoint rejection.
- [ ] Run model tests; verify missing-model failure.
- [ ] Implement `Obstacle`, `SimulationModel`, and `SceneValidationError`.
- [ ] Re-run model and padding tests; expect all pass.

### Task 3: Production ARA* adapter

- [ ] Write failing tests for direct production import, empty-scene path, detour/no-path behavior, and metrics passthrough.
- [ ] Run adapter tests; verify missing-adapter failure.
- [ ] Implement `PlannerAdapter` and `WAITING_FOR_START_GOAL`.
- [ ] Re-run all pure tests; expect all pass.

### Task 4: Desktop UI

- [ ] Implement `AppWindow` with fixed 20-cube wireframe grid, side controls, active XY pick plane, numeric start/goal inputs, obstacle speed editing, delete/clear actions, auto-replan, and grouped PyVista actors.
- [ ] Implement `main.py` with clear dependency error and Qt event loop.
- [ ] Add desktop dependencies to `requirements.txt`.
- [ ] Byte-compile all simulator modules.

### Task 5: Verification

- [ ] Run `python -m unittest discover -s astar_simulation/tests -v`.
- [ ] Run `python -m py_compile astar_simulation/*.py`.
- [ ] Install desktop dependencies when compatible with the active Python runtime.
- [ ] Run headless import/smoke checks for `app_window.py` and `main.py`.
- [ ] Review diff against design spec and report any environment-only limitations.
