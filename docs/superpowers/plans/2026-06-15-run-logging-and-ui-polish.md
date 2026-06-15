# Run Logging and UI Polish Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add complete CSV experiment logging, finish-reset behavior, and clearer single-obstacle deletion controls.

**Architecture:** Extend ARA* result metrics without changing search behavior. Add a pure CSV logger and keep logging orchestration in the desktop window. Add obstacle-list synchronization and right-click deletion to the existing UI.

**Tech Stack:** Python 3, csv, unittest, PyVista, PySide6

---

### Task 1: ARA* experiment metrics

- [ ] Add failing tests for `first_solution_time_ms` and `iterations`.
- [ ] Extend `scripts/astar_improved_3d.py` to record both metrics.
- [ ] Run algorithm and simulator tests.

### Task 2: CSV run logger

- [ ] Add failing tests for schema, padded density, path distance, scenario ID, and replan flag.
- [ ] Create `astar_simulation/run_logger.py`.
- [ ] Run logger and full tests.

### Task 3: Reset and obstacle editor UI

- [ ] Reset robot marker to original start after finishing while preserving paths.
- [ ] Add `voxel_size_m` input and write one CSV row after every plan attempt.
- [ ] Add obstacle list synchronized by stable obstacle ID.
- [ ] Add list-based single deletion and right-click deletion.
- [ ] Improve side-panel grouping.

### Task 4: Verification

- [ ] Update README.
- [ ] Run all tests, compile, imports, and pip check.
- [ ] Live-smoke finish reset and CSV row creation.
