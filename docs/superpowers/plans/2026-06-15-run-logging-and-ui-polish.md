# Run Logging and UI Polish Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add complete CSV experiment logging, finish-reset behavior, and clearer single-obstacle deletion controls.

**Architecture:** Extend ARA* result metrics without changing search behavior. Add a pure CSV logger and keep logging orchestration in the desktop window. Add obstacle-list synchronization and right-click deletion to the existing UI.

**Tech Stack:** Python 3, csv, unittest, PyVista, PySide6

---

### Task 1: ARA* experiment metrics

- [x] Add failing tests for `first_solution_time_ms` and `iterations`.
- [x] Extend `scripts/astar_improved_3d.py` to record both metrics.
- [x] Run algorithm and simulator tests.

### Task 2: CSV run logger

- [x] Add failing tests for schema, padded density, path distance, scenario ID, and replan flag.
- [x] Create `astar_simulation/run_logger.py`.
- [x] Run logger and full tests.

### Task 3: Reset and obstacle editor UI

- [x] Reset robot marker to original start after finishing while preserving paths.
- [x] Add `voxel_size_m` input and write one CSV row after every plan attempt.
- [x] Add obstacle list synchronized by stable obstacle ID.
- [x] Add list-based single deletion and right-click deletion.
- [x] Improve side-panel grouping.

### Task 4: Verification

- [x] Update README.
- [x] Run all tests, compile, imports, and pip check.
- [x] Live-smoke finish reset, CSV row creation, and list deletion.
