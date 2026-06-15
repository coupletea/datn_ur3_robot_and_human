# Staged ARA* Animation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Change the simulator to a `10 x 10 x 10` grid with explicit Start/Stop, sequential obstacle scanning, six-direction planning, and voxel-by-voxel robot movement.

**Architecture:** Extend the pure scene model with current robot position and stable scan entries. Configure the production ARA* adapter for six-direction unsmoothed paths. Replace UI auto-replanning with a non-blocking Qt timer state machine.

**Tech Stack:** Python 3, unittest, PyVista, PyVistaQt, PySide6

---

### Task 1: Model and scan data

- [ ] Update failing model tests to expect a default `10 x 10 x 10` grid, current robot initialization from start, and stable obstacle scan entries.
- [ ] Run model tests and verify failures against current behavior.
- [ ] Implement `current_robot`, `initialize_robot`, `move_robot`, and `scan_entries`.
- [ ] Re-run model tests.

### Task 2: Six-direction planner

- [ ] Add a failing adapter test asserting every planned edge changes exactly one axis by one voxel and planning starts from `current_robot`.
- [ ] Run adapter tests and verify failures.
- [ ] Configure `AStarImproved3D(diagonal=False, smooth=False)` and use current robot as planning start.
- [ ] Re-run all pure tests.

### Task 3: Start/Stop UI state machine

- [ ] Replace auto-replan timer with explicit `Start` and `Stop` controls.
- [ ] Add scan and robot-step timing controls.
- [ ] Implement states `EDITING`, `SCANNING`, `MOVING`, `STOPPED`, and `FINISHED`.
- [ ] Lock editing during scan/movement and unlock on stop/failure/finish.
- [ ] Highlight scan entries sequentially, plan after scanning, draw planned path, move robot one voxel per timer tick, and draw traveled path separately.
- [ ] Update grid/pick/input bounds to `0..9`.

### Task 4: Documentation and verification

- [ ] Update simulator README for the new execution flow.
- [ ] Run all unit tests and byte-compile modules.
- [ ] Run dependency/import checks.
- [ ] Launch desktop app and verify it remains alive without stderr.
