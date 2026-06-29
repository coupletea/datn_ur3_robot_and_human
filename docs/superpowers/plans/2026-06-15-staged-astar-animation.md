# Staged ARA* Animation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Change the simulator to a `10 x 10 x 10` grid with explicit Start/Stop, sequential obstacle scanning, six-direction planning, and voxel-by-voxel robot movement.

**Architecture:** Extend the pure scene model with current robot position and stable scan entries. Configure the production ARA* adapter for six-direction unsmoothed paths. Replace UI auto-replanning with a non-blocking Qt timer state machine.

**Tech Stack:** Python 3, unittest, PyVista, PyVistaQt, PySide6

---

### Task 1: Model and scan data

- [x] Update failing model tests to expect a default `10 x 10 x 10` grid, current robot initialization from start, and stable obstacle scan entries.
- [x] Run model tests and verify failures against current behavior.
- [x] Implement `current_robot`, `initialize_robot`, `move_robot`, and `scan_entries`.
- [x] Re-run model tests.

### Task 2: Six-direction planner

- [x] Add a failing adapter test asserting every planned edge changes exactly one axis by one voxel and planning starts from `current_robot`.
- [x] Run adapter tests and verify failures.
- [x] Configure `AStarImproved3D(diagonal=False, smooth=False)` and use current robot as planning start.
- [x] Re-run all pure tests.

### Task 3: Start/Stop UI state machine

- [x] Replace auto-replan timer with explicit `Start` and `Stop` controls.
- [x] Add scan and robot-step timing controls.
- [x] Implement states `EDITING`, `SCANNING`, `MOVING`, `STOPPED`, and `FINISHED`.
- [x] Lock editing during scan/movement and unlock on stop/failure/finish.
- [x] Highlight scan entries sequentially, plan after scanning, draw planned path, move robot one voxel per timer tick, and draw traveled path separately.
- [x] Update grid/pick/input bounds to `0..9`.

### Task 4: Documentation and verification

- [x] Update simulator README for the new execution flow.
- [x] Run all unit tests and byte-compile modules.
- [x] Run dependency/import checks.
- [x] Launch desktop app and verify scan/move plus Stop/edit/restart flows.
