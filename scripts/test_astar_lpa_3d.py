#!/usr/bin/env python3
"""Offline assert-based tests for LPAStar3D (no ROS).

Run: python3 scripts/test_astar_lpa_3d.py

Cross-checks LPA* against the existing AStarImproved3D (ARA*) baseline on small
grids. smooth=False everywhere so path steps stay single-voxel and grid-optimal
cost is directly comparable (epsilon=1.0 => optimal for both)."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from astar_lpa_3d import LPAStar3D
from astar_improved_3d import (
    AStarImproved3D,
    START_BLOCKED,
    GOAL_BLOCKED,
)

GRID = (12, 12, 4)


def _step_cost(a, b):
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def _path_cost(path):
    return sum(_step_cost(path[i], path[i + 1]) for i in range(len(path) - 1))


def _assert_valid_path(path, start, goal, obstacles, grid):
    assert path, "path empty"
    assert path[0] == start, f"path start {path[0]} != {start}"
    assert path[-1] == goal, f"path goal {path[-1]} != {goal}"
    sx, sy, sz = grid
    obs = set(obstacles)
    for v in path:
        x, y, z = v
        assert 0 <= x < sx and 0 <= y < sy and 0 <= z < sz, f"{v} out of bounds"
        assert v not in obs, f"{v} sits in obstacle"
    for i in range(len(path) - 1):
        d = [abs(path[i][k] - path[i + 1][k]) for k in range(3)]
        assert max(d) == 1, f"non-adjacent step {path[i]}->{path[i+1]}"


def _new_lpa(smooth=False, **kw):
    # Generous time budget: these tests assert correctness + reuse decision,
    # not wall-clock speed. (Production budget comes from ~ara_max_time_ms.)
    kw.setdefault("max_time_ms", 5000.0)
    return LPAStar3D(GRID[0], GRID[1], GRID[2], diagonal=True,
                     smooth=smooth, epsilon=1.0, **kw)


def _new_ara():
    # epsilon fixed at 1.0 -> optimal, comparable cost to LPA*
    return AStarImproved3D(GRID[0], GRID[1], GRID[2], diagonal=True,
                           epsilon_start=1.0, epsilon_final=1.0, smooth=False)


def test_empty_grid():
    lpa = _new_lpa()
    start, goal = (0, 0, 0), (6, 5, 2)
    r = lpa.plan_with_info(start, goal, set())
    assert r.success and r.reason == "OK", f"empty grid failed: {r.reason}"
    _assert_valid_path(r.path, start, goal, set(), GRID)


def test_static_obstacle_cost_matches_ara():
    start, goal = (0, 0, 0), (10, 0, 0)
    # wall on x=5 blocking the straight line, leaving a gap at y>=2
    obstacles = {(5, y, z) for y in range(0, 2) for z in range(GRID[2])}
    obstacles.discard(start)
    obstacles.discard(goal)

    lpa = _new_lpa()
    rl = lpa.plan_with_info(start, goal, obstacles)
    assert rl.success, f"LPA* failed: {rl.reason}"
    _assert_valid_path(rl.path, start, goal, obstacles, GRID)

    ara = _new_ara()
    ra = ara.plan_with_info(start, goal, obstacles)
    assert ra.success, f"ARA* failed: {ra.reason}"

    cl, ca = _path_cost(rl.path), _path_cost(ra.path)
    assert abs(cl - ca) < 1e-6, f"cost mismatch LPA*={cl} ARA*={ca}"


def test_start_blocked():
    lpa = _new_lpa()
    start, goal = (3, 3, 1), (9, 9, 2)
    r = lpa.plan_with_info(start, goal, {start})
    assert not r.success and r.reason == START_BLOCKED, f"got {r.reason}"


def test_goal_blocked():
    lpa = _new_lpa()
    start, goal = (0, 0, 0), (9, 9, 2)
    r = lpa.plan_with_info(start, goal, {goal})
    assert not r.success and r.reason == GOAL_BLOCKED, f"got {r.reason}"


def test_repair_on_small_obstacle_change():
    lpa = _new_lpa()
    start, goal = (0, 0, 0), (10, 0, 0)
    obs0 = {(5, 0, 0)}
    r0 = lpa.plan_with_info(start, goal, obs0)
    assert r0.success, f"initial plan failed: {r0.reason}"

    # move the single blocking voxel one cell over
    obs1 = {(5, 1, 0)}
    r1 = lpa.replan_with_info(start, obs1)
    assert r1.success, f"repair plan failed: {r1.reason}"
    _assert_valid_path(r1.path, start, goal, obs1, GRID)
    assert r1.metrics.get("reuse_mode") == "REPAIR", \
        f"expected REPAIR, got {r1.metrics.get('reuse_mode')}"


def test_large_obstacle_diff_resets():
    lpa = _new_lpa(max_changed_obstacles_for_repair=5)
    start, goal = (0, 0, 0), (10, 10, 3)
    r0 = lpa.plan_with_info(start, goal, set())
    assert r0.success, f"initial plan failed: {r0.reason}"

    big = {(2, y, z) for y in range(8) for z in range(GRID[2])}  # > 5 changed
    big.discard(start)
    big.discard(goal)
    r1 = lpa.replan_with_info(start, big)
    assert r1.metrics.get("reuse_mode") == "RESET", \
        f"expected RESET, got {r1.metrics.get('reuse_mode')}"
    assert r1.success, f"reset plan failed: {r1.reason}"
    _assert_valid_path(r1.path, start, goal, big, GRID)


def run():
    tests = [
        test_empty_grid,
        test_static_obstacle_cost_matches_ara,
        test_start_blocked,
        test_goal_blocked,
        test_repair_on_small_obstacle_change,
        test_large_obstacle_diff_resets,
    ]
    for t in tests:
        t()
        print(f"  OK  {t.__name__}")
    print("LPAStar3D self-check OK")


if __name__ == "__main__":
    run()
