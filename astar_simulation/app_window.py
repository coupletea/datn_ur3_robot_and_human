from __future__ import annotations

import math
from typing import Dict, Iterable, List, Optional

import pyvista as pv
from PySide6 import QtCore, QtWidgets
from pyvistaqt import QtInteractor

from astar_simulation.planner_adapter import PlannerAdapter, PlanningOutcome
from astar_simulation.simulation_model import (
    ScanEntry,
    SceneValidationError,
    SimulationModel,
    Voxel,
)


class AppWindow(QtWidgets.QMainWindow):
    MODE_SELECT = "Select"
    MODE_OBSTACLE = "Add Obstacle"
    MODE_START = "Set Start"
    MODE_GOAL = "Set Goal"

    STATE_EDITING = "EDITING"
    STATE_SCANNING = "SCANNING"
    STATE_MOVING = "MOVING"
    STATE_STOPPED = "STOPPED"
    STATE_FINISHED = "FINISHED"

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ARA* Improved 3D Simulator")
        self.resize(1400, 900)

        self.model = SimulationModel()
        self.planner = PlannerAdapter(max_time_ms=500.0)
        self.selected_obstacle_id: Optional[int] = None
        self.state = self.STATE_EDITING
        self._actors: Dict[str, object] = {}
        self._updating_controls = False
        self._editable_widgets: List[QtWidgets.QWidget] = []
        self._scan_entries: List[ScanEntry] = []
        self._scan_index = 0
        self._planned_path: List[Voxel] = []
        self._traveled_path: List[Voxel] = []
        self._move_index = 0

        self.plotter = QtInteractor(self)
        self.setCentralWidget(self.plotter.interactor)
        self._build_controls()
        self._build_scene()

        self._animation_timer = QtCore.QTimer(self)
        self._animation_timer.setSingleShot(True)
        self._animation_timer.timeout.connect(self._on_animation_tick)

        self.plotter.enable_point_picking(
            callback=self._on_point_picked,
            show_message=False,
            show_point=False,
            pickable_window=True,
            left_clicking=True,
        )
        self._refresh_scene()
        self._show_status(self.STATE_EDITING)

    @property
    def grid_max(self) -> int:
        return self.model.grid_size[0] - 1

    def _build_controls(self) -> None:
        dock = QtWidgets.QDockWidget("Controls", self)
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)

        layout.addWidget(QtWidgets.QLabel("Edit mode"))
        self.mode_group = QtWidgets.QButtonGroup(self)
        self.mode_group.setExclusive(True)
        for index, mode in enumerate(
            (self.MODE_SELECT, self.MODE_OBSTACLE, self.MODE_START, self.MODE_GOAL)
        ):
            button = QtWidgets.QRadioButton(mode)
            button.setChecked(index == 0)
            self.mode_group.addButton(button)
            self._editable_widgets.append(button)
            layout.addWidget(button)

        layout.addWidget(QtWidgets.QLabel("Pick plane Z"))
        self.z_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.z_slider.setRange(0, self.grid_max)
        self.z_slider.valueChanged.connect(self._on_z_changed)
        self.z_label = QtWidgets.QLabel("Z = 0")
        self._editable_widgets.append(self.z_slider)
        layout.addWidget(self.z_slider)
        layout.addWidget(self.z_label)

        self.start_inputs = self._add_coordinate_editor(layout, "Start", self._apply_start)
        self.goal_inputs = self._add_coordinate_editor(layout, "Goal", self._apply_goal)

        obstacle_box = QtWidgets.QGroupBox("Selected obstacle")
        obstacle_layout = QtWidgets.QFormLayout(obstacle_box)
        self.selected_label = QtWidgets.QLabel("None")
        self.speed_spin = QtWidgets.QDoubleSpinBox()
        self.speed_spin.setRange(0.0, 9.0)
        self.speed_spin.setSingleStep(0.25)
        self.speed_spin.valueChanged.connect(self._on_speed_changed)
        self.delete_button = QtWidgets.QPushButton("Delete Selected")
        self.delete_button.clicked.connect(self._delete_selected)
        obstacle_layout.addRow("ID / voxel", self.selected_label)
        obstacle_layout.addRow("Speed", self.speed_spin)
        obstacle_layout.addRow(self.delete_button)
        self._editable_widgets.extend((self.speed_spin, self.delete_button))
        layout.addWidget(obstacle_box)

        self.gain_spin = QtWidgets.QDoubleSpinBox()
        self.gain_spin.setRange(0.0, 10.0)
        self.gain_spin.setSingleStep(0.25)
        self.gain_spin.setValue(self.model.padding_gain)
        self.gain_spin.valueChanged.connect(self._on_gain_changed)
        self._editable_widgets.append(self.gain_spin)
        layout.addWidget(QtWidgets.QLabel("Padding gain"))
        layout.addWidget(self.gain_spin)

        self.clear_button = QtWidgets.QPushButton("Clear Obstacles")
        self.clear_button.clicked.connect(self._clear_obstacles)
        self._editable_widgets.append(self.clear_button)
        layout.addWidget(self.clear_button)

        timing_box = QtWidgets.QGroupBox("Animation timing")
        timing_layout = QtWidgets.QFormLayout(timing_box)
        self.robot_step_spin = QtWidgets.QDoubleSpinBox()
        self.robot_step_spin.setRange(0.1, 2.0)
        self.robot_step_spin.setSingleStep(0.1)
        self.robot_step_spin.setValue(0.5)
        self.robot_step_spin.setSuffix(" s/voxel")
        self.scan_step_spin = QtWidgets.QDoubleSpinBox()
        self.scan_step_spin.setRange(0.05, 1.0)
        self.scan_step_spin.setSingleStep(0.05)
        self.scan_step_spin.setValue(0.2)
        self.scan_step_spin.setSuffix(" s/obstacle")
        timing_layout.addRow("Robot", self.robot_step_spin)
        timing_layout.addRow("Scan", self.scan_step_spin)
        layout.addWidget(timing_box)

        button_row = QtWidgets.QHBoxLayout()
        self.start_button = QtWidgets.QPushButton("Start")
        self.start_button.clicked.connect(self._start_run)
        self.stop_button = QtWidgets.QPushButton("Stop")
        self.stop_button.clicked.connect(self._stop_run)
        self.stop_button.setEnabled(False)
        button_row.addWidget(self.start_button)
        button_row.addWidget(self.stop_button)
        layout.addLayout(button_row)

        result_box = QtWidgets.QGroupBox("ARA* result")
        result_layout = QtWidgets.QVBoxLayout(result_box)
        self.result_label = QtWidgets.QLabel(self.STATE_EDITING)
        self.result_label.setWordWrap(True)
        result_layout.addWidget(self.result_label)
        layout.addWidget(result_box)
        layout.addStretch(1)

        dock.setWidget(panel)
        self.addDockWidget(QtCore.Qt.DockWidgetArea.RightDockWidgetArea, dock)

    def _add_coordinate_editor(self, layout, title: str, callback):
        box = QtWidgets.QGroupBox(title)
        form = QtWidgets.QFormLayout(box)
        row = QtWidgets.QHBoxLayout()
        inputs = []
        for axis in "XYZ":
            spin = QtWidgets.QSpinBox()
            spin.setRange(0, self.grid_max)
            spin.setPrefix(f"{axis}:")
            row.addWidget(spin)
            inputs.append(spin)
            self._editable_widgets.append(spin)
        button = QtWidgets.QPushButton(f"Set {title}")
        button.clicked.connect(callback)
        self._editable_widgets.append(button)
        form.addRow(row)
        form.addRow(button)
        layout.addWidget(box)
        return tuple(inputs)

    def _build_scene(self) -> None:
        self.plotter.set_background("#101722")
        dimensions = tuple(size + 1 for size in self.model.grid_size)
        grid = pv.ImageData(dimensions=dimensions, spacing=(1, 1, 1), origin=(-0.5, -0.5, -0.5))
        self.plotter.add_mesh(
            grid.extract_all_edges(), color="#53677d", opacity=0.25, line_width=1, pickable=False
        )
        upper = self.grid_max + 0.5
        self.plotter.show_bounds(
            bounds=(-0.5, upper, -0.5, upper, -0.5, upper),
            grid="back",
            location="outer",
            xtitle="X",
            ytitle="Y",
            ztitle="Z",
        )
        self.plotter.camera_position = "iso"
        self.plotter.reset_camera()

    def _set_editing_enabled(self, enabled: bool) -> None:
        for widget in self._editable_widgets:
            widget.setEnabled(enabled)
        if enabled:
            self._sync_selected_controls()
        self.start_button.setEnabled(enabled)
        self.stop_button.setEnabled(not enabled)

    def _current_mode(self) -> str:
        button = self.mode_group.checkedButton()
        return button.text() if button else self.MODE_SELECT

    def _on_z_changed(self, value: int) -> None:
        self.z_label.setText(f"Z = {value}")
        self._render_pick_plane()

    def _on_point_picked(self, point) -> None:
        if point is None or self.state in (self.STATE_SCANNING, self.STATE_MOVING):
            return
        try:
            mode = self._current_mode()
            if mode == self.MODE_OBSTACLE:
                self.selected_obstacle_id = self.model.add_obstacle(self._snap_to_voxel(point))
            elif mode == self.MODE_START:
                voxel = self._snap_to_voxel(point)
                self.model.set_start(voxel)
                self._set_coordinate_inputs(self.start_inputs, voxel)
            elif mode == self.MODE_GOAL:
                voxel = self._snap_to_voxel(point)
                self.model.set_goal(voxel)
                self._set_coordinate_inputs(self.goal_inputs, voxel)
            else:
                self._select_nearest_obstacle(point)
            self._scene_changed()
        except SceneValidationError as exc:
            self._show_error(str(exc))

    def _snap_to_voxel(self, point) -> Voxel:
        x = max(0, min(self.grid_max, int(round(float(point[0])))))
        y = max(0, min(self.grid_max, int(round(float(point[1])))))
        return (x, y, self.z_slider.value())

    def _select_nearest_obstacle(self, point) -> None:
        if not self.model.obstacles:
            self.selected_obstacle_id = None
            return
        selected = min(self.model.obstacles.values(), key=lambda item: math.dist(item.voxel, point))
        self.selected_obstacle_id = selected.obstacle_id if math.dist(selected.voxel, point) <= 1 else None

    def _apply_start(self) -> None:
        self._apply_endpoint(self.start_inputs, self.model.set_start)

    def _apply_goal(self) -> None:
        self._apply_endpoint(self.goal_inputs, self.model.set_goal)

    def _apply_endpoint(self, inputs, setter) -> None:
        try:
            setter(tuple(spin.value() for spin in inputs))
            self._scene_changed()
        except SceneValidationError as exc:
            self._show_error(str(exc))

    @staticmethod
    def _set_coordinate_inputs(inputs, voxel: Voxel) -> None:
        for spin, value in zip(inputs, voxel):
            spin.setValue(value)

    def _on_speed_changed(self, value: float) -> None:
        if self._updating_controls or self.selected_obstacle_id is None:
            return
        try:
            self.model.set_obstacle_speed(self.selected_obstacle_id, value)
            self._scene_changed()
        except SceneValidationError as exc:
            self._show_error(str(exc))
            self._sync_selected_controls()

    def _on_gain_changed(self, value: float) -> None:
        if self._updating_controls:
            return
        try:
            self.model.set_padding_gain(value)
            self._scene_changed()
        except SceneValidationError as exc:
            self._show_error(str(exc))
            self._updating_controls = True
            self.gain_spin.setValue(self.model.padding_gain)
            self._updating_controls = False

    def _delete_selected(self) -> None:
        if self.selected_obstacle_id is not None:
            self.model.delete_obstacle(self.selected_obstacle_id)
            self.selected_obstacle_id = None
            self._scene_changed()

    def _clear_obstacles(self) -> None:
        self.model.clear_obstacles()
        self.selected_obstacle_id = None
        self._scene_changed()

    def _scene_changed(self) -> None:
        self._remove_actor("scan_highlight")
        self._refresh_scene()
        self._show_status(self.STATE_EDITING if self.model.current_robot is None else self.STATE_STOPPED)

    def _start_run(self) -> None:
        try:
            self.model.initialize_robot()
            if self.model.goal is None:
                raise SceneValidationError("goal is required")
        except SceneValidationError as exc:
            self._show_error(str(exc))
            return
        self._scan_entries = self.model.scan_entries()
        self._scan_index = 0
        self._set_editing_enabled(False)
        self.state = self.STATE_SCANNING
        self._show_status(self.STATE_SCANNING)
        self._animation_timer.start(1)

    def _stop_run(self) -> None:
        self._animation_timer.stop()
        self._remove_actor("scan_highlight")
        self.state = self.STATE_STOPPED
        self._set_editing_enabled(True)
        self._refresh_scene()
        self._show_status(self.STATE_STOPPED)

    def _on_animation_tick(self) -> None:
        if self.state == self.STATE_SCANNING:
            self._scan_tick()
        elif self.state == self.STATE_MOVING:
            self._move_tick()

    def _scan_tick(self) -> None:
        if self._scan_index >= len(self._scan_entries):
            self._remove_actor("scan_highlight")
            self._plan_and_begin_movement()
            return
        entry = self._scan_entries[self._scan_index]
        self._render_voxel_group(
            "scan_highlight", entry.padded_voxels, "#ffe066", 0.85, pickable=False
        )
        self._show_status(f"SCANNING obstacle {entry.obstacle_id}")
        self._scan_index += 1
        self._animation_timer.start(int(self.scan_step_spin.value() * 1000))

    def _plan_and_begin_movement(self) -> None:
        try:
            outcome = self.planner.plan(self.model)
        except Exception as exc:
            self._finish_without_motion(f"Planning error: {exc}")
            return
        self._show_outcome(outcome)
        if outcome.result is None or not outcome.result.success:
            self._finish_without_motion(outcome.status)
            return
        self._planned_path = list(outcome.result.path)
        self._traveled_path = [self.model.current_robot] if self.model.current_robot else []
        self._move_index = 1
        self._render_polyline("planned_path", self._planned_path, "#64a5ff", 5)
        self._render_polyline("traveled_path", self._traveled_path, "#35c98f", 8)
        self.state = self.STATE_MOVING
        self._show_status(self.STATE_MOVING)
        self._animation_timer.start(int(self.robot_step_spin.value() * 1000))

    def _move_tick(self) -> None:
        if self._move_index >= len(self._planned_path):
            self.state = self.STATE_FINISHED
            self._set_editing_enabled(True)
            self._show_status(self.STATE_FINISHED)
            return
        voxel = self._planned_path[self._move_index]
        self.model.move_robot(voxel)
        self._traveled_path.append(voxel)
        self._move_index += 1
        self._render_robot()
        self._render_polyline("traveled_path", self._traveled_path, "#35c98f", 8)
        self._show_status(f"MOVING {self._move_index}/{len(self._planned_path)}")
        self._animation_timer.start(int(self.robot_step_spin.value() * 1000))

    def _finish_without_motion(self, message: str) -> None:
        self.state = self.STATE_STOPPED
        self._set_editing_enabled(True)
        self._show_error(message)

    def _refresh_scene(self) -> None:
        self._render_pick_plane()
        self._render_voxels()
        self._render_endpoint("start", self.model.start, "#35c98f")
        self._render_endpoint("goal", self.model.goal, "#b879ff")
        self._render_robot()
        self._sync_selected_controls()
        self.plotter.render()

    def _render_pick_plane(self) -> None:
        size = self.model.grid_size[0]
        plane = pv.Plane(
            center=(self.grid_max / 2, self.grid_max / 2, self.z_slider.value()),
            direction=(0, 0, 1),
            i_size=size,
            j_size=size,
            i_resolution=size,
            j_resolution=size,
        )
        self._replace_actor(
            "pick_plane", plane, color="#e1c45c", opacity=0.12, show_edges=True, pickable=True
        )

    def _render_voxels(self) -> None:
        base = {obstacle.voxel for obstacle in self.model.obstacles.values()}
        self._render_voxel_group(
            "padding", self.model.padded_obstacles() - base, "#ed9644", 0.28, False
        )
        self._render_voxel_group("obstacles", base, "#df4f4f", 0.95, True)

    def _render_voxel_group(
        self, name: str, voxels: Iterable[Voxel], color: str, opacity: float, pickable: bool
    ) -> None:
        points = list(voxels)
        if not points:
            self._remove_actor(name)
            return
        float_points = [tuple(float(value) for value in point) for point in points]
        mesh = pv.PolyData(float_points).glyph(geom=pv.Cube(), scale=False, orient=False)
        self._replace_actor(
            name, mesh, color=color, opacity=opacity, show_edges=True, pickable=pickable
        )

    def _render_endpoint(self, name: str, voxel: Optional[Voxel], color: str) -> None:
        if voxel is None:
            self._remove_actor(name)
            return
        self._replace_actor(name, pv.Sphere(radius=0.42, center=voxel), color=color, pickable=False)

    def _render_robot(self) -> None:
        if self.model.current_robot is None:
            self._remove_actor("robot")
            return
        self._replace_actor(
            "robot",
            pv.Sphere(radius=0.34, center=self.model.current_robot),
            color="#4de3ff",
            pickable=False,
        )

    def _render_polyline(self, name: str, points: List[Voxel], color: str, width: int) -> None:
        if not points:
            self._remove_actor(name)
        else:
            float_points = [tuple(float(value) for value in point) for point in points]
            mesh = (
                pv.PolyData(float_points)
                if len(float_points) == 1
                else pv.lines_from_points(float_points)
            )
            self._replace_actor(name, mesh, color=color, line_width=width, pickable=False)
        self.plotter.render()

    def _replace_actor(self, name: str, mesh, **kwargs) -> None:
        self._remove_actor(name)
        self._actors[name] = self.plotter.add_mesh(mesh, name=name, **kwargs)

    def _remove_actor(self, name: str) -> None:
        actor = self._actors.pop(name, None)
        if actor is not None:
            self.plotter.remove_actor(actor, render=False)

    def _sync_selected_controls(self) -> None:
        selected = self.model.obstacles.get(self.selected_obstacle_id)
        self._updating_controls = True
        if selected is None:
            self.selected_obstacle_id = None
            self.selected_label.setText("None")
            self.speed_spin.setEnabled(False)
            self.delete_button.setEnabled(False)
            self.speed_spin.setValue(0.0)
        else:
            self.selected_label.setText(f"{selected.obstacle_id} / {selected.voxel}")
            self.speed_spin.setEnabled(True)
            self.delete_button.setEnabled(True)
            self.speed_spin.setValue(selected.speed)
        self._updating_controls = False

    def _show_outcome(self, outcome: PlanningOutcome) -> None:
        if outcome.result is None:
            self.result_label.setText(outcome.status)
            return
        metrics = outcome.result.metrics
        self.result_label.setText(
            f"{outcome.result.reason}\nPlanning: {metrics.get('planning_time_ms', 0.0):.3f} ms\n"
            f"Cost: {metrics.get('path_cost')}\nPath nodes: {metrics.get('path_length')}\n"
            f"Expanded: {metrics.get('expanded_steps')}\n"
            f"Padded obstacles: {metrics.get('obstacle_count')}"
        )

    def _show_status(self, status: str) -> None:
        self.statusBar().showMessage(status)
        if not self.result_label.text().startswith(("OK", "NO_PATH", "ERROR")):
            self.result_label.setText(status)

    def _show_error(self, message: str) -> None:
        self.statusBar().showMessage(message, 5000)
        self.result_label.setText(f"ERROR\n{message}")

    def closeEvent(self, event) -> None:
        self._animation_timer.stop()
        self.plotter.close()
        super().closeEvent(event)
