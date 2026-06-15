from __future__ import annotations

import math
from typing import Dict, Iterable, Optional

import pyvista as pv
from PySide6 import QtCore, QtWidgets
from pyvistaqt import QtInteractor

from astar_simulation.planner_adapter import PlannerAdapter, PlanningOutcome
from astar_simulation.simulation_model import SceneValidationError, SimulationModel, Voxel


class AppWindow(QtWidgets.QMainWindow):
    MODE_SELECT = "Select"
    MODE_OBSTACLE = "Add Obstacle"
    MODE_START = "Set Start"
    MODE_GOAL = "Set Goal"

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ARA* Improved 3D Simulator")
        self.resize(1400, 900)

        self.model = SimulationModel()
        self.planner = PlannerAdapter(max_time_ms=500.0)
        self.selected_obstacle_id: Optional[int] = None
        self._actors: Dict[str, object] = {}
        self._updating_controls = False

        self.plotter = QtInteractor(self)
        self.setCentralWidget(self.plotter.interactor)
        self._build_controls()
        self._build_scene()

        self._replan_timer = QtCore.QTimer(self)
        self._replan_timer.setSingleShot(True)
        self._replan_timer.setInterval(80)
        self._replan_timer.timeout.connect(self._replan_and_render)

        self.plotter.enable_point_picking(
            callback=self._on_point_picked,
            show_message=False,
            show_point=False,
            pickable_window=True,
            left_clicking=True,
        )
        self._refresh_scene()
        self._schedule_replan()

    def _build_controls(self) -> None:
        dock = QtWidgets.QDockWidget("Controls", self)
        dock.setFeatures(QtWidgets.QDockWidget.DockWidgetMovable)
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
            layout.addWidget(button)

        layout.addSpacing(8)
        layout.addWidget(QtWidgets.QLabel("Pick plane Z"))
        self.z_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.z_slider.setRange(0, 19)
        self.z_slider.setValue(0)
        self.z_label = QtWidgets.QLabel("Z = 0")
        self.z_slider.valueChanged.connect(self._on_z_changed)
        layout.addWidget(self.z_slider)
        layout.addWidget(self.z_label)

        self.start_inputs = self._add_coordinate_editor(layout, "Start", self._apply_start)
        self.goal_inputs = self._add_coordinate_editor(layout, "Goal", self._apply_goal)

        obstacle_box = QtWidgets.QGroupBox("Selected obstacle")
        obstacle_layout = QtWidgets.QFormLayout(obstacle_box)
        self.selected_label = QtWidgets.QLabel("None")
        self.speed_spin = QtWidgets.QDoubleSpinBox()
        self.speed_spin.setRange(0.0, 19.0)
        self.speed_spin.setDecimals(2)
        self.speed_spin.setSingleStep(0.25)
        self.speed_spin.valueChanged.connect(self._on_speed_changed)
        obstacle_layout.addRow("ID / voxel", self.selected_label)
        obstacle_layout.addRow("Speed", self.speed_spin)
        self.delete_button = QtWidgets.QPushButton("Delete Selected")
        self.delete_button.clicked.connect(self._delete_selected)
        obstacle_layout.addRow(self.delete_button)
        layout.addWidget(obstacle_box)

        gain_box = QtWidgets.QGroupBox("Padding")
        gain_layout = QtWidgets.QFormLayout(gain_box)
        self.gain_spin = QtWidgets.QDoubleSpinBox()
        self.gain_spin.setRange(0.0, 10.0)
        self.gain_spin.setDecimals(2)
        self.gain_spin.setSingleStep(0.25)
        self.gain_spin.setValue(self.model.padding_gain)
        self.gain_spin.valueChanged.connect(self._on_gain_changed)
        gain_layout.addRow("Linear gain", self.gain_spin)
        layout.addWidget(gain_box)

        self.clear_button = QtWidgets.QPushButton("Clear Obstacles")
        self.clear_button.clicked.connect(self._clear_obstacles)
        layout.addWidget(self.clear_button)

        result_box = QtWidgets.QGroupBox("ARA* result")
        result_layout = QtWidgets.QVBoxLayout(result_box)
        self.result_label = QtWidgets.QLabel("WAITING_FOR_START_GOAL")
        self.result_label.setWordWrap(True)
        result_layout.addWidget(self.result_label)
        layout.addWidget(result_box)

        self.help_label = QtWidgets.QLabel(
            "Left drag: rotate\nWheel: zoom\nRight drag: pan\n"
            "Placement uses X/Y click + active Z."
        )
        self.help_label.setWordWrap(True)
        layout.addWidget(self.help_label)
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
            spin.setRange(0, 19)
            spin.setPrefix(f"{axis}:")
            row.addWidget(spin)
            inputs.append(spin)
        apply_button = QtWidgets.QPushButton(f"Set {title}")
        apply_button.clicked.connect(callback)
        form.addRow(row)
        form.addRow(apply_button)
        layout.addWidget(box)
        return tuple(inputs)

    def _build_scene(self) -> None:
        self.plotter.set_background("#101722")
        grid = pv.ImageData(dimensions=(21, 21, 21), spacing=(1, 1, 1), origin=(-0.5, -0.5, -0.5))
        self.plotter.add_mesh(
            grid.extract_all_edges(),
            color="#53677d",
            opacity=0.18,
            line_width=1,
            pickable=False,
        )
        self.plotter.show_bounds(
            bounds=(-0.5, 19.5, -0.5, 19.5, -0.5, 19.5),
            grid="back",
            location="outer",
            xtitle="X",
            ytitle="Y",
            ztitle="Z",
        )
        self.plotter.camera_position = "iso"
        self.plotter.reset_camera()

    def _current_mode(self) -> str:
        checked = self.mode_group.checkedButton()
        return checked.text() if checked is not None else self.MODE_SELECT

    def _on_z_changed(self, value: int) -> None:
        self.z_label.setText(f"Z = {value}")
        self._render_pick_plane()

    def _on_point_picked(self, point) -> None:
        if point is None:
            return
        try:
            mode = self._current_mode()
            if mode == self.MODE_OBSTACLE:
                voxel = self._snap_to_voxel(point, self.z_slider.value())
                self.selected_obstacle_id = self.model.add_obstacle(voxel)
            elif mode == self.MODE_START:
                voxel = self._snap_to_voxel(point, self.z_slider.value())
                self.model.set_start(voxel)
                self._set_coordinate_inputs(self.start_inputs, voxel)
            elif mode == self.MODE_GOAL:
                voxel = self._snap_to_voxel(point, self.z_slider.value())
                self.model.set_goal(voxel)
                self._set_coordinate_inputs(self.goal_inputs, voxel)
            else:
                self._select_nearest_obstacle(point)
            self._refresh_scene()
            self._schedule_replan()
        except SceneValidationError as exc:
            self._show_error(str(exc))

    @staticmethod
    def _snap_to_voxel(point, z_value: int) -> Voxel:
        x = max(0, min(19, int(round(float(point[0])))))
        y = max(0, min(19, int(round(float(point[1])))))
        return (x, y, int(z_value))

    def _select_nearest_obstacle(self, point) -> None:
        if not self.model.obstacles:
            self.selected_obstacle_id = None
            return
        selected = min(
            self.model.obstacles.values(),
            key=lambda obstacle: math.dist(obstacle.voxel, point),
        )
        self.selected_obstacle_id = (
            selected.obstacle_id if math.dist(selected.voxel, point) <= 1.0 else None
        )

    def _apply_start(self) -> None:
        self._apply_endpoint(self.start_inputs, self.model.set_start)

    def _apply_goal(self) -> None:
        self._apply_endpoint(self.goal_inputs, self.model.set_goal)

    def _apply_endpoint(self, inputs, setter) -> None:
        try:
            setter(tuple(spin.value() for spin in inputs))
            self._refresh_scene()
            self._schedule_replan()
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
            self._refresh_scene()
            self._schedule_replan()
        except SceneValidationError as exc:
            self._show_error(str(exc))
            self._sync_selected_controls()

    def _on_gain_changed(self, value: float) -> None:
        if self._updating_controls:
            return
        try:
            self.model.set_padding_gain(value)
            self._refresh_scene()
            self._schedule_replan()
        except SceneValidationError as exc:
            self._show_error(str(exc))
            self._updating_controls = True
            self.gain_spin.setValue(self.model.padding_gain)
            self._updating_controls = False

    def _delete_selected(self) -> None:
        if self.selected_obstacle_id is None:
            return
        self.model.delete_obstacle(self.selected_obstacle_id)
        self.selected_obstacle_id = None
        self._refresh_scene()
        self._schedule_replan()

    def _clear_obstacles(self) -> None:
        self.model.clear_obstacles()
        self.selected_obstacle_id = None
        self._refresh_scene()
        self._schedule_replan()

    def _schedule_replan(self) -> None:
        self._replan_timer.start()

    def _replan_and_render(self) -> None:
        try:
            outcome = self.planner.plan(self.model)
            self._render_path(outcome)
            self._show_outcome(outcome)
        except Exception as exc:
            self._show_error(f"Planning error: {exc}")

    def _refresh_scene(self) -> None:
        self._render_pick_plane()
        self._render_voxels()
        self._render_endpoint("start", self.model.start, "#35c98f")
        self._render_endpoint("goal", self.model.goal, "#b879ff")
        self._sync_selected_controls()
        self.plotter.render()

    def _render_pick_plane(self) -> None:
        plane = pv.Plane(
            center=(9.5, 9.5, self.z_slider.value()),
            direction=(0, 0, 1),
            i_size=20,
            j_size=20,
            i_resolution=20,
            j_resolution=20,
        )
        self._replace_actor(
            "pick_plane",
            plane,
            color="#e1c45c",
            opacity=0.12,
            show_edges=True,
            pickable=True,
        )

    def _render_voxels(self) -> None:
        base = {obstacle.voxel for obstacle in self.model.obstacles.values()}
        padded_only = self.model.padded_obstacles() - base
        self._render_voxel_group("padding", padded_only, "#ed9644", 0.28, pickable=False)
        self._render_voxel_group("obstacles", base, "#df4f4f", 0.95, pickable=True)

    def _render_voxel_group(
        self,
        name: str,
        voxels: Iterable[Voxel],
        color: str,
        opacity: float,
        pickable: bool,
    ) -> None:
        points = list(voxels)
        if not points:
            self._remove_actor(name)
            return
        mesh = pv.PolyData(points).glyph(geom=pv.Cube(), scale=False)
        self._replace_actor(
            name,
            mesh,
            color=color,
            opacity=opacity,
            show_edges=True,
            pickable=pickable,
        )

    def _render_endpoint(self, name: str, voxel: Optional[Voxel], color: str) -> None:
        if voxel is None:
            self._remove_actor(name)
            return
        self._replace_actor(
            name,
            pv.Sphere(radius=0.48, center=voxel),
            color=color,
            opacity=1.0,
            pickable=False,
        )

    def _render_path(self, outcome: PlanningOutcome) -> None:
        if outcome.result is None or not outcome.result.path:
            self._remove_actor("path")
            self.plotter.render()
            return
        points = outcome.result.path
        if len(points) == 1:
            mesh = pv.PolyData(points)
        else:
            mesh = pv.lines_from_points(points)
        self._replace_actor("path", mesh, color="#64a5ff", line_width=6, pickable=False)
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
            f"{outcome.result.reason}\n"
            f"Planning: {metrics.get('planning_time_ms', 0.0):.3f} ms\n"
            f"Cost: {metrics.get('path_cost')}\n"
            f"Path nodes: {metrics.get('path_length')}\n"
            f"Expanded: {metrics.get('expanded_steps')}\n"
            f"Padded obstacles: {metrics.get('obstacle_count')}"
        )

    def _show_error(self, message: str) -> None:
        self.statusBar().showMessage(message, 5000)
        self.result_label.setText(f"ERROR\n{message}")

    def closeEvent(self, event) -> None:
        self.plotter.close()
        super().closeEvent(event)
