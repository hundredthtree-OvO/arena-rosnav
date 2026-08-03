"""Standalone authored-scenario editor for the toilet benchmark."""

from __future__ import annotations

import json
import math
from pathlib import Path

from python_qt_binding import QtCore, QtWidgets

from toilet_benchmark.episodes.schema import EpisodeSpec
from toilet_benchmark.walkable_map_planner import (
    WalkableMapPlanner,
    WalkableMapPlannerConfig,
)

from .editor_model import HoldDraft, MapSnapshot, ScenarioDraft
from .map_canvas import MapCanvas
from .ros_bridge import RosWorker


DEFAULT_MAP = Path(
    "/home/stardust/resources/arena_ws/arena_assets/navigation/"
    "shenxinfu_841837.walkable.json"
)


class EditorWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Toilet Benchmark - 场景路线编辑器")
        self.resize(1280, 820)
        self._map: MapSnapshot | None = None
        self._map_path = DEFAULT_MAP
        self._scenario = ScenarioDraft.default()
        self._scenario.add_pedestrian("toilet_agent_01")
        self._selected_actor_id = "toilet_agent_01"

        self._canvas = MapCanvas()
        self._canvas.route_changed.connect(self._route_changed)
        self._canvas.spawn_clicked.connect(self._spawn_clicked)
        self._canvas.cursor_world_changed.connect(self._cursor_changed)
        self._canvas.edit_finished.connect(self._validate)
        self._canvas.waypoint_selected.connect(self._waypoint_selected)

        self._build_toolbar()
        self._build_layout()
        self._refresh_actor_list()
        self._refresh_canvas()
        if self._map_path.is_file():
            self._load_map_path(self._map_path)

        self._bridge = RosWorker()
        self._bridge.map_ready.connect(self._map_ready)
        self._bridge.people_ready.connect(self._canvas.set_people)
        self._bridge.robot_ready.connect(self._canvas.set_robot)
        self._bridge.anchors_ready.connect(self._canvas.set_anchors)
        self._bridge.ros_error.connect(self._status)
        self._bridge.start()
        self._status("可离线加载地图并编辑；在线状态仅作为预览，不会改变草稿。")

    def _build_toolbar(self) -> None:
        toolbar = self.addToolBar("场景编辑")
        toolbar.setMovable(False)
        group = QtWidgets.QActionGroup(self)
        group.setExclusive(True)
        for label, mode in (("选择", "select"), ("画路线", "draw"),
                            ("编辑路线点", "edit"), ("设置出生点", "spawn")):
            action = QtWidgets.QAction(label, self)
            action.setCheckable(True)
            action.setChecked(mode == "select")
            action.triggered.connect(lambda _checked=False, value=mode: self._set_mode(value))
            group.addAction(action)
            toolbar.addAction(action)
        toolbar.addSeparator()
        for label, handler in (("删除路线点", self._delete_waypoint),
                               ("清空当前路线", self._clear_route),
                               ("适配地图", self._canvas.fit_map)):
            action = QtWidgets.QAction(label, self)
            action.triggered.connect(handler)
            toolbar.addAction(action)

    def _build_layout(self) -> None:
        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal, self)
        splitter.setChildrenCollapsible(False)
        splitter.setHandleWidth(8)
        splitter.addWidget(self._canvas)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(320)
        panel = QtWidgets.QWidget()
        root = QtWidgets.QVBoxLayout(panel)

        scenario_box = QtWidgets.QGroupBox("场景")
        scenario_form = QtWidgets.QFormLayout(scenario_box)
        self._scenario_id = QtWidgets.QLineEdit(self._scenario.scenario_id)
        self._scenario_id.editingFinished.connect(self._sync_scenario_fields)
        scenario_form.addRow("场景 ID", self._scenario_id)
        self._seed = QtWidgets.QSpinBox()
        self._seed.setRange(0, 2_000_000_000)
        self._seed.setValue(self._scenario.seed)
        self._seed.valueChanged.connect(self._sync_scenario_fields)
        scenario_form.addRow("Seed", self._seed)
        self._radius = QtWidgets.QDoubleSpinBox()
        self._radius.setRange(0.0, 0.8)
        self._radius.setSingleStep(0.01)
        self._radius.setValue(0.30)
        self._radius.setSuffix(" m")
        self._radius.valueChanged.connect(self._validate)
        scenario_form.addRow("静态验证半径", self._radius)
        root.addWidget(scenario_box)

        actor_box = QtWidgets.QGroupBox("参与者")
        actor_layout = QtWidgets.QVBoxLayout(actor_box)
        self._actor_list = QtWidgets.QListWidget()
        self._actor_list.setMaximumHeight(150)
        self._actor_list.currentTextChanged.connect(self._actor_selected)
        actor_layout.addWidget(self._actor_list)
        actor_buttons = QtWidgets.QHBoxLayout()
        add_button = QtWidgets.QPushButton("添加行人")
        add_button.clicked.connect(self._add_pedestrian)
        remove_button = QtWidgets.QPushButton("删除行人")
        remove_button.clicked.connect(self._remove_pedestrian)
        actor_buttons.addWidget(add_button)
        actor_buttons.addWidget(remove_button)
        actor_layout.addLayout(actor_buttons)
        root.addWidget(actor_box)

        properties = QtWidgets.QGroupBox("选中参与者")
        form = QtWidgets.QFormLayout(properties)
        self._kind = QtWidgets.QLabel("-")
        form.addRow("类型", self._kind)
        self._yaw = QtWidgets.QDoubleSpinBox()
        self._yaw.setRange(-180.0, 180.0)
        self._yaw.setSuffix(" deg")
        self._yaw.valueChanged.connect(self._actor_fields_changed)
        form.addRow("出生朝向", self._yaw)
        self._speed = QtWidgets.QDoubleSpinBox()
        self._speed.setRange(0.05, 2.5)
        self._speed.setSingleStep(0.05)
        self._speed.setSuffix(" m/s")
        self._speed.valueChanged.connect(self._actor_fields_changed)
        form.addRow("行人速度", self._speed)
        self._constrain = QtWidgets.QCheckBox("严格沿编辑路线")
        self._constrain.toggled.connect(self._actor_fields_changed)
        form.addRow("路线约束", self._constrain)
        self._actor_summary = QtWidgets.QLabel("-")
        self._actor_summary.setWordWrap(True)
        form.addRow("草稿", self._actor_summary)
        root.addWidget(properties)

        hold_box = QtWidgets.QGroupBox("阶段性停留")
        hold_form = QtWidgets.QFormLayout(hold_box)
        self._selected_point = QtWidgets.QLabel("未选择路线点")
        hold_form.addRow("路线点", self._selected_point)
        self._hold_duration = QtWidgets.QDoubleSpinBox()
        self._hold_duration.setRange(0.1, 300.0)
        self._hold_duration.setValue(2.0)
        self._hold_duration.setSuffix(" s")
        hold_form.addRow("停留时间", self._hold_duration)
        hold_buttons = QtWidgets.QHBoxLayout()
        add_hold = QtWidgets.QPushButton("在选中点停留")
        add_hold.clicked.connect(self._add_hold)
        remove_hold = QtWidgets.QPushButton("移除停留")
        remove_hold.clicked.connect(self._remove_hold)
        hold_buttons.addWidget(add_hold)
        hold_buttons.addWidget(remove_hold)
        hold_form.addRow(hold_buttons)
        root.addWidget(hold_box)

        files = QtWidgets.QGroupBox("文件与验证")
        file_layout = QtWidgets.QGridLayout(files)
        controls = (("加载地图", self._load_map), ("加载场景", self._load_scenario),
                    ("保存场景", self._save_scenario), ("验证场景", self._validate_with_planner))
        for index, (label, handler) in enumerate(controls):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(handler)
            file_layout.addWidget(button, index // 2, index % 2)
        self._validation = QtWidgets.QLabel("未验证")
        self._validation.setWordWrap(True)
        file_layout.addWidget(self._validation, 2, 0, 1, 2)
        root.addWidget(files)

        self._cursor = QtWidgets.QLabel("光标: -")
        self._status_label = QtWidgets.QLabel()
        self._status_label.setWordWrap(True)
        root.addWidget(self._cursor)
        root.addWidget(self._status_label)
        root.addStretch(1)
        scroll.setWidget(panel)
        splitter.addWidget(scroll)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 0)
        splitter.setSizes([930, 350])
        self.setCentralWidget(splitter)

    def _actor(self):
        try:
            return self._scenario.actor(self._selected_actor_id)
        except KeyError:
            return None

    def _refresh_actor_list(self) -> None:
        selected = self._selected_actor_id
        self._actor_list.blockSignals(True)
        self._actor_list.clear()
        self._actor_list.addItem(self._scenario.robot.actor_id)
        self._actor_list.addItems([actor.actor_id for actor in self._scenario.pedestrians])
        matches = self._actor_list.findItems(selected, QtCore.Qt.MatchExactly)
        self._actor_list.setCurrentItem(matches[0] if matches else self._actor_list.item(0))
        self._actor_list.blockSignals(False)
        self._actor_selected(self._actor_list.currentItem().text())

    def _actor_selected(self, actor_id: str) -> None:
        if not actor_id:
            return
        self._selected_actor_id = str(actor_id)
        actor = self._actor()
        if actor is None:
            return
        self._kind.setText("机器人" if actor.kind == "robot" else "行人")
        self._yaw.blockSignals(True)
        self._yaw.setValue(0.0 if actor.spawn_pose is None else math.degrees(actor.spawn_pose[3]))
        self._yaw.blockSignals(False)
        pedestrian = actor.kind == "pedestrian"
        self._speed.setEnabled(pedestrian)
        self._constrain.setEnabled(pedestrian)
        self._speed.blockSignals(True)
        self._speed.setValue(actor.velocity_mps)
        self._speed.blockSignals(False)
        self._constrain.blockSignals(True)
        self._constrain.setChecked(actor.constrain_to_path)
        self._constrain.blockSignals(False)
        self._waypoint_selected(None)
        self._refresh_canvas()
        self._update_summary()

    def _add_pedestrian(self) -> None:
        number = 1
        existing = {actor.actor_id for actor in self._scenario.pedestrians}
        while f"toilet_agent_{number:02d}" in existing:
            number += 1
        actor = self._scenario.add_pedestrian(f"toilet_agent_{number:02d}")
        self._selected_actor_id = actor.actor_id
        self._refresh_actor_list()
        self._validate()

    def _remove_pedestrian(self) -> None:
        actor = self._actor()
        if actor is None or actor.kind != "pedestrian":
            self._status("机器人不能从场景中删除。")
            return
        self._scenario.remove_pedestrian(actor.actor_id)
        self._selected_actor_id = self._scenario.robot.actor_id
        self._refresh_actor_list()
        self._validate()

    def _set_mode(self, mode: str) -> None:
        if mode in ("draw", "edit") and (self._actor() is None or self._actor().kind != "pedestrian"):
            self._status("机器人只设置出生位；请先选择一个行人再编辑路线。")
            self._canvas.set_mode("select")
            return
        self._canvas.set_mode(mode)
        self._status(f"编辑模式: {mode}")

    def _spawn_clicked(self, point) -> None:
        actor = self._actor()
        if actor is None:
            return
        z = 0.03 if actor.kind == "robot" else 0.0
        yaw = 0.0 if actor.spawn_pose is None else actor.spawn_pose[3]
        actor.spawn_pose = (float(point[0]), float(point[1]), z, yaw)
        self._refresh_canvas()
        self._update_summary()
        self._validate()

    def _actor_fields_changed(self, *_args) -> None:
        actor = self._actor()
        if actor is None:
            return
        if actor.spawn_pose is not None:
            actor.spawn_pose = (*actor.spawn_pose[:3], math.radians(self._yaw.value()))
        if actor.kind == "pedestrian":
            actor.velocity_mps = float(self._speed.value())
            actor.constrain_to_path = bool(self._constrain.isChecked())
        self._refresh_canvas()

    def _route_changed(self, _points) -> None:
        self._update_summary()
        self._validate()

    def _waypoint_selected(self, index) -> None:
        self._selected_point.setText("未选择路线点" if index is None else f"waypoint {index}")

    def _add_hold(self) -> None:
        actor = self._actor()
        index = self._canvas.selected_waypoint
        if actor is None or actor.kind != "pedestrian" or index is None:
            self._status("请先选择一个行人的路线点。")
            return
        actor.holds = [hold for hold in actor.holds if hold.waypoint_index != index]
        actor.holds.append(HoldDraft(index, float(self._hold_duration.value())))
        actor.holds.sort(key=lambda hold: hold.waypoint_index)
        self._refresh_canvas()
        self._update_summary()
        self._validate()

    def _remove_hold(self) -> None:
        actor = self._actor()
        index = self._canvas.selected_waypoint
        if actor is None or index is None:
            return
        actor.holds = [hold for hold in actor.holds if hold.waypoint_index != index]
        self._refresh_canvas()
        self._update_summary()

    def _delete_waypoint(self) -> None:
        if not self._canvas.delete_selected_waypoint():
            self._status("请先选中一个路线点。")

    def _clear_route(self) -> None:
        actor = self._actor()
        if actor is None or actor.kind != "pedestrian":
            return
        actor.route.clear()
        actor.holds.clear()
        self._refresh_canvas()
        self._update_summary()
        self._validate()

    def _sync_scenario_fields(self, *_args) -> None:
        self._scenario.scenario_id = str(self._scenario_id.text().strip() or "authored_scenario")
        self._scenario.seed = int(self._seed.value())

    def _refresh_canvas(self) -> None:
        self._canvas.set_scenario(self._scenario, self._selected_actor_id)

    def _update_summary(self) -> None:
        actor = self._actor()
        if actor is None:
            return
        spawn = "未设置" if actor.spawn_pose is None else f"({actor.spawn_pose[0]:.2f}, {actor.spawn_pose[1]:.2f})"
        self._actor_summary.setText(
            f"出生点: {spawn}\n路线点: {len(actor.route)}\n停留点: {len(actor.holds)}"
        )

    def _cursor_changed(self, point) -> None:
        self._cursor.setText(f"光标: ({point[0]:.3f}, {point[1]:.3f})")

    def _map_ready(self, grid: MapSnapshot) -> None:
        self._map = grid
        self._canvas.set_map(grid)
        self._validate()

    def _load_map(self) -> None:
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "加载 walkable map", str(self._map_path), "Walkable map (*.json)"
        )
        if not filename:
            return
        self._load_map_path(Path(filename))

    def _load_map_path(self, path: Path) -> None:
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            if payload.get("schema") != "arena.walkable_map.v1":
                raise ValueError("不是 arena.walkable_map.v1 文件")
            self._map_path = path
            self._map_ready(MapSnapshot.from_payload(payload))
            self._status(f"地图已加载: {path}")
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            self._status(f"地图加载失败: {exc}")

    def _save_scenario(self) -> None:
        self._sync_scenario_fields()
        errors = self._scenario_errors(plan_routes=True)
        if errors:
            self._status("场景未通过验证，拒绝保存。")
            self._show_validation(errors)
            return
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "保存可执行场景", str(Path.home() / f"{self._scenario.scenario_id}.json"),
            "Toilet episode (*.json)"
        )
        if not filename:
            return
        try:
            episode = self._scenario.to_episode_spec(map_path=str(self._map_path))
            Path(filename).write_text(json.dumps(episode.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")
            self._status(f"已保存可执行 episode: {filename}")
        except (OSError, ValueError) as exc:
            self._status(f"保存失败: {exc}")

    def _load_scenario(self) -> None:
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "加载可执行场景", str(Path.home()), "Toilet episode (*.json)"
        )
        if not filename:
            return
        try:
            payload = json.loads(Path(filename).read_text(encoding="utf-8"))
            episode = EpisodeSpec.from_mapping(payload)
            if episode.task_type != "authored_route":
                raise ValueError(f"task_type 必须是 authored_route，实际为 {episode.task_type}")
            self._scenario = ScenarioDraft.from_episode_spec(episode)
            map_path = episode.assets.get("walkable_map")
            if map_path:
                self._map_path = Path(str(map_path))
                if self._map_path.is_file():
                    self._load_map_path(self._map_path)
            self._scenario_id.setText(self._scenario.scenario_id)
            self._seed.setValue(self._scenario.seed)
            self._selected_actor_id = self._scenario.robot.actor_id
            self._refresh_actor_list()
            self._validate()
            self._status(f"场景已加载: {filename}")
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            self._status(f"场景加载失败: {exc}")

    def _validate(self, *_args) -> list[str]:
        errors = self._scenario_errors(plan_routes=False)
        self._show_validation(errors)
        return errors

    def _validate_with_planner(self, *_args) -> list[str]:
        errors = self._scenario_errors(plan_routes=True)
        self._show_validation(errors)
        return errors

    def _scenario_errors(self, *, plan_routes: bool) -> list[str]:
        self._sync_scenario_fields()
        errors = self._scenario.validate(self._map, radius_m=float(self._radius.value()))
        if not plan_routes or errors or not self._map_path.is_file():
            return errors
        try:
            planner = WalkableMapPlanner.from_file(
                WalkableMapPlannerConfig(
                    map_path=str(self._map_path),
                    agent_radius_m=float(self._radius.value()),
                )
            )
            for actor in self._scenario.pedestrians:
                cursor = actor.spawn_pose
                if cursor is None:
                    continue
                for target_index, target in enumerate(actor.route):
                    try:
                        planner.plan(list(cursor[:3]), list(target), z=float(target[2]))
                    except Exception as exc:
                        errors.append(
                            f"{actor.actor_id}: cannot plan to target {target_index}: {exc}"
                        )
                        break
                    cursor = target
        except (OSError, ValueError, KeyError, TypeError) as exc:
            errors.append(f"walkable map planner unavailable: {exc}")
        return errors

    def _show_validation(self, errors: list[str]) -> None:
        if errors:
            self._validation.setStyleSheet("color: #c84b54")
            self._validation.setText("REJECT\n" + "\n".join(f"- {error}" for error in errors))
        else:
            self._validation.setStyleSheet("color: #2e9d62")
            self._validation.setText("PASS: 场景参与者、路线、停留点均有效")

    def _status(self, message: str) -> None:
        self._status_label.setText(str(message))

    def closeEvent(self, event) -> None:
        self._bridge.stop()
        event.accept()


def main(args=None) -> int:
    app = QtWidgets.QApplication(args or [])
    window = EditorWindow()
    window.show()
    return int(app.exec_())
