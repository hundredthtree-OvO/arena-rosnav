"""Interactive map canvas for authored multi-actor scenarios."""

from __future__ import annotations

import math

from python_qt_binding import QtCore, QtGui, QtWidgets


class MapCanvas(QtWidgets.QGraphicsView):
    route_changed = QtCore.Signal(object)
    spawn_clicked = QtCore.Signal(object)
    cursor_world_changed = QtCore.Signal(object)
    edit_finished = QtCore.Signal()
    waypoint_selected = QtCore.Signal(object)

    _ACTOR_COLORS = ("#f5c451", "#67d5b5", "#ff7f7f", "#72a7ff", "#d291ff")

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setScene(QtWidgets.QGraphicsScene(self))
        self.setRenderHint(QtGui.QPainter.Antialiasing, True)
        self.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QtWidgets.QGraphicsView.AnchorViewCenter)
        self.setBackgroundBrush(QtGui.QBrush(QtGui.QColor("#20242a")))
        self.setMouseTracking(True)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self._map = None
        self._map_item = None
        self._mode = "select"
        self._scenario = None
        self._selected_actor_id = ""
        self._people: list[dict] = []
        self._robot: dict | None = None
        self._anchors: list[dict] = []
        self._overlay_items: list[object] = []
        self._drag_index: int | None = None
        self._selected_waypoint: int | None = None
        self._selection_radius_px = 10.0

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def selected_waypoint(self) -> int | None:
        return self._selected_waypoint

    def set_mode(self, mode: str) -> None:
        self._mode = str(mode)
        cursor = QtCore.Qt.CrossCursor if self._mode in ("draw", "spawn") else QtCore.Qt.ArrowCursor
        self.setCursor(cursor)

    def set_map(self, grid) -> None:
        self._map = grid
        image = QtGui.QImage(grid.width, grid.height, QtGui.QImage.Format_RGB32)
        for py in range(grid.height):
            source_y = grid.height - 1 - py
            for px in range(grid.width):
                value = grid.data[source_y * grid.width + px]
                color = QtGui.QColor(218, 222, 226)
                if value < 0:
                    color = QtGui.QColor(138, 143, 150)
                elif value >= 50:
                    color = QtGui.QColor(38, 42, 48)
                elif value > 0:
                    color = QtGui.QColor(182, 187, 193)
                image.setPixelColor(px, py, color)
        if self._map_item is not None:
            self.scene().removeItem(self._map_item)
        self._map_item = self.scene().addPixmap(QtGui.QPixmap.fromImage(image))
        self._map_item.setZValue(-100.0)
        self.scene().setSceneRect(0.0, 0.0, grid.width, grid.height)
        self.fit_map()
        self._redraw()

    def set_scenario(self, scenario, selected_actor_id: str) -> None:
        self._scenario = scenario
        self._selected_actor_id = str(selected_actor_id)
        actor = self._selected_actor()
        if actor is None or actor.kind != "pedestrian":
            self._selected_waypoint = None
        elif self._selected_waypoint is not None and self._selected_waypoint >= len(actor.route):
            self._selected_waypoint = None
        self._redraw()

    def set_people(self, people: list[dict]) -> None:
        self._people = list(people)
        self._redraw()

    def set_robot(self, robot: dict | None) -> None:
        self._robot = robot
        self._redraw()

    def set_anchors(self, anchors: list[dict]) -> None:
        self._anchors = list(anchors)
        self._redraw()

    def fit_map(self) -> None:
        if self._map_item is not None:
            self.fitInView(self.scene().sceneRect(), QtCore.Qt.KeepAspectRatio)

    def delete_selected_waypoint(self) -> bool:
        if self._selected_waypoint is None:
            return False
        actor = self._selected_actor()
        if actor is None or actor.kind != "pedestrian":
            return False
        index = self._selected_waypoint
        actor.route.pop(index)
        actor.holds = [
            hold for hold in actor.holds if hold.waypoint_index != index
        ]
        for hold in actor.holds:
            if hold.waypoint_index > index:
                hold.waypoint_index -= 1
        self._selected_waypoint = None
        self.route_changed.emit(list(actor.route))
        self.waypoint_selected.emit(None)
        self._redraw()
        return True

    def _selected_actor(self):
        if self._scenario is None or not self._selected_actor_id:
            return None
        try:
            return self._scenario.actor(self._selected_actor_id)
        except KeyError:
            return None

    def _scene_point(self, world_xy):
        px, py = self._map.world_to_pixel(float(world_xy[0]), float(world_xy[1]))
        return QtCore.QPointF(px, py)

    def _world_point(self, scene_point):
        if self._map is None:
            return None
        return self._map.pixel_to_world(scene_point.x(), scene_point.y())

    def _add(self, item, z: float = 10.0):
        item.setZValue(z)
        self._overlay_items.append(item)
        return item

    def _path(self, points, color: str, *, selected: bool) -> None:
        if not points:
            return
        path = QtGui.QPainterPath(self._scene_point(points[0]))
        for point in points[1:]:
            path.lineTo(self._scene_point(point))
        pen = QtGui.QPen(QtGui.QColor(color), 3.0 if selected else 1.5)
        pen.setCosmetic(True)
        if not selected:
            pen.setStyle(QtCore.Qt.DashLine)
        item = QtWidgets.QGraphicsPathItem(path)
        self.scene().addItem(item)
        self._add(item, 8.0)
        item.setPen(pen)

    def _redraw(self) -> None:
        for item in self._overlay_items:
            if item.scene() is self.scene():
                self.scene().removeItem(item)
        self._overlay_items.clear()
        if self._map is None:
            return
        for anchor in self._anchors:
            self._circle((anchor["x"], anchor["y"]), 0.04, "#4fd18b")
        if self._scenario is not None:
            for index, actor in enumerate(self._scenario.pedestrians):
                selected = actor.actor_id == self._selected_actor_id
                color = self._ACTOR_COLORS[index % len(self._ACTOR_COLORS)]
                self._path(actor.route, color, selected=selected)
                if actor.spawn_pose is not None:
                    self._circle(actor.spawn_pose, 0.11, color, label=f"{actor.actor_id} 起点")
                    self._arrow(actor.spawn_pose, actor.spawn_pose[3], color, 0.35)
                for point_index, point in enumerate(actor.route):
                    is_selected = selected and point_index == self._selected_waypoint
                    self._circle(point, 0.055, "#ffffff" if is_selected else color, label=str(point_index))
                for hold in actor.holds:
                    if hold.waypoint_index < len(actor.route):
                        self._ring(actor.route[hold.waypoint_index], color, label=f"停 {hold.duration_sec:g}s")
            robot = self._scenario.robot
            if robot.spawn_pose is not None:
                self._square(robot.spawn_pose, "#b18cff", label="机器人起点")
                self._arrow(robot.spawn_pose, robot.spawn_pose[3], "#b18cff", 0.4)
        for person in self._people:
            self._circle(person["position"], 0.18, "#63b3ed", label=person.get("id", ""))
        if self._robot is not None:
            self._square(self._robot["position"], "#d6b6ff", label="机器人实时")

    def _radius_px(self, metres: float, minimum: float = 1.5, maximum: float = 5.0) -> float:
        return max(minimum, min(maximum, metres / self._map.resolution))

    def _circle(self, point, metres: float, color: str, label: str = "") -> None:
        center = self._scene_point(point)
        radius = self._radius_px(metres)
        pen = QtGui.QPen(QtGui.QColor(color), 1.5)
        pen.setCosmetic(True)
        self._add(self.scene().addEllipse(center.x() - radius, center.y() - radius,
                                          radius * 2.0, radius * 2.0, pen,
                                          QtGui.QBrush(QtGui.QColor(color))))
        self._label(center, label, color, radius)

    def _ring(self, point, color: str, label: str) -> None:
        center = self._scene_point(point)
        radius = self._radius_px(0.10, maximum=6.0)
        pen = QtGui.QPen(QtGui.QColor(color), 2.0)
        pen.setCosmetic(True)
        self._add(self.scene().addEllipse(center.x() - radius, center.y() - radius,
                                          radius * 2.0, radius * 2.0, pen))
        self._label(center, label, color, radius)

    def _square(self, point, color: str, label: str) -> None:
        center = self._scene_point(point)
        radius = self._radius_px(0.14)
        pen = QtGui.QPen(QtGui.QColor(color), 1.5)
        pen.setCosmetic(True)
        self._add(self.scene().addRect(center.x() - radius, center.y() - radius,
                                       radius * 2.0, radius * 2.0, pen,
                                       QtGui.QBrush(QtGui.QColor(color))))
        self._label(center, label, color, radius)

    def _label(self, center, label: str, color: str, offset: float) -> None:
        if not label:
            return
        text = self._add(self.scene().addText(str(label)), 15.0)
        text.setDefaultTextColor(QtGui.QColor(color))
        text.setFlag(QtWidgets.QGraphicsItem.ItemIgnoresTransformations, True)
        text.setPos(center.x() + offset + 2.0, center.y() - offset - 2.0)

    def _arrow(self, point, yaw: float, color: str, length_m: float) -> None:
        start = self._scene_point(point)
        end = self._scene_point((point[0] + math.cos(yaw) * length_m,
                                 point[1] + math.sin(yaw) * length_m))
        pen = QtGui.QPen(QtGui.QColor(color), 1.5)
        pen.setCosmetic(True)
        self._add(self.scene().addLine(QtCore.QLineF(start, end), pen), 12.0)

    def _nearest_waypoint(self, scene_point) -> int | None:
        actor = self._selected_actor()
        if actor is None or actor.kind != "pedestrian":
            return None
        nearest = None
        distance_limit = self._selection_radius_px
        for index, point in enumerate(actor.route):
            location = self._scene_point(point)
            distance = math.hypot(location.x() - scene_point.x(), location.y() - scene_point.y())
            if distance <= distance_limit:
                nearest, distance_limit = index, distance
        return nearest

    def _select_waypoint(self, index: int | None) -> None:
        self._selected_waypoint = index
        self.waypoint_selected.emit(index)
        self._redraw()

    def mousePressEvent(self, event) -> None:
        if self._map is None:
            return super().mousePressEvent(event)
        scene_point = self.mapToScene(event.pos())
        world = self._world_point(scene_point)
        actor = self._selected_actor()
        if world is None or actor is None:
            return super().mousePressEvent(event)
        if event.button() == QtCore.Qt.RightButton:
            self._select_waypoint(self._nearest_waypoint(scene_point))
            self.delete_selected_waypoint()
            return
        if event.button() != QtCore.Qt.LeftButton:
            return super().mousePressEvent(event)
        if self._mode == "spawn":
            self.spawn_clicked.emit((world[0], world[1]))
            self.set_mode("select")
        elif actor.kind == "pedestrian" and self._mode == "draw":
            actor.route.append((world[0], world[1], 0.0))
            self.route_changed.emit(list(actor.route))
            self._redraw()
        elif actor.kind == "pedestrian" and self._mode == "edit":
            self._drag_index = self._nearest_waypoint(scene_point)
            self._select_waypoint(self._drag_index)
        elif self._mode == "select":
            self._select_waypoint(self._nearest_waypoint(scene_point))
        self.cursor_world_changed.emit(world)

    def mouseMoveEvent(self, event) -> None:
        world = self._world_point(self.mapToScene(event.pos()))
        if world is not None:
            self.cursor_world_changed.emit(world)
        actor = self._selected_actor()
        if self._drag_index is not None and world is not None and actor is not None:
            actor.route[self._drag_index] = (world[0], world[1], 0.0)
            self.route_changed.emit(list(actor.route))
            self._redraw()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        if self._drag_index is not None:
            self._drag_index = None
            self.edit_finished.emit()
        super().mouseReleaseEvent(event)

    def keyPressEvent(self, event) -> None:
        if event.key() in (QtCore.Qt.Key_Delete, QtCore.Qt.Key_Backspace):
            if self.delete_selected_waypoint():
                event.accept()
                return
        super().keyPressEvent(event)

    def wheelEvent(self, event) -> None:
        self.scale(1.15 if event.angleDelta().y() > 0 else 1.0 / 1.15,
                   1.15 if event.angleDelta().y() > 0 else 1.0 / 1.15)
