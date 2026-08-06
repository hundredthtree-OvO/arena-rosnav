import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from python_qt_binding import QtWidgets

from toilet_benchmark_ui.editor_model import MapSnapshot, ScenarioDraft
from toilet_benchmark_ui.map_canvas import MapCanvas, oriented_box_corners

_APP = None


def _canvas(scenario: ScenarioDraft, actor_id: str) -> MapCanvas:
    global _APP
    _APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    canvas = MapCanvas()
    canvas.set_map(
        MapSnapshot(
            frame_id="map",
            resolution=0.1,
            width=20,
            height=20,
            origin=(-1.0, -1.0),
            data=(0,) * 400,
        )
    )
    canvas.set_scenario(scenario, actor_id)
    return canvas


def test_overlapping_pedestrian_spawn_wins_deterministic_edit_hit() -> None:
    scenario = ScenarioDraft.default()
    actor = scenario.add_pedestrian("toilet_agent_01")
    actor.spawn_pose = (0.0, 0.0, 0.0, 0.0)
    actor.route = [(0.0, 0.0, 0.0)]
    canvas = _canvas(scenario, actor.actor_id)

    assert canvas._nearest_edit_handle(canvas._scene_point((0.0, 0.0))) == ("spawn", None)


def test_robot_edit_hit_includes_goal_only_when_robot_is_selected() -> None:
    scenario = ScenarioDraft.default()
    scenario.robot.goal_pose = (0.8, 0.8, 0.0)
    pedestrian = scenario.add_pedestrian("toilet_agent_01")
    pedestrian.spawn_pose = (-0.8, -0.8, 0.0, 0.0)
    canvas = _canvas(scenario, scenario.robot.actor_id)

    point = canvas._scene_point(scenario.robot.goal_pose)
    assert canvas._nearest_edit_handle(point) == ("goal", None)

    canvas.set_scenario(scenario, pedestrian.actor_id)
    assert canvas._nearest_edit_handle(point) is None


def test_robot_footprint_corners_preserve_metric_length_width_and_yaw() -> None:
    corners = oriented_box_corners((1.0, 2.0), yaw=0.5 * 3.141592653589793,
                                   length=0.70, width=0.42)

    assert corners[0] == pytest.approx((0.79, 2.35))
    assert corners[1] == pytest.approx((0.79, 1.65))
    assert corners[2] == pytest.approx((1.21, 1.65))
    assert corners[3] == pytest.approx((1.21, 2.35))


def test_canvas_footprints_match_isaac_scene_profile() -> None:
    assert MapCanvas.PEDESTRIAN_FOOTPRINT_RADIUS_M == pytest.approx(0.26)
    assert MapCanvas.ROBOT_FOOTPRINT_LENGTH_M == pytest.approx(0.70)
    assert MapCanvas.ROBOT_FOOTPRINT_WIDTH_M == pytest.approx(0.42)
