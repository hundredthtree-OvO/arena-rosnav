from toilet_benchmark.domain import AgentTaskFeedback, DirectiveType, WorldSnapshot
from toilet_benchmark.episodes.schema import (
    EpisodeSpec,
    PedestrianEpisodeSpec,
    RobotEpisodeSpec,
    TerminationSpec,
    TrackType,
)
from toilet_benchmark.scenario import (
    DirectorScenarioAdapter,
    EnterUseExit,
    ObjectKind,
    ScenarioAgentAssignment,
    ScenarioRuntime,
    ScenarioRuntimeMode,
    SmartObject,
    SmartObjectRegistry,
    SmartObjectSlot,
    OrientedRegion,
)
from toilet_benchmark.resource_manager import QueueSlot, Resource, ResourceManager


def _episode():
    return EpisodeSpec(
        episode_id="golden_enter_use_exit",
        scene_id="test_toilet",
        task_type="enter_use_exit",
        track=TrackType.INTERACTIVE,
        seed=7,
        robot=RobotEpisodeSpec(
            model="test_robot",
            start_pose=(0.0, 0.0, 0.0, 0.0),
            goal_pose=(1.0, 0.0, 0.0),
        ),
        pedestrians=(
            PedestrianEpisodeSpec(agent_id="agent_01", semantic_goal="urinal_1"),
            PedestrianEpisodeSpec(agent_id="agent_02", semantic_goal="urinal_1"),
        ),
        termination=TerminationSpec(timeout_sec=30.0, goal_tolerance_m=0.2),
    )


def _runtime():
    resource = SmartObject(
        object_id="urinal_1",
        kind=ObjectKind.URINAL,
        interaction_slots=(SmartObjectSlot("urinal_1:use", (1.0, 0.0, 0.0)),),
        queue_slots=(SmartObjectSlot("urinal_1:queue:1", (0.0, 0.0, 0.0)),),
    )
    return ScenarioRuntime(
        SmartObjectRegistry((resource,)),
        EnterUseExit(exit_target_id="exit_main", activity_duration_sec=2.0),
    )


def _run_golden_trace():
    runtime = _runtime()
    runtime.reset(_episode())
    frames = (
        WorldSnapshot(0.0),
        WorldSnapshot(
            1.0,
            task_feedback={"agent_01": AgentTaskFeedback(reached_target_id="urinal_1")},
        ),
        WorldSnapshot(3.0),
        WorldSnapshot(
            4.0,
            task_feedback={
                "agent_01": AgentTaskFeedback(reached_target_id="exit_main"),
                "agent_02": AgentTaskFeedback(reached_target_id="urinal_1"),
            },
        ),
        WorldSnapshot(6.0),
        WorldSnapshot(
            7.0,
            task_feedback={"agent_02": AgentTaskFeedback(reached_target_id="exit_main")},
        ),
    )
    return runtime, tuple(runtime.tick(frame) for frame in frames)


def test_enter_use_exit_golden_event_sequence_and_directives():
    runtime, results = _run_golden_trace()

    event_sequence = [
        (event.agent_id, event.event_type)
        for result in results
        for event in result.events
    ]
    assert event_sequence == [
        ("agent_01", "smart_object_claimed"),
        ("agent_02", "smart_object_queued"),
        ("agent_01", "smart_object_reached"),
        ("agent_01", "activity_started"),
        ("agent_01", "activity_completed"),
        ("agent_01", "smart_object_released"),
        ("agent_02", "smart_object_promoted"),
        ("agent_01", "agent_completed"),
        ("agent_02", "smart_object_reached"),
        ("agent_02", "activity_started"),
        ("agent_02", "activity_completed"),
        ("agent_02", "smart_object_released"),
        ("agent_02", "agent_completed"),
    ]
    assert [directive.directive_type for directive in results[0].directives] == [
        DirectiveType.MOVE_TO_OBJECT,
        DirectiveType.WAIT_AT_QUEUE,
    ]
    assert results[2].directives[0].directive_type == DirectiveType.MOVE_TO_EXIT
    assert results[2].directives[0].object_id is None
    assert results[-1].completed is True
    assert runtime.registry.snapshot()["urinal_1"] == {"owners": {}, "queue": []}


def test_runtime_reset_replays_identical_trace():
    _, first = _run_golden_trace()
    _, second = _run_golden_trace()

    def serialize(results):
        return [
            {
                "directives": [
                    (item.agent_id, item.directive_type.value, item.target_id, item.slot_id)
                    for item in result.directives
                ],
                "events": [item.to_payload() for item in result.events],
                "completed": result.completed,
            }
            for result in results
        ]

    assert serialize(first) == serialize(second)


def test_runtime_requires_episode_reset():
    runtime = _runtime()

    try:
        runtime.tick(WorldSnapshot(0.0))
    except RuntimeError as exc:
        assert "reset" in str(exc)
    else:
        raise AssertionError("runtime tick should require an episode")


def test_adjacent_resources_do_not_share_a_room_wide_passage_lock():
    def resource(object_id, x):
        return SmartObject(
            object_id=object_id,
            kind=ObjectKind.URINAL,
            interaction_slots=(
                SmartObjectSlot(
                    f"{object_id}:use",
                    (x, 1.0, 1.5707963267948966),
                    occupancy_region=OrientedRegion(
                        (x, 1.0), 1.5707963267948966, 0.42, 0.26
                    ),
                    passage_region=OrientedRegion(
                        (x, 0.55), 1.5707963267948966, 0.45, 0.42
                    ),
                ),
            ),
        )

    runtime = ScenarioRuntime(
        SmartObjectRegistry((resource("urinal_1", 0.0), resource("urinal_2", 0.77))),
        EnterUseExit(exit_target_id="exit_main", activity_duration_sec=1.0),
    )
    episode = EpisodeSpec(
        episode_id="adjacent_passages",
        scene_id="test_toilet",
        task_type="enter_use_exit",
        track=TrackType.INTERACTIVE,
        seed=7,
        robot=RobotEpisodeSpec(
            model="test_robot",
            start_pose=(0.0, 0.0, 0.0, 0.0),
            goal_pose=(1.0, 0.0, 0.0),
        ),
        pedestrians=(
            PedestrianEpisodeSpec(agent_id="agent_01", semantic_goal="urinal_1"),
            PedestrianEpisodeSpec(agent_id="agent_02", semantic_goal="urinal_2"),
        ),
        termination=TerminationSpec(timeout_sec=30.0, goal_tolerance_m=0.2),
    )
    runtime.reset(episode)

    first = runtime.tick(WorldSnapshot(0.0))
    assert [item.directive_type for item in first.directives] == [
        DirectiveType.MOVE_TO_OBJECT,
        DirectiveType.MOVE_TO_OBJECT,
    ]
    assert first.directives[0].target_id == "urinal_1"
    assert first.directives[1].target_id == "urinal_2"

    second = runtime.tick(
        WorldSnapshot(
            1.0,
            task_feedback={
                "agent_01": AgentTaskFeedback(reached_target_id="urinal_1"),
                "agent_02": AgentTaskFeedback(reached_target_id="urinal_2"),
            },
        )
    )
    assert [item.directive_type for item in second.directives] == [
        DirectiveType.START_ACTIVITY,
        DirectiveType.START_ACTIVITY,
    ]

    third = runtime.tick(WorldSnapshot(2.0))
    assert [item.directive_type for item in third.directives] == [
        DirectiveType.MOVE_TO_PASSAGE,
        DirectiveType.START_ACTIVITY,
    ]

    fourth = runtime.tick(
        WorldSnapshot(
            3.0,
            task_feedback={
                "agent_01": AgentTaskFeedback(
                    reached_target_id="urinal_1:passage:egress"
                ),
                "agent_02": AgentTaskFeedback(
                    reached_target_id="urinal_2:passage:egress"
                ),
            },
        )
    )
    assert [item.directive_type for item in fourth.directives] == [
        DirectiveType.MOVE_TO_EXIT,
        DirectiveType.MOVE_TO_PASSAGE,
    ]


def test_adapter_uses_geometry_heading_instead_of_character_root_yaw():
    manager = ResourceManager(
        {
            "urinal_1": Resource(
                resource_id="urinal_1",
                category="urinals",
                position=[2.0, 1.0, 0.0],
                yaw=3.14159,
                body_yaw=1.5708,
                passage_yaw=1.5708,
            )
        }
    )
    adapter = DirectorScenarioAdapter.from_legacy_resources(
        manager,
        exit_target_id="exit_main",
        passage_depth_m=0.9,
    )

    staging = adapter.passage_staging_pose("urinal_1")
    holding = adapter.passage_holding_pose("urinal_1")

    assert staging is not None
    assert abs(staging[0] - 2.0) < 1e-4
    assert abs(staging[1] - 0.1) < 1e-4
    assert holding is not None
    assert abs(holding[0] - 2.0) < 1e-4
    assert abs(holding[1] + 0.35) < 1e-4


def test_director_adapter_mirrors_takeover_ownership_and_promotion():
    manager = ResourceManager(
        {
            "urinal_1": Resource(
                resource_id="urinal_1",
                category="urinals",
                position=[1.0, 0.0, 0.0],
                queue_slots=[QueueSlot("queue_1", [0.0, 0.0, 0.0])],
            )
        }
    )
    adapter = DirectorScenarioAdapter.from_legacy_resources(
        manager,
        exit_target_id="exit_main",
        mode=ScenarioRuntimeMode.TAKEOVER,
    )
    adapter.reset(
        (
            ScenarioAgentAssignment("agent_01", "urinal_1"),
            ScenarioAgentAssignment("agent_02", "urinal_1"),
        ),
        episode_id="adapter_test",
        seed=3,
        timestamp_sec=0.0,
    )
    adapter.sync_legacy_resources(manager)

    assert manager.resources["urinal_1"].occupied_by == "agent_01"
    assert manager.resources["urinal_1"].queue_slots[0].occupant_id == "agent_02"

    adapter.advance(
        1.0,
        {"agent_01": AgentTaskFeedback(reached_target_id="urinal_1:passage")},
    )
    adapter.advance(
        2.0,
        {"agent_01": AgentTaskFeedback(reached_target_id="urinal_1")},
    )
    adapter.advance(
        3.0,
        {"agent_01": AgentTaskFeedback(activity_complete=True)},
    )
    adapter.sync_legacy_resources(manager)

    assert manager.resources["urinal_1"].occupied_by == "agent_02"
    assert manager.resources["urinal_1"].queue_slots[0].occupant_id is None
    assert adapter.compare_legacy_resources(manager) == ()


def test_promoted_queue_agent_allows_legacy_queueing_transition():
    manager = ResourceManager(
        {
            "urinal_1": Resource(
                resource_id="urinal_1",
                category="urinals",
                position=[1.0, 0.0, 0.0],
                queue_slots=[QueueSlot("queue_1", [0.0, 0.0, 0.0])],
            )
        }
    )
    adapter = DirectorScenarioAdapter.from_legacy_resources(
        manager,
        exit_target_id="exit_main",
        mode=ScenarioRuntimeMode.SHADOW,
    )
    adapter.reset(
        (
            ScenarioAgentAssignment("agent_01", "urinal_1"),
            ScenarioAgentAssignment("agent_02", "urinal_1"),
        ),
        episode_id="promotion_transition_test",
        seed=3,
        timestamp_sec=0.0,
    )
    adapter.advance(
        1.0,
        {"agent_01": AgentTaskFeedback(reached_target_id="urinal_1:passage")},
    )
    adapter.advance(
        2.0,
        {"agent_01": AgentTaskFeedback(reached_target_id="urinal_1")},
    )
    adapter.advance(
        3.0,
        {"agent_01": AgentTaskFeedback(activity_complete=True)},
    )

    # Promotion lets the waiter approach the staging point, while the same
    # station's egress token still protects the final local passage.
    assert adapter.directive_for("agent_02").directive_type == DirectiveType.MOVE_TO_PASSAGE
    assert adapter.compare_legacy_phases({"agent_02": "QUEUEING"}) == ()
    adapter.advance(
        4.0,
        {
            "agent_01": AgentTaskFeedback(
                reached_target_id="urinal_1:passage:egress"
            )
        },
    )
    assert adapter.directive_for("agent_02").directive_type == DirectiveType.MOVE_TO_PASSAGE


def test_shadow_adapter_reports_resource_owner_drift_without_mutating_legacy():
    manager = ResourceManager(
        {
            "urinal_1": Resource(
                resource_id="urinal_1",
                category="urinals",
                position=[1.0, 0.0, 0.0],
            )
        }
    )
    adapter = DirectorScenarioAdapter.from_legacy_resources(
        manager,
        exit_target_id="exit_main",
        mode=ScenarioRuntimeMode.SHADOW,
    )
    adapter.reset(
        (ScenarioAgentAssignment("agent_01", "urinal_1"),),
        episode_id="shadow_test",
        seed=3,
        timestamp_sec=0.0,
    )

    mismatches = adapter.compare_legacy_resources(manager)

    assert manager.resources["urinal_1"].occupied_by is None
    assert len(mismatches) == 1
    assert "owner mismatch" in mismatches[0].reason


def test_cancelling_owner_releases_and_promotes_waiting_agent():
    runtime = _runtime()
    runtime.reset(_episode())
    runtime.tick(WorldSnapshot(0.0))

    result = runtime.cancel_agent(
        "agent_01",
        1.0,
        reason="motion recovery exhausted",
    )

    assert [event.event_type for event in result.events] == [
        "smart_object_released",
        "agent_cancelled",
        "smart_object_promoted",
    ]
    assert runtime.registry.claim_for_agent("agent_01") is None
    assert runtime.registry.claim_for_agent("agent_02").status.value == "acquired"
    ScenarioAgentAssignment,
    ScenarioRuntimeMode,
