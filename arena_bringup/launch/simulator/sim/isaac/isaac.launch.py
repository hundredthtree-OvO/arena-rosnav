import os

import launch
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription


def generate_launch_description():
    ros2isaacsim_launch = None
    try:
        candidate = os.path.join(
            get_package_share_directory("ros2isaacsim"),
            "launch",
            "run_isaacsim.launch.py",
        )
        if os.path.exists(candidate):
            ros2isaacsim_launch = candidate
    except PackageNotFoundError:
        ros2isaacsim_launch = None

    if ros2isaacsim_launch is None:
        # Fallback for source-only checkouts where ros2isaacsim is not installed
        # into the current overlay but arena-isaac exists in the same workspace.
        ros2isaacsim_launch = os.path.normpath(
            os.path.join(
                os.path.dirname(__file__),
                "/home/stardust/resources/arena_ws/src/arena/arena-isaac/ros2isaacsim/launch/run_isaacsim.launch.py",
            )
        )

    if not os.path.exists(ros2isaacsim_launch):
        raise FileNotFoundError(
            f"Could not locate ros2isaacsim launch file. Tried: {ros2isaacsim_launch}"
        )

    isaac_path = launch.substitutions.LaunchConfiguration("isaac_path")
    logger = launch.substitutions.LaunchConfiguration("log_level")
    enable_people_stack = launch.substitutions.LaunchConfiguration("enable_people_stack")
    enable_character_services = launch.substitutions.LaunchConfiguration("enable_character_services")
    people_extension_mode = launch.substitutions.LaunchConfiguration("people_extension_mode")
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "isaac_path",
            default_value=os.environ.get(
                "ISAAC_PATH",
                os.path.expanduser("~/resources/isaac-sim-4.5.0"),
            ),
            description="Path to Isaac Sim installation directory.",
        ),
        launch.actions.DeclareLaunchArgument(
            "log_level",
            default_value=["debug"],
            description="Logging level",
        ),
        launch.actions.DeclareLaunchArgument(
            "enable_people_stack",
            default_value="true",
            description="Enable Isaac People stack for arena-rosnav Isaac human simulation.",
        ),
        launch.actions.DeclareLaunchArgument(
            "enable_character_services",
            default_value="true",
            description="Enable Isaac pedestrian spawn/move services for upper-layer human simulators.",
        ),
        launch.actions.DeclareLaunchArgument(
            "people_extension_mode",
            default_value="replicator_agent_core",
            description="People extension mode used for Isaac human simulation.",
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                ros2isaacsim_launch
            ),
            launch_arguments={
                "isaac_path": isaac_path,
                "log_level": logger,
                "enable_people_stack": enable_people_stack,
                "enable_character_services": enable_character_services,
                "people_extension_mode": people_extension_mode,
            }.items(),
        ),
    ])
