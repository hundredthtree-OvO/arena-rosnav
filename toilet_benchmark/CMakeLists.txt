cmake_minimum_required(VERSION 3.8)
project(toilet_benchmark)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}/
)

install(
  FILES
    package.xml
    README.md
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/manual_collection_node
    scripts/toilet_manifest
    scripts/toilet_replay
    scripts/toilet_replay_export
    scripts/toilet_replay_isaac
    scripts/toilet_authored_scenario
    scripts/pedestrian_diagnostic_recorder
    scripts/walkable_map_publisher
    scripts/animgraph_phase_probe
    scripts/toilet_dataset
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
