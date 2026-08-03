cmake_minimum_required(VERSION 3.8)
project(toilet_benchmark_ui)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  FILES
    package.xml
    README.md
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY
    docs
    examples
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/route_editor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
