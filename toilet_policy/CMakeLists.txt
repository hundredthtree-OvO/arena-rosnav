cmake_minimum_required(VERSION 3.8)
project(toilet_policy)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
install(FILES package.xml README.md DESTINATION share/${PROJECT_NAME}/)
install(PROGRAMS
  scripts/toilet_policy_export
  scripts/toilet_policy_eval
  scripts/toilet_policy_inference
  scripts/toilet_policy_train
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
