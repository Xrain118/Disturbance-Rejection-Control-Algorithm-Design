cmake_minimum_required(VERSION 3.16)
project(dragonfly_onboard)

find_package(ament_cmake REQUIRED)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
ament_package()
