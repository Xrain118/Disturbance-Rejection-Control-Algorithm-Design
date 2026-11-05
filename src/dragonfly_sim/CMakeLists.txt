cmake_minimum_required(VERSION 3.16)
project(dragonfly_sim)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

add_library(dragonfly_physics SHARED src/physics.cpp)
target_compile_features(dragonfly_physics PUBLIC cxx_std_17)
target_link_libraries(dragonfly_physics gz-sim8::gz-sim8 gz-plugin2::register)

install(TARGETS dragonfly_physics DESTINATION lib)
install(
  DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(PROGRAMS scripts/demo_trajectory.py DESTINATION lib/${PROJECT_NAME} RENAME demo_trajectory)

ament_package()
