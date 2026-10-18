cmake_minimum_required(VERSION 3.8)
project(dragonfly_arm)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SolveIK.srv"
  DEPENDENCIES geometry_msgs
)

ament_python_install_package(dragonfly_arm_kinematics)

install(
  DIRECTORY config launch urdf worlds docs
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  PROGRAMS scripts/demo_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME demo_trajectory
)

install(
  PROGRAMS scripts/kinematics_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME kinematics_node
)

if(BUILD_TESTING)
  # The host has user-installed pytest plugins that are newer than Ubuntu's
  # pytest. Keep this package's self-contained tests isolated from them.
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_urdf
    test/test_urdf.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  ament_add_pytest_test(
    test_kinematics
    test/test_kinematics.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  ament_add_pytest_test(
    test_kinematics_node
    test/test_kinematics_node.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1" "ROS_DOMAIN_ID=71" "ROS_LOCALHOST_ONLY=1"
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
