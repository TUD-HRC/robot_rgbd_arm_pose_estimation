cmake_minimum_required(VERSION 3.8)
project(arm_pose_estimation)

# The EKF has a real-time budget: build optimized unless told otherwise.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type (empty means Release)" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(geometry_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# --- EKF ---------------------------------------------------------------------
# arm_pose_ekf_core: the filter, kinematics and measurement models, without ROS.
add_library(arm_pose_ekf_core STATIC
  src/ekf/joint_state_ekf.cpp
  src/ekf/arm_kinematics.cpp
  src/ekf/robot_kinematics.cpp
  src/ekf/measurements.cpp
)
target_include_directories(arm_pose_ekf_core PUBLIC include)
target_compile_features(arm_pose_ekf_core PUBLIC cxx_std_17)
target_link_libraries(arm_pose_ekf_core PUBLIC pinocchio::pinocchio)

add_executable(arm_pose_ekf_node
  src/arm_pose_ekf_node.cpp
  src/ekf/ekf_config.cpp
)
ament_target_dependencies(arm_pose_ekf_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
target_link_libraries(arm_pose_ekf_node arm_pose_ekf_core)
install(TARGETS arm_pose_ekf_node DESTINATION lib/${PROJECT_NAME})

# --- Vision front end ----------------------------------------------------------
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/keypoint_extraction
  scripts/swivel_solver
  DESTINATION lib/${PROJECT_NAME}
)

# mediapipe and opencv come from pip, in a virtual environment. An environment
# hook appends its site-packages to PYTHONPATH, so it never has to be activated.
# Empty: no hook (enough to replay recordings, which do not run the front end).
set(ARM_POSE_VENV "$ENV{VIRTUAL_ENV}" CACHE PATH
  "Virtual environment providing mediapipe and opencv; empty for none.")
if(ARM_POSE_VENV)
  if(NOT EXISTS "${ARM_POSE_VENV}/bin/python3")
    message(FATAL_ERROR "ARM_POSE_VENV '${ARM_POSE_VENV}' is not a virtual environment.")
  endif()
  execute_process(
    COMMAND "${ARM_POSE_VENV}/bin/python3" -c
      "import sys, sysconfig; print(sysconfig.get_path('purelib')); print('%d.%d' % sys.version_info[:2])"
    OUTPUT_VARIABLE _venv_query
    OUTPUT_STRIP_TRAILING_WHITESPACE
    COMMAND_ERROR_IS_FATAL ANY
  )
  string(REPLACE "\n" ";" _venv_query "${_venv_query}")
  list(GET _venv_query 0 ARM_POSE_VENV_SITE_PACKAGES)
  list(GET _venv_query 1 _venv_python_version)
  # Compiled packages in the venv must match the interpreter that runs the nodes.
  set(_python_version "${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}")
  if(NOT _venv_python_version VERSION_EQUAL _python_version)
    message(FATAL_ERROR "The virtual environment is Python ${_venv_python_version}, but ROS "
      "uses Python ${_python_version}. Recreate it with `python${_python_version} -m venv`.")
  endif()
  message(STATUS "Vision dependencies from: ${ARM_POSE_VENV_SITE_PACKAGES}")
  ament_environment_hooks(env-hooks/arm_pose_estimation_venv.dsv.in)
endif()

# --- Resources -----------------------------------------------------------------
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # No license headers; the C++ uses 4-space indentation, which cpplint and
  # uncrustify reject.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_joint_state_ekf test/test_joint_state_ekf.cpp)
  target_link_libraries(test_joint_state_ekf arm_pose_ekf_core)
  ament_add_gtest(test_measurements test/test_measurements.cpp)
  target_link_libraries(test_measurements arm_pose_ekf_core)
  target_compile_definitions(test_measurements PRIVATE
    ARM_XACRO_PATH="${CMAKE_CURRENT_SOURCE_DIR}/urdf/right_arm.urdf.xacro")

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_shoulder_anchor test/test_shoulder_anchor.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
endif()

ament_package()
