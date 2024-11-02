cmake_minimum_required(VERSION 3.8)
project(clifford_sim_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)         # Add any necessary dependencies
find_package(std_msgs REQUIRED)        # Add additional dependencies as needed
# Add more find_package calls for other libraries you might use

# Add the executable
add_executable(clifford_joint_keyboard src/clifford_sim_1/clifford_joint_keyboard.cpp) # Update the path if necessary

# Link dependencies
ament_target_dependencies(clifford_joint_keyboard
  rclcpp
  std_msgs
  # Add other dependencies if required
)

# Install the executable
install(TARGETS
  clifford_joint_keyboard
  DESTINATION lib/${PROJECT_NAME}
)

# Install other directories
install(
  DIRECTORY launch meshes rviz urdf 
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  scripts/joy_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # Comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # Comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
