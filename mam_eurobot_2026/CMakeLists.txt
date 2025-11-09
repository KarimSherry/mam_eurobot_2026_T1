cmake_minimum_required(VERSION 3.8)
project(mam_eurobot_2026)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Include Cpp "include" directory
include_directories(include)

# Create and install Cpp executable
add_executable(cpp_test src/cpp_test.cpp)
ament_target_dependencies(cpp_test rclcpp geometry_msgs)
install(TARGETS cpp_test DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install worlds files
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

# Install models files
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS ${PROJECT_NAME}/py_test.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ${PROJECT_NAME}/vision/aruco_detector.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ${PROJECT_NAME}/vision/color_detector.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ${PROJECT_NAME}/vision/world_to_topcamera.py DESTINATION lib/${PROJECT_NAME})

ament_package()
