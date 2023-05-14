cmake_minimum_required(VERSION 3.8)
project(epti4abot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(
  epti4abot
  SHARED
  src/dfrobot.cpp
)
target_link_libraries(
  epti4abot
)
target_include_directories(
  epti4abot
  PUBLIC
  include
)

# add_executable(
#   your_executer path_to_cpp.cpp
# )
# target_include_directories(
#   your_executer
#   PUBLIC 
#   include
#   other_folder
# )
# ament_target_dependencies(
#   your_executer
#   all_the_package_you_needed
# )

add_executable(
  wit_imu_sensor src/witimunode.cpp
)
target_include_directories(
  wit_imu_sensor 
  PUBLIC 
  include
)
ament_target_dependencies(
  wit_imu_sensor
  rclcpp
  std_msgs
  tf2_ros
  sensor_msgs
  nav_msgs
  tf2_msgs
  tf2_geometry_msgs
  rclcpp_lifecycle
)

ament_target_dependencies(
  epti4abot
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)

# prevent pluginlib from using boost
target_compile_definitions(epti4abot PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface epti4abot.xml)

install(
  TARGETS
  epti4abot
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)


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
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_dfrobot test/test_dfrobot.cpp)
  target_include_directories(test_dfrobot PRIVATE include)
  ament_target_dependencies(
    test_dfrobot
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  epti4abot
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  std_msgs
  rclcpp_lifecycle
)

install(TARGETS
  wit_imu_sensor
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config description launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
