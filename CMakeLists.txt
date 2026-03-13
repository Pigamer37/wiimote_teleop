cmake_minimum_required(VERSION 3.8)
project(wiimote_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(joy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wiimote_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package( OpenCV REQUIRED )
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(wiimote_handler src/wiimote_handler.cpp)
target_include_directories(wiimote_handler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(wiimote_handler PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(wiimote_handler ${OpenCV_LIBS})

ament_target_dependencies(wiimote_handler
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  joy
  wiimote_msgs
  sensor_msgs
  std_msgs
  kdl_parser
)

install(TARGETS wiimote_handler
  DESTINATION lib/${PROJECT_NAME})

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

# ament_export_targets(wiimote_handler HAS_LIBRARY_TARGET)
# ament_export_dependencies(
#   hardware_interface
#   pluginlib
#   rclcpp
#   rclcpp_lifecycle
#   joy
#   sensor_msgs
#   std_msgs
#   kdl_parser
# )

ament_package()
