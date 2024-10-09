cmake_minimum_required(VERSION 3.8)
project(tut_6)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
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
endif()

# Define executables
add_executable(map_processor_node src/map_processor_node.cpp)
add_executable(tutorial_6 src/tutorial_6.cpp)
add_executable(slo_3_3 src/slo_3_3.cpp)
add_executable(moveforwardservicenode src/moveforwardservicenode.cpp)


# Link dependencies to each target
ament_target_dependencies(map_processor_node std_srvs rclcpp tf2_geometry_msgs tf2 nav_msgs geometry_msgs std_msgs sensor_msgs cv_bridge image_transport OpenCV)
ament_target_dependencies(tutorial_6 rclcpp std_srvs tf2_geometry_msgs tf2 nav_msgs geometry_msgs std_msgs sensor_msgs cv_bridge image_transport OpenCV)
ament_target_dependencies(slo_3_3 std_srvs rclcpp tf2_geometry_msgs tf2 nav_msgs geometry_msgs std_msgs sensor_msgs cv_bridge image_transport OpenCV)
ament_target_dependencies(moveforwardservicenode std_srvs rclcpp tf2_geometry_msgs tf2 nav_msgs geometry_msgs std_msgs sensor_msgs cv_bridge image_transport OpenCV)


# Link libraries for OpenCV
target_link_libraries(map_processor_node ${OpenCV_LIBRARIES})
target_link_libraries(tutorial_6 ${OpenCV_LIBRARIES})
target_link_libraries(slo_3_3 ${OpenCV_LIBRARIES})
target_link_libraries(moveforwardservicenode ${OpenCV_LIBRARIES})



# Install executables
install(TARGETS
  map_processor_node
  tutorial_6
  slo_3_3
  moveforwardservicenode
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
