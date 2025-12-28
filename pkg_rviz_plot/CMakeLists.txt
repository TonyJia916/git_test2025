cmake_minimum_required(VERSION 3.8)
project(pkg_rviz_plot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(planning_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(lomp_msgs REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(vehicle_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(rclcpp_components REQUIRED)
find_package(visualization_msgs  REQUIRED)
find_package(geometry_msgs  REQUIRED)


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



# file(GLOB RVIZ_PLOT_SOURCES 
#     ${CMAKE_CURRENT_SOURCE_DIR}/../../rviz_plot/src/*.cpp
# )

include_directories(
    include
  )

set(RVIZ_PLOT_SRC
  src/math/aabox2d.cpp
  src/math/box2d.cpp
  src/math/line_segment2d.cpp
  src/math/math_utils.cpp
  src/math/polygon2d.cpp
  src/math/vec2d.cpp
  src/visualization/plot.cpp
  src/visualization/color.cpp)  
 
add_executable(rviz_plot_node src/rviz_plot_node.cpp ${RVIZ_PLOT_SRC})

 
ament_target_dependencies(rviz_plot_node
  tf2
  rclcpp
  planning_msgs
  perception_msgs
  control_msgs
  lomp_msgs
  vehicle_msgs
  rclcpp_components
  visualization_msgs
  geometry_msgs
)

install(TARGETS
  rviz_plot_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  ${CMAKE_CURRENT_LIST_DIR}/config/configros2.rviz
  ${CMAKE_CURRENT_LIST_DIR}/config/pkg_rviz_plot.yaml
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()