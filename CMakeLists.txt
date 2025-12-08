cmake_minimum_required(VERSION 3.8)
project(obstacle_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fpermissive)
endif()

# -----------------------------------------------------------------------
# dependency package
# -----------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


find_package(Armadillo REQUIRED) 


# -----------------------------------------------------------------------
# create message
# -----------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CircleObstacle.msg"
  "msg/SegmentObstacle.msg"
  "msg/Obstacles.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# -----------------------------------------------------------------------
# build lib
# -----------------------------------------------------------------------
set(DEPS
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)


add_library(obstacle_extractor SHARED src/obstacle_extractor.cpp)
ament_target_dependencies(obstacle_extractor ${DEPS})
rosidl_target_interfaces(obstacle_extractor
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(obstacle_extractor "${cpp_typesupport_target}" ${ARMADILLO_LIBRARIES})

# -----------------------------------------------------------------------
# node excutor
# -----------------------------------------------------------------------

add_executable(obstacle_extractor_node src/nodes/obstacle_extractor_node.cpp)

target_link_libraries(obstacle_extractor_node obstacle_extractor)
ament_target_dependencies(obstacle_extractor_node ${DEPS})

target_include_directories(obstacle_extractor PUBLIC include) 
ament_target_dependencies(obstacle_extractor ${DEPS})

# --- obstacle_visualizer ---
add_library(obstacle_visualizer src/obstacle_visualizer.cpp)
target_include_directories(obstacle_visualizer PUBLIC include)
ament_target_dependencies(obstacle_visualizer ${DEPS})
rosidl_target_interfaces(obstacle_visualizer
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(obstacle_visualizer_node src/nodes/obstacle_visualizer_node.cpp)
target_link_libraries(obstacle_visualizer_node obstacle_visualizer)
ament_target_dependencies(obstacle_visualizer_node ${DEPS})


# -----------------------------------------------------------------------
# Install
# -----------------------------------------------------------------------

install(TARGETS 
  obstacle_extractor
  obstacle_extractor_node
  obstacle_visualizer
  obstacle_visualizer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------------------------------------------------
# Ament package
ament_export_include_directories(include)
ament_export_libraries(obstacle_extractor obstacle_visualizer)
ament_export_dependencies(${DEPS})
ament_package()
