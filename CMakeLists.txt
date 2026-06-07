cmake_minimum_required(VERSION 3.5)
project(finenav_terrain_map)

find_package(Eigen3 REQUIRED)

fins_add_node(${PROJECT_NAME}
  terrain_map.cpp
)

fins_link_ros_dependencies(${PROJECT_NAME}
  geometry_msgs
  sensor_msgs
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  Eigen3::Eigen
)
