cmake_minimum_required(VERSION 2.8.3)
project(calibration)


find_package( OpenCV REQUIRED )

message("${OpenCV_INCLUDE_DIRS}")
message("${OpenCV_LIBRARIES}")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  nodelet
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  std_msgs
  visualization_msgs
)

add_message_files(
FILES
BoundingBox.msg
BoundingBoxArray.msg
)

generate_messages(
DEPENDENCIES
std_msgs
sensor_msgs
geometry_msgs
)


catkin_package(CATKIN_DEPENDS message_runtime sensor_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable( c src/c.cpp)
target_link_libraries(  c  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}         -lpthread)


