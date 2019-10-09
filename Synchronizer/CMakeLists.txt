cmake_minimum_required(VERSION 2.8.3)
project(Synchronizer)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
std_msgs
sensor_msgs
jsk_recognition_msgs
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs 
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/msg_Queue.cpp )
add_executable(${PROJECT_NAME}_asdnode src/pub.cpp )

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)


target_link_libraries(${PROJECT_NAME}_asdnode
  ${catkin_LIBRARIES}
)