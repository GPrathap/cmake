cmake_minimum_required(VERSION 3.2)
project(trajectory_estimator)


AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR} SUB_SOURCES)
SET( common ${SUB_SOURCES})
ADD_LIBRARY ( common  ${common})
target_link_libraries(common ${catkin_LIBRARIES})
