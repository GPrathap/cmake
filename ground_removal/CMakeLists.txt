cmake_minimum_required(VERSION 3.2)
project(trajectory_estimator)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR} SUB_SOURCES)
SET( ground_removal ${SUB_SOURCES})


ADD_LIBRARY ( ground_removal  ${ground_removal} )
target_link_libraries(ground_removal ${OpenCV_LIBS})