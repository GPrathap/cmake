cmake_minimum_required(VERSION 3.2)
project(trajectory_estimator)


AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR} SUB_SOURCES)
SET( rrt_star ${SUB_SOURCES})
ADD_LIBRARY ( rrt_star  ${rrt_star}  )
target_link_libraries(rrt_star ${TRACKING_ESTIMATION_LIB})
