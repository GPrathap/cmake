cmake_minimum_required(VERSION 3.2)
project(trajectory_estimator)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR} SUB_SOURCES)
SET( trajectory_planning ${SUB_SOURCES})

ADD_LIBRARY ( trajectory_planning  ${trajectory_planning})