cmake_minimum_required(VERSION 3.2)
project(trajectory_estimator)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR} SUB_SOURCES)
SET(a_star ${SUB_SOURCES})
ADD_LIBRARY(a_star  ${a_star})