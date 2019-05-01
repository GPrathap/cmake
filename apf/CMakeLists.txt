cmake_minimum_required(VERSION 3.2)
project(trajectory_estimator)

AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR} SUB_SOURCES)
SET( apf ${SUB_SOURCES})

ADD_LIBRARY ( apf  SHARED ${apf} )

target_link_libraries(apf ${Boost_LIBRARIES})