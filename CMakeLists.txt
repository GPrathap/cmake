cmake_minimum_required (VERSION 3.0)
project (trajectory_estimator)

SET(CMAKE_C_FLAGS_DEBUG "-D_DEBUG")
add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC -fopenmp -DBOOST_LOG_DYN_LINK")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -ggdb")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message("${CMAKE_MODULE_PATH}")
message("${EIGEN3_INCLUDE_DIR}")
find_package(PCL)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost COMPONENTS program_options log log_setup regex atomic date_time system chrono filesystem thread graph REQUIRED)
set(Boost_USE_STATIC_LIBS OFF)
include_directories(${BOOST_INCLUDE_DIRS})


message( STATUS "Include sub-directories into build process:" )
foreach( DIR rrt_star a_star apf common spline utils ground_removal projections local_maxima quadtree trajectory_planning)
    message( STATUS "  - include/" ${DIR} )
    include_directories( "${DIR}" )
    link_directories("${DIR}")
    add_subdirectory( "${DIR}" )
endforeach()



include_directories(/usr/local/include/)
link_directories(/usr/local/lib/)

set(TRACKING_LIB ${TRACKING_LIB} ${Boost_LIBRARIES})
set(TRACKING_LIB ${TRACKING_ESTIMATION_LIB} ${PCL_LIBRARIES})
set(TRACKING_LIB ${TRACKING_LIB} rrt_star a_star apf utils common spline projections ground_removal cnpy z  local_maxima quadtree trajectory_planning)
set(TRACKING_LIB ${TRACKING_LIB} ${OpenCV_LIBS})

add_executable(${PROJECT_NAME}_node main.cpp)
target_link_libraries(${PROJECT_NAME}_node -Wl,--start-group  ${TRACKING_LIB} -Wl,--end-group)
