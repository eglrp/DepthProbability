cmake_minimum_required(VERSION 2.8)
add_compile_options(-std=c++11)
project(DepthProbability)


################################################################################
# Include CMake dependencies
################################################################################

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

################################################################################
# dependency
################################################################################
#- Eigen
include(cmake/eigen_dep.cmake)
#- Ceres
include(cmake/ceres_dep.cmake)
#- Glog
include(cmake/glog_dep.cmake)
#- Boost
include(cmake/boost_dep.cmake)
#- python
include(cmake/python_dep.cmake)
#- opencv
include(cmake/opencv_dep.cmake)

################################################################################
# source
################################################################################

set(SRC src/common.hpp
        src/common.cpp
        src/estimator.hpp
        src/estimator.cpp src/matchingtest.cpp)
include_directories(/src)

add_library(depthProb SHARED ${SRC})
target_link_libraries(depthProb
        ${PYTHON_LIBRARY}
        boost_python
        ${CERES_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${OpenCV_LIBRARIES}
        gomp)

add_executable(matching_test src/matchingtest.cpp)
target_link_libraries(matching_test
        ${OpenCV_LIBRARIES}
        ${GLOG_LIBRARIES}
        )



