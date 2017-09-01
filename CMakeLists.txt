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

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_DIR})

################################################################################
# source
################################################################################

set(SRC src/common.hpp
        src/common.cpp
        src/estimator.hpp
        src/estimator.cpp)
include_directories(/src)

add_library(depthProb SHARED ${SRC})
target_link_libraries(depthProb ${PYTHON_LIBRARY} boost_python ${CERES_LIBRARIES})