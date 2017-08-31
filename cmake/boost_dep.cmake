find_package(Boost COMPONENTS system thread python REQUIRED)

MESSAGE(STATUS "Include Boost include dirs : ${Boost_INCLUDE_DIRS}")
include_directories(${BOOST_INCLUDE_DIRS})