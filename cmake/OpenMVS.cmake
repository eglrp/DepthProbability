find_package(OpenMVS REQUIRED)

include_directories(${OpenMVS_INCLUDE_DIRS})
message(STATUS "Found OpenMVS : ${OpenMVS_INCLUDE_DIRS}")