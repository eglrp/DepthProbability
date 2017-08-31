find_package(Ceres REQUIRED)

MESSAGE(STATUS "Include Ceres include dirs : ${CERES_INCLUDE_DIRS}")
include_directories(${CERES_INCLUDE_DIR})

MESSAGE(STATUS "Ceres LIB : ${CERES_LIBRARIES}")