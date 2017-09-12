find_package(OpenCV REQUIRED)
include_directories({OpenCV_INCLUDE_DIRS})
MESSAGE(STATUS "OpenCV LIB : ${OpenCV_LIBRARIES}\n-- OpenCV include: ${OpenCV_INCLUDE_DIRS}")