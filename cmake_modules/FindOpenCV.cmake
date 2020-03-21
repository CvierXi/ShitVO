# - Find OpenCV

set(OpenCV_DIR "~/ws/3rd/opencv/opencv-3.4.5_contrib/build")

find_package(OpenCV QUIET COMPONENTS viz)
if (OpenCV_FOUND)
    add_definitions(-DHAVE_VIZ)
    message(STATUS "OpenCV-module viz is found. Build with viz.")
else()
    message(STATUS "OpenCV-module viz is not found. Build without viz.")
endif ()

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
