cmake_minimum_required(VERSION 3.10)
project(ShitVO)

set(CMAKE_CXX_STANDARD 11)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

include(cmake_modules/FindEigen.cmake)
include(cmake_modules/FindOpenCV.cmake)

include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(core)
add_subdirectory(pc)

add_executable(demo_homo_vo demo/demo_homo_vo.cpp)
target_link_libraries(demo_homo_vo pc ${OpenCV_LIBS})