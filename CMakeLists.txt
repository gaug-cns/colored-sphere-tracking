cmake_minimum_required(VERSION 2.8)
project(tracker)


set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++11 -O")


list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/)


find_package(Eigen REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)


include_directories(include ${CERES_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

file(GLOB SRCFILES src/*.cpp)
add_executable(tracker ${SRCFILES})

target_link_libraries(tracker ${CERES_LIBRARIES} ${OpenCV_LIBS})
