cmake_minimum_required(VERSION 3.16.3)
project(pnp)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(OpenCV REQUIRED QUIET)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(include ./common)

set(TARGET_LIBRARIES "")
list(APPEND TARGET_LIBRARIES ${GLOG_LIBRARIES})
list(APPEND TARGET_LIBRARIES ${OpenCV_LIBRARIES})

add_executable(epnp src/epnp.cpp test/epnp_node.cpp)
target_link_libraries(epnp ${TARGET_LIBRARIES} gflags)

add_executable(p3p src/p3p.cpp test/p3p_node.cpp)
target_link_libraries(p3p ${TARGET_LIBRARIES} gflags)
