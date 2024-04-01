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

add_executable(epnp epnp/epnp.cpp)
target_link_libraries(epnp ${TARGET_LIBRARIES} gflags)

# add_executable(laplace laplace/laplace.cpp)
# target_link_libraries(laplace ${TARGET_LIBRARIES} gflags)

# add_executable(prewitt prewitt/prewitt.cpp)
# target_link_libraries(prewitt ${TARGET_LIBRARIES} gflags)

# add_executable(roberts roberts/roberts.cpp)
# target_link_libraries(roberts ${TARGET_LIBRARIES} gflags)

# add_executable(sobel sobel/sobel.cpp)
# target_link_libraries(sobel ${TARGET_LIBRARIES} gflags)

# ---------------------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------------------
# install(TARGETS canny prewitt
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY
# launch
# config
# DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )