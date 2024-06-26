cmake_minimum_required(VERSION 3.0.2)
project(hba)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++17)
set(CMAKE_CXX_FLAGS "-std=c++17 -O3" )
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++0x -std=c++17 -fexceptions")
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../bin)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosbag
  std_msgs
  geometry_msgs
  visualization_msgs
  tf
)

find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED QUIET)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs visualization_msgs tf
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

# Hierarchical Bundle Adjustment
add_executable(hba source/hba.cpp)
target_link_libraries(hba ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)

add_executable(hbam source/hbam.cpp)
target_link_libraries(hbam ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam yaml-cpp)

add_executable(visualize source/visualize.cpp)
target_link_libraries(visualize ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(visualizem source/visualizem.cpp)
target_link_libraries(visualizem ${catkin_LIBRARIES} ${PCL_LIBRARIES} yaml-cpp)

add_executable(record_tum_tra source/record_tum_tra.cpp)
target_link_libraries(record_tum_tra ${catkin_LIBRARIES})

# add_executable(hba_online source/hba_online.cpp)
# target_link_libraries(hba_online ${catkin_LIBRARIES} ${PCL_LIBRARIES} yaml-cpp)