cmake_minimum_required(VERSION 3.1.2)
project(mrs_llcp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  nodelet
  )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES Llcp
  CATKIN_DEPENDS roscpp
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(Llcp src/llcp.cpp src/serial_port.cpp)
target_link_libraries(Llcp
  ${catkin_LIBRARIES}
  )
