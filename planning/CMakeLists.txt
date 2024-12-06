cmake_minimum_required(VERSION 2.8.3)
project(map_frontiers)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14 -Wall -Wextra)

#############
# LIBRARIES #
#############

############
# BINARIES #
############

cs_add_executable(voxblox_map
  src/voxblox_map.cc
)
target_link_libraries(voxblox_map ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
