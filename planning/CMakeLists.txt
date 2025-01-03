cmake_minimum_required(VERSION 2.8.3)
project(map_frontiers)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14 -Wall -Wextra)

#############
# LIBRARIES #
#############

cs_add_library(${PROJECT_NAME}
  src/NBV_Selector.cpp
)

############
# BINARIES #
############

cs_add_executable(NBV_Selector
  src/NBV_Selector.cpp
)
target_link_libraries(NBV_Selector ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
#cs_export()
