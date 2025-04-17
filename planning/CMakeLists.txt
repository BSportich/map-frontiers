cmake_minimum_required(VERSION 2.8.3)
project(map_frontiers)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin_simple REQUIRED COMPONENTS rospy)
catkin_simple(ALL_DEPS_REQUIRED)

catkin_python_setup()

catkin_package()

add_definitions(-std=c++14 -Wall -Wextra)


#############
# LIBRARIES #
#############

cs_add_library(${PROJECT_NAME}
  src/ViewGenerator.cpp
)

############
# BINARIES #
############

# Add the NBV_Selector executable, but do not compile it into the library.
cs_add_executable(NBV_Selector
  src/NBV_Selector.cpp
)
# Link the NBV_Selector executable against the library.
target_link_libraries(NBV_Selector ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)
