cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(mr_rrt_planner
  src/CentralizedRRTPlanner.cc)

IF (NOT APPLE)
  target_link_libraries(mr_rrt_planner
    gmodule-2.0 gthread-2.0 gobject-2.0 glib-2.0)
  include_directories(/usr/include/glib-2.0 /usr/lib/glib-2.0/include /opt/local/include)
ELSE (NOT APPLE)
  target_link_libraries(mr_rrt_planner
    gmodule-2.0 gthread-2.0 gobject-2.0 glib-2.0 intl iconv)
  include_directories(/opt/local/include/glib-2.0 /opt/local/lib/glib-2.0/include /opt/local/include)
ENDIF (NOT APPLE)

rosbuild_add_library(mr_rrt_simple_planner
  src/CentralizedSimplePlanner.cc)
target_link_libraries(mr_rrt_simple_planner
  mr_rrt_planner)

IF (NOT APPLE)
  target_link_libraries(mr_rrt_simple_planner
    gmodule-2.0 gthread-2.0 gobject-2.0 glib-2.0)
  include_directories(/usr/include/glib-2.0 /usr/lib/glib-2.0/include /opt/local/include)
ELSE (NOT APPLE)
  target_link_libraries(mr_rrt_simple_planner
    gmodule-2.0 gthread-2.0 gobject-2.0 glib-2.0 intl iconv)
  include_directories(/opt/local/include/glib-2.0 /opt/local/lib/glib-2.0/include /opt/local/include)
ENDIF (NOT APPLE)

