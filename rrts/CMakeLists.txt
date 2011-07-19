cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99 -D_REENTRANT -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE")

rosbuild_add_library(rrts
  src/opttree.c
  src/optsystem.c
  src/kdtree.c
  src/kdtree2.c)

IF (NOT APPLE)
  target_link_libraries(rrts
    gmodule-2.0 gthread-2.0 gobject-2.0 glib-2.0)
  include_directories(/usr/include/glib-2.0 /usr/lib/glib-2.0/include /opt/local/include)
ELSE (NOT APPLE)
  target_link_libraries(rrts
    gmodule-2.0 gthread-2.0 gobject-2.0 glib-2.0 intl iconv)
  include_directories(/opt/local/include/glib-2.0 /opt/local/lib/glib-2.0/include /opt/local/include)
ENDIF (NOT APPLE)