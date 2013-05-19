cmake_minimum_required (VERSION 2.6)
project (pdva-pilot)

set (pdva-pilot_VERSION_MAJOR 0)
set (pdva-pilot_VERSION_MINOR 1)

find_library (LIBCONFIG_LIBRARIES NAMES config libconfig)

set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99")
#include_directories (./mavlink)

add_library (param param.c)
add_library (comm comm.c)

add_executable (pdva-pilot pdva-pilot.c)
target_link_libraries (pdva-pilot param comm rt ${LIBCONFIG_LIBRARIES})

install (TARGETS pdva-pilot DESTINATION bin)
