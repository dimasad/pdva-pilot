cmake_minimum_required (VERSION 2.6)
project (pdva-pilot)

set (pdva-pilot_VERSION_MAJOR 0)
set (pdva-pilot_VERSION_MINOR 1)

# Set cmake module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

# Find glib2
find_package(glib2 REQUIRED)
include_directories(${glib2_INCLUDE_DIRS})

find_library (LIBCONFIG_LIBRARIES NAMES config libconfig)

set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99")


#add_library (comm comm.c)
#add_library (control control.c)
add_library (param param.c)
add_library (pid pid.c)

add_executable (pdva-pilot pdva-pilot.c)
target_link_libraries (
  pdva-pilot param pid rt #comm control
  ${LIBCONFIG_LIBRARIES} ${glib2_LIBRARIES})

install (TARGETS pdva-pilot DESTINATION bin)
