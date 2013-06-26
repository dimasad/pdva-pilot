cmake_minimum_required (VERSION 2.6)

# Configure the project
project (pdva-pilot)
set (pdva-pilot_VERSION_MAJOR 0)
set (pdva-pilot_VERSION_MINOR 1)

# Set cmake module path
set (
  CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} 
  "${CMAKE_SOURCE_DIR}/cmake/Modules/")

# Find glib2
find_package (glib2 REQUIRED)
include_directories (${glib2_INCLUDE_DIRS})

# Find libconfig
find_package (libconfig REQUIRED)
include_directories (${libconfig_INCLUDE_DIRS})

# Set compiler options
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99")

# Add the project libraries
#add_library (comm comm.c)
#add_library (control control.c)
add_library (param param.c)
add_library (pid pid.c)

# Add the project executables
add_executable (pdva-pilot pdva-pilot.c)
target_link_libraries (
  pdva-pilot param pid rt #comm control
  ${libconfig_LIBRARIES} ${glib2_LIBRARIES})

# Define the installation
install (TARGETS pdva-pilot RUNTIME DESTINATION bin)

# Add the tests if the testing framework is available
find_package (gtest)
if (gtest_FOUND)
  include_directories (${gtest_INCLUDE_DIRS})

  # Add each individual test
  add_executable (test_param tests/test_param.cpp)
  target_link_libraries (
    test_param param pthread
    ${libconfig_LIBRARIES} ${glib2_LIBRARIES} ${gtest_LIBRARIES})
  
endif (gtest_FOUND)