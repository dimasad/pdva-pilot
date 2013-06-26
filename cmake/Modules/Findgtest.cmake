# - Try to find gtest
# Once done, this will define
#
#  gtest_FOUND - system has gtest
#  gtest_INCLUDE_DIRS - the gtest include directories
#  gtest_LIBRARIES - link these to use gtest

include(LibFindMacros)

# Dependencies
libfind_package(gtest Threads)

# Include dir for gtest/gtest.h
find_path(gtest_INCLUDE_DIR
  NAMES gtest/gtest.h
)

# Find the library for the tests
find_library(gtest_LIBRARY NAMES gtest)

# Find the library for the test entry point
find_library(gtest_main_LIBRARY NAMES gtest_main)

# Call the LibFind macros
set(
  gtest_PROCESS_INCLUDES 
  gtest_INCLUDE_DIR)

set(
  gtest_PROCESS_LIBS 
  gtest_LIBRARY gtest_main_LIBRARY ${CMAKE_THREAD_LIBS_INIT})
libfind_process(gtest)
