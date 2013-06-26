# - Try to find libconfig
# Once done, this will define
#
#  libconfig_FOUND - system has libconfig
#  libconfig_INCLUDE_DIRS - the libconfig include directories
#  libconfig_LIBRARIES - link these to use libconfig

include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(libconfig_PKGCONF libconfig)

# Include dir for libconfig.h
find_path(libconfig_INCLUDE_DIR
  NAMES libconfig.h
  PATHS ${libconfig_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(libconfig_LIBRARY
  NAMES config libconfig
  PATHS ${libconfig_PKGCONF_LIBRARY_DIRS}
)

set(libconfig_PROCESS_INCLUDES libconfig_INCLUDE_DIR)
set(libconfig_PROCESS_LIBS libconfig_LIBRARY)
libfind_process(libconfig)
