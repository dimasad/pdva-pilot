# - Try to find glib2
# Once done, this will define
#
#  glib2_FOUND - system has glib2
#  glib2_INCLUDE_DIRS - the glib2 include directories
#  glib2_LIBRARIES - link these to use glib2

include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(glib2_PKGCONF glib-2.0)

# Include dir for glib.h
find_path(glib_INCLUDE_DIR
  NAMES glib.h
  PATHS ${glib2_PKGCONF_INCLUDE_DIRS}
  PATH_SUFFIXES glib-2.0
)

# Include dir for glibconfig.h
find_path(glibconfig_INCLUDE_DIR
  NAMES glibconfig.h
  PATHS ${glib2_PKGCONF_INCLUDE_DIRS}
  PATH_SUFFIXES glib-2.0
)

# Finally the library itself
find_library(glib2_LIBRARY
  NAMES glib-2.0
  PATHS ${glib_PKGCONF_LIBRARY_DIRS}
)

set(glib2_PROCESS_INCLUDES glib_INCLUDE_DIR glibconfig_INCLUDE_DIR)
set(glib2_PROCESS_LIBS glib2_LIBRARY)
libfind_process(glib2)
