# Tries to find libosmosdr.
#
# Usage of this module as follows:
#
# find_package(LibOsmoSDR)
#
#
# Variables defined by this module:
#
# LIBOSMOSDR_FOUND System has libosmosdr libs/headers
# LIBOSMOSDR_LIBRARIES The libosmosdr libraries 
# LIBOSMOSDR_INCLUDE_DIR The location of libosmosdr headers


if(NOT LIBOSMOSDR_FOUND)
  pkg_check_modules (LIBOSMOSDR_PKG libosmosdr)
  find_path(LIBOSMOSDR_INCLUDE_DIR NAMES osmosdr.h
    PATHS
    ${LIBOSMOSDR_PKG_INCLUDE_DIRS}
    /usr/include
    /usr/local/include
  )

 find_library(LIBOSMOSDR_LIBRARIES NAMES osmosdr
    PATHS
    ${LIBOSMOSDR_PKG_LIBRARY_DIRS}
    /usr/lib
    /usr/local/lib
  )

  if(LIBOSMOSDR_INCLUDE_DIR AND LIBOSMOSDR_LIBRARIES)
    set(LIBOSMOSDR_FOUND TRUE CACHE INTERNAL "libosmosdr found")
    message(STATUS "Found libosmosdr: ${LIBOSMOSDR_INCLUDE_DIR}, ${LIBOSMOSDR_LIBRARIES}")
  else(LIBOSMOSDR_INCLUDE_DIR AND LIBOSMOSDR_LIBRARIES)
    set(LIBOSMOSDR_FOUND FALSE CACHE INTERNAL "libosmosdr found")
    message(STATUS "libosmosdr not found.")
  endif(LIBOSMOSDR_INCLUDE_DIR AND LIBOSMOSDR_LIBRARIES)

mark_as_advanced(LIBOSMOSDR_INCLUDE_DIR LIBOSMOSDR_LIBRARIES)

endif(NOT LIBOSMOSDR_FOUND)
