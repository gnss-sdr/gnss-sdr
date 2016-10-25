# Tries to find gr-osmosdr.
#
# Usage of this module as follows:
#
# find_package(GrOsmoSDR)
#
# Variables used by this module, they can change the default behaviour and need
# to be set before calling find_package:
#
# GrOsmoSDR_ROOT_DIR Set this variable to the root installation of
# gr-osmosdr if the module has problems finding
# the proper installation path.
#
# Variables defined by this module:
#
# GROSMOSDR_FOUND System has gr-osmosdr libs/headers
# GROSMOSDR_LIBRARIES The gr-osmosdr libraries (gnuradio-osmosdr)
# GROSMOSDR_INCLUDE_DIR The location of gr-osmosdr headers

if(NOT GROSMOSDR_FOUND)
  pkg_check_modules (GROSMOSDR_PKG gnuradio-osmosdr)
  find_path(GROSMOSDR_INCLUDE_DIR 
    NAMES osmosdr/source.h
	  osmosdr/api.h
    PATHS
    ${GROSMOSDR_PKG_INCLUDE_DIRS}
    /usr/include
    /usr/local/include
  )

 find_library(GROSMOSDR_LIBRARIES 
    NAMES gnuradio-osmosdr
    PATHS
    ${GROSMOSDR_PKG_LIBRARY_DIRS}
    /usr/lib
    /usr/local/lib
  )

  if(GROSMOSDR_INCLUDE_DIR AND GROSMOSDR_LIBRARIES)
    set(GROSMOSDR_FOUND TRUE CACHE INTERNAL "gnuradio-osmosdr found")
    message(STATUS "Found gnuradio-osmosdr: ${GROSMOSDR_INCLUDE_DIR}, ${GROSMOSDR_LIBRARIES}")
  else(GROSMOSDR_INCLUDE_DIR AND GROSMOSDR_LIBRARIES)
    set(GROSMOSDR_FOUND FALSE CACHE INTERNAL "gnuradio-osmosdr found")
    message(STATUS "gnuradio-osmosdr not found.")
  endif(GROSMOSDR_INCLUDE_DIR AND GROSMOSDR_LIBRARIES)

mark_as_advanced(GROSMOSDR_INCLUDE_DIR GROSMOSDR_LIBRARIES)

endif(NOT GROSMOSDR_FOUND)
