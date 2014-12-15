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



FIND_PATH(GROSMOSDR_INCLUDE_DIR 
     NAMES osmosdr/source.h 
           osmosdr/api.h
     HINTS ${GrOsmoSDR_ROOT_DIR}/include
     PATHS /usr/local/include
           /usr/include
)


find_library(GROSMOSDR_LIBRARIES
  NAMES gnuradio-osmosdr
  HINTS ${GrOsmoSDR_ROOT_DIR}/lib
  PATHS /usr/local/lib
        /usr/lib
  )


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  GrOsmoSDR
  DEFAULT_MSG
  GROSMOSDR_LIBRARIES
  GROSMOSDR_INCLUDE_DIR
)

mark_as_advanced(
  GrOsmoSDR_ROOT_DIR
  GROSMOSDR_LIBRARIES
  GROSMOSDR_INCLUDE_DIR
)
