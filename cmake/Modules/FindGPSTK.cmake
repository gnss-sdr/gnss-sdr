# - Find gpstk library
# Find the native gpstk includes and library
# This module defines
#  GPSTK_INCLUDE_DIR, where to find Rinex3ObsBase.hpp, etc.
#  GPSTK_LIBRARIES, libraries to link against to use GPSTK.
#  GPSTK_FOUND, If false, do not try to use GPSTK.
# also defined, but not for general use are
#  GPSTK_LIBRARY, where to find the GPSTK library.

FIND_PATH(GPSTK_INCLUDE_DIR Rinex3ObsBase.hpp)

SET(GPSTK_NAMES ${GPSTK_NAMES} gpstk libgpstk)
FIND_LIBRARY(GPSTK_LIBRARY NAMES ${GPSTK_NAMES} )

# handle the QUIETLY and REQUIRED arguments and set GPSTK_FOUND to TRUE if 
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GPSTK  DEFAULT_MSG  GPSTK_LIBRARY  GPSTK_INCLUDE_DIR)

IF(GPSTK_FOUND)
  SET( GPSTK_LIBRARIES ${GPSTK_LIBRARY} )
ENDIF(GPSTK_FOUND)

MARK_AS_ADVANCED(GPSTK_INCLUDE_DIR GPSTK_LIBRARY)
