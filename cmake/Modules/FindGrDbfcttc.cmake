########################################################################
# Find  GR-DBFCTTC Module
########################################################################

INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_GR_DBFCTTC gr-dbfcttc)

FIND_PATH(
    GR_DBFCTTC_INCLUDE_DIRS
    NAMES dbfcttc/api.h
    HINTS $ENV{GR_DBFCTTC_DIR}/include
          ${PC_GR_DBFCTTC_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/include 
          /usr/local/include
)

FIND_LIBRARY(
    GR_DBFCTTC_LIBRARIES
    NAMES gnuradio-dbfcttc
    HINTS $ENV{GR_DBFCTTC_DIR}/lib
          ${PC_GR_DBFCTTC_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/lib
          /usr/lib64
          /usr/local/lib
          /usr/local/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_DBFCTTC DEFAULT_MSG GR_DBFCTTC_LIBRARIES GR_DBFCTTC_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_DBFCTTC_LIBRARIES GR_DBFCTTC_INCLUDE_DIRS)
