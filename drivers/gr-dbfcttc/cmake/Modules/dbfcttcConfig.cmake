INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_DBFCTTC dbfcttc)

FIND_PATH(
    DBFCTTC_INCLUDE_DIRS
    NAMES dbfcttc/api.h
    HINTS $ENV{DBFCTTC_DIR}/include
        ${PC_DBFCTTC_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREEFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    DBFCTTC_LIBRARIES
    NAMES gnuradio-dbfcttc
    HINTS $ENV{DBFCTTC_DIR}/lib
        ${PC_DBFCTTC_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(DBFCTTC DEFAULT_MSG DBFCTTC_LIBRARIES DBFCTTC_INCLUDE_DIRS)
MARK_AS_ADVANCED(DBFCTTC_LIBRARIES DBFCTTC_INCLUDE_DIRS)

