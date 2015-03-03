INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_TELEORBIT teleorbit)

FIND_PATH(
    TELEORBIT_INCLUDE_DIRS
    NAMES teleorbit/api.h
    HINTS $ENV{TELEORBIT_DIR}/include
        ${PC_TELEORBIT_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    TELEORBIT_LIBRARIES
    NAMES gnuradio-teleorbit
    HINTS $ENV{TELEORBIT_DIR}/lib
        ${PC_TELEORBIT_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(TELEORBIT DEFAULT_MSG TELEORBIT_LIBRARIES TELEORBIT_INCLUDE_DIRS)
MARK_AS_ADVANCED(TELEORBIT_LIBRARIES TELEORBIT_INCLUDE_DIRS)
