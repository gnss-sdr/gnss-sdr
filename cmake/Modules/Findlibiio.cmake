INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_LIBIIO libiio)

FIND_PATH(
    LIBIIO_INCLUDE_DIRS
    NAMES gnuradio/iio/api.h
    HINTS $ENV{LIBIIO_DIR}/include
        ${PC_LIBIIO_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    LIBIIO_LIBRARIES
    NAMES libiio.so
    HINTS $ENV{LIBIIO_DIR}/lib
        ${PC_LIBIIO_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          /usr/lib/x86_64-linux-gnu
)

message("find libiio:")
message(${LIBIIO_LIBRARIES})
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(LIBIIO DEFAULT_MSG LIBIIO_LIBRARIES LIBIIO_INCLUDE_DIRS)
MARK_AS_ADVANCED(LIBIIO_LIBRARIES LIBIIO_INCLUDE_DIRS)
