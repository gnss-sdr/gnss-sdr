INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_IIO gnuradio-iio)

FIND_PATH(
    IIO_INCLUDE_DIRS
    NAMES iio/api.h
    HINTS $ENV{IIO_DIR}/include
        ${PC_IIO_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
          ${CMAKE_INSTALL_PREFIX}/include/gnuradio
          /usr/local/include/gnuradio
          /usr/include/gnuradio
)

FIND_LIBRARY(
    IIO_LIBRARIES
    NAMES gnuradio-iio
    HINTS $ENV{IIO_DIR}/lib
        ${PC_IIO_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(IIO DEFAULT_MSG IIO_LIBRARIES IIO_INCLUDE_DIRS)
MARK_AS_ADVANCED(IIO_LIBRARIES IIO_INCLUDE_DIRS)
