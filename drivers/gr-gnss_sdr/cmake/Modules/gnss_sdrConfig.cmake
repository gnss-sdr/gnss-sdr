INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_GNSS_SDR gnss_sdr)

set(FOUND_GNSS_SDR_HE_DRIVER 1)

FIND_PATH(
    GNSS_SDR_INCLUDE_DIRS
    NAMES gnss_sdr/api.h
    HINTS $ENV{GNSS_SDR_DIR}/include
        ${PC_GNSS_SDR_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GNSS_SDR_LIBRARIES
    NAMES gnuradio-gnss_sdr
    HINTS $ENV{GNSS_SDR_DIR}/lib
        ${PC_GNSS_SDR_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GNSS_SDR DEFAULT_MSG GNSS_SDR_LIBRARIES GNSS_SDR_INCLUDE_DIRS)
MARK_AS_ADVANCED(GNSS_SDR_LIBRARIES GNSS_SDR_INCLUDE_DIRS)

