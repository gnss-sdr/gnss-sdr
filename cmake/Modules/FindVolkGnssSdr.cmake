########################################################################
# Find VOLK (Vector-Optimized Library of Kernels) GNSS-SDR library
########################################################################

INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_VOLK_GNSSSDR volk_gnsssdr)

FIND_PATH(
    VOLK_GNSSSDR_INCLUDE_DIRS
    NAMES volk_gnsssdr/volk_gnsssdr.h
    HINTS $ENV{VOLK_GNSSSDR_DIR}/include
        ${PC_VOLK_GNSSSDR_INCLUDEDIR}
    PATHS /usr/local/include
          /usr/include
          ${GNURADIO_INSTALL_PREFIX}/include
)

FIND_LIBRARY(
    VOLK_GNSSSDR_LIBRARIES
    NAMES volk_gnsssdr
    HINTS $ENV{VOLK_GNSSSDR_DIR}/lib
        ${PC_VOLK_GNSSSDR_LIBDIR}
    PATHS /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          ${GNURADIO_INSTALL_PREFIX}/lib
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(VOLK_GNSSSDR DEFAULT_MSG VOLK_GNSSSDR_LIBRARIES VOLK_GNSSSDR_INCLUDE_DIRS)
MARK_AS_ADVANCED(VOLK_GNSSSDR_LIBRARIES VOLK_GNSSSDR_INCLUDE_DIRS)