########################################################################
# Find the library for the USRP Hardware Driver
########################################################################

INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_UHD uhd)

FIND_PATH(
    UHD_INCLUDE_DIRS
    NAMES uhd/config.hpp
    HINTS $ENV{UHD_DIR}/include
        ${PC_UHD_INCLUDEDIR}
    PATHS /usr/local/include
          /usr/include
          ${GNURADIO_INSTALL_PREFIX}/include
)

FIND_LIBRARY(
    UHD_LIBRARIES
    NAMES uhd
    HINTS $ENV{UHD_DIR}/lib
        ${PC_UHD_LIBDIR}
    PATHS /usr/local/lib
          /usr/lib/x86_64-linux-gnu
          /usr/lib/i386-linux-gnu
          /usr/lib/arm-linux-gnueabihf
          /usr/lib/arm-linux-gnueabi
          /usr/lib/aarch64-linux-gnu
          /usr/lib/mipsel-linux-gnu
          /usr/lib/mips-linux-gnu
          /usr/lib/mips64el-linux-gnuabi64
          /usr/lib/powerpc-linux-gnu
          /usr/lib/powerpc64-linux-gnu
          /usr/lib/powerpc64le-linux-gnu
          /usr/lib/powerpc-linux-gnuspe
          /usr/lib/hppa-linux-gnu
          /usr/lib/s390x-linux-gnu
          /usr/lib/i386-gnu
          /usr/lib/hppa-linux-gnu
          /usr/lib/x86_64-kfreebsd-gnu
          /usr/lib/i386-kfreebsd-gnu
          /usr/lib/m68k-linux-gnu
          /usr/lib/sh4-linux-gnu
          /usr/lib/sparc64-linux-gnu
          /usr/lib/x86_64-linux-gnux32
          /usr/lib/alpha-linux-gnu
          /usr/lib64
          /usr/lib
          ${GNURADIO_INSTALL_PREFIX}/lib
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(UHD DEFAULT_MSG UHD_LIBRARIES UHD_INCLUDE_DIRS)
MARK_AS_ADVANCED(UHD_LIBRARIES UHD_INCLUDE_DIRS)
