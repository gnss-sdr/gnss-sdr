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
          /opt/local/include
)

FIND_LIBRARY(
    LIBIIO_LIBRARIES
    NAMES libiio.so iio
    HINTS $ENV{LIBIIO_DIR}/lib
          ${PC_LIBIIO_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          /usr/lib/x86_64-linux-gnu
          /usr/lib/gcc/alpha-linux-gnu
          /usr/lib/gcc/aarch64-linux-gnu
          /usr/lib/gcc/arm-linux-gnueabi
          /usr/lib/gcc/arm-linux-gnueabihf
          /usr/lib/gcc/hppa-linux-gnu
          /usr/lib/gcc/i686-gnu
          /usr/lib/gcc/i686-linux-gnu
          /usr/lib/gcc/x86_64-kfreebsd-gnu
          /usr/lib/gcc/i686-kfreebsd-gnu
          /usr/lib/gcc/m68k-linux-gnu
          /usr/lib/gcc/mips-linux-gnu
          /usr/lib/gcc/mips64el-linux-gnuabi64
          /usr/lib/gcc/mipsel-linux-gnu
          /usr/lib/gcc/powerpc-linux-gnu
          /usr/lib/gcc/powerpc-linux-gnuspe
          /usr/lib/gcc/powerpc64-linux-gnu
          /usr/lib/gcc/powerpc64le-linux-gnu
          /usr/lib/gcc/s390x-linux-gnu
          /usr/lib/gcc/sparc64-linux-gnu
          /usr/lib/gcc/x86_64-linux-gnux32
          /usr/lib/gcc/sh4-linux-gnu
          /Library/Frameworks/iio.framework/
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(LIBIIO DEFAULT_MSG LIBIIO_LIBRARIES LIBIIO_INCLUDE_DIRS)
MARK_AS_ADVANCED(LIBIIO_LIBRARIES LIBIIO_INCLUDE_DIRS)
