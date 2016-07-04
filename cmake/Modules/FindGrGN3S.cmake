########################################################################
# Find  GR-GN3S Module
########################################################################

INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_GR_GN3S gr-gn3s)

FIND_PATH(
    GR_GN3S_INCLUDE_DIRS
    NAMES gn3s/gn3s_api.h
    HINTS $ENV{GR_GN3S_DIR}/include
          ${PC_GR_GN3S_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GR_GN3S_LIBRARIES
    NAMES gr-gn3s
    HINTS $ENV{GR_GN3S_DIR}/lib
          ${PC_GR_GN3S_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_GN3S DEFAULT_MSG GR_GN3S_LIBRARIES GR_GN3S_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_GN3S_LIBRARIES GR_GN3S_INCLUDE_DIRS)
