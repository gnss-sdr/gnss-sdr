# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2024 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT GNSSSDR_LIB_PATHS)
    include(GnsssdrLibPaths)
endif()

if(NOT PKG_CONFIG_FOUND)
    include(FindPkgConfig)
endif()
pkg_check_modules(PC_GMP "gmp")

set(GMP_DEFINITIONS ${PC_GMP_CFLAGS_OTHER})

find_path(GMP_INCLUDE_DIR
    NAMES gmpxx.h
    HINTS ${PC_GMP_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
          /opt/local/include
)

set(GMP_INCLUDE_DIRS ${GMP_INCLUDE_DIR})
set(GMP_PC_ADD_CFLAGS "-I${GMP_INCLUDE_DIR}")

find_library(GMPXX_LIBRARY
    NAMES gmpxx
    HINTS ${PC_GMP_LIBDIR}
    PATHS ${GNSSSDR_LIB_PATHS}
          ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
)

find_library(GMP_LIBRARY
    NAMES gmp
    HINTS ${PC_GMP_LIBDIR}
    PATHS ${GNSSSDR_LIB_PATHS}
          ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
)

set(GMP_LIBRARIES ${GMPXX_LIBRARY} ${GMP_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GMP DEFAULT_MSG GMPXX_LIBRARY GMP_LIBRARY GMP_INCLUDE_DIR)

if(GMP_FOUND AND NOT TARGET Gmp::gmp)
    add_library(Gmp::gmp SHARED IMPORTED)
    set_target_properties(Gmp::gmp PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${GMPXX_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${GMP_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES "${GMP_LIBRARIES}"
    )
endif()

set_package_properties(GMP PROPERTIES
    URL "https://gmplib.org/"
)

if(PC_GMP_VERSION)
    set_package_properties(GMP PROPERTIES
        DESCRIPTION "The GNU Multiple Precision Arithmetic Library (found: v.${PC_GMP_VERSION})"
    )
else()
    set_package_properties(GMP PROPERTIES
        DESCRIPTION "The GNU Multiple Precision Arithmetic Library"
    )
endif()

mark_as_advanced(GMPXX_LIBRARY GMP_LIBRARY GMP_INCLUDE_DIR)
