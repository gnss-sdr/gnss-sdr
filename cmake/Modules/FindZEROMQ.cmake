# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2023 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

#
# Provides the following imported target:
# ZeroMQ::ZeroMQ
#

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT GNSSSDR_LIB_PATHS)
    include(GnsssdrLibPaths)
endif()

if(NOT PKG_CONFIG_FOUND)
    include(FindPkgConfig)
endif()

find_package(PkgConfig)
pkg_check_modules(PC_ZEROMQ "libzmq")

find_path(ZEROMQ_INCLUDE_DIRS
    NAMES zmq.hpp
    HINTS ${PC_ZEROMQ_INCLUDE_DIR} ${CMAKE_INSTALL_PREFIX}/include
    PATHS /usr/local/include /usr/include /opt/local/include
)

find_library(ZEROMQ_LIBRARIES
    NAMES zmq libzmq.so.5 ${ZEROMQ_LIBRARY_NAME}
    HINTS ${PC_ZEROMQ_LIBDIR} ${CMAKE_INSTALL_PREFIX}/lib ${CMAKE_INSTALL_PREFIX}/lib64
    PATHS ${GNSSSDR_LIB_PATHS}
)

include(FindPackageHandleStandardArgs)
# This is just to detect presence, include files not required
find_package_handle_standard_args(ZEROMQ DEFAULT_MSG ZEROMQ_LIBRARIES)
mark_as_advanced(ZEROMQ_LIBRARIES ZEROMQ_INCLUDE_DIRS)

set_package_properties(ZEROMQ PROPERTIES URL "https://zeromq.org/")
if(PC_ZEROMQ_VERSION)
    set_package_properties(ZEROMQ PROPERTIES
        DESCRIPTION "An open-source universal messaging library (found: v${PC_ZEROMQ_VERSION})"
    )
else()
    set_package_properties(ZEROMQ PROPERTIES
        DESCRIPTION "An open-source universal messaging library"
    )
endif()

if(ZEROMQ_FOUND AND ZEROMQ_INCLUDE_DIRS AND NOT TARGET ZeroMQ::ZeroMQ)
    add_library(ZeroMQ::ZeroMQ INTERFACE IMPORTED)
    set_target_properties(ZeroMQ::ZeroMQ PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${ZEROMQ_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${ZEROMQ_LIBRARIES}"
    )
endif()