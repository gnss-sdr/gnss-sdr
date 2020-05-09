# Copyright (C) 2011-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later

########################################################################
# Find GNU Radio
########################################################################

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()
include(FindPackageHandleStandardArgs)

# if GR_REQUIRED_COMPONENTS is not defined, it will be set to the following list
if(NOT GR_REQUIRED_COMPONENTS)
  set(GR_REQUIRED_COMPONENTS RUNTIME PMT BLOCKS FFT FILTER ANALOG)
endif()

# Allows us to use all .cmake files in this directory
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR})

# Easily access all libraries and includes of GNU Radio
set(GNURADIO_ALL_LIBRARIES "")
set(GNURADIO_ALL_INCLUDE_DIRS "")

macro(LIST_CONTAINS var value)
    set(${var})
    foreach(value2 ${ARGN})
        if(${value} STREQUAL ${value2})
            set(${var} TRUE)
        endif()
    endforeach()
endmacro()

if(NOT GNURADIO_INSTALL_PREFIX)
    set(GNURADIO_INSTALL_PREFIX_USER_PROVIDED /usr)
else()
    set(GNURADIO_INSTALL_PREFIX_USER_PROVIDED ${GNURADIO_INSTALL_PREFIX})
endif()
if(GNURADIO_ROOT)
    set(GNURADIO_INSTALL_PREFIX_USER_PROVIDED
        ${GNURADIO_INSTALL_PREFIX_USER_PROVIDED}
        ${GNURADIO_ROOT}
    )
endif()
if(DEFINED ENV{GNURADIO_ROOT})
    set(GNURADIO_INSTALL_PREFIX_USER_PROVIDED
        ${GNURADIO_INSTALL_PREFIX_USER_PROVIDED}
        $ENV{GNURADIO_ROOT}
    )
endif()
if(DEFINED ENV{GNURADIO_RUNTIME_DIR})
    set(GNURADIO_INSTALL_PREFIX_USER_PROVIDED
        ${GNURADIO_INSTALL_PREFIX_USER_PROVIDED}
        $ENV{GNURADIO_RUNTIME_DIR}
    )
endif()
set(GNURADIO_INSTALL_PREFIX_USER_PROVIDED
    ${GNURADIO_INSTALL_PREFIX_USER_PROVIDED}
    ${CMAKE_INSTALL_PREFIX}
)

function(GR_MODULE EXTVAR PCNAME INCFILE LIBFILE)
    list_contains(REQUIRED_MODULE ${EXTVAR} ${GR_REQUIRED_COMPONENTS})
    if(NOT REQUIRED_MODULE)
        #message("Ignoring GNU Radio Module ${EXTVAR}")
        return()
    endif()

    message(STATUS "Checking for GNU Radio Module: ${EXTVAR}")

    # check for .pc hints
    pkg_check_modules(PC_GNURADIO_${EXTVAR} QUIET ${PCNAME})

    if(NOT PC_GNURADIO_${EXTVAR}_FOUND)
        set(PC_GNURADIO_${EXTVAR}_LIBRARIES ${LIBFILE})
    endif()

    set(INCVAR_NAME "GNURADIO_${EXTVAR}_INCLUDE_DIRS")
    set(LIBVAR_NAME "GNURADIO_${EXTVAR}_LIBRARIES")
    set(PC_INCDIR ${PC_GNURADIO_${EXTVAR}_INCLUDEDIR})
    set(PC_LIBDIR ${PC_GNURADIO_${EXTVAR}_LIBDIR})

    # look for include files
    find_path(${INCVAR_NAME}
        NAMES ${INCFILE}
        HINTS ${PC_INCDIR}
        PATHS ${GNURADIO_INSTALL_PREFIX_USER_PROVIDED}/include
              /usr/include
              /usr/local/include
              /opt/local/include
    )

    # look for libs
    foreach(libname ${PC_GNURADIO_${EXTVAR}_LIBRARIES})
        find_library(${LIBVAR_NAME}_${libname}
            NAMES ${libname} ${libname}-${PC_GNURADIO_RUNTIME_VERSION}
            HINTS ${PC_LIBDIR}
            PATHS ${GNURADIO_INSTALL_PREFIX_USER_PROVIDED}/lib
                  ${GNURADIO_INSTALL_PREFIX_USER_PROVIDED}/lib64
                  /usr/lib
                  /usr/lib64
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
                  /usr/lib/riscv64-linux-gnu
                  /usr/local/lib
                  /usr/local/lib64
                  /opt/local/lib
        )
        list(APPEND ${LIBVAR_NAME} ${${LIBVAR_NAME}_${libname}})
    endforeach()

    set(${LIBVAR_NAME} ${${LIBVAR_NAME}} PARENT_SCOPE)

    # show results
    message(STATUS " * INCLUDES=${GNURADIO_${EXTVAR}_INCLUDE_DIRS}")
    message(STATUS " * LIBS=${GNURADIO_${EXTVAR}_LIBRARIES}")

    # append to all includes and libs list
    set(GNURADIO_ALL_INCLUDE_DIRS ${GNURADIO_ALL_INCLUDE_DIRS} ${GNURADIO_${EXTVAR}_INCLUDE_DIRS} PARENT_SCOPE)
    set(GNURADIO_ALL_LIBRARIES    ${GNURADIO_ALL_LIBRARIES}    ${GNURADIO_${EXTVAR}_LIBRARIES}    PARENT_SCOPE)

    if(GNURADIO_${EXTVAR}_LIBRARIES AND GNURADIO_${EXTVAR}_INCLUDE_DIRS)
        set(GNURADIO_${EXTVAR}_FOUND TRUE)
    endif()
    message(STATUS "GNURADIO_${EXTVAR}_FOUND = ${GNURADIO_${EXTVAR}_FOUND}")
    set(GNURADIO_${EXTVAR}_FOUND ${GNURADIO_${EXTVAR}_FOUND} PARENT_SCOPE)

    # generate an error if the module is missing
    if(NOT GNURADIO_${EXTVAR}_FOUND)
        message(STATUS "Required GNU Radio Component: ${EXTVAR} missing!")
        set(GNURADIO_FOUND FALSE) # Trick for feature_summary
    endif()

    # Create imported target
    string(TOLOWER ${EXTVAR} gnuradio_component)
    if(NOT TARGET Gnuradio::${gnuradio_component})
        add_library(Gnuradio::${gnuradio_component} SHARED IMPORTED)
        set(GNURADIO_LIBRARY ${GNURADIO_${EXTVAR}_LIBRARIES})
        list(GET GNURADIO_LIBRARY 0 FIRST_DIR)
        get_filename_component(GNURADIO_DIR ${FIRST_DIR} ABSOLUTE)
        set_target_properties(Gnuradio::${gnuradio_component} PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
            IMPORTED_LOCATION "${GNURADIO_DIR}"
            INTERFACE_INCLUDE_DIRECTORIES "${GNURADIO_${EXTVAR}_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES "${GNURADIO_LIBRARY}"
        )
    endif()

    mark_as_advanced(GNURADIO_${EXTVAR}_LIBRARIES GNURADIO_${EXTVAR}_INCLUDE_DIRS)
endfunction()

gr_module(RUNTIME gnuradio-runtime gnuradio/top_block.h gnuradio-runtime)
gr_module(PMT gnuradio-runtime pmt/pmt.h gnuradio-pmt)
gr_module(BLOCKS gnuradio-blocks gnuradio/blocks/api.h gnuradio-blocks)
gr_module(FEC gnuradio-fec gnuradio/fec/api.h gnuradio-fec)
gr_module(FFT gnuradio-fft gnuradio/fft/api.h gnuradio-fft)
gr_module(FILTER gnuradio-filter gnuradio/filter/api.h gnuradio-filter)
gr_module(ANALOG gnuradio-analog gnuradio/analog/api.h gnuradio-analog)
gr_module(DIGITAL gnuradio-digital gnuradio/digital/api.h gnuradio-digital)
gr_module(AUDIO gnuradio-audio gnuradio/audio/api.h gnuradio-audio)
gr_module(CHANNELS gnuradio-channels gnuradio/channels/api.h gnuradio-channels)
gr_module(QTGUI gnuradio-qtgui gnuradio/qtgui/api.h gnuradio-qtgui)
gr_module(TRELLIS gnuradio-trellis gnuradio/trellis/api.h gnuradio-trellis)
gr_module(UHD gnuradio-uhd gnuradio/uhd/api.h gnuradio-uhd)
gr_module(VOCODER gnuradio-vocoder gnuradio/vocoder/api.h gnuradio-vocoder)
gr_module(WAVELET gnuradio-wavelet gnuradio/wavelet/api.h gnuradio-wavelet)


list(REMOVE_DUPLICATES GNURADIO_ALL_INCLUDE_DIRS)
list(REMOVE_DUPLICATES GNURADIO_ALL_LIBRARIES)

if(NOT PC_GNURADIO_RUNTIME_VERSION)
    set(OLD_PACKAGE_VERSION ${PACKAGE_VERSION})
    unset(PACKAGE_VERSION)
    list(GET GNURADIO_BLOCKS_LIBRARIES 0 FIRST_DIR)
    get_filename_component(GNURADIO_BLOCKS_DIR ${FIRST_DIR} DIRECTORY)
    if(EXISTS ${GNURADIO_BLOCKS_DIR}/cmake/gnuradio/GnuradioConfigVersion.cmake)
        set(PACKAGE_FIND_VERSION_MAJOR 3)
        set(PACKAGE_FIND_VERSION_MINOR 7)
        set(PACKAGE_FIND_VERSION_PATCH 4)
        include(${GNURADIO_BLOCKS_DIR}/cmake/gnuradio/GnuradioConfigVersion.cmake)
    endif()
    if(PACKAGE_VERSION)
        set(PC_GNURADIO_RUNTIME_VERSION ${PACKAGE_VERSION})
    endif()
    set(PACKAGE_VERSION ${OLD_PACKAGE_VERSION})
endif()

# Trick to find out that GNU Radio is >= 3.7.4 if pkgconfig is not present
if(NOT PC_GNURADIO_RUNTIME_VERSION)
    find_file(GNURADIO_VERSION_GREATER_THAN_373
        NAMES gnuradio/blocks/tsb_vector_sink_f.h
        PATHS ${GNURADIO_INSTALL_PREFIX_USER_PROVIDED}/include
              /usr/include
              /usr/local/include
              /opt/local/include
    )
    if(GNURADIO_VERSION_GREATER_THAN_373)
        set(PC_GNURADIO_RUNTIME_VERSION "3.7.4+")
    endif()

    find_file(GNURADIO_VERSION_GREATER_THAN_38
        NAMES gnuradio/filter/mmse_resampler_cc.h
        PATHS ${GNURADIO_INSTALL_PREFIX_USER_PROVIDED}/include
              /usr/include
              /usr/local/include
              /opt/local/include
    )
    if(GNURADIO_VERSION_GREATER_THAN_38)
        set(PC_GNURADIO_RUNTIME_VERSION "3.8.0+")
    endif()
endif()

set(GNURADIO_VERSION ${PC_GNURADIO_RUNTIME_VERSION})

if(NOT GNSSSDR_GNURADIO_MIN_VERSION)
    set(GNSSSDR_GNURADIO_MIN_VERSION "3.7.3")
endif()

if(GNURADIO_VERSION)
    if(GNURADIO_VERSION VERSION_LESS ${GNSSSDR_GNURADIO_MIN_VERSION})
        unset(GNURADIO_RUNTIME_FOUND)
        message(STATUS "The GNU Radio version installed in your system (v${GNURADIO_VERSION}) is too old.")
        if(${CMAKE_SYSTEM_NAME} MATCHES "Linux|kFreeBSD|GNU")
            message("Go to https://github.com/gnuradio/pybombs")
            message("and follow the instructions to install GNU Radio in your system.")
        endif()
        if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
            message("You can install it easily via Macports:")
            message("  sudo port install gnuradio ")
            message("Alternatively, you can use homebrew:")
            message("  brew install gnuradio")
        endif()
        message(FATAL_ERROR "GNU Radio v${GNSSSDR_GNURADIO_MIN_VERSION} or later is required to build gnss-sdr.")
    endif()
    set_package_properties(GNURADIO PROPERTIES
        DESCRIPTION "The free and open software radio ecosystem (found: v${GNURADIO_VERSION})"
    )
else()
    set_package_properties(GNURADIO PROPERTIES
        DESCRIPTION "The free and open software radio ecosystem"
    )
endif()

find_package_handle_standard_args(GNURADIO DEFAULT_MSG GNURADIO_RUNTIME_FOUND)

# Search for IIO component
if(GNURADIO_VERSION VERSION_GREATER 3.8.99)
    pkg_check_modules(PC_GNURADIO_IIO QUIET gnuradio-iio)
    # look for include files
    message(STATUS "Checking for GNU Radio Module: IIO")
    find_path(GNURADIO_IIO_INCLUDE_DIRS
        NAMES gnuradio/iio/api.h
        HINTS ${PC_GNURADIO_IIO_INCLUDEDIR}
        PATHS ${GNURADIO_INSTALL_PREFIX_USER_PROVIDED}/include
              /usr/include
              /usr/local/include
              /opt/local/include
    )

    # look for libs
    find_library(GNURADIO_IIO_LIBRARIES
        NAMES gnuradio-iio gnuradio-iio-${GNURADIO_VERSION}
        HINTS ${PC_GNURADIO_IIO_LIBDIR}
        PATHS ${GNURADIO_INSTALL_PREFIX_USER_PROVIDED}/lib
              ${GNURADIO_INSTALL_PREFIX_USER_PROVIDED}/lib64
              /usr/lib
              /usr/lib64
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
              /usr/lib/riscv64-linux-gnu
              /usr/local/lib
              /usr/local/lib64
              /opt/local/lib
    )

    if(GNURADIO_IIO_LIBRARIES)
        message(STATUS " * INCLUDES=${GNURADIO_IIO_INCLUDE_DIRS}")
        message(STATUS " * LIBS=${GNURADIO_IIO_LIBRARIES}")
    endif()
    if(GNURADIO_IIO_LIBRARIES AND GNURADIO_IIO_INCLUDE_DIRS)
        set(GNURADIO_IIO_FOUND TRUE)
    endif()
    if(GNURADIO_IIO_FOUND)
        message(STATUS "GNURADIO_IIO_FOUND = ${GNURADIO_IIO_FOUND}")
        # append to all includes and libs list
        set(GNURADIO_ALL_INCLUDE_DIRS ${GNURADIO_ALL_INCLUDE_DIRS} ${GNURADIO_IIO_INCLUDE_DIRS})
        set(GNURADIO_ALL_LIBRARIES ${GNURADIO_ALL_LIBRARIES} ${GNURADIO_IIO_LIBRARIES})

        # Create imported target
        if(NOT TARGET Gnuradio::iio)
            add_library(Gnuradio::iio SHARED IMPORTED)
            set(GNURADIO_LIBRARY ${GNURADIO_IIO_LIBRARIES})
            list(GET GNURADIO_LIBRARY 0 FIRST_DIR)
            get_filename_component(GNURADIO_DIR ${FIRST_DIR} ABSOLUTE)
            set_target_properties(Gnuradio::iio PROPERTIES
                IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
                IMPORTED_LOCATION "${GNURADIO_DIR}"
                INTERFACE_INCLUDE_DIRECTORIES "${GNURADIO_IIO_INCLUDE_DIRS}"
                INTERFACE_LINK_LIBRARIES "${GNURADIO_LIBRARY}"
            )
        endif()
    endif()
endif()


set_package_properties(GNURADIO PROPERTIES
    URL "https://www.gnuradio.org/"
)
