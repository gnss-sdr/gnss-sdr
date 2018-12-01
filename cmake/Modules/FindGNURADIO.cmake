# Copyright (C) 2011-2018 (see AUTHORS file for a list of contributors)
#
# This file is part of GNSS-SDR.
#
# GNSS-SDR is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# GNSS-SDR is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.

########################################################################
# Find GNU Radio
########################################################################

include(FindPkgConfig)
include(FindPackageHandleStandardArgs)

# if GR_REQUIRED_COMPONENTS is not defined, it will be set to the following list
if(NOT GR_REQUIRED_COMPONENTS)
  set(GR_REQUIRED_COMPONENTS RUNTIME ANALOG BLOCKS DIGITAL FFT FILTER PMT FEC TRELLIS UHD)
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

function(GR_MODULE EXTVAR PCNAME INCFILE LIBFILE)
    list_contains(REQUIRED_MODULE ${EXTVAR} ${GR_REQUIRED_COMPONENTS})
    if(NOT REQUIRED_MODULE)
        #message("Ignoring GNU Radio Module ${EXTVAR}")
        return()
    endif()

    message(STATUS "Checking for GNU Radio Module: ${EXTVAR}")

    # check for .pc hints
    pkg_check_modules(PC_GNURADIO_${EXTVAR} ${PCNAME})

    if(NOT PC_GNURADIO_${EXTVAR}_FOUND)
        set(PC_GNURADIO_${EXTVAR}_LIBRARIES ${LIBFILE})
    endif()

    set(INCVAR_NAME "GNURADIO_${EXTVAR}_INCLUDE_DIRS")
    set(LIBVAR_NAME "GNURADIO_${EXTVAR}_LIBRARIES")
    set(PC_INCDIR ${PC_GNURADIO_${EXTVAR}_INCLUDEDIR})
    set(PC_LIBDIR ${PC_GNURADIO_${EXTVAR}_LIBDIR})

    # look for include files
    find_path(
        ${INCVAR_NAME}
        NAMES ${INCFILE}
        HINTS $ENV{GNURADIO_RUNTIME_DIR}/include
            ${PC_INCDIR}
            ${CMAKE_INSTALL_PREFIX}/include
            ${GNURADIO_INSTALL_PREFIX}/include
        PATHS /usr/local/include
              /usr/include
              ${GNURADIO_INSTALL_PREFIX}/include
    )

    # look for libs
    foreach(libname ${PC_GNURADIO_${EXTVAR}_LIBRARIES})
        find_library(
            ${LIBVAR_NAME}_${libname}
            NAMES ${libname} ${libname}-${PC_GNURADIO_RUNTIME_VERSION}
            HINTS $ENV{GNURADIO_RUNTIME_DIR}/lib
                ${PC_LIBDIR}
                ${CMAKE_INSTALL_PREFIX}/lib/
                ${CMAKE_INSTALL_PREFIX}/lib64/
                ${GNURADIO_INSTALL_PREFIX}/lib/
                ${GNURADIO_INSTALL_PREFIX}/lib64
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
        list(APPEND ${LIBVAR_NAME} ${${LIBVAR_NAME}_${libname}})
    endforeach()

    set(${LIBVAR_NAME} ${${LIBVAR_NAME}} PARENT_SCOPE)

    # show results
    message(STATUS " * INCLUDES=${GNURADIO_${EXTVAR}_INCLUDE_DIRS}")
    message(STATUS " * LIBS=${GNURADIO_${EXTVAR}_LIBRARIES}")

    # append to all includes and libs list
    set(GNURADIO_ALL_INCLUDE_DIRS ${GNURADIO_ALL_INCLUDE_DIRS} ${GNURADIO_${EXTVAR}_INCLUDE_DIRS} PARENT_SCOPE)
    set(GNURADIO_ALL_LIBRARIES    ${GNURADIO_ALL_LIBRARIES}    ${GNURADIO_${EXTVAR}_LIBRARIES}    PARENT_SCOPE)

    find_package_handle_standard_args(GNURADIO_${EXTVAR} DEFAULT_MSG GNURADIO_${EXTVAR}_LIBRARIES GNURADIO_${EXTVAR}_INCLUDE_DIRS)
    message(STATUS "GNURADIO_${EXTVAR}_FOUND = ${GNURADIO_${EXTVAR}_FOUND}")
    set(GNURADIO_${EXTVAR}_FOUND ${GNURADIO_${EXTVAR}_FOUND} PARENT_SCOPE)

    # generate an error if the module is missing
    if(NOT GNURADIO_${EXTVAR}_FOUND)
        message(STATUS "Required GNU Radio Component: ${EXTVAR} missing!")
    endif()

    mark_as_advanced(GNURADIO_${EXTVAR}_LIBRARIES GNURADIO_${EXTVAR}_INCLUDE_DIRS)
endfunction()

gr_module(RUNTIME gnuradio-runtime gnuradio/top_block.h gnuradio-runtime)
gr_module(ANALOG gnuradio-analog gnuradio/analog/api.h gnuradio-analog)
gr_module(AUDIO gnuradio-audio gnuradio/audio/api.h gnuradio-audio)
gr_module(BLOCKS gnuradio-blocks gnuradio/blocks/api.h gnuradio-blocks)
gr_module(CHANNELS gnuradio-channels gnuradio/channels/api.h gnuradio-channels)
gr_module(DIGITAL gnuradio-digital gnuradio/digital/api.h gnuradio-digital)
gr_module(FCD gnuradio-fcd gnuradio/fcd_api.h gnuradio-fcd)
gr_module(FEC gnuradio-fec gnuradio/fec/api.h gnuradio-fec)
gr_module(FFT gnuradio-fft gnuradio/fft/api.h gnuradio-fft)
gr_module(FILTER gnuradio-filter gnuradio/filter/api.h gnuradio-filter)
gr_module(NOAA gnuradio-noaa gnuradio/noaa/api.h gnuradio-noaa)
gr_module(PAGER gnuradio-pager gnuradio/pager/api.h gnuradio-pager)
gr_module(QTGUI gnuradio-qtgui gnuradio/qtgui/api.h gnuradio-qtgui)
gr_module(TRELLIS gnuradio-trellis gnuradio/trellis/api.h gnuradio-trellis)
gr_module(UHD gnuradio-uhd gnuradio/uhd/api.h gnuradio-uhd)
gr_module(VOCODER gnuradio-vocoder gnuradio/vocoder/api.h gnuradio-vocoder)
gr_module(WAVELET gnuradio-wavelet gnuradio/wavelet/api.h gnuradio-wavelet)
gr_module(WXGUI gnuradio-wxgui gnuradio/wxgui/api.h gnuradio-wxgui)
gr_module(PMT gnuradio-runtime pmt/pmt.h gnuradio-pmt)

list(REMOVE_DUPLICATES GNURADIO_ALL_INCLUDE_DIRS)
list(REMOVE_DUPLICATES GNURADIO_ALL_LIBRARIES)

# Trick to find out that GNU Radio is >= 3.7.4 if pkgconfig is not present
if(NOT PC_GNURADIO_RUNTIME_VERSION)
    find_file(GNURADIO_VERSION_GREATER_THAN_373
              NAMES gnuradio/blocks/tsb_vector_sink_f.h
              HINTS $ENV{GNURADIO_RUNTIME_DIR}/include
                    ${CMAKE_INSTALL_PREFIX}/include
                    ${GNURADIO_INSTALL_PREFIX}/include
              PATHS /usr/local/include
                    /usr/include
                    ${GNURADIO_INSTALL_PREFIX}/include
              )
    if(GNURADIO_VERSION_GREATER_THAN_373)
        set(PC_GNURADIO_RUNTIME_VERSION "3.7.4+")
    endif()

    find_file(GNURADIO_VERSION_GREATER_THAN_38
              NAMES gnuradio/filter/mmse_resampler_cc.h
              HINTS $ENV{GNURADIO_RUNTIME_DIR}/include
                    ${CMAKE_INSTALL_PREFIX}/include
                    ${GNURADIO_INSTALL_PREFIX}/include
              PATHS /usr/local/include
                    /usr/include
                    ${GNURADIO_INSTALL_PREFIX}/include
              )
    if(GNURADIO_VERSION_GREATER_THAN_38)
        set(PC_GNURADIO_RUNTIME_VERSION "3.8.0+")
    endif()
endif()
