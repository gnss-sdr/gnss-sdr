# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2021-2025 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

if(NOT DEFINED GNSSSDR_LIB_PATHS)
    include(GnsssdrFindPaths)
endif()

find_program(GNUPLOT_EXECUTABLE
    NAMES
        gnuplot
        pgnuplot
    PATHS
        ${GNSSSDR_BIN_PATHS}
    ONLY_CMAKE_FIND_ROOT_PATH
)

if(NOT CMAKE_CROSSCOMPILING)
    if(GNUPLOT_EXECUTABLE)
        execute_process(COMMAND "${GNUPLOT_EXECUTABLE}" --version
            OUTPUT_VARIABLE GNUPLOT_OUTPUT_VARIABLE
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        string(REGEX REPLACE "^gnuplot ([0-9\\.]+)( patchlevel )?" "\\1." GNUPLOT_VERSION_STRING "${GNUPLOT_OUTPUT_VARIABLE}")
        string(REGEX REPLACE "\\.$" "" GNUPLOT_VERSION_STRING "${GNUPLOT_VERSION_STRING}")
        unset(GNUPLOT_OUTPUT_VARIABLE)
    endif()
else()
    if(GNUPLOT_EXECUTABLE)
        if(CMAKE_SYSROOT)
            string(LENGTH "${CMAKE_SYSROOT}" _gnuplot_sysroot_len)
            string(SUBSTRING "${GNUPLOT_EXECUTABLE}" 0 ${_gnuplot_sysroot_len} _gnuplot_sysroot_prefix)
            if(_gnuplot_sysroot_prefix STREQUAL "${CMAKE_SYSROOT}")
                string(SUBSTRING "${GNUPLOT_EXECUTABLE}" ${_gnuplot_sysroot_len} -1 GNUPLOT_EXECUTABLE)
            endif()
        elseif(DEFINED ENV{OECORE_TARGET_SYSROOT})
            string(LENGTH "$ENV{OECORE_TARGET_SYSROOT}" _gnuplot_sysroot_len)
            string(SUBSTRING "${GNUPLOT_EXECUTABLE}" 0 ${_gnuplot_sysroot_len} _gnuplot_sysroot_prefix)
            if(_gnuplot_sysroot_prefix STREQUAL "$ENV{OECORE_TARGET_SYSROOT}")
                string(SUBSTRING "${GNUPLOT_EXECUTABLE}" ${_gnuplot_sysroot_len} -1 GNUPLOT_EXECUTABLE)
            endif()
        endif()
    else()
        message(STATUS "Warning: Gnuplot is not found, you can install it later.")
        message(STATUS "  Setting default path to /usr/bin/gnuplot")
        set(GNUPLOT_EXECUTABLE "/usr/bin/gnuplot")
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GNUPLOT DEFAULT_MSG GNUPLOT_EXECUTABLE)
unset(_gnuplot_sysroot_len)
unset(_gnuplot_sysroot_prefix)
