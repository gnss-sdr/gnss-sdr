# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2021 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


find_program(GNUPLOT_EXECUTABLE
    NAMES
        gnuplot
        pgnuplot
    PATHS
        /usr/bin
        /usr/local/bin
        /opt/local/bin
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
            string(REGEX REPLACE "${CMAKE_SYSROOT}" "" GNUPLOT_EXECUTABLE "${GNUPLOT_EXECUTABLE}")
        elseif(DEFINED ENV{OECORE_TARGET_SYSROOT})
            string(REGEX REPLACE "$ENV{OECORE_TARGET_SYSROOT}" "" GNUPLOT_EXECUTABLE "${GNUPLOT_EXECUTABLE}")
        endif()
    else()
        message(STATUS "Warning: Gnuplot is not found, you can install it later.")
        message(STATUS "  Setting default path to /usr/bin/gnuplot")
        set(GNUPLOT_EXECUTABLE "/usr/bin/gnuplot")
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GNUPLOT DEFAULT_MSG GNUPLOT_EXECUTABLE)
