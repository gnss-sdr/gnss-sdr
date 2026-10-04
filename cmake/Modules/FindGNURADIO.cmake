# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2026 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

########################################################################
# Find GNU Radio
########################################################################

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT PKG_CONFIG_FOUND)
    include(FindPkgConfig)
endif()

if(NOT DEFINED GNSSSDR_LIB_PATHS)
    include(GnsssdrFindPaths)
endif()

include(FindPackageHandleStandardArgs)

# if GR_REQUIRED_COMPONENTS is not defined, it will be set to the following list
if(NOT DEFINED GR_REQUIRED_COMPONENTS OR
        "${GR_REQUIRED_COMPONENTS}" STREQUAL "")
    set(GR_REQUIRED_COMPONENTS RUNTIME PMT BLOCKS FFT FILTER ANALOG)
endif()

# Allows us to use all .cmake files in this directory
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_LIST_DIR}")

# Easily access all libraries and includes of GNU Radio
set(GNURADIO_ALL_LIBRARIES "")
set(GNURADIO_ALL_INCLUDE_DIRS "")
set(GNURADIO_REQUIRED_COMPONENTS_FOUND TRUE)

macro(LIST_CONTAINS var value)
    set(${var} FALSE)
    foreach(value2 ${ARGN})
        if("${value}" STREQUAL "${value2}")
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
    list_contains(REQUIRED_MODULE "${EXTVAR}" ${GR_REQUIRED_COMPONENTS})
    if(NOT REQUIRED_MODULE)
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

    unset(${LIBVAR_NAME})

    # look for include files
    find_path(${INCVAR_NAME}
        NAMES ${INCFILE}
        HINTS ${PC_INCDIR}
        PATHS ${GNURADIO_INSTALL_PREFIX_USER_PROVIDED}/include
              ${GNSSSDR_INCLUDE_PATHS}
    )

    # look for libs
    foreach(libname ${PC_GNURADIO_${EXTVAR}_LIBRARIES})
        find_library(${LIBVAR_NAME}_${libname}
            NAMES ${libname} ${libname}-${PC_GNURADIO_RUNTIME_VERSION}
            HINTS ${PC_LIBDIR}
            PATHS ${GNURADIO_INSTALL_PREFIX_USER_PROVIDED}/${CMAKE_INSTALL_LIBDIR}
                  ${GNSSSDR_LIB_PATHS}
        )
        if(${LIBVAR_NAME}_${libname})
            list(APPEND ${LIBVAR_NAME} ${${LIBVAR_NAME}_${libname}})
        endif()
    endforeach()

    set(${LIBVAR_NAME} ${${LIBVAR_NAME}} PARENT_SCOPE)

    # show results
    message(STATUS " * INCLUDES=${GNURADIO_${EXTVAR}_INCLUDE_DIRS}")
    message(STATUS " * LIBS=${GNURADIO_${EXTVAR}_LIBRARIES}")

    # append to all includes and libs list
    set(GNURADIO_ALL_INCLUDE_DIRS ${GNURADIO_ALL_INCLUDE_DIRS}
        ${GNURADIO_${EXTVAR}_INCLUDE_DIRS} PARENT_SCOPE)
    set(GNURADIO_ALL_LIBRARIES ${GNURADIO_ALL_LIBRARIES}
        ${GNURADIO_${EXTVAR}_LIBRARIES} PARENT_SCOPE)

    if(GNURADIO_${EXTVAR}_LIBRARIES AND GNURADIO_${EXTVAR}_INCLUDE_DIRS)
        set(GNURADIO_${EXTVAR}_FOUND TRUE)
    else()
        set(GNURADIO_${EXTVAR}_FOUND FALSE)
    endif()
    message(STATUS "GNURADIO_${EXTVAR}_FOUND = ${GNURADIO_${EXTVAR}_FOUND}")
    set(GNURADIO_${EXTVAR}_FOUND ${GNURADIO_${EXTVAR}_FOUND} PARENT_SCOPE)

    # generate an error if the module is missing
    if(NOT GNURADIO_${EXTVAR}_FOUND)
        message(STATUS "Required GNU Radio Component: ${EXTVAR} missing!")
        set(GNURADIO_FOUND FALSE PARENT_SCOPE) # Trick for feature_summary
        set(GNURADIO_REQUIRED_COMPONENTS_FOUND FALSE PARENT_SCOPE)
    endif()

    # Create imported target
    string(TOLOWER "${EXTVAR}" gnuradio_component)
    if(NOT TARGET Gnuradio::${gnuradio_component}
            AND GNURADIO_${EXTVAR}_LIBRARIES)
        add_library(Gnuradio::${gnuradio_component} UNKNOWN IMPORTED)
        set(GNURADIO_LIBRARY ${GNURADIO_${EXTVAR}_LIBRARIES})
        list(GET GNURADIO_LIBRARY 0 FIRST_LIBRARY)
        set_target_properties(Gnuradio::${gnuradio_component} PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
            IMPORTED_LOCATION "${FIRST_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES
                "${GNURADIO_${EXTVAR}_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES "${GNURADIO_LIBRARY}"
        )
    endif()

    mark_as_advanced(GNURADIO_${EXTVAR}_LIBRARIES
        GNURADIO_${EXTVAR}_INCLUDE_DIRS)
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
gr_module(ZEROMQ gnuradio-zeromq gnuradio/zeromq/api.h gnuradio-zeromq)

list(REMOVE_DUPLICATES GNURADIO_ALL_INCLUDE_DIRS)
list(REMOVE_DUPLICATES GNURADIO_ALL_LIBRARIES)

if(NOT PC_GNURADIO_RUNTIME_VERSION)
    if(DEFINED PACKAGE_VERSION)
        set(OLD_PACKAGE_VERSION "${PACKAGE_VERSION}")
        set(_HAD_PACKAGE_VERSION TRUE)
    else()
        unset(OLD_PACKAGE_VERSION)
        set(_HAD_PACKAGE_VERSION FALSE)
    endif()

    if(DEFINED PACKAGE_FIND_VERSION_MAJOR)
        set(OLD_PACKAGE_FIND_VERSION_MAJOR "${PACKAGE_FIND_VERSION_MAJOR}")
        set(_HAD_PACKAGE_FIND_VERSION_MAJOR TRUE)
    else()
        set(_HAD_PACKAGE_FIND_VERSION_MAJOR FALSE)
    endif()

    if(DEFINED PACKAGE_FIND_VERSION_MINOR)
        set(OLD_PACKAGE_FIND_VERSION_MINOR "${PACKAGE_FIND_VERSION_MINOR}")
        set(_HAD_PACKAGE_FIND_VERSION_MINOR TRUE)
    else()
        set(_HAD_PACKAGE_FIND_VERSION_MINOR FALSE)
    endif()

    if(DEFINED PACKAGE_FIND_VERSION_PATCH)
        set(OLD_PACKAGE_FIND_VERSION_PATCH "${PACKAGE_FIND_VERSION_PATCH}")
        set(_HAD_PACKAGE_FIND_VERSION_PATCH TRUE)
    else()
        set(_HAD_PACKAGE_FIND_VERSION_PATCH FALSE)
    endif()

    unset(PACKAGE_VERSION)
    if(GNURADIO_BLOCKS_LIBRARIES)
        list(GET GNURADIO_BLOCKS_LIBRARIES 0 FIRST_LIBRARY)
        get_filename_component(GNURADIO_BLOCKS_DIR "${FIRST_LIBRARY}" DIRECTORY)
        if(EXISTS "${GNURADIO_BLOCKS_DIR}/cmake/gnuradio/GnuradioConfigVersion.cmake")
            set(PACKAGE_FIND_VERSION_MAJOR 3)
            set(PACKAGE_FIND_VERSION_MINOR 7)
            set(PACKAGE_FIND_VERSION_PATCH 4)
            include("${GNURADIO_BLOCKS_DIR}/cmake/gnuradio/GnuradioConfigVersion.cmake")
        endif()
    endif()
    if(PACKAGE_VERSION)
        set(PC_GNURADIO_RUNTIME_VERSION "${PACKAGE_VERSION}")
    endif()

    if(_HAD_PACKAGE_VERSION)
        set(PACKAGE_VERSION "${OLD_PACKAGE_VERSION}")
    else()
        unset(PACKAGE_VERSION)
    endif()
    if(_HAD_PACKAGE_FIND_VERSION_MAJOR)
        set(PACKAGE_FIND_VERSION_MAJOR "${OLD_PACKAGE_FIND_VERSION_MAJOR}")
    else()
        unset(PACKAGE_FIND_VERSION_MAJOR)
    endif()
    if(_HAD_PACKAGE_FIND_VERSION_MINOR)
        set(PACKAGE_FIND_VERSION_MINOR "${OLD_PACKAGE_FIND_VERSION_MINOR}")
    else()
        unset(PACKAGE_FIND_VERSION_MINOR)
    endif()
    if(_HAD_PACKAGE_FIND_VERSION_PATCH)
        set(PACKAGE_FIND_VERSION_PATCH "${OLD_PACKAGE_FIND_VERSION_PATCH}")
    else()
        unset(PACKAGE_FIND_VERSION_PATCH)
    endif()
endif()

# Trick to find out that GNU Radio is >= 3.7.4 if pkgconfig is not present
if(NOT PC_GNURADIO_RUNTIME_VERSION)
    find_file(GNURADIO_VERSION_GREATER_THAN_373
        NAMES gnuradio/blocks/tsb_vector_sink_f.h
        PATHS ${GNURADIO_INSTALL_PREFIX_USER_PROVIDED}/include
              ${GNSSSDR_INCLUDE_PATHS}
    )
    if(GNURADIO_VERSION_GREATER_THAN_373)
        set(PC_GNURADIO_RUNTIME_VERSION "3.7.4+")
    endif()

    find_file(GNURADIO_VERSION_GREATER_THAN_38
        NAMES gnuradio/filter/mmse_resampler_cc.h
        PATHS ${GNURADIO_INSTALL_PREFIX_USER_PROVIDED}/include
              ${GNSSSDR_INCLUDE_PATHS}
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
        if("${CMAKE_SYSTEM_NAME}" MATCHES "Linux|kFreeBSD|GNU")
            message("Go to https://wiki.gnuradio.org/index.php/InstallingGR")
            message("and follow the instructions to install a newer GNU Radio version in your system.")
        endif()
        if("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
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

find_package_handle_standard_args(GNURADIO DEFAULT_MSG
    GNURADIO_RUNTIME_FOUND GNURADIO_REQUIRED_COMPONENTS_FOUND)

# Detect if using standard pointers
set(GNURADIO_USES_STD_POINTERS FALSE)
if(GNURADIO_VERSION VERSION_GREATER 3.8.99)
    file(STRINGS "${GNURADIO_RUNTIME_INCLUDE_DIRS}/gnuradio/basic_block.h" _basic_block)
    foreach(_loop_var IN LISTS _basic_block)
        string(STRIP "${_loop_var}" _file_line)
        if("public std::enable_shared_from_this<basic_block>" STREQUAL "${_file_line}")
            set(GNURADIO_USES_STD_POINTERS TRUE)
        endif()
    endforeach()
endif()

# Detect if FFT are templates
if(EXISTS "${GNURADIO_FFT_INCLUDE_DIRS}/gnuradio/fft/fft_vfc.h")
    set(GNURADIO_FFT_USES_TEMPLATES FALSE)
else()
    set(GNURADIO_FFT_USES_TEMPLATES TRUE)
endif()

# Search for IIO component
if(GNURADIO_VERSION VERSION_GREATER 3.8.99)
    pkg_check_modules(PC_GNURADIO_IIO QUIET gnuradio-iio)
    # look for include files
    message(STATUS "Checking for GNU Radio Module: IIO")
    find_path(GNURADIO_IIO_INCLUDE_DIRS
        NAMES gnuradio/iio/api.h
        HINTS ${PC_GNURADIO_IIO_INCLUDEDIR}
        PATHS ${GNURADIO_INSTALL_PREFIX_USER_PROVIDED}/include
              ${GNSSSDR_INCLUDE_PATHS}
    )

    # look for libs
    find_library(GNURADIO_IIO_LIBRARIES
        NAMES gnuradio-iio gnuradio-iio-${GNURADIO_VERSION}
        HINTS ${PC_GNURADIO_IIO_LIBDIR}
        PATHS ${GNURADIO_INSTALL_PREFIX_USER_PROVIDED}/lib
              ${GNURADIO_INSTALL_PREFIX_USER_PROVIDED}/lib64
              ${GNSSSDR_LIB_PATHS}
    )

    if(GNURADIO_IIO_LIBRARIES)
        message(STATUS " * INCLUDES=${GNURADIO_IIO_INCLUDE_DIRS}")
        message(STATUS " * LIBS=${GNURADIO_IIO_LIBRARIES}")
    else()
        message(STATUS " * IIO GNU Radio Module not found.")
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
            add_library(Gnuradio::iio UNKNOWN IMPORTED)
            set(GNURADIO_LIBRARY ${GNURADIO_IIO_LIBRARIES})
            list(GET GNURADIO_LIBRARY 0 FIRST_LIBRARY)
            set_target_properties(Gnuradio::iio PROPERTIES
                IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
                IMPORTED_LOCATION "${FIRST_LIBRARY}"
                INTERFACE_INCLUDE_DIRECTORIES "${GNURADIO_IIO_INCLUDE_DIRS}"
                INTERFACE_LINK_LIBRARIES "${GNURADIO_LIBRARY}"
            )
        endif()

        # check templatized API
        if(NOT EXISTS "${GNURADIO_IIO_INCLUDE_DIRS}/gnuradio/iio/pluto_source.h")
            set(GR_IIO_TEMPLATIZED_API TRUE)
        endif()
    endif()
endif()

# Check if PMT uses boost::any or std::any
if(GNURADIO_PMT_INCLUDE_DIRS)
    file(STRINGS "${GNURADIO_PMT_INCLUDE_DIRS}/pmt/pmt.h" _pmt_content)
    set(_uses_boost TRUE)
    foreach(_loop_var IN LISTS _pmt_content)
        string(STRIP "${_loop_var}" _file_line)
        if("#include <any>" STREQUAL "${_file_line}")
            set(_uses_boost FALSE)
        endif()
    endforeach()
    if(_uses_boost)
        set(PMT_USES_BOOST_ANY TRUE)
    endif()
endif()

# Check if GNU Radio uses log4cpp or spdlog
if(GNURADIO_RUNTIME_INCLUDE_DIRS)
    if(EXISTS "${GNURADIO_RUNTIME_INCLUDE_DIRS}/gnuradio/logger.h")
        file(STRINGS "${GNURADIO_RUNTIME_INCLUDE_DIRS}/gnuradio/logger.h" _logger_content)
        set(_uses_log4cpp FALSE)
        set(_uses_spdlog FALSE)
        foreach(_loop_var IN LISTS _logger_content)
            string(STRIP "${_loop_var}" _file_line)
            if("#include <log4cpp/Category.hh>" STREQUAL "${_file_line}")
                set(_uses_log4cpp TRUE)
            endif()
            if("#include <spdlog/spdlog.h>" STREQUAL "${_file_line}")
                set(_uses_spdlog TRUE)
            endif()
        endforeach()
        if(_uses_log4cpp)
            find_package(LOG4CPP)
            set_package_properties(LOG4CPP PROPERTIES
                PURPOSE "Required by GNU Radio."
                TYPE REQUIRED
            )
            if(CMAKE_VERSION VERSION_GREATER 3.13)
                if(TARGET Gnuradio::filter)
                    target_link_libraries(Gnuradio::filter INTERFACE Log4cpp::log4cpp)
                endif()
                if(TARGET Gnuradio::runtime)
                    target_link_libraries(Gnuradio::runtime INTERFACE Log4cpp::log4cpp)
                endif()
            else()
                if(TARGET Gnuradio::filter)
                    set_target_properties(Gnuradio::filter PROPERTIES INTERFACE_LINK_LIBRARIES Log4cpp::log4cpp)
                endif()
                if(TARGET Gnuradio::runtime)
                    set_target_properties(Gnuradio::runtime PROPERTIES INTERFACE_LINK_LIBRARIES Log4cpp::log4cpp)
                endif()
            endif()
        endif()
        if(_uses_spdlog)
            find_package(spdlog REQUIRED CONFIG)
            set_package_properties(spdlog PROPERTIES
                URL "https://github.com/gabime/spdlog"
                DESCRIPTION "Very fast, header-only/compiled, C++ logging library (found: v${spdlog_VERSION})"
                PURPOSE "Required by GNU Radio."
                TYPE REQUIRED
            )
            set(GNURADIO_USES_SPDLOG TRUE)
            if(CMAKE_VERSION VERSION_GREATER 3.13)
                if(TARGET Gnuradio::runtime)
                    target_link_libraries(Gnuradio::runtime INTERFACE spdlog::spdlog)
                endif()
                if(TARGET Gnuradio::blocks)
                    target_link_libraries(Gnuradio::blocks INTERFACE spdlog::spdlog)
                endif()
            else()
                if(TARGET Gnuradio::runtime)
                    set_target_properties(Gnuradio::runtime PROPERTIES INTERFACE_LINK_LIBRARIES spdlog::spdlog)
                endif()
                if(TARGET Gnuradio::blocks)
                    set_target_properties(Gnuradio::blocks PROPERTIES INTERFACE_LINK_LIBRARIES spdlog::spdlog)
                endif()
            endif()
        endif()
    endif()
endif()

set_package_properties(GNURADIO PROPERTIES
    URL "https://www.gnuradio.org/"
)
