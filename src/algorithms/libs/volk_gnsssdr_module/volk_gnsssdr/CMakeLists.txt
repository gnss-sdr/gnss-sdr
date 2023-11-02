# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2021 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


########################################################################
# Project setup
########################################################################
cmake_minimum_required(VERSION 2.8.12...3.27)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose build type: None Debug Release RelWithDebInfo MinSizeRel")
project(volk_gnsssdr)
enable_language(CXX)
enable_language(C)
enable_testing()

set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}) # allows this to be a sub-project
set(PROJECT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}) # allows this to be a sub-project
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules) # location for custom "Modules"

if(POLICY CMP0042)
    cmake_policy(SET CMP0042 NEW)
endif()
if(POLICY CMP0068)
    cmake_policy(SET CMP0068 NEW)
endif()
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

# Set compiler flags
set(GNSSSDR_CLANG_MIN_VERSION "3.4.0")
set(GNSSSDR_APPLECLANG_MIN_VERSION "500")
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    execute_process(COMMAND
        ${CMAKE_CXX_COMPILER} -v
        RESULT_VARIABLE _res ERROR_VARIABLE _err
        ERROR_STRIP_TRAILING_WHITESPACE)
    if(${_res} STREQUAL "0")
        # output is in error stream
        string(REGEX MATCH "^Apple.*" IS_APPLE ${_err})
        if("${IS_APPLE}" STREQUAL "")
            set(MIN_VERSION ${GNSSSDR_CLANG_MIN_VERSION})
            set(APPLE_STR "")
            # retrieve the compiler's version from it
            string(REGEX MATCH "clang version [0-9.]+" CLANG_OTHER_VERSION ${_err})
            string(REGEX MATCH "[0-9.]+" CLANG_VERSION ${CLANG_OTHER_VERSION})
        else()
            set(MIN_VERSION ${GNSSSDR_APPLECLANG_MIN_VERSION})
            set(APPLE_STR "Apple ")
            # retrieve the compiler's version from it
            string(REGEX MATCH "(clang-[0-9.]+)" CLANG_APPLE_VERSION ${_err})
            string(REGEX MATCH "[0-9.]+" CLANG_VERSION ${CLANG_APPLE_VERSION})
        endif()
        if(${CLANG_VERSION} VERSION_LESS "${MIN_VERSION}")
            message(WARNING "\nThe compiler selected to build VOLK-GNSSSDR (${APPLE_STR}Clang version ${CLANG_VERSION} : ${CMAKE_CXX_COMPILER}) is older than that officially supported (${MIN_VERSION} minimum). This build may or not work. We highly recommend using Apple Clang version ${APPLECLANG_MIN_VERSION} or more recent, or Clang version ${CLANG_MIN_VERSION} or more recent.")
        endif()
    else()
        message(WARNING "\nCannot determine the version of the compiler selected to build VOLK-GNSSSDR (${APPLE_STR}Clang : ${CMAKE_CXX_COMPILER}). This build may or not work. We highly recommend using Apple Clang version ${APPLECLANG_MIN_VERSION} or more recent, or Clang version ${CLANG_MIN_VERSION} or more recent.")
    endif()
endif()

#### Set C++ standard
set(CMAKE_CXX_EXTENSIONS OFF)

# Check if we have std::filesystem
if(CMAKE_VERSION VERSION_LESS 3.8)
    set(FILESYSTEM_FOUND FALSE)
else()
    find_package(FILESYSTEM COMPONENTS Final Experimental)
endif()

set(MY_CXX_FLAGS "")
if((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND NOT WIN32)
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "6.1.1")
        if(CMAKE_VERSION VERSION_LESS "3.1")
            set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++11")
        else()
            set(CMAKE_CXX_STANDARD 11)
        endif()
    else()
        if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "8.0.0")
            set(CMAKE_CXX_STANDARD 14)
        else()
            if(${FILESYSTEM_FOUND})
                if(CMAKE_VERSION VERSION_LESS 3.12)
                    set(CMAKE_CXX_STANDARD 17)
                else()
                    set(CMAKE_CXX_STANDARD 20)
                endif()
                set(CMAKE_CXX_STANDARD_REQUIRED ON)
            else()
                set(CMAKE_CXX_STANDARD 14)
            endif()
        endif()
    endif()
    set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -Wall -Wextra")  # Add warning flags: For "-Wall" see https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    if(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
        # See https://trac.macports.org/wiki/XcodeVersionInfo for Apple Clang version equivalences
        if(CLANG_VERSION VERSION_LESS "600")
            set(CMAKE_CXX_STANDARD 11)
        else()
            if(CLANG_VERSION VERSION_LESS "1000")
                set(CMAKE_CXX_STANDARD 14)
            else()
                if(${FILESYSTEM_FOUND})
                    set(CMAKE_CXX_STANDARD 17)
                    set(CMAKE_CXX_STANDARD_REQUIRED ON)
                else()
                    set(CMAKE_CXX_STANDARD 14)
                endif()
            endif()
        endif()
    else()
        if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "3.5.0")
            if(CMAKE_VERSION VERSION_LESS "3.1")
                set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++11")
            else()
                set(CMAKE_CXX_STANDARD 11)
            endif()
        else()
            if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "6.0.0")
                set(CMAKE_CXX_STANDARD 14)
            else()
                if(${FILESYSTEM_FOUND})
                    if(CMAKE_VERSION VERSION_LESS 3.12)
                        set(CMAKE_CXX_STANDARD 17)
                    else()
                        set(CMAKE_CXX_STANDARD 20)
                        if(CMAKE_VERSION VERSION_GREATER 3.20.99)
                            if(((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND NOT (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "11.0.0")) OR
                                ((CMAKE_CXX_COMPILER_ID STREQUAL "Clang") AND NOT (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "12.0")))
                                set(CMAKE_CXX_STANDARD 23)
                            endif()
                        endif()
                    endif()
                    set(CMAKE_CXX_STANDARD_REQUIRED ON)
                else()
                    set(CMAKE_CXX_STANDARD 14)
                endif()
            endif()
        endif()
    endif()
endif()

if(NOT ((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND NOT WIN32) AND NOT (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
    if(NOT (CMAKE_VERSION VERSION_LESS "3.1"))
        set(CMAKE_C_STANDARD 11)
        set(CMAKE_CXX_STANDARD 11)
    endif()
endif()

########################################################################
# Common compile flags
########################################################################

# Disable complex math NaN/INFO range checking for performance
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-fcx-limited-range HAVE_CX_LIMITED_RANGE)
if(HAVE_CX_LIMITED_RANGE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcx-limited-range")
endif()

include(CheckCCompilerFlag)
check_c_compiler_flag(-fcx-limited-range HAVE_C_LIMITED_RANGE)
if(HAVE_C_LIMITED_RANGE)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcx-limited-range")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MY_CXX_FLAGS} -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

if(CMAKE_C_COMPILER_ID MATCHES "Clang|GNU")
    # Abort compilation if kernel implementations have inconsistent function
    # prototypes, i.e. if
    #
    #     kernel_foo_sse(uint32_t *dst, lv32fc_t *src)
    #     kernel_foo_avx(uint16_t *dst, lv32fc_t *src)
    #
    # are defined. Note the different data type of the first argument). By
    # default 'incompatible-pointer-types' is a warning only and 'pointer-sign'
    # is a warning enabled by '-Wall'. These warnings are only applicable to C.
    if(NOT (CMAKE_C_COMPILER_ID MATCHES "GNU" AND CMAKE_C_COMPILER_VERSION VERSION_LESS "5.0"))
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Werror=incompatible-pointer-types -Werror=pointer-sign")
    endif()
endif()

option(ENABLE_STRIP "Create a stripped volk_gnsssdr_profile binary (without debugging symbols)" OFF)
if(ENABLE_STRIP)
    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()

include(VolkBuildTypes)
# select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    message(STATUS "Build type not specified: defaulting to release.")
endif()
volk_check_build_type(${CMAKE_BUILD_TYPE})
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")
message(STATUS "Build type set to ${CMAKE_BUILD_TYPE}.")

set(VERSION_INFO_MAJOR_VERSION 0)
set(VERSION_INFO_MINOR_VERSION 0)
set(VERSION_INFO_MAINT_VERSION 18)
include(VolkGnsssdrVersion) # setup version info



########################################################################
# Environment setup
########################################################################
if(NOT DEFINED BOOST_ROOT AND NOT DEFINED ENV{BOOST_ROOT})
    set(BOOST_ROOT ${CMAKE_INSTALL_PREFIX})
endif()

if(NOT DEFINED CROSSCOMPILE_MULTILIB)
    set(CROSSCOMPILE_MULTILIB "")
endif()
set(CROSSCOMPILE_MULTILIB ${CROSSCOMPILE_MULTILIB} CACHE STRING "Define \"true\" if you have and want to use multiple C development libs installed for cross compile")

if(MSVC)
    if(CMAKE_VERSION VERSION_GREATER 3.12.0)
        add_compile_definitions(_USE_MATH_DEFINES) # enables math constants on all supported versions of MSVC
    else()
        add_definitions(-D_USE_MATH_DEFINES) # enables math constants on all supported versions of MSVC
    endif()
    add_compile_options(/W1) # reduce warnings
    add_compile_options(/wo4309)
    add_compile_options(/wd4752)
    add_compile_options(/wo4273)
    add_compile_options(/wo4838)
endif()

# allow 'large' files in 32 bit builds
if(UNIX)
    if(CMAKE_VERSION VERSION_GREATER 3.12.0)
        add_compile_definitions(_LARGEFILE_SOURCE _FILE_OFFSET_BITS=64 _LARGE_FILES)
    else()
        add_definitions(-D_LARGEFILE_SOURCE -D_FILE_OFFSET_BITS=64 -D_LARGE_FILES)
    endif()
endif()


########################################################################
# Dependencies setup
########################################################################

# cpu_features

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^cortex")
    set(CMAKE_SYSTEM_PROCESSOR arm-${CMAKE_SYSTEM_PROCESSOR})
endif()

# cpu_features - sensible defaults, user settable option
if(CMAKE_SYSTEM_PROCESSOR MATCHES
    "(^mips)|(^arm)|(^aarch64)|(x86_64)|(AMD64|amd64)|(^i.86$)|(^powerpc)|(^ppc)|(^s390x)|^riscv")
    option(VOLK_CPU_FEATURES "volk-gnsssdr uses cpu_features" ON)
else()
    option(VOLK_CPU_FEATURES "volk-gnsssdr uses cpu_features" OFF)
endif()

if(CMAKE_VERSION VERSION_GREATER 3.0 AND VOLK_CPU_FEATURES)
    find_package(CPUFEATURES)
    if(CPUFEATURES_FOUND AND (CMAKE_SYSTEM_PROCESSOR MATCHES "(^s390x)|^riscv") AND (CPUFEATURES_VERSION VERSION_LESS "0.8.0"))
        set(USE_CPU_FEATURES OFF)
        unset(CPUFEATURES_FOUND CACHE)
        message(STATUS "Building volk-gnsssdr without cpu_features, installed version v${CPUFEATURES_VERSION} does not support the ${CMAKE_SYSTEM_PROCESSOR} architecture")
    else()
        set(USE_CPU_FEATURES ON)
    endif()
    if(NOT CPUFEATURES_FOUND)
        message(STATUS "Building volk-gnsssdr with cpu_features")
        set(BUILD_TESTING OFF CACHE BOOL "Build cpu_features without tests." FORCE)
        set(BUILD_PIC ON CACHE BOOL
            "Build cpu_features with Position Independent Code (PIC)."
            FORCE
        )
        set(CMAKE_POSITION_INDEPENDENT_CODE ON CACHE BOOL
            "Build cpu_features with Position Independent Code (PIC)."
            FORCE
        )
        set(BUILD_SHARED_LIBS_SAVED "${BUILD_SHARED_LIBS}")
        set(BUILD_SHARED_LIBS OFF)
        add_subdirectory(cpu_features)
        set(BUILD_SHARED_LIBS "${BUILD_SHARED_LIBS_SAVED}")
    endif()
else()
    message(STATUS "Building volk-gnsssdr without cpu_features")
endif()


# Python
include(VolkPython) # sets PYTHON_EXECUTABLE

if(NOT PYTHON_EXECUTABLE)
    message(FATAL_ERROR "Python 2.7 or greater required to build VOLK_GNSSSDR")
endif()

# Mako
if(NOT MAKO_FOUND)
    message(FATAL_ERROR "Mako templates required to build VOLK_GNSSSDR")
endif()

# Six
if(PYTHON_VERSION_STRING VERSION_LESS "3.0")
    if(NOT SIX_FOUND)
        message(FATAL_ERROR "six - python 2 and 3 compatibility library required to build VOLK_GNSSSDR")
    endif()
endif()


# Boost
if(MSVC)
    if(NOT DEFINED BOOST_ALL_DYN_LINK)
        set(BOOST_ALL_DYN_LINK TRUE)
    endif()
    set(BOOST_ALL_DYN_LINK "${BOOST_ALL_DYN_LINK}" CACHE BOOL "boost enable dynamic linking")
    if(BOOST_ALL_DYN_LINK)
        if(CMAKE_VERSION VERSION_GREATER 3.12.0)
            add_compile_definitions(BOOST_ALL_DYN_LINK) # enables math constants on all supported versions of MSVC
        else()
            add_definitions(-DBOOST_ALL_DYN_LINK) # setup boost auto-linking in msvc
        endif()
    else()
        unset(BOOST_REQUIRED_COMPONENTS) # empty components list for static link
    endif()
endif()
if(NOT ${FILESYSTEM_FOUND})
    include(VolkBoost)
    if(NOT Boost_FOUND)
        message(FATAL_ERROR "VOLK-GNSSSDR requires Boost to build")
    endif()
endif()


########################################################################
# check for aligned_alloc, since some compilers lack this C11 feature.
# For Apple-clang use `posix_memalign`
# For MSVC use `_aligned_malloc`.
########################################################################
include(CheckSymbolExists)
if(NOT (${CMAKE_SYSTEM_NAME} MATCHES "Darwin"))
    check_symbol_exists(aligned_alloc stdlib.h USE_ALIGNED_ALLOC)
endif()
if(NOT USE_ALIGNED_ALLOC)
    check_symbol_exists(posix_memalign stdlib.h HAVE_POSIX_MEMALIGN)
endif()


########################################################################
# Check if Orc is available
########################################################################
option(ENABLE_ORC "Enable Orc" TRUE)
if(ENABLE_ORC)
    find_package(ORC)
else()
    message(STATUS "Disabling use of ORC")
endif()


########################################################################
# Setup doxygen
########################################################################
find_package(Doxygen)
if(DOXYGEN_FOUND)
    add_subdirectory(docs)
endif()


########################################################################
# Detect /lib versus /lib64
########################################################################
if(${CMAKE_INSTALL_LIBDIR} MATCHES lib64)
    set(LIB_SUFFIX 64)
endif()


########################################################################
# Setup the package config file
########################################################################
# set variables found in the pc.in file
if(NOT LIB_SUFFIX)
    set(LIB_SUFFIX "")
endif()
set(prefix ${CMAKE_INSTALL_PREFIX})
set(exec_prefix "\${prefix}")
set(libdir "\${exec_prefix}/lib${LIB_SUFFIX}")
set(includedir "\${prefix}/include")

configure_file(
    ${PROJECT_SOURCE_DIR}/tmpl/volk_gnsssdr.pc.in
    ${PROJECT_BINARY_DIR}/volk_gnsssdr.pc
    @ONLY
)

install(
    FILES ${PROJECT_BINARY_DIR}/volk_gnsssdr.pc
    DESTINATION lib${LIB_SUFFIX}/pkgconfig
    COMPONENT "volk_gnsssdr_devel"
)

########################################################################
# Install all headers in the include directories
########################################################################
set(VOLK_RUNTIME_DIR bin)
set(VOLK_LIBRARY_DIR lib${LIB_SUFFIX})
set(VOLK_INCLUDE_DIR include)

install(
    DIRECTORY ${PROJECT_SOURCE_DIR}/kernels/volk_gnsssdr
    DESTINATION include COMPONENT "volk_gnsssdr_devel"
    FILES_MATCHING PATTERN "*.h"
)

install(FILES
    ${PROJECT_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_alloc.h
    ${PROJECT_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_prefs.h
    ${PROJECT_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_complex.h
    ${PROJECT_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_common.h
    ${PROJECT_SOURCE_DIR}/include/volk_gnsssdr/saturation_arithmetic.h
    ${PROJECT_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_avx_intrinsics.h
    ${PROJECT_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_sse_intrinsics.h
    ${PROJECT_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_sse3_intrinsics.h
    ${PROJECT_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_neon_intrinsics.h
    ${PROJECT_BINARY_DIR}/include/volk_gnsssdr/volk_gnsssdr.h
    ${PROJECT_BINARY_DIR}/include/volk_gnsssdr/volk_gnsssdr_cpu.h
    ${PROJECT_BINARY_DIR}/include/volk_gnsssdr/volk_gnsssdr_config_fixed.h
    ${PROJECT_BINARY_DIR}/include/volk_gnsssdr/volk_gnsssdr_typedefs.h
    ${PROJECT_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_malloc.h
    ${PROJECT_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_sine_table.h
    ${PROJECT_SOURCE_DIR}/include/volk_gnsssdr/constants.h
    DESTINATION include/volk_gnsssdr
    COMPONENT "volk_gnsssdr_devel"
)


########################################################################
# On Apple only, set install name and use rpath correctly, if not already set
########################################################################
if(APPLE)
    if(NOT CMAKE_INSTALL_NAME_DIR)
        set(CMAKE_INSTALL_NAME_DIR
            ${CMAKE_INSTALL_PREFIX}/${VOLK_LIBRARY_DIR} CACHE
            PATH "Library Install Name Destination Directory" FORCE
        )
    endif()
    if(NOT CMAKE_INSTALL_RPATH)
        set(CMAKE_INSTALL_RPATH
            ${CMAKE_INSTALL_PREFIX}/${VOLK_LIBRARY_DIR} CACHE
            PATH "Library Install RPath" FORCE
        )
    endif()
    if(NOT CMAKE_BUILD_WITH_INSTALL_RPATH)
        set(CMAKE_BUILD_WITH_INSTALL_RPATH ON CACHE
            BOOL "Do Build Using Library Install RPath" FORCE
        )
    endif()
endif()


########################################################################
# Create uninstall target
########################################################################
configure_file(
    ${PROJECT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in
    ${PROJECT_BINARY_DIR}/cmake_uninstall.cmake
    @ONLY
)

# Only add the target if there isn't one defined already
if(NOT TARGET uninstall)
    add_custom_target(uninstall
        ${CMAKE_COMMAND} -P ${PROJECT_BINARY_DIR}/cmake_uninstall.cmake
    )
endif()



########################################################################
# Install our Cmake modules into $prefix/lib/cmake/volk_gnsssdr
# See "Package Configuration Files" on page:
#    https://www.cmake.org/Wiki/CMake/Tutorials/Packaging
########################################################################

configure_file(
    ${PROJECT_SOURCE_DIR}/cmake/Modules/VolkGnsssdrConfig.cmake.in
    ${PROJECT_BINARY_DIR}/cmake/Modules/VolkGnsssdrConfig.cmake
    @ONLY
)

configure_file(
    ${PROJECT_SOURCE_DIR}/cmake/Modules/VolkGnsssdrConfigVersion.cmake.in
    ${PROJECT_BINARY_DIR}/cmake/Modules/VolkGnsssdrConfigVersion.cmake
    @ONLY
)


########################################################################
# Install cmake search routine for external use
########################################################################

if(NOT CMAKE_MODULES_DIR)
    set(CMAKE_MODULES_DIR lib${LIB_SUFFIX}/cmake)
endif()

install(
    FILES
    ${PROJECT_BINARY_DIR}/cmake/Modules/VolkGnsssdrConfig.cmake
    ${PROJECT_BINARY_DIR}/cmake/Modules/VolkGnsssdrConfigVersion.cmake
    DESTINATION ${CMAKE_MODULES_DIR}/volk_gnsssdr
    COMPONENT "volk_gnsssdr_devel"
)

install(EXPORT VOLK_GNSSSDR-export FILE VolkGnsssdrTargets.cmake
    NAMESPACE VolkGnsssdr:: DESTINATION ${CMAKE_MODULES_DIR}/volk_gnsssdr
)

########################################################################
# Option to enable QA testing, on by default
########################################################################
option(ENABLE_TESTING "Enable QA testing" ON)
if(ENABLE_TESTING)
    message(STATUS "QA Testing is enabled.")
else()
    message(STATUS "QA Testing is disabled.")
endif()
message(STATUS "  Modify using: -DENABLE_TESTING=ON/OFF")


########################################################################
# Option to enable post-build profiling using volk_profile, off by default
########################################################################
option(ENABLE_PROFILING "Launch system profiler after build" OFF)
if(ENABLE_PROFILING)
    set(ENABLE_STATIC_LIBS ON)
    if(DEFINED VOLK_CONFIGPATH)
        get_filename_component(VOLK_CONFIGPATH ${VOLK_CONFIGPATH} ABSOLUTE)
        set(VOLK_CONFIGPATH "${VOLK_CONFIGPATH}/volk_gnsssdr")
        message(STATUS "System profiling is enabled, using path: ${VOLK_CONFIGPATH}")
    elseif(DEFINED ENV{VOLK_CONFIGPATH})
        set(VOLK_CONFIGPATH "$ENV{VOLK_CONFIGPATH}/volk_gnsssdr")
        message(STATUS "System profiling is enabled, using env path: $ENV{VOLK_CONFIGPATH}")
    else()
        message(STATUS "System profiling is enabled with default paths.")
        if(DEFINED ENV{HOME})
            set(VOLK_CONFIGPATH "$ENV{HOME}/.volk_gnsssdr")
        elseif(DEFINED ENV{APPDATA})
            set(VOLK_CONFIGPATH "$ENV{APPDATA}/.volk_gnsssdr")
        endif()
    endif()
else()
    message(STATUS "System profiling is disabled.")
endif()
message(STATUS "  Modify using: -DENABLE_PROFILING=ON/OFF")
########################################################################

########################################################################
# Setup the library
########################################################################
add_subdirectory(lib)

########################################################################
# And the utility apps
########################################################################
add_subdirectory(apps)
add_subdirectory(python/volk_gnsssdr_modtool)


########################################################################
# Print summary
########################################################################
message(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")
