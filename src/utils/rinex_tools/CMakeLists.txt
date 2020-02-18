# Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

############################################################################
# Local installation of GPSTk https://github.com/SGL-UT/GPSTk
############################################################################
find_package(GPSTK)
set_package_properties(GPSTK PROPERTIES
    PURPOSE "Used in some Extra Tests."
    TYPE REQUIRED
)
if(NOT GPSTK_FOUND OR ENABLE_OWN_GPSTK)
    set_package_properties(GPSTK PROPERTIES
        PURPOSE "GPSTk v${GNSSSDR_GPSTK_LOCAL_VERSION} will be automatically downloaded and built when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'."
    )
    if("${TOOLCHAIN_ARG}" STREQUAL "")
        set(TOOLCHAIN_ARG "-DCMAKE_CXX_FLAGS=-Wno-deprecated")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated")
    endif()
    set(GPSTK_BUILD_COMMAND "${CMAKE_MAKE_PROGRAM}")
    if(CMAKE_GENERATOR STREQUAL Xcode)
        set(GPSTK_BUILD_COMMAND "xcodebuild" "-configuration" "${CMAKE_BUILD_TYPE}")
    endif()
    include(GNUInstallDirs)
    if(CMAKE_VERSION VERSION_LESS 3.2)
        ExternalProject_Add(gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}
            GIT_REPOSITORY https://github.com/SGL-UT/GPSTk
            GIT_TAG v${GNSSSDR_GPSTK_LOCAL_VERSION}
            SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}
            BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/../../gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}
            CMAKE_ARGS ${GTEST_COMPILER} ${TOOLCHAIN_ARG} -DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install -DBUILD_EXT=OFF -DBUILD_PYTHON=OFF -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
            BUILD_COMMAND ${GPSTK_BUILD_COMMAND}
            UPDATE_COMMAND ""
            PATCH_COMMAND ""
        )
    else()
        ExternalProject_Add(gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}
            GIT_REPOSITORY https://github.com/SGL-UT/GPSTk
            GIT_TAG v${GNSSSDR_GPSTK_LOCAL_VERSION}
            SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}
            BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/../../gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}
            CMAKE_ARGS ${GTEST_COMPILER} ${TOOLCHAIN_ARG} -DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install -DBUILD_EXT=OFF -DBUILD_PYTHON=OFF -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
            BUILD_COMMAND ${GPSTK_BUILD_COMMAND}
            BUILD_BYPRODUCTS ${CMAKE_SOURCE_DIR}/thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install/${CMAKE_INSTALL_LIBDIR}/${CMAKE_FIND_LIBRARY_PREFIXES}gpstk${CMAKE_SHARED_LIBRARY_SUFFIX}
            UPDATE_COMMAND ""
            PATCH_COMMAND ""
        )
    endif()
    set(GPSTK_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install/include CACHE PATH "Local GPSTK headers")
    set(GPSTK_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install/${CMAKE_INSTALL_LIBDIR}/${CMAKE_FIND_LIBRARY_PREFIXES}gpstk${CMAKE_SHARED_LIBRARY_SUFFIX})
    set(GPSTK_BINDIR ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install/bin/)
    add_definitions(-DGPSTK_BINDIR="${GPSTK_BINDIR}")
    add_library(Gpstk::gpstk SHARED IMPORTED)
    add_dependencies(Gpstk::gpstk gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION})
    file(MAKE_DIRECTORY ${GPSTK_INCLUDE_DIRS})
    file(MAKE_DIRECTORY ${GPSTK_INCLUDE_DIRS}/gpstk)
    set_target_properties(Gpstk::gpstk PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${GPSTK_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${GPSTK_INCLUDE_DIRS};${GPSTK_INCLUDE_DIRS}/gpstk"
        INTERFACE_LINK_LIBRARIES "${GPSTK_LIBRARY}"
    )
else()
    set(GPSTK_BINDIR ${GPSTK_INCLUDE_DIR}/../bin/)
    add_definitions(-DGPSTK_BINDIR="${GPSTK_BINDIR}")
endif()


find_package(Gnuplot)
if(GNUPLOT_FOUND)
    add_definitions(-DGNUPLOT_EXECUTABLE="${GNUPLOT_EXECUTABLE}")
endif()
set_package_properties(Gnuplot PROPERTIES
    URL "http://www.gnuplot.info"
    PURPOSE "Used to generate plots in some tests."
    TYPE OPTIONAL
)
if(GNUPLOT_FOUND AND GNUPLOT_VERSION_STRING)
    set_package_properties(Gnuplot PROPERTIES
        DESCRIPTION "A portable command-line driven graphing utility (found: v${GNUPLOT_VERSION_STRING})"
    )
else()
    set_package_properties(Gnuplot PROPERTIES
        DESCRIPTION "A portable command-line driven graphing utility"
    )
endif()


set(LIST_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/src/tests/common-files
)

include_directories(${LIST_INCLUDE_DIRS})

#set(obsdiff_SOURCES obsdiff.cc)
set(obsdiff_HEADERS obsdiff_flags.h)
source_group(Headers FILES ${obsdiff_HEADERS})


if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(obsdiff_lib
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

add_executable(obsdiff ${CMAKE_CURRENT_SOURCE_DIR}/obsdiff.cc)

################################################################################
# Detect availability of std::filesystem and set C++ standard accordingly
################################################################################
set(FILESYSTEM_FOUND FALSE)
if(NOT (GNURADIO_VERSION VERSION_LESS 3.8) AND LOG4CPP_READY_FOR_CXX17)
    # Check if we have std::filesystem
    if(NOT (CMAKE_VERSION VERSION_LESS 3.8))
        find_package(FILESYSTEM COMPONENTS Final Experimental)
        set_package_properties(FILESYSTEM PROPERTIES
            URL "https://en.cppreference.com/w/cpp/filesystem"
            DESCRIPTION "Provides facilities for performing operations on file systems and their components"
            PURPOSE "Work with paths, regular files, and directories."
            TYPE OPTIONAL
        )
        if(${FILESYSTEM_FOUND})
            if(CMAKE_VERSION VERSION_LESS 3.13)
                set(CMAKE_CXX_STANDARD 17)
            else()
                set(CMAKE_CXX_STANDARD 20)
            endif()
            set(CMAKE_CXX_STANDARD_REQUIRED ON)
        endif()
    endif()
endif()




target_link_libraries(obsdiff
    PUBLIC
        Boost::headers
    PRIVATE
        Armadillo::armadillo
        Threads::Threads
        channel_libs
        algorithms_libs
        core_receiver
        core_libs
        Gflags::gflags
        Glog::glog
        Gpstk::gpstk
)

target_compile_definitions(obsdiff
    PUBLIC -DGNSS_SDR_VERSION="${VERSION}"
    PUBLIC -DGNSSSDR_INSTALL_DIR="${CMAKE_INSTALL_PREFIX}"
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(obsdiff
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

add_custom_command(TARGET obsdiff POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:obsdiff>
        ${CMAKE_SOURCE_DIR}/install/$<TARGET_FILE_NAME:obsdiff>)

install(TARGETS obsdiff
    RUNTIME DESTINATION bin
    COMPONENT "obsdiff"
)

find_program(GZIP gzip
    /bin
    /usr/bin
    /usr/local/bin
    /opt/local/bin
    /sbin
)

