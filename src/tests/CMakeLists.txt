# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


add_subdirectory(unit-tests/signal-processing-blocks/libs)
add_subdirectory(system-tests/libs)

################################################################################
# Google Test - https://github.com/google/googletest
################################################################################
set(GTEST_COMPILER -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER} -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER})
set(TOOLCHAIN_ARG "")
if(DEFINED ENV{OECORE_TARGET_SYSROOT})
    set(GTEST_COMPILER "")
    set(TOOLCHAIN_ARG "-DCMAKE_TOOLCHAIN_FILE=${GNSSSDR_SOURCE_DIR}/cmake/Toolchains/oe-sdk_cross.cmake")
else()
    if(CMAKE_TOOLCHAIN_FILE)
        set(TOOLCHAIN_ARG "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
    endif()
    if(ENABLE_PACKAGING)
        set(TOOLCHAIN_ARG "-DCMAKE_VERBOSE_MAKEFILE=ON")
    endif()
endif()

if(NOT GOOGLETEST_FOUND)
    set(GTEST_BUILD_COMMAND "${CMAKE_COMMAND}"
        "--build" "${GNSSSDR_BINARY_DIR}/gtest-${GNSSSDR_GTEST_LOCAL_VERSION}"
        "--config" $<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
    )
    if(CMAKE_VERSION VERSION_GREATER 3.12 AND NOT CMAKE_CROSSCOMPILING)
        # Parallel building of gtest causes problems in some environments (e.g. Macports buildings)
        set(GTEST_BUILD_COMMAND ${GTEST_BUILD_COMMAND} "--parallel 1")
    endif()
    if(CMAKE_GENERATOR STREQUAL Xcode)
        set(GTEST_BUILD_COMMAND "xcodebuild" "-configuration" $<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel> "-target" "gtest_main")
    endif()
    if(GNSSSDR_GTEST_LOCAL_VERSION VERSION_LESS 1.12.0)
        set(DEBUG_DECORATION "d")
    else()
        set(DEBUG_DECORATION "")
    endif()
    if(CMAKE_VERSION VERSION_LESS 3.2)
        ExternalProject_Add(gtest-${GNSSSDR_GTEST_LOCAL_VERSION}
            GIT_REPOSITORY https://github.com/google/googletest
            GIT_TAG v${GNSSSDR_GTEST_LOCAL_VERSION}
            SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/gtest/gtest-${GNSSSDR_GTEST_LOCAL_VERSION}
            BINARY_DIR ${GNSSSDR_BINARY_DIR}/gtest-${GNSSSDR_GTEST_LOCAL_VERSION}
            CMAKE_ARGS ${GTEST_COMPILER}
                -DINSTALL_GTEST=OFF
                -DBUILD_GMOCK=OFF
                -Dgtest_hide_internal_symbols=ON
                ${TOOLCHAIN_ARG}
                -DCMAKE_BUILD_TYPE=$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
            BUILD_COMMAND ${GTEST_BUILD_COMMAND}
            UPDATE_COMMAND ""
            PATCH_COMMAND ""
            INSTALL_COMMAND ""
        )
    else()
        set(GOOGLETEST_BUILD_BYPRODUCTS
            ${GNSSSDR_BINARY_DIR}/gtest-${GNSSSDR_GTEST_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_STATIC_LIBRARY_SUFFIX}
            ${GNSSSDR_BINARY_DIR}/gtest-${GNSSSDR_GTEST_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_STATIC_LIBRARY_SUFFIX}
        )

        if((CMAKE_BUILD_TYPE STREQUAL Debug) OR (CMAKE_BUILD_TYPE STREQUAL NoOptWithASM) OR
            (CMAKE_BUILD_TYPE STREQUAL Coverage) OR (CMAKE_BUILD_TYPE STREQUAL ASAN))  # Workaround for Ninja generator
            set(GOOGLETEST_BUILD_BYPRODUCTS
                ${GNSSSDR_BINARY_DIR}/gtest-${GNSSSDR_GTEST_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${DEBUG_DECORATION}${CMAKE_STATIC_LIBRARY_SUFFIX}
                ${GNSSSDR_BINARY_DIR}/gtest-${GNSSSDR_GTEST_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${DEBUG_DECORATION}${CMAKE_STATIC_LIBRARY_SUFFIX}
            )
        endif()
        ExternalProject_Add(gtest-${GNSSSDR_GTEST_LOCAL_VERSION}
            GIT_REPOSITORY https://github.com/google/googletest
            GIT_TAG v${GNSSSDR_GTEST_LOCAL_VERSION}
            SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/gtest/gtest-${GNSSSDR_GTEST_LOCAL_VERSION}
            BINARY_DIR ${GNSSSDR_BINARY_DIR}/gtest-${GNSSSDR_GTEST_LOCAL_VERSION}
            CMAKE_ARGS ${GTEST_COMPILER}
                -DINSTALL_GTEST=OFF
                -DBUILD_GMOCK=OFF
                -Dgtest_hide_internal_symbols=ON
                ${TOOLCHAIN_ARG}
                -DCMAKE_BUILD_TYPE=$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
            BUILD_COMMAND ${GTEST_BUILD_COMMAND}
            UPDATE_COMMAND ""
            PATCH_COMMAND ""
            BUILD_BYPRODUCTS ${GOOGLETEST_BUILD_BYPRODUCTS}
            INSTALL_COMMAND ""
        )
    endif()
    ExternalProject_Get_Property(gtest-${GNSSSDR_GTEST_LOCAL_VERSION} binary_dir)

    if(NOT TARGET GTest::GTest)
        file(MAKE_DIRECTORY ${GNSSSDR_BINARY_DIR}/thirdparty/gtest/gtest-${GNSSSDR_GTEST_LOCAL_VERSION}/googletest/include)
        add_library(GTest::GTest STATIC IMPORTED)
        add_dependencies(GTest::GTest gtest-${GNSSSDR_GTEST_LOCAL_VERSION})
        set_target_properties(GTest::GTest PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
            IMPORTED_CONFIGURATIONS "None;Debug;Release;RelWithDebInfo;MinSizeRel"
            MAP_IMPORTED_CONFIG_NOOPTWITHASM Debug
            MAP_IMPORTED_CONFIG_COVERAGE Debug
            MAP_IMPORTED_CONFIG_O2WITHASM RelWithDebInfo
            MAP_IMPORTED_CONFIG_O3WITHASM RelWithDebInfo
            MAP_IMPORTED_CONFIG_ASAN Debug
            IMPORTED_LOCATION_NONE ${binary_dir}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_STATIC_LIBRARY_SUFFIX}
            IMPORTED_LOCATION_DEBUG ${binary_dir}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_STATIC_LIBRARY_SUFFIX}
            IMPORTED_LOCATION_RELEASE ${binary_dir}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_STATIC_LIBRARY_SUFFIX}
            IMPORTED_LOCATION_RELWITHDEBINFO ${binary_dir}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_STATIC_LIBRARY_SUFFIX}
            IMPORTED_LOCATION_MINSIZEREL ${binary_dir}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_STATIC_LIBRARY_SUFFIX}
            INTERFACE_INCLUDE_DIRECTORIES ${GNSSSDR_BINARY_DIR}/thirdparty/gtest/gtest-${GNSSSDR_GTEST_LOCAL_VERSION}/googletest/include
            INTERFACE_LINK_LIBRARIES ${binary_dir}/lib/gtest$<$<CONFIG:Debug>:${DEBUG_DECORATION}>${CMAKE_STATIC_LIBRARY_SUFFIX}
        )
        if((CMAKE_GENERATOR STREQUAL Xcode) OR MSVC)
            set_target_properties(GTest::GTest PROPERTIES
                IMPORTED_LOCATION_DEBUG ${binary_dir}/lib/Debug/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${DEBUG_DECORATION}${CMAKE_STATIC_LIBRARY_SUFFIX}
                IMPORTED_LOCATION_RELEASE ${binary_dir}/lib/Release/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_STATIC_LIBRARY_SUFFIX}
                IMPORTED_LOCATION_RELWITHDEBINFO ${binary_dir}/lib/RelWithDebInfo/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_STATIC_LIBRARY_SUFFIX}
                IMPORTED_LOCATION_MINSIZEREL ${binary_dir}/lib/MinSizeRel/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_STATIC_LIBRARY_SUFFIX}
                INTERFACE_LINK_LIBRARIES ${binary_dir}/lib/$<$<CONFIG:Debug>:Debug/>$<$<CONFIG:Release>:Release/>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo/>$<$<CONFIG:MinSizeRel>:MinSizeRel/>gtest$<$<CONFIG:Debug>:${DEBUG_DECORATION}>${CMAKE_STATIC_LIBRARY_SUFFIX}
            )
        endif()
        set_target_properties(GTest::GTest PROPERTIES
            INTERFACE_LINK_LIBRARIES Threads::Threads
        )
    endif()
    if(NOT TARGET GTest::Main)
        add_library(GTest::Main STATIC IMPORTED)
        add_dependencies(GTest::Main gtest-${GNSSSDR_GTEST_LOCAL_VERSION})
        set_target_properties(GTest::Main PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
            IMPORTED_CONFIGURATIONS "None;Debug;Release;RelWithDebInfo;MinSizeRel"
            MAP_IMPORTED_CONFIG_NOOPTWITHASM Debug
            MAP_IMPORTED_CONFIG_COVERAGE Debug
            MAP_IMPORTED_CONFIG_O2WITHASM RelWithDebInfo
            MAP_IMPORTED_CONFIG_O3WITHASM RelWithDebInfo
            MAP_IMPORTED_CONFIG_ASAN Debug
            IMPORTED_LOCATION_NONE ${binary_dir}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_STATIC_LIBRARY_SUFFIX}
            IMPORTED_LOCATION_DEBUG ${binary_dir}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${DEBUG_DECORATION}${CMAKE_STATIC_LIBRARY_SUFFIX}
            IMPORTED_LOCATION_RELEASE ${binary_dir}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_STATIC_LIBRARY_SUFFIX}
            IMPORTED_LOCATION_RELWITHDEBINFO ${binary_dir}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_STATIC_LIBRARY_SUFFIX}
            IMPORTED_LOCATION_MINSIZEREL ${binary_dir}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_STATIC_LIBRARY_SUFFIX}
            INTERFACE_INCLUDE_DIRECTORIES ${GNSSSDR_BINARY_DIR}/thirdparty/gtest/gtest-${GNSSSDR_GTEST_LOCAL_VERSION}/googletest/include
            INTERFACE_LINK_LIBRARIES ${binary_dir}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main$<$<CONFIG:Debug>:${DEBUG_DECORATION}>${CMAKE_STATIC_LIBRARY_SUFFIX}
        )
        if((CMAKE_GENERATOR STREQUAL Xcode) OR MSVC)
            set_target_properties(GTest::Main PROPERTIES
                IMPORTED_LOCATION_DEBUG ${binary_dir}/lib/Debug/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${DEBUG_DECORATION}${CMAKE_STATIC_LIBRARY_SUFFIX}
                IMPORTED_LOCATION_RELEASE ${binary_dir}/lib/Release/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_STATIC_LIBRARY_SUFFIX}
                IMPORTED_LOCATION_RELWITHDEBINFO ${binary_dir}/lib/RelWithDebInfo/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_STATIC_LIBRARY_SUFFIX}
                IMPORTED_LOCATION_MINSIZEREL ${binary_dir}/lib/MinSizeRel/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_STATIC_LIBRARY_SUFFIX}
                INTERFACE_LINK_LIBRARIES ${binary_dir}/lib/$<$<CONFIG:Debug>:Debug/>$<$<CONFIG:Release>:Release/>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo/>$<$<CONFIG:MinSizeRel>:MinSizeRel/>${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main$<$<CONFIG:Debug>:${DEBUG_DECORATION}>${CMAKE_STATIC_LIBRARY_SUFFIX}
            )
        endif()
    endif()
else()
    if(GTEST_INCLUDE_DIRS)
        set(GTEST_INCLUDE_DIRECTORIES ${GTEST_INCLUDE_DIRS} ${LIBGTEST_DEV_DIR})
        add_library(gtest STATIC ${LIBGTEST_DEV_DIR}/src/gtest-all.cc)
        target_include_directories(gtest
            PUBLIC
                ${GTEST_INCLUDE_DIRECTORIES}
        )
        add_library(gtest_main STATIC ${LIBGTEST_DEV_DIR}/src/gtest_main.cc)
        target_include_directories(gtest_main
            PUBLIC
                ${GTEST_INCLUDE_DIRECTORIES}
        )
        if(NOT TARGET GTest::GTest)
            add_library(GTest::GTest STATIC IMPORTED)
            add_dependencies(GTest::GTest gtest)
            set_target_properties(GTest::GTest PROPERTIES
                IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
                IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_STATIC_LIBRARY_SUFFIX}
                INTERFACE_INCLUDE_DIRECTORIES ${GTEST_INCLUDE_DIRS}
                INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_STATIC_LIBRARY_SUFFIX}
            )
            if((CMAKE_GENERATOR STREQUAL Xcode) OR MSVC)
                set_target_properties(GTest::GTest PROPERTIES
                    IMPORTED_CONFIGURATIONS "Debug;Release;RelWithDebInfo;MinSizeRel"
                    IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/Debug/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_STATIC_LIBRARY_SUFFIX}
                    IMPORTED_LOCATION_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/Release/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_STATIC_LIBRARY_SUFFIX}
                    IMPORTED_LOCATION_RELWITHDEBINFO ${CMAKE_CURRENT_BINARY_DIR}/RelWithDebInfo/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_STATIC_LIBRARY_SUFFIX}
                    IMPORTED_LOCATION_MINSIZEREL ${CMAKE_CURRENT_BINARY_DIR}/MinSizeRel/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_STATIC_LIBRARY_SUFFIX}
                    INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_BINARY_DIR}/$<$<CONFIG:Debug>:Debug/>$<$<CONFIG:Release>:Release/>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo/>$<$<CONFIG:MinSizeRel>:MinSizeRel/>${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_STATIC_LIBRARY_SUFFIX}
                )
            endif()
            set_target_properties(GTest::GTest PROPERTIES
                INTERFACE_LINK_LIBRARIES Threads::Threads
            )
        endif()
        if(NOT TARGET GTest::Main)
            add_library(GTest::Main STATIC IMPORTED)
            add_dependencies(GTest::Main gtest_main)
            set_target_properties(GTest::Main PROPERTIES
                IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
                IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_STATIC_LIBRARY_SUFFIX}
                INTERFACE_INCLUDE_DIRECTORIES ${GTEST_INCLUDE_DIRS}
                INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_STATIC_LIBRARY_SUFFIX}
            )
            if((CMAKE_GENERATOR STREQUAL Xcode) OR MSVC)
                set_target_properties(GTest::Main PROPERTIES
                    IMPORTED_CONFIGURATIONS "Debug;Release;RelWithDebInfo;MinSizeRel"
                    IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/Debug/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_STATIC_LIBRARY_SUFFIX}
                    IMPORTED_LOCATION_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/Release/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_STATIC_LIBRARY_SUFFIX}
                    IMPORTED_LOCATION_RELWITHDEBINFO ${CMAKE_CURRENT_BINARY_DIR}/RelWithDebInfo/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_STATIC_LIBRARY_SUFFIX}
                    IMPORTED_LOCATION_MINSIZEREL ${CMAKE_CURRENT_BINARY_DIR}/MinSizeRel/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_STATIC_LIBRARY_SUFFIX}
                    INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_BINARY_DIR}/$<$<CONFIG:Debug>:Debug/>$<$<CONFIG:Release>:Release/>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo/>$<$<CONFIG:MinSizeRel>:MinSizeRel/>${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_STATIC_LIBRARY_SUFFIX}
                )
            endif()
        endif()
    endif()
endif()



################################################################################
# Definitions
################################################################################

if(GNURADIO_IS_38_OR_GREATER)
    add_definitions(-DGR_GREATER_38=1)
endif()

if(OPENSSL_FOUND)
    add_definitions(-DUSE_OPENSSL_FALLBACK=1)
endif()

if(ENABLE_OPENCL)
    add_definitions(-DOPENCL_BLOCKS_TEST=1)
endif()

if(ENABLE_CUDA)
    add_definitions(-DCUDA_BLOCKS_TEST=1)
endif()

if(ENABLE_FPGA)
    add_definitions(-DFPGA_BLOCKS_TEST=1)
endif()

if(ARMADILLO_VERSION_STRING VERSION_GREATER 8.400)
    # mvnrnd() requires 8.400
    add_definitions(-DARMADILLO_HAVE_MVNRND=1)
endif()

if(ENABLE_PACKAGING)
    add_definitions(-DEXCLUDE_TESTS_REQUIRING_BINARIES=1)
endif()

if(has_std_plus_void)
    add_definitions(-DCOMPILER_HAS_STD_PLUS_VOID=1)
endif()

find_package(GNUPLOT)
if(GNUPLOT_FOUND)
    add_definitions(-DGNUPLOT_EXECUTABLE="${GNUPLOT_EXECUTABLE}")
endif()
set_package_properties(GNUPLOT PROPERTIES
    URL "http://www.gnuplot.info"
    PURPOSE "Used to generate plots in some tests."
    TYPE OPTIONAL
)
if(GNUPLOT_FOUND AND GNUPLOT_VERSION_STRING)
    set_package_properties(GNUPLOT PROPERTIES
        DESCRIPTION "A portable command-line driven graphing utility (found: v${GNUPLOT_VERSION_STRING})"
    )
else()
    set_package_properties(GNUPLOT PROPERTIES
        DESCRIPTION "A portable command-line driven graphing utility"
    )
endif()

if(ENABLE_UNIT_TESTING_MINIMAL)
    add_definitions(-DUNIT_TESTING_MINIMAL=1)
endif()


################################################################################
# gnsstk and optional generator
################################################################################

if(ENABLE_UNIT_TESTING_EXTRA OR ENABLE_SYSTEM_TESTING_EXTRA OR ENABLE_FPGA)
    if(ENABLE_FPGA)
        set(CROSS_INSTALL_DIR "-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}")
        if(DEFINED ENV{OECORE_TARGET_SYSROOT})
            set(CROSS_INSTALL_DIR "${CROSS_INSTALL_DIR} -DBOOST_ROOT=$ENV{OECORE_TARGET_SYSROOT}/usr")
        endif()
    else()
        set(CROSS_INSTALL_DIR "")
    endif()
    find_package(GNSSSIMULATOR QUIET)
    if(GNSSSIMULATOR_FOUND OR NOT ENABLE_GNSS_SIM_INSTALL)
        if(NOT GNSSSIMULATOR_FOUND)
            message(STATUS "Warning: gnss_sim has not been found, you can install it later.")
            message(STATUS "  See https://bitbucket.org/jarribas/gnss-simulator")
            message(STATUS "  It is expected at /usr/bin/gnss_sim")
            set(SW_GENERATOR_BIN "/usr/bin/gnss_sim")
        endif()
        add_definitions(-DSW_GENERATOR_BIN="${SW_GENERATOR_BIN}")
        add_definitions(-DDEFAULT_RINEX_NAV="${CMAKE_INSTALL_PREFIX}/share/gnss-sim/brdc3540.14n")
        add_definitions(-DDEFAULT_POSITION_FILE="${CMAKE_INSTALL_PREFIX}/share/gnss-sim/circle.csv")
    else()
        set(GNSS_SIM_BUILD_COMMAND "${CMAKE_MAKE_PROGRAM}")
        if(CMAKE_GENERATOR STREQUAL Xcode)
            set(GNSS_SIM_BUILD_COMMAND "xcodebuild" "-configuration" "${CMAKE_BUILD_TYPE}" "-target" "gnss_sim")
        endif()
        if(TOOLCHAIN_ARG)
            set(TOOLCHAIN_ARG_GNSS_SIM "-DCMAKE_NO_SYSTEM_FROM_IMPORTED=ON")
        else()
            set(TOOLCHAIN_ARG_GNSS_SIM "")
        endif()
        ExternalProject_Add(gnss-sim
            GIT_REPOSITORY https://bitbucket.org/jarribas/gnss-simulator
            GIT_TAG ${GNSSSDR_GNSS_SIM_LOCAL_VERSION}
            SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/gnss-sim
            BINARY_DIR ${GNSSSDR_BINARY_DIR}/gnss-sim
            CMAKE_ARGS ${GTEST_COMPILER} ${TOOLCHAIN_ARG} ${CROSS_INSTALL_DIR} ${TOOLCHAIN_ARG_GNSS_SIM}
            BUILD_COMMAND ${GNSS_SIM_BUILD_COMMAND}
            UPDATE_COMMAND ""
            PATCH_COMMAND ""
            INSTALL_COMMAND ""
        )
        if(ENABLE_INSTALL_TESTS)
            install(PROGRAMS ${GNSSSDR_BINARY_DIR}/gnss-sim/gnss_sim DESTINATION bin)
            install(FILES ${GNSSSDR_BINARY_DIR}/thirdparty/gnss-sim/brdc3540.14n DESTINATION share/gnss-sim)
            install(FILES ${GNSSSDR_BINARY_DIR}/thirdparty/gnss-sim/circle.csv DESTINATION share/gnss-sim)
            set(SW_GENERATOR_BIN ${CMAKE_INSTALL_PREFIX}/bin/gnss_sim)
            add_definitions(-DSW_GENERATOR_BIN="${SW_GENERATOR_BIN}")
            add_definitions(-DDEFAULT_RINEX_NAV="${CMAKE_INSTALL_PREFIX}/share/gnss-sim/brdc3540.14n")
            add_definitions(-DDEFAULT_POSITION_FILE="${CMAKE_INSTALL_PREFIX}/share/gnss-sim/circle.csv")
        else()
            set(SW_GENERATOR_BIN ${GNSSSDR_BINARY_DIR}/gnss-sim/gnss_sim)
            if(CMAKE_GENERATOR STREQUAL Xcode)
                set(SW_GENERATOR_BIN ${GNSSSDR_BINARY_DIR}/gnss-sim/${CMAKE_BUILD_TYPE}/gnss_sim)
            endif()
            add_definitions(-DSW_GENERATOR_BIN="${SW_GENERATOR_BIN}")
            add_definitions(-DDEFAULT_RINEX_NAV="${GNSSSDR_BINARY_DIR}/thirdparty/gnss-sim/brdc3540.14n")
            add_definitions(-DDEFAULT_POSITION_FILE="${GNSSSDR_BINARY_DIR}/thirdparty/gnss-sim/circle.csv")
        endif()
    endif()

    ############################################################################
    # Local installation of gnsstk https://github.com/SGL-UT/gnsstk
    ############################################################################
    if(NOT GNSSTK_FOUND OR ENABLE_OWN_GNSSTK)
        set_package_properties(GNSSTK PROPERTIES
            PURPOSE "GNSSTk v${GNSSSDR_GNSSTK_LOCAL_VERSION} will be automatically downloaded and built when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'."
        )
        if("${TOOLCHAIN_ARG}" STREQUAL "")
            set(TOOLCHAIN_ARG "-DCMAKE_CXX_FLAGS=-Wno-deprecated")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated")
        endif()
        set(GNSSTK_BUILD_COMMAND "${CMAKE_MAKE_PROGRAM}")
        if(CMAKE_GENERATOR STREQUAL Xcode)
            set(GNSSTK_BUILD_COMMAND "xcodebuild" "-configuration" "${CMAKE_BUILD_TYPE}")
        endif()
        include(GNUInstallDirs)
        find_program(Patch_EXECUTABLE NAME patch PATHS ENV PATH)
        if(NOT Patch_EXECUTABLE)
            message(FATAL_ERROR "The patch command is not found. It is required to build GNSSTk. Please check your OS documentation and install the patch command.")
        endif()
        if(CMAKE_VERSION VERSION_GREATER 3.17.0)
            set(GNSSTK_PATCH_COMMAND
                cd ${GNSSSDR_BINARY_DIR}/thirdparty/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION} &&
                ${Patch_EXECUTABLE} ${GNSSSDR_BINARY_DIR}/thirdparty/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}/CMakeLists.txt < ${GNSSSDR_SOURCE_DIR}/src/tests/data/gnsstk_static14.patch &&
                ${Patch_EXECUTABLE} ${GNSSSDR_BINARY_DIR}/thirdparty/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}/core/lib/GNSSCore/ObsID.hpp < ${GNSSSDR_SOURCE_DIR}/src/tests/data/gnsstk_gcc13.patch
            )
        else()
            set(GNSSTK_PATCH_COMMAND
                cd ${GNSSSDR_BINARY_DIR}/thirdparty/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION} &&
                ${Patch_EXECUTABLE} ${GNSSSDR_BINARY_DIR}/thirdparty/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}/CMakeLists.txt < ${GNSSSDR_SOURCE_DIR}/src/tests/data/gnsstk_static13.patch &&
                ${Patch_EXECUTABLE} ${GNSSSDR_BINARY_DIR}/thirdparty/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}/core/lib/GNSSCore/ObsID.hpp < ${GNSSSDR_SOURCE_DIR}/src/tests/data/gnsstk_gcc13.patch
            )
        endif()
        # Patch only once
        if(EXISTS ${GNSSSDR_BINARY_DIR}/thirdparty/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}/CMakeLists.txt)
            set(GNSSTK_PATCH_COMMAND "")
        endif()
        if(CMAKE_VERSION VERSION_LESS 3.2)
            ExternalProject_Add(gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}
                GIT_REPOSITORY https://github.com/SGL-UT/gnsstk
                GIT_TAG v${GNSSSDR_GNSSTK_LOCAL_VERSION}
                SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}
                BINARY_DIR ${GNSSSDR_BINARY_DIR}/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}
                PATCH_COMMAND ${GNSSTK_PATCH_COMMAND}
                CMAKE_ARGS ${GTEST_COMPILER} ${TOOLCHAIN_ARG}
                    -DCMAKE_INSTALL_PREFIX=${GNSSSDR_BINARY_DIR}/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}/install
                    -DBUILD_EXT=ON
                    -DBUILD_PYTHON=OFF
                    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                    -DCMAKE_CXX_STANDARD=11
                    -DCMAKE_CXX_EXTENSIONS=ON
                    -DCMAKE_C_STANDARD=11
                    -DCMAKE_C_EXTENSIONS=ON
                BUILD_COMMAND ${GNSSTK_BUILD_COMMAND}
                UPDATE_COMMAND ""
            )
        else()
            if(CMAKE_MAKE_PROGRAM MATCHES "make")
                include(ProcessorCount)
                ProcessorCount(NUMBER_OF_PROCESSORS)
                if(NUMBER_OF_PROCESSORS GREATER 1)
                    set(GNSSTK_PARALLEL_BUILD "-j${NUMBER_OF_PROCESSORS}")
                endif()
            endif()
            if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
                set(GNSSTK_FLAGS "-DCMAKE_CXX_FLAGS:STRING=-w")  # Fix for clang in aarch64
            endif()
            if(CMAKE_VERSION VERSION_GREATER 3.17.0)
                ExternalProject_Add(gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}
                    GIT_REPOSITORY https://github.com/SGL-UT/gnsstk
                    GIT_TAG v${GNSSSDR_GNSSTK_LOCAL_VERSION}
                    GIT_SUBMODULES_RECURSE OFF
                    GIT_SUBMODULES ""
                    SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}
                    BINARY_DIR ${GNSSSDR_BINARY_DIR}/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}
                    PATCH_COMMAND ${GNSSTK_PATCH_COMMAND}
                    CMAKE_ARGS ${GTEST_COMPILER} ${TOOLCHAIN_ARG}
                        -DCMAKE_INSTALL_PREFIX=${GNSSSDR_BINARY_DIR}/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}/install
                        -DBUILD_EXT=ON
                        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                        -DCMAKE_CXX_STANDARD=17
                        -DCMAKE_CXX_EXTENSIONS=ON
                        -DCMAKE_C_STANDARD=11
                        -DCMAKE_C_EXTENSIONS=ON
                        "${GNSSTK_FLAGS}"
                    BUILD_COMMAND ${GNSSTK_BUILD_COMMAND} ${GNSSTK_PARALLEL_BUILD}
                    BUILD_BYPRODUCTS ${GNSSSDR_BINARY_DIR}/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}/install/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gnsstk${CMAKE_STATIC_LIBRARY_SUFFIX}
                    UPDATE_COMMAND ""
                )
            else()
                ExternalProject_Add(gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}
                    GIT_REPOSITORY https://github.com/SGL-UT/gnsstk
                    GIT_TAG v${GNSSSDR_GNSSTK_LOCAL_VERSION}
                    SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}
                    BINARY_DIR ${GNSSSDR_BINARY_DIR}/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}
                    PATCH_COMMAND ${GNSSTK_PATCH_COMMAND}
                    CMAKE_ARGS ${GTEST_COMPILER} ${TOOLCHAIN_ARG}
                        -DCMAKE_INSTALL_PREFIX=${GNSSSDR_BINARY_DIR}/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}/install
                        -DBUILD_EXT=ON
                        -DBUILD_PYTHON=OFF
                        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                        -DCMAKE_CXX_STANDARD=14
                        -DCMAKE_CXX_EXTENSIONS=ON
                        -DCMAKE_C_STANDARD=11
                        -DCMAKE_C_EXTENSIONS=ON
                    BUILD_COMMAND ${GNSSTK_BUILD_COMMAND} ${GNSSTK_PARALLEL_BUILD}
                    BUILD_BYPRODUCTS ${GNSSSDR_BINARY_DIR}/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}/install/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gnsstk${CMAKE_STATIC_LIBRARY_SUFFIX}
                    UPDATE_COMMAND ""
                )
            endif()
        endif()
        set(GNSSTK_INCLUDE_DIRS ${GNSSSDR_BINARY_DIR}/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}/install/include CACHE PATH "Local GNSSTK headers")
        set(GNSSTK_LIBRARY ${GNSSSDR_BINARY_DIR}/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}/install/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gnsstk${CMAKE_STATIC_LIBRARY_SUFFIX})
        set(GNSSTK_BINDIR ${GNSSSDR_BINARY_DIR}/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}/install/bin/)
        add_definitions(-DGNSSTK_BINDIR="${GNSSTK_BINDIR}")
        add_library(Gnsstk::gnsstk STATIC IMPORTED)
        add_dependencies(Gnsstk::gnsstk gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION})
        file(MAKE_DIRECTORY ${GNSSTK_INCLUDE_DIRS})
        file(MAKE_DIRECTORY ${GNSSTK_INCLUDE_DIRS}/gnsstk)
        set_target_properties(Gnsstk::gnsstk PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
            IMPORTED_LOCATION "${GNSSTK_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${GNSSTK_INCLUDE_DIRS};${GNSSTK_INCLUDE_DIRS}/gnsstk"
            INTERFACE_LINK_LIBRARIES "${GNSSTK_LIBRARY}"
        )
    else()
        set(GNSSTK_BINDIR ${GNSSTK_INCLUDE_DIR}/../bin/)
        add_definitions(-DGNSSTK_BINDIR="${GNSSTK_BINDIR}")
    endif()
endif()

if(NOT (ENABLE_PACKAGING OR ENABLE_UNIT_TESTING_MINIMAL))
    message(STATUS "Downloading some data files for testing ...")
    if(NOT EXISTS ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat)
        message(STATUS "Downloading file: Galileo_E1_ID_1_Fs_4Msps_8ms.dat")
        file(DOWNLOAD https://sourceforge.net/projects/gnss-sdr/files/data/Galileo_E1_ID_1_Fs_4Msps_8ms.dat
                ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat
            SHOW_PROGRESS
            EXPECTED_HASH MD5=d57a02d3c7361bba2e137329b66458ef
        )
    endif()
    if(NOT EXISTS ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat)
        message(STATUS "Downloading file: GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat")
        file(DOWNLOAD https://sourceforge.net/projects/gnss-sdr/files/data/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat
                ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat
            SHOW_PROGRESS
            EXPECTED_HASH MD5=f12ada80a2ad1bab061262e010643529
        )
    endif()
    if(NOT EXISTS ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat)
        message(STATUS "Downloading file: GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat")
        file(DOWNLOAD https://sourceforge.net/projects/gnss-sdr/files/data/GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat
                ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat
            SHOW_PROGRESS
            EXPECTED_HASH MD5=b98d6d82885354f168f279817de284b5
        )
    endif()
    if(NOT EXISTS ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/NT1065_GLONASS_L1_20160831_fs6625e6_if0e3_4ms.bin)
        message(STATUS "Downloading file: NT1065_GLONASS_L1_20160831_fs6625e6_if0e3_4ms.bin")
        file(DOWNLOAD https://sourceforge.net/projects/gnss-sdr/files/data/NT1065_GLONASS_L1_20160831_fs6625e6_if0e3_4ms.bin
                ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/NT1065_GLONASS_L1_20160831_fs6625e6_if0e3_4ms.bin
            SHOW_PROGRESS
            EXPECTED_HASH MD5=0e2dc212309141d236897bc0af187074
        )
    endif()
    message(STATUS "Done.")
endif()

if(ENABLE_UNIT_TESTING_EXTRA)
    add_definitions(-DEXTRA_TESTS)
    message(STATUS "Downloading some extra data files for testing ...")
    if(NOT EXISTS ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/gps_l2c_m_prn7_5msps.dat)
        message(STATUS "Downloading file: gps_l2c_m_prn7_5msps.dat")
        file(DOWNLOAD https://sourceforge.net/projects/gnss-sdr/files/data/gps_l2c_m_prn7_5msps.dat
                ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/gps_l2c_m_prn7_5msps.dat
            SHOW_PROGRESS
            EXPECTED_HASH MD5=a6fcbefe155137945d3c33c5ef7bd0f9
        )
    endif()
    if(NOT EXISTS ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/Glonass_L1_CA_SIM_Fs_62Msps_4ms.dat)
        message(STATUS "Downloading file: Glonass_L1_CA_SIM_Fs_62Msps_4ms.dat")
        file(DOWNLOAD https://sourceforge.net/projects/gnss-sdr/files/data/Glonass_L1_CA_SIM_Fs_62Msps_4ms.dat
                ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/Glonass_L1_CA_SIM_Fs_62Msps_4ms.dat
            SHOW_PROGRESS
            EXPECTED_HASH MD5=ffb72fc63c116be58d5e5ccb1daaed3a
        )
    endif()
    if(NOT EXISTS ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/BdsB1IStr01_fs25e6_if0_4ms.dat)
        message(STATUS "Downloading file: BdsB1IStr01_fs25e6_if0_4ms.dat")
        file(DOWNLOAD https://sourceforge.net/projects/gnss-sdr/files/data/BdsB1IStr01_fs25e6_if0_4ms.dat
                ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/BdsB1IStr01_fs25e6_if0_4ms.dat
            SHOW_PROGRESS
            EXPECTED_HASH MD5=5a4336dad9d80f3313a16dec4fff9233
        )
    endif()
    if(NOT EXISTS ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/BdsB3IStr01_fs50e6_if0_4ms.dat)
        message(STATUS "Downloading file: BdsB3IStr01_fs50e6_if0_4ms.dat")
        file(DOWNLOAD https://sourceforge.net/projects/gnss-sdr/files/data/BdsB3IStr01_fs50e6_if0_4ms.dat
                ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/BdsB3IStr01_fs50e6_if0_4ms.dat
            SHOW_PROGRESS
            EXPECTED_HASH MD5=066d0d8434a8bc81e161778b7c34cc07
        )
    endif()
    message(STATUS "Done.")
    if(ENABLE_INSTALL_TESTS)
        install(FILES ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/gps_l2c_m_prn7_5msps.dat DESTINATION share/gnss-sdr/signal_samples)
        install(FILES ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/Glonass_L1_CA_SIM_Fs_62Msps_4ms.dat DESTINATION share/gnss-sdr/signal_samples)
        install(FILES ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/BdsB1IStr01_fs25e6_if0_4ms.dat DESTINATION share/gnss-sdr/signal_samples)
        install(FILES ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/BdsB1IStr01_fs25e6_if0_4ms.dat DESTINATION share/gnss-sdr/signal_samples)
    endif()
endif()

if(NOT ENABLE_PACKAGING)
    if(ENABLE_INSTALL_TESTS)
        install(FILES ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat DESTINATION share/gnss-sdr/signal_samples)
        install(FILES ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat DESTINATION share/gnss-sdr/signal_samples)
        install(FILES ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat DESTINATION share/gnss-sdr/signal_samples)
        install(FILES ${GNSSSDR_BINARY_DIR}/thirdparty/signal_samples/NT1065_GLONASS_L1_20160831_fs6625e6_if0e3_4ms.bin DESTINATION share/gnss-sdr/signal_samples)
        install(FILES ${GNSSSDR_SOURCE_DIR}/src/tests/data/rtklib_test/obs_test1.xml DESTINATION share/gnss-sdr/data/rtklib_test)
        install(FILES ${GNSSSDR_SOURCE_DIR}/src/tests/data/rtklib_test/eph_GPS_L1CA_test1.xml DESTINATION share/gnss-sdr/data/rtklib_test)
        add_definitions(-DTEST_PATH="${CMAKE_INSTALL_PREFIX}/share/gnss-sdr/")
    else()
        file(COPY ${GNSSSDR_SOURCE_DIR}/src/tests/data/rtklib_test/obs_test1.xml DESTINATION ${GNSSSDR_BINARY_DIR}/thirdparty/data/rtklib_test)
        file(COPY ${GNSSSDR_SOURCE_DIR}/src/tests/data/rtklib_test/eph_GPS_L1CA_test1.xml DESTINATION ${GNSSSDR_BINARY_DIR}/thirdparty/data/rtklib_test)
        add_definitions(-DTEST_PATH="${GNSSSDR_BINARY_DIR}/thirdparty/")
    endif()
endif()



################################################################################
# Unit testing
################################################################################
if(ENABLE_UNIT_TESTING)
    if(USE_CMAKE_TARGET_SOURCES)
        add_executable(run_tests)
        target_sources(run_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test_main.cc)
    else()
        add_executable(run_tests ${CMAKE_CURRENT_SOURCE_DIR}/test_main.cc)
    endif()

    target_link_libraries(run_tests
        PRIVATE
            Boost::thread
            Armadillo::armadillo
            Gflags::gflags
            Glog::glog
            Gnuradio::runtime
            Gnuradio::blocks
            Gnuradio::filter
            Gnuradio::analog
            GTest::GTest
            GTest::Main
            Matio::matio
            Volk::volk
            Volkgnsssdr::volkgnsssdr
            signal_source_adapters
            data_type_adapters
            input_filter_adapters
            resampler_adapters
            channel_adapters
            acquisition_adapters
            tracking_adapters
            tracking_libs
            telemetry_decoder_adapters
            obs_adapters
            signal_generator_adapters
            pvt_adapters
            pvt_libs
            algorithms_libs
            core_monitor
            signal_processing_testing_lib
            system_testing_lib
            core_receiver
    )
    target_include_directories(run_tests
        INTERFACE
            ${GNSSSDR_SOURCE_DIR}/src/tests/common-files
    )
    if(GNURADIO_USES_STD_POINTERS)
        target_compile_definitions(run_tests
            PRIVATE -DGNURADIO_USES_STD_POINTERS=1
        )
    endif()
    if(PMT_USES_BOOST_ANY)
        target_compile_definitions(run_tests
            PRIVATE -DPMT_USES_BOOST_ANY=1
        )
    endif()
    if(ENABLE_UNIT_TESTING_EXTRA)
        target_link_libraries(run_tests PRIVATE Gnsstk::gnsstk)
        if(GNSSTK_OLDER_THAN_8)
            target_compile_definitions(run_tests PRIVATE -DOLD_GPSTK=1)
        endif()
        if(GNSSTK_USES_GPSTK_NAMESPACE)
            target_compile_definitions(run_tests PRIVATE -DGNSSTK_USES_GPSTK_NAMESPACE=1)
        endif()
        if(GNSSTK_OLDER_THAN_9)
            target_compile_definitions(run_tests PRIVATE -DGNSSTK_OLDER_THAN_9=1)
        endif()
    endif()
    if(ENABLE_STRIP)
        set_target_properties(run_tests PROPERTIES LINK_FLAGS "-s")
    endif()
    if(ENABLE_INSTALL_TESTS)
        if(EXISTS ${LOCAL_INSTALL_BASE_DIR}/install/run_tests)
            file(REMOVE ${LOCAL_INSTALL_BASE_DIR}/install/run_tests)
        endif()
        install(TARGETS run_tests RUNTIME DESTINATION bin COMPONENT "run_tests")
    else()
        add_custom_command(TARGET run_tests POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:run_tests>
            ${LOCAL_INSTALL_BASE_DIR}/install/$<TARGET_FILE_NAME:run_tests>
        )
    endif()
    if(ENABLE_GPERFTOOLS)
        if(GPERFTOOLS_FOUND)
            target_link_libraries(run_tests
                PRIVATE
                    Gperftools::gperftools
            )
        endif()
    endif()
    if(ENABLE_CLANG_TIDY)
        if(CLANG_TIDY_EXE)
            set_target_properties(run_tests
                PROPERTIES
                    CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
            )
        endif()
    endif()
    if(ENABLE_CUDA AND NOT CMAKE_VERSION VERSION_GREATER 3.11)
        target_link_libraries(run_tests
            PRIVATE
                ${CUDA_LIBRARIES}
        )
        target_include_directories(run_tests
            PRIVATE
                ${CUDA_INCLUDE_DIRS}
        )
    endif()
    if(USE_GENERIC_LAMBDAS)
        set(has_generic_lambdas HAS_GENERIC_LAMBDA=1)
        set(no_has_generic_lambdas HAS_GENERIC_LAMBDA=0)
        target_compile_definitions(run_tests
            PRIVATE
                "$<$<COMPILE_FEATURES:cxx_generic_lambdas>:${has_generic_lambdas}>"
                "$<$<NOT:$<COMPILE_FEATURES:cxx_generic_lambdas>>:${no_has_generic_lambdas}>"
        )
    else()
        target_compile_definitions(run_tests
            PRIVATE
                -DHAS_GENERIC_LAMBDA=0
        )
    endif()
    if(USE_BOOST_BIND_PLACEHOLDERS)
        target_compile_definitions(run_tests
            PRIVATE
                -DUSE_BOOST_BIND_PLACEHOLDERS=1
        )
    endif()
endif()

if(ENABLE_FPGA)
    set(GPS_L1_CA_DLL_PLL_TRACKING_TEST_FPGA_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/tracking/gps_l1_ca_dll_pll_tracking_test_fpga.cc
    )
    if(USE_CMAKE_TARGET_SOURCES)
        add_executable(gps_l1_ca_dll_pll_tracking_test_fpga)
        target_sources(gps_l1_ca_dll_pll_tracking_test_fpga PRIVATE ${GPS_L1_CA_DLL_PLL_TRACKING_TEST_FPGA_SOURCES})
    else()
        add_executable(gps_l1_ca_dll_pll_tracking_test_fpga ${GPS_L1_CA_DLL_PLL_TRACKING_TEST_FPGA_SOURCES})
    endif()

    if(USE_GENERIC_LAMBDAS)
        set(has_generic_lambdas HAS_GENERIC_LAMBDA=1)
        set(no_has_generic_lambdas HAS_GENERIC_LAMBDA=0)
        target_compile_definitions(gps_l1_ca_dll_pll_tracking_test_fpga
            PRIVATE
                "$<$<COMPILE_FEATURES:cxx_generic_lambdas>:${has_generic_lambdas}>"
                "$<$<NOT:$<COMPILE_FEATURES:cxx_generic_lambdas>>:${no_has_generic_lambdas}>"
        )
    else()
        target_compile_definitions(gps_l1_ca_dll_pll_tracking_test_fpga
            PRIVATE
                -DHAS_GENERIC_LAMBDA=0
        )
    endif()
    if(USE_BOOST_BIND_PLACEHOLDERS)
        target_compile_definitions(gps_l1_ca_dll_pll_tracking_test_fpga
            PRIVATE
                -DUSE_BOOST_BIND_PLACEHOLDERS=1
        )
    endif()
    if(PMT_USES_BOOST_ANY)
        target_compile_definitions(gps_l1_ca_dll_pll_tracking_test_fpga
            PRIVATE -DPMT_USES_BOOST_ANY=1
        )
    endif()

    target_link_libraries(gps_l1_ca_dll_pll_tracking_test_fpga
        PRIVATE
            Armadillo::armadillo
            Boost::thread
            Gflags::gflags
            Glog::glog
            Gnuradio::runtime
            Gnuradio::blocks
            GTest::GTest
            GTest::Main
            Matio::matio
            Volk::volk
            channel_libs
            data_type_adapters
            tracking_adapters
            signal_processing_testing_lib
            algorithms_libs
            core_receiver
    )
    target_include_directories(gps_l1_ca_dll_pll_tracking_test_fpga
        INTERFACE ${GNSSSDR_SOURCE_DIR}/src/tests/common-files
    )
    install(TARGETS gps_l1_ca_dll_pll_tracking_test_fpga
        RUNTIME DESTINATION bin
        COMPONENT "fpga-test"
    )
endif()


################################################################################
# System testing
################################################################################
function(add_system_test executable)
    ## Please call this function with variables OPT_INCLUDES_ and OPT_LIBS_
    ## already defined.
    if(NOT OPT_INCLUDES_)
        set(OPT_INCLUDES_ "")
    endif()
    if(NOT OPT_LIBS_)
        set(OPT_LIBS_ "")
    endif()
    set(SYSTEM_TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/system-tests/${executable}.cc)
    # Ensure that executable is rebuilt if it was previously built and then removed
    if(NOT EXISTS ${LOCAL_INSTALL_BASE_DIR}/install/${executable})
        execute_process(COMMAND ${CMAKE_COMMAND} -E touch ${SYSTEM_TEST_SOURCES})
    endif()
    if(USE_CMAKE_TARGET_SOURCES)
        add_executable(${executable})
        target_sources(${executable} PRIVATE ${SYSTEM_TEST_SOURCES})
    else()
        add_executable(${executable} ${SYSTEM_TEST_SOURCES})
    endif()
    target_include_directories(${executable}
        PRIVATE ${OPT_INCLUDES_}
        INTERFACE ${GNSSSDR_SOURCE_DIR}/src/tests/common-files
    )
    target_link_libraries(${executable} PRIVATE ${OPT_LIBS_} algorithms_libs)
    if(GNURADIO_USES_STD_POINTERS)
        target_compile_definitions(${executable}
            PRIVATE -DGNURADIO_USES_STD_POINTERS=1
        )
    endif()

    if(ENABLE_STRIP)
        set_target_properties(${executable} PROPERTIES LINK_FLAGS "-s")
    endif()

    if(ENABLE_INSTALL_TESTS)
        if(EXISTS ${LOCAL_INSTALL_BASE_DIR}/install/${executable})
            file(REMOVE ${LOCAL_INSTALL_BASE_DIR}/install/${executable})
        endif()
        install(TARGETS ${executable} RUNTIME DESTINATION bin COMPONENT "${executable}_test")
    else()
        add_custom_command(TARGET ${executable} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${executable}>
            ${LOCAL_INSTALL_BASE_DIR}/install/$<TARGET_FILE_NAME:${executable}>
        )
    endif()
    if(ENABLE_CLANG_TIDY)
        if(CLANG_TIDY_EXE)
            set_target_properties(${executable}
                PROPERTIES
                    CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
            )
        endif()
    endif()
endfunction()


if(ENABLE_SYSTEM_TESTING)
    set(HOST_SYSTEM "Unknown")
    if(${CMAKE_SYSTEM_NAME} MATCHES "Linux|kFreeBSD|GNU")
        set(HOST_SYSTEM "GNU/Linux ${LINUX_DISTRIBUTION} ${LINUX_VER} ${ARCHITECTURE_STRING}")
        string(REPLACE "\n" "" HOST_SYSTEM "${HOST_SYSTEM}")
        string(REPLACE "\"" "" HOST_SYSTEM "${HOST_SYSTEM}")
    endif()
    if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        set(HOST_SYSTEM "macOS")
    endif()
    add_definitions(-DHOST_SYSTEM="${HOST_SYSTEM}")

    #### TTFF
    set(OPT_LIBS_ Boost::thread Boost::date_time
        Threads::Threads Gflags::gflags Glog::glog
        Gnuradio::runtime GTest::GTest GTest::Main
        Gnuradio::blocks Gnuradio::filter
        Gnuradio::analog algorithms_libs
        core_receiver
    )
    if(NOT ENABLE_PACKAGING)
        add_system_test(ttff)
    endif()

    if(ENABLE_SYSTEM_TESTING_EXTRA)
        #### POSITION_TEST
        set(OPT_LIBS_
            algorithms_libs core_receiver core_system_parameters gnss_sdr_flags
            system_testing_lib signal_processing_testing_lib
            Boost::thread Threads::Threads Gflags::gflags Glog::glog
            GTest::GTest GTest::Main Gnuradio::runtime
            Gnuradio::blocks Gnuradio::filter
            Gnuradio::analog Matio::matio Volkgnsssdr::volkgnsssdr
        )
        add_system_test(position_test)
        if(ENABLE_GPERFTOOLS)
            if(GPERFTOOLS_FOUND)
                target_link_libraries(position_test
                    PRIVATE
                        Gperftools::gperftools
                )
            endif()
        endif()
        if(NOT GNSSSIMULATOR_FOUND OR ENABLE_GNSS_SIM_INSTALL)
            if(NOT CMAKE_CROSSCOMPILING)
                add_dependencies(position_test gnss-sim)
            endif()
        endif()
    else()
        # Avoid working with old executables if they were switched ON and then OFF
        if(EXISTS ${LOCAL_INSTALL_BASE_DIR}/install/position_test)
            file(REMOVE ${LOCAL_INSTALL_BASE_DIR}/install/position_test)
        endif()
    endif()
else()
    # Avoid working with old executables if they were switched ON and then OFF
    if(EXISTS ${LOCAL_INSTALL_BASE_DIR}/install/ttff)
        file(REMOVE ${LOCAL_INSTALL_BASE_DIR}/install/ttff)
    endif()
    if(EXISTS ${LOCAL_INSTALL_BASE_DIR}/install/position_test)
        file(REMOVE ${LOCAL_INSTALL_BASE_DIR}/install/position_test)
    endif()
endif()


#########################################################
#  Adding Tests to Ctest
#########################################################
set(CMAKE_CTEST_COMMAND ctest -V)
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})

if(NOT ENABLE_PACKAGING AND NOT ENABLE_FPGA)
    set(FLOWGRAPH_TEST_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/control-plane/gnss_flowgraph_test.cc
    )
    if(USE_CMAKE_TARGET_SOURCES)
        add_executable(flowgraph_test)
        target_sources(flowgraph_test PRIVATE ${FLOWGRAPH_TEST_SOURCES})
    else()
        add_executable(flowgraph_test ${FLOWGRAPH_TEST_SOURCES})
    endif()

    target_link_libraries(flowgraph_test
        PRIVATE
            Boost::thread
            Gflags::gflags
            Glog::glog
            Gnuradio::runtime
            GTest::GTest
            GTest::Main
            Volkgnsssdr::volkgnsssdr
            signal_source_adapters
            input_filter_adapters
            channel_adapters
            core_receiver
            algorithms_libs
    )

    target_include_directories(flowgraph_test
        PRIVATE
            ${GNSSSDR_SOURCE_DIR}/src/algorithms/libs
    )

    add_test(flowgraph_test flowgraph_test)

    set_property(TEST flowgraph_test PROPERTY TIMEOUT 30)
endif()


#########################################################
if(NOT ENABLE_PACKAGING AND NOT ENABLE_FPGA)
    set(GNSS_BLOCK_TEST_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/sources/file_signal_source_test.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/filter/fir_filter_test.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/filter/pulse_blanking_filter_test.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/filter/notch_filter_test.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/filter/notch_filter_lite_test.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/adapter/pass_through_test.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/adapter/adapter_test.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/control-plane/gnss_block_factory_test.cc
    )
    if(USE_CMAKE_TARGET_SOURCES)
        add_executable(gnss_block_test)
        target_sources(gnss_block_test PRIVATE ${GNSS_BLOCK_TEST_SOURCES})
    else()
        add_executable(gnss_block_test ${GNSS_BLOCK_TEST_SOURCES})
    endif()

    target_link_libraries(gnss_block_test
        PRIVATE
            Boost::thread
            Gflags::gflags
            Glog::glog
            Gnuradio::runtime
            Gnuradio::blocks
            Gnuradio::filter
            Gnuradio::analog
            GTest::GTest
            GTest::Main
            Volkgnsssdr::volkgnsssdr
            signal_source_adapters
            data_type_adapters
            input_filter_adapters
            channel_adapters
            core_receiver
            algorithms_libs
    )

    target_include_directories(gnss_block_test
        PRIVATE ${GNSSSDR_SOURCE_DIR}/src/algorithms/libs
        INTERFACE ${GNSSSDR_SOURCE_DIR}/src/tests/common-files
    )

    if(ENABLE_FPGA)
        target_compile_definitions(gnss_block_test PRIVATE -DENABLE_FPGA=1)
    endif()

    add_test(gnss_block_test gnss_block_test)

    set_property(TEST gnss_block_test PROPERTY TIMEOUT 60)
endif()


#########################################################
if(NOT ENABLE_PACKAGING AND NOT ENABLE_FPGA)
    set(GNURADIO_BLOCK_TEST_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/sources/unpack_2bit_samples_test.cc
    )
    if(USE_CMAKE_TARGET_SOURCES)
        add_executable(gnuradio_block_test)
        target_sources(gnuradio_block_test PRIVATE ${GNURADIO_BLOCK_TEST_SOURCES})
    else()
        add_executable(gnuradio_block_test ${GNURADIO_BLOCK_TEST_SOURCES})
    endif()

    target_link_libraries(gnuradio_block_test
        PRIVATE
            Boost::thread
            Gflags::gflags
            Glog::glog
            Gnuradio::runtime
            Gnuradio::blocks
            Gnuradio::filter
            Gnuradio::analog
            GTest::GTest
            GTest::Main
            Volkgnsssdr::volkgnsssdr
            signal_source_gr_blocks
            signal_source_libs
            core_receiver
            algorithms_libs
    )

    add_test(gnuradio_block_test gnuradio_block_test)

    set_property(TEST gnuradio_block_test PROPERTY TIMEOUT 30)
endif()


#########################################################
set(MATIO_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/arithmetic/matio_test.cc
)
if(USE_CMAKE_TARGET_SOURCES)
    add_executable(matio_test)
    target_sources(matio_test PRIVATE ${MATIO_TEST_SOURCES})
else()
    add_executable(matio_test ${MATIO_TEST_SOURCES})
endif()

target_link_libraries(matio_test
    PRIVATE
        algorithms_libs
        Gflags::gflags
        Glog::glog
        GTest::GTest
        GTest::Main
        Matio::matio
        core_receiver
)

target_include_directories(matio_test
    INTERFACE
        ${GNSSSDR_SOURCE_DIR}/src/tests/common-files
)

add_test(matio_test matio_test)

set_property(TEST matio_test PROPERTY TIMEOUT 30)


#########################################################
if(NOT ENABLE_PACKAGING AND NOT ENABLE_FPGA)
    set(ACQ_TEST_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/acquisition/gps_l1_ca_pcps_acquisition_test.cc
    )
    if(USE_CMAKE_TARGET_SOURCES)
        add_executable(acq_test)
        target_sources(acq_test PRIVATE ${ACQ_TEST_SOURCES})
    else()
        add_executable(acq_test ${ACQ_TEST_SOURCES})
    endif()

    target_link_libraries(acq_test
        PRIVATE
            Boost::thread
            Gflags::gflags
            Glog::glog
            Gnuradio::runtime
            Gnuradio::blocks
            Gnuradio::filter
            Gnuradio::analog
            GTest::GTest
            GTest::Main
            Volkgnsssdr::volkgnsssdr
            signal_generator_gr_blocks
            signal_source_gr_blocks
            signal_source_libs
            acquisition_adapters
            algorithms_libs
            signal_processing_testing_lib
            core_receiver
    )
    target_include_directories(acq_test
        INTERFACE
            ${GNSSSDR_SOURCE_DIR}/src/tests/common-files
    )
    if(PMT_USES_BOOST_ANY)
        target_compile_definitions(acq_test
            PRIVATE
                -DPMT_USES_BOOST_ANY=1
        )
    endif()
    add_test(acq_test acq_test)

    if(USE_GENERIC_LAMBDAS)
        set(has_generic_lambdas HAS_GENERIC_LAMBDA=1)
        set(no_has_generic_lambdas HAS_GENERIC_LAMBDA=0)
        target_compile_definitions(acq_test
            PRIVATE
                "$<$<COMPILE_FEATURES:cxx_generic_lambdas>:${has_generic_lambdas}>"
                "$<$<NOT:$<COMPILE_FEATURES:cxx_generic_lambdas>>:${no_has_generic_lambdas}>"
        )
    else()
        target_compile_definitions(acq_test
            PRIVATE
                -DHAS_GENERIC_LAMBDA=0
        )
    endif()
    if(USE_BOOST_BIND_PLACEHOLDERS)
        target_compile_definitions(acq_test
            PRIVATE
                -DUSE_BOOST_BIND_PLACEHOLDERS=1
        )
    endif()
    set_property(TEST acq_test PROPERTY TIMEOUT 30)
endif()


#########################################################
if(NOT ENABLE_PACKAGING AND NOT ENABLE_FPGA)
    set(NONLINEAR_SOURCES "")
    if(ARMADILLO_VERSION_STRING VERSION_GREATER 8.400)
        set(NONLINEAR_SOURCES
            ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/tracking/cubature_filter_test.cc
            # ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/tracking/unscented_filter_test.cc
        )
    endif()
    set(TRKTEST_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/tracking/galileo_e1_dll_pll_veml_tracking_test.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/tracking/tracking_loop_filter_test.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/tracking/cpu_multicorrelator_real_codes_test.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/tracking/bayesian_estimation_test.cc
        ${NONLINEAR_SOURCES}
    )
    if(USE_CMAKE_TARGET_SOURCES)
        add_executable(trk_test)
        target_sources(trk_test PRIVATE ${TRKTEST_SOURCES})
    else()
        add_executable(trk_test ${TRKTEST_SOURCES})
    endif()

    target_link_libraries(trk_test
        PRIVATE
            Boost::thread
            Gflags::gflags
            Glog::glog
            Gnuradio::runtime
            Gnuradio::blocks
            Gnuradio::filter
            Gnuradio::analog
            GTest::GTest
            GTest::Main
            Volkgnsssdr::volkgnsssdr
            signal_source_gr_blocks
            signal_source_libs
            algorithms_libs
            tracking_adapters
            signal_generator_gr_blocks
            core_receiver
    )
    if(USE_GENERIC_LAMBDAS)
        set(has_generic_lambdas HAS_GENERIC_LAMBDA=1)
        set(no_has_generic_lambdas HAS_GENERIC_LAMBDA=0)
        target_compile_definitions(trk_test
            PRIVATE
                "$<$<COMPILE_FEATURES:cxx_generic_lambdas>:${has_generic_lambdas}>"
                "$<$<NOT:$<COMPILE_FEATURES:cxx_generic_lambdas>>:${no_has_generic_lambdas}>"
        )
    else()
        target_compile_definitions(trk_test
            PRIVATE
                -DHAS_GENERIC_LAMBDA=0
        )
    endif()
    if(USE_BOOST_BIND_PLACEHOLDERS)
        target_compile_definitions(trk_test
            PRIVATE
                -DUSE_BOOST_BIND_PLACEHOLDERS=1
        )
    endif()
    if(PMT_USES_BOOST_ANY)
        target_compile_definitions(trk_test
            PRIVATE
                -DPMT_USES_BOOST_ANY=1
        )
    endif()

    add_test(trk_test trk_test)

    set_property(TEST trk_test PROPERTY TIMEOUT 30)
endif()

#########################################################

if(NOT ENABLE_PACKAGING AND NOT ENABLE_FPGA)
    set(CONTROL_THREAD_TEST_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/control-plane/control_thread_test.cc
    )
    if(USE_CMAKE_TARGET_SOURCES)
        add_executable(control_thread_test)
        target_sources(control_thread_test PRIVATE ${CONTROL_THREAD_TEST_SOURCES})
    else()
        add_executable(control_thread_test ${CONTROL_THREAD_TEST_SOURCES})
    endif()

    target_link_libraries(control_thread_test
        PRIVATE
            Boost::thread
            Gflags::gflags
            Glog::glog
            GTest::GTest
            GTest::Main
            signal_source_adapters
            algorithms_libs
            core_receiver
    )

    add_test(control_thread_test control_thread_test)

    set_property(TEST control_thread_test PROPERTY TIMEOUT 30)
endif()

#########################################################

if(ENABLE_PACKAGING)
    add_dependencies(check matio_test)
else()
    if(ENABLE_FPGA)
        add_dependencies(check matio_test)
    else()
        add_dependencies(check
            control_thread_test
            flowgraph_test
            gnss_block_test
            gnuradio_block_test
            acq_test
            trk_test
            matio_test
        )
    endif()
endif()

if(ENABLE_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()
