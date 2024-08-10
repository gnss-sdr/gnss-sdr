# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


find_package(benchmark QUIET)

if(NOT benchmark_FOUND)
    set(BENCHMARK_COMPILER -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER})
    set(BENCHMARK_TOOLCHAIN_ARG "")
    if(DEFINED ENV{OECORE_TARGET_SYSROOT})
        set(BENCHMARK_COMPILER "")
        set(BENCHMARK_TOOLCHAIN_ARG "-DCMAKE_TOOLCHAIN_FILE=${GNSSSDR_SOURCE_DIR}/cmake/Toolchains/oe-sdk_cross.cmake")
    else()
        if(CMAKE_TOOLCHAIN_FILE)
            set(BENCHMARK_TOOLCHAIN_ARG "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
        endif()
        if(ENABLE_PACKAGING)
            set(BENCHMARK_TOOLCHAIN_ARG "-DCMAKE_VERBOSE_MAKEFILE=ON")
        endif()
    endif()
    set(BENCHMARK_BUILD_COMMAND "${CMAKE_COMMAND}"
        "--build" "${GNSSSDR_BINARY_DIR}/benchmark-${GNSSSDR_BENCHMARK_LOCAL_VERSION}"
        "--config" $<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
    )
    if(CMAKE_GENERATOR STREQUAL Xcode)
        set(VOLK_GNSSSDR_BUILD_COMMAND "xcodebuild"
            "-configuration" $<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
        )
    endif()

    if((CMAKE_VERSION VERSION_GREATER 3.12.0) AND NOT (CMAKE_GENERATOR STREQUAL Xcode))
        set(BENCHMARK_PARALLEL_BUILD "--parallel 2")
    endif()

    ExternalProject_Add(google-benchmark-${GNSSSDR_BENCHMARK_LOCAL_VERSION}
        GIT_REPOSITORY https://github.com/google/benchmark
        GIT_TAG v${GNSSSDR_BENCHMARK_LOCAL_VERSION}
        SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/benchmark
        BINARY_DIR ${GNSSSDR_BINARY_DIR}/benchmark-${GNSSSDR_BENCHMARK_LOCAL_VERSION}
        CMAKE_ARGS ${BENCHMARK_COMPILER}
            -DBENCHMARK_ENABLE_TESTING=OFF
            -DBENCHMARK_USE_BUNDLED_GTEST=OFF
            ${BENCHMARK_TOOLCHAIN_ARG}
            -DCMAKE_BUILD_TYPE=$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
        BUILD_COMMAND "${BENCHMARK_BUILD_COMMAND} ${BENCHMARK_PARALLEL_BUILD}"
        UPDATE_COMMAND ""
        PATCH_COMMAND ""
        BUILD_BYPRODUCTS ${GNSSSDR_BINARY_DIR}/benchmark-${GNSSSDR_BENCHMARK_LOCAL_VERSION}/src/${CMAKE_FIND_LIBRARY_PREFIXES}benchmark${CMAKE_STATIC_LIBRARY_SUFFIX}
        INSTALL_COMMAND ""
    )

    add_library(benchmark::benchmark STATIC IMPORTED)
    add_dependencies(benchmark::benchmark google-benchmark-${GNSSSDR_BENCHMARK_LOCAL_VERSION})
    file(MAKE_DIRECTORY ${GNSSSDR_BINARY_DIR}/thirdparty/benchmark/include)
    file(MAKE_DIRECTORY ${GNSSSDR_BINARY_DIR}/benchmark-${GNSSSDR_BENCHMARK_LOCAL_VERSION}/include)
    set_target_properties(benchmark::benchmark PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_CONFIGURATIONS "None;Debug;Release;RelWithDebInfo;MinSizeRel"
        MAP_IMPORTED_CONFIG_NOOPTWITHASM Debug
        MAP_IMPORTED_CONFIG_COVERAGE Debug
        MAP_IMPORTED_CONFIG_O2WITHASM RelWithDebInfo
        MAP_IMPORTED_CONFIG_O3WITHASM RelWithDebInfo
        MAP_IMPORTED_CONFIG_ASAN Debug
        IMPORTED_LOCATION_NONE ${GNSSSDR_BINARY_DIR}/benchmark-${GNSSSDR_BENCHMARK_LOCAL_VERSION}/src/${CMAKE_FIND_LIBRARY_PREFIXES}benchmark${CMAKE_STATIC_LIBRARY_SUFFIX}
        IMPORTED_LOCATION_DEBUG ${GNSSSDR_BINARY_DIR}/benchmark-${GNSSSDR_BENCHMARK_LOCAL_VERSION}/src/${CMAKE_FIND_LIBRARY_PREFIXES}benchmark${CMAKE_STATIC_LIBRARY_SUFFIX}
        IMPORTED_LOCATION_RELEASE ${GNSSSDR_BINARY_DIR}/benchmark-${GNSSSDR_BENCHMARK_LOCAL_VERSION}/src/${CMAKE_FIND_LIBRARY_PREFIXES}benchmark${CMAKE_STATIC_LIBRARY_SUFFIX}
        IMPORTED_LOCATION_RELWITHDEBINFO ${GNSSSDR_BINARY_DIR}/benchmark-${GNSSSDR_BENCHMARK_LOCAL_VERSION}/src/${CMAKE_FIND_LIBRARY_PREFIXES}benchmark${CMAKE_STATIC_LIBRARY_SUFFIX}
        IMPORTED_LOCATION_MINSIZEREL ${GNSSSDR_BINARY_DIR}/benchmark-${GNSSSDR_BENCHMARK_LOCAL_VERSION}/src/${CMAKE_FIND_LIBRARY_PREFIXES}benchmark${CMAKE_STATIC_LIBRARY_SUFFIX}
        INTERFACE_INCLUDE_DIRECTORIES "${GNSSSDR_BINARY_DIR}/thirdparty/benchmark/include"
        INTERFACE_LINK_LIBRARIES "${GNSSSDR_BINARY_DIR}/benchmark-${GNSSSDR_BENCHMARK_LOCAL_VERSION}/src/${CMAKE_FIND_LIBRARY_PREFIXES}benchmark${CMAKE_STATIC_LIBRARY_SUFFIX};${GNSSSDR_BINARY_DIR}/benchmark-${GNSSSDR_BENCHMARK_LOCAL_VERSION}/src/${CMAKE_FIND_LIBRARY_PREFIXES}benchmark_main${CMAKE_STATIC_LIBRARY_SUFFIX}"
    )
    if((CMAKE_GENERATOR STREQUAL Xcode) OR MSVC)
        set_target_properties(benchmark::benchmark PROPERTIES
            IMPORTED_LOCATION_DEBUG ${GNSSSDR_BINARY_DIR}/benchmark-${GNSSSDR_BENCHMARK_LOCAL_VERSION}/src/Debug/${CMAKE_FIND_LIBRARY_PREFIXES}benchmark${CMAKE_STATIC_LIBRARY_SUFFIX}
            IMPORTED_LOCATION_RELEASE ${GNSSSDR_BINARY_DIR}/benchmark-${GNSSSDR_BENCHMARK_LOCAL_VERSION}/src/Release/${CMAKE_FIND_LIBRARY_PREFIXES}benchmark${CMAKE_STATIC_LIBRARY_SUFFIX}
            IMPORTED_LOCATION_RELWITHDEBINFO ${GNSSSDR_BINARY_DIR}/benchmark-${GNSSSDR_BENCHMARK_LOCAL_VERSION}/src/RelWithDebInfo/${CMAKE_FIND_LIBRARY_PREFIXES}benchmark${CMAKE_STATIC_LIBRARY_SUFFIX}
            IMPORTED_LOCATION_MINSIZEREL ${GNSSSDR_BINARY_DIR}/benchmark-${GNSSSDR_BENCHMARK_LOCAL_VERSION}/src/MinSizeRel/${CMAKE_FIND_LIBRARY_PREFIXES}benchmark${CMAKE_STATIC_LIBRARY_SUFFIX}
            INTERFACE_LINK_LIBRARIES "${GNSSSDR_BINARY_DIR}/benchmark-${GNSSSDR_BENCHMARK_LOCAL_VERSION}/src/$<$<CONFIG:Debug>:Debug/>$<$<CONFIG:Release>:Release/>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo/>$<$<CONFIG:MinSizeRel>:MinSizeRel/>benchmark${CMAKE_STATIC_LIBRARY_SUFFIX};
                ${GNSSSDR_BINARY_DIR}/benchmark-${GNSSSDR_BENCHMARK_LOCAL_VERSION}/src/$<$<CONFIG:Debug>:Debug/>$<$<CONFIG:Release>:Release/>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo/>$<$<CONFIG:MinSizeRel>:MinSizeRel/>benchmark_main${CMAKE_STATIC_LIBRARY_SUFFIX}"
        )
    endif()
    set_target_properties(benchmark::benchmark PROPERTIES
        INTERFACE_LINK_LIBRARIES Threads::Threads
    )
endif()

macro(add_benchmark)
    # First argument is the benchmark name, the rest are dependencies
    add_executable(${ARGV0} ${ARGV0}.cc)
    target_link_libraries(${ARGV0} PRIVATE benchmark::benchmark)
    if(${ARGC} GREATER 1)
        set(list_dependencies "${ARGN}")
        list(REMOVE_AT list_dependencies 0)
        foreach(dependency IN LISTS list_dependencies)
            target_link_libraries(${ARGV0} PRIVATE ${dependency})
        endforeach()
    endif()
    add_custom_command(TARGET ${ARGV0} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${ARGV0}>
            ${LOCAL_INSTALL_BASE_DIR}/install/$<TARGET_FILE_NAME:${ARGV0}>
    )
endmacro()

set(EXTRA_BENCHMARK_DEPENDENCIES "")
if(ENABLE_GLOG_AND_GFLAGS)
    set(EXTRA_BENCHMARK_DEPENDENCIES "Gflags::gflags;Glog::glog")
endif()

add_benchmark(benchmark_atan2 Gnuradio::runtime)
add_benchmark(benchmark_copy)
add_benchmark(benchmark_crypto core_libs Boost::headers ${EXTRA_BENCHMARK_DEPENDENCIES})
add_benchmark(benchmark_detector core_system_parameters ${EXTRA_BENCHMARK_DEPENDENCIES})
add_benchmark(benchmark_preamble core_system_parameters ${EXTRA_BENCHMARK_DEPENDENCIES})
add_benchmark(benchmark_reed_solomon core_system_parameters ${EXTRA_BENCHMARK_DEPENDENCIES})

if(has_std_plus_void)
    target_compile_definitions(benchmark_detector PRIVATE -DCOMPILER_HAS_STD_PLUS_VOID=1)
endif()
if(has_transform_reduce)
    target_compile_definitions(benchmark_detector PRIVATE -DCOMPILER_HAS_STD_TRANSFORM_REDUCE=1)
endif()
if(has_transform_reduce_with_execution_policy)
    target_compile_definitions(benchmark_detector PRIVATE -DCOMPILER_HAS_STD_TRANSFORM_REDUCE_WITH_POLICY=1)
endif()
