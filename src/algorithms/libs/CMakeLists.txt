# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


add_subdirectory(rtklib)

set(GNSS_SPLIBS_SOURCES
    beidou_b1i_signal_replica.cc
    beidou_b3i_signal_replica.cc
    galileo_e1_signal_replica.cc
    galileo_e5_signal_replica.cc
    galileo_e6_signal_replica.cc
    glonass_l1_signal_replica.cc
    glonass_l2_signal_replica.cc
    gps_l2c_signal_replica.cc
    gps_l5_signal_replica.cc
    gnss_signal_replica.cc
    gps_sdr_signal_replica.cc
    byte_x2_to_complex_byte.cc
    complex_byte_to_float_x2.cc
    complex_float_to_complex_byte.cc
    conjugate_cc.cc
    conjugate_sc.cc
    conjugate_ic.cc
    cshort_to_float_x2.cc
    gnss_sdr_create_directory.cc
    geofunctions.cc
    item_type_helpers.cc
    pass_through.cc
    short_x2_to_cshort.cc
)

set(GNSS_SPLIBS_HEADERS
    beidou_b1i_signal_replica.h
    beidou_b3i_signal_replica.h
    galileo_e1_signal_replica.h
    galileo_e5_signal_replica.h
    galileo_e6_signal_replica.h
    glonass_l1_signal_replica.h
    glonass_l2_signal_replica.h
    gps_l2c_signal_replica.h
    gps_l5_signal_replica.h
    gnss_signal_replica.h
    gps_sdr_signal_replica.h
    byte_x2_to_complex_byte.h
    complex_byte_to_float_x2.h
    complex_float_to_complex_byte.h
    conjugate_cc.h
    conjugate_sc.h
    conjugate_ic.h
    cshort_to_float_x2.h
    gnss_sdr_create_directory.h
    gnss_sdr_make_unique.h
    gnss_circular_deque.h
    geofunctions.h
    item_type_helpers.h
    pass_through.h
    short_x2_to_cshort.h
)

if(ENABLE_OPENCL)
    set(GNSS_SPLIBS_SOURCES ${GNSS_SPLIBS_SOURCES}
        opencl/fft_execute.cc # Needs OpenCL
        opencl/fft_setup.cc # Needs OpenCL
        opencl/fft_kernelstring.cc # Needs OpenCL
    )
endif()

list(SORT GNSS_SPLIBS_HEADERS)
list(SORT GNSS_SPLIBS_SOURCES)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(algorithms_libs STATIC)
    target_sources(algorithms_libs
        PRIVATE
            ${GNSS_SPLIBS_SOURCES}
        PUBLIC
            ${GNSS_SPLIBS_HEADERS}
    )
else()
    source_group(Headers FILES ${GNSS_SPLIBS_HEADERS})
    add_library(algorithms_libs ${GNSS_SPLIBS_SOURCES} ${GNSS_SPLIBS_HEADERS})
endif()

if(${FILESYSTEM_FOUND})
    target_compile_definitions(algorithms_libs PRIVATE -DHAS_STD_FILESYSTEM=1)
    if(${find_experimental})
        target_compile_definitions(algorithms_libs PRIVATE -DHAS_STD_FILESYSTEM_EXPERIMENTAL=1)
    endif()
    target_link_libraries(algorithms_libs PRIVATE std::filesystem)
else()
    target_link_libraries(algorithms_libs PRIVATE Boost::filesystem Boost::system)
endif()

if(NOT has_span)
    target_include_directories(algorithms_libs
        PUBLIC
            ${CMAKE_SOURCE_DIR}/src/algorithms/libs/gsl/include
    )
endif()

target_link_libraries(algorithms_libs
    PUBLIC
        Armadillo::armadillo
        Boost::headers
        Gnuradio::runtime
        Gnuradio::blocks
    PRIVATE
        core_system_parameters
        Volk::volk
        Volkgnsssdr::volkgnsssdr
        Gflags::gflags
        Glog::glog
)

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(algorithms_libs
        PUBLIC -DGNURADIO_USES_STD_POINTERS=1
    )
endif()

if(has_span)
    target_compile_definitions(algorithms_libs
        PUBLIC -DHAS_STD_SPAN=1
    )
endif()

if(ENABLE_OPENCL)
    target_link_libraries(algorithms_libs PUBLIC OpenCL::OpenCL)
    target_include_directories(algorithms_libs PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/opencl
    )
endif()

if(ENABLE_ARMA_NO_DEBUG)
    target_compile_definitions(algorithms_libs
        PUBLIC -DARMA_NO_BOUND_CHECKING=1
    )
endif()

target_include_directories(algorithms_libs
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
)

target_compile_definitions(algorithms_libs
    PUBLIC -DGNSSSDR_INSTALL_DIR="${CMAKE_INSTALL_PREFIX}"
)

if(DO_NOT_USE_LAMBDAS)
    target_compile_definitions(algorithms_libs
        PRIVATE -DDO_NOT_USE_LAMBDAS=1
    )
endif()

set_property(TARGET algorithms_libs
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(algorithms_libs
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

###############################################################################

if(USE_CMAKE_TARGET_SOURCES)
    add_library(gnss_sdr_flags STATIC)
    target_sources(gnss_sdr_flags
        PRIVATE
            gnss_sdr_flags.cc
        PUBLIC
            gnss_sdr_flags.h
    )
else()
    source_group(Headers FILES gnss_sdr_flags.h)
    add_library(gnss_sdr_flags gnss_sdr_flags.cc gnss_sdr_flags.h)
endif()

if(${FILESYSTEM_FOUND})
    target_compile_definitions(gnss_sdr_flags PRIVATE -DHAS_STD_FILESYSTEM=1)
    if(${find_experimental})
        target_compile_definitions(gnss_sdr_flags PRIVATE -DHAS_STD_FILESYSTEM_EXPERIMENTAL=1)
    endif()
    target_link_libraries(gnss_sdr_flags PRIVATE std::filesystem)
else()
    target_link_libraries(gnss_sdr_flags PRIVATE Boost::filesystem)
endif()

target_link_libraries(gnss_sdr_flags
    PUBLIC
        Gflags::gflags
)

if(${GFLAGS_GREATER_20})
    target_compile_definitions(gnss_sdr_flags PRIVATE -DGFLAGS_GREATER_2_0=1)
endif()

target_compile_definitions(gnss_sdr_flags
    PRIVATE -DGNSSSDR_INSTALL_DIR="${CMAKE_INSTALL_PREFIX}"
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(gnss_sdr_flags
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET gnss_sdr_flags
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
