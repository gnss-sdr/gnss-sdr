# Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#


if(CMAKE_VERSION VERSION_GREATER 3.13)
    add_library(front_end_cal_lib STATIC)
    target_sources(front_end_cal_lib
        PRIVATE
            front_end_cal.cc
        PUBLIC
            front_end_cal.h
    )
else()
    source_group(Headers FILES front_end_cal.h)
    add_library(front_end_cal_lib front_end_cal.cc front_end_cal.h)
endif()

target_link_libraries(front_end_cal_lib
    PUBLIC
        Armadillo::armadillo
        Threads::Threads
        acquisition_adapters
        gnss_sdr_flags
        channel_libs
        algorithms_libs
        core_receiver
        core_libs
    PRIVATE
        Boost::headers
        Gflags::gflags
        Glog::glog
        Gnuradio::blocks
        Gnuradio::runtime
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(front_end_cal_lib
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

if(CMAKE_VERSION VERSION_GREATER 3.13)
    add_executable(front-end-cal)
    target_sources(front-end-cal PRIVATE main.cc)
else()
    add_executable(front-end-cal main.cc)
endif()

if(${FILESYSTEM_FOUND})
    target_compile_definitions(front-end-cal PRIVATE -DHAS_STD_FILESYSTEM=1)
    if(${find_experimental})
        target_compile_definitions(front-end-cal PRIVATE -DHAS_STD_FILESYSTEM_EXPERIMENTAL=1)
    endif()
    target_link_libraries(front-end-cal PRIVATE std::filesystem)
else()
    target_link_libraries(front-end-cal PRIVATE Boost::filesystem Boost::system)
endif()

target_link_libraries(front-end-cal
    PRIVATE
        core_receiver
        front_end_cal_lib
        gnss_sdr_flags
        Boost::headers
        Glog::glog
)

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(front-end-cal
        PRIVATE -DGNURADIO_USES_STD_POINTERS=1
    )
endif()

target_compile_definitions(front-end-cal
    PRIVATE -DGNSS_SDR_VERSION="${VERSION}"
    PRIVATE -DGNSSSDR_INSTALL_DIR="${CMAKE_INSTALL_PREFIX}"
)

if(CMAKE_VERSION VERSION_GREATER 3.1 AND NOT ((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7.0)))
    set(has_generic_lambdas HAS_GENERIC_LAMBDA=1)
    set(no_has_generic_lambdas HAS_GENERIC_LAMBDA=0)
    target_compile_definitions(front-end-cal
        PRIVATE
            "$<$<COMPILE_FEATURES:cxx_generic_lambdas>:${has_generic_lambdas}>"
            "$<$<NOT:$<COMPILE_FEATURES:cxx_generic_lambdas>>:${no_has_generic_lambdas}>"
    )
else()
    target_compile_definitions(front-end-cal
        PRIVATE
            -DHAS_GENERIC_LAMBDA=0
    )
endif()

if(Boost_VERSION_STRING VERSION_GREATER 1.72.99)
    target_compile_definitions(front-end-cal
        PRIVATE
            -DBOOST_173_OR_GREATER=1
    )
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(front-end-cal
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

add_custom_command(TARGET front-end-cal POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:front-end-cal>
        ${CMAKE_SOURCE_DIR}/install/$<TARGET_FILE_NAME:front-end-cal>)

install(TARGETS front-end-cal
    RUNTIME DESTINATION bin
    COMPONENT "front-end-cal"
)

find_program(GZIP gzip
    /bin
    /usr/bin
    /usr/local/bin
    /opt/local/bin
    /sbin
)

if(NOT GZIP_NOTFOUND)
    execute_process(COMMAND gzip -9 -c ${CMAKE_SOURCE_DIR}/docs/manpage/front-end-cal-manpage
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR} OUTPUT_FILE "${CMAKE_BINARY_DIR}/front-end-cal.1.gz")
    install(FILES ${CMAKE_BINARY_DIR}/front-end-cal.1.gz DESTINATION share/man/man1)
endif()
