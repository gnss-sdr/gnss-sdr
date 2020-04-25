# Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#


set(FRONT_END_CAL_SOURCES front_end_cal.cc)
set(FRONT_END_CAL_HEADERS front_end_cal.h)

add_library(front_end_cal_lib ${FRONT_END_CAL_SOURCES} ${FRONT_END_CAL_HEADERS})
source_group(Headers FILES ${FRONT_END_CAL_HEADERS})

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

add_executable(front-end-cal ${CMAKE_CURRENT_SOURCE_DIR}/main.cc)

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
    PUBLIC
        core_libs
        core_receiver
        front_end_cal_lib
        gnss_sdr_flags
        Boost::headers
    PRIVATE
        Gflags::gflags
        Glog::glog
)

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(front-end-cal
        PUBLIC -DGNURADIO_USES_STD_POINTERS=1
    )
endif()

target_compile_definitions(front-end-cal
    PUBLIC -DGNSS_SDR_VERSION="${VERSION}"
    PUBLIC -DGNSSSDR_INSTALL_DIR="${CMAKE_INSTALL_PREFIX}"
)

if(CMAKE_VERSION VERSION_GREATER 3.1)
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
