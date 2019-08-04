# Copyright (C) 2012-2019  (see AUTHORS file for a list of contributors)
#
# This file is part of GNSS-SDR.
#
# GNSS-SDR is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# GNSS-SDR is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
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
        Boost::boost
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
        Boost::boost
    PRIVATE
        Gflags::gflags
        Glog::glog
)

target_compile_definitions(front-end-cal
    PUBLIC -DGNSS_SDR_VERSION="${VERSION}"
    PUBLIC -DGNSSSDR_INSTALL_DIR="${CMAKE_INSTALL_PREFIX}"
)

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
