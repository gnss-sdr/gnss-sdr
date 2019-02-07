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


add_executable(gnss-sdr ${CMAKE_CURRENT_SOURCE_DIR}/main.cc)

target_link_libraries(gnss-sdr
    PUBLIC
        Boost::filesystem
        Boost::system
        Gflags::gflags
        Glog::glog
        gnss_rx
        gnss_sdr_flags
        Threads::Threads
)

# Disable internal logging
if(NOT ENABLE_LOG)
    target_compile_definitions(gnss-sdr PRIVATE -DGOOGLE_STRIP_LOG=1)
endif()

target_compile_definitions(gnss-sdr PRIVATE -DGNSS_SDR_VERSION="${VERSION}")

if(ENABLE_CUDA)
    target_link_libraries(gnss-sdr
        PUBLIC
            ${CUDA_LIBRARIES}
    )
    target_include_directories(gnss-sdr
        PUBLIC
            ${CUDA_INCLUDE_DIRS}
    )
    target_compile_definitions(gnss-sdr PRIVATE -DCUDA_GPU_ACCEL=1)
endif()

if(ENABLE_GPERFTOOLS)
    if(GPERFTOOLS_FOUND)
        target_link_libraries(gnss-sdr
            PUBLIC
                ${GPERFTOOLS_PROFILER}
                ${GPERFTOOLS_TCMALLOC}
        )
        target_include_directories(gnss-sdr
            PRIVATE
                ${GPERFTOOLS_INCLUDE_DIR}
        )
    endif()
endif()

if(OS_IS_MACOSX)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_link_libraries(gnss-sdr
            PUBLIC
                "-lc++"
        )
    endif()
endif()

add_custom_command(TARGET gnss-sdr
    POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:gnss-sdr>
        ${CMAKE_SOURCE_DIR}/install/$<TARGET_FILE_NAME:gnss-sdr>
)

install(TARGETS gnss-sdr
    RUNTIME DESTINATION bin
    COMPONENT "gnss-sdr")

install(DIRECTORY ${CMAKE_SOURCE_DIR}/conf DESTINATION share/gnss-sdr
    FILES_MATCHING PATTERN "*.conf")

install(FILES ${CMAKE_SOURCE_DIR}/conf/gnss-sdr.conf DESTINATION share/gnss-sdr/conf
    RENAME default.conf)

if(NOT VOLKGNSSSDR_FOUND)
    install(PROGRAMS ${CMAKE_BINARY_DIR}/volk_gnsssdr_module/build/apps/volk_gnsssdr_profile DESTINATION bin COMPONENT "volk_gnsssdr")
    install(PROGRAMS ${CMAKE_BINARY_DIR}/volk_gnsssdr_module/build/apps/volk_gnsssdr-config-info DESTINATION bin COMPONENT "volk_gnsssdr")
endif()

find_program(GZIP
    gzip
    /bin
    /usr/bin
    /usr/local/bin
    /opt/local/bin
    /sbin
)

if(NOT GZIP_NOTFOUND)
    execute_process(COMMAND gzip -9 -c ${CMAKE_SOURCE_DIR}/docs/manpage/gnss-sdr-manpage
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR} OUTPUT_FILE "${CMAKE_BINARY_DIR}/gnss-sdr.1.gz")

    install(FILES ${CMAKE_BINARY_DIR}/gnss-sdr.1.gz DESTINATION share/man/man1)

    execute_process(COMMAND gzip -9 -c ${CMAKE_SOURCE_DIR}/docs/changelog
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR} OUTPUT_FILE "${CMAKE_BINARY_DIR}/changelog.gz")

    install(FILES ${CMAKE_BINARY_DIR}/changelog.gz DESTINATION share/doc/gnss-sdr)

    if(NOT VOLKGNSSSDR_FOUND)
        execute_process(COMMAND gzip -9 -c ${CMAKE_SOURCE_DIR}/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/cmake/Packaging/volk_gnsssdr_profile-manpage
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR} OUTPUT_FILE "${CMAKE_BINARY_DIR}/volk_gnsssdr_profile.1.gz")
        execute_process(COMMAND gzip -9 -c ${CMAKE_SOURCE_DIR}/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/cmake/Packaging/volk_gnsssdr-config-info-manpage
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR} OUTPUT_FILE "${CMAKE_BINARY_DIR}/volk_gnsssdr-config-info.1.gz")
        install(FILES ${CMAKE_BINARY_DIR}/volk_gnsssdr_profile.1.gz DESTINATION share/man/man1)
        install(FILES ${CMAKE_BINARY_DIR}/volk_gnsssdr-config-info.1.gz DESTINATION share/man/man1)
    endif()
endif()
