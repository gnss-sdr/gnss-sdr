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

set(GNSS_SDR_OPTIONAL_LIBS "")
set(GNSS_SDR_OPTIONAL_HEADERS "")

if(ENABLE_GPERFTOOLS)
    if(GPERFTOOLS_FOUND)
        set(GNSS_SDR_OPTIONAL_LIBS "${GNSS_SDR_OPTIONAL_LIBS};${GPERFTOOLS_PROFILER};${GPERFTOOLS_TCMALLOC}")
        set(GNSS_SDR_OPTIONAL_HEADERS "${GNSS_SDR_OPTIONAL_HEADERS};${GPERFTOOLS_INCLUDE_DIR}")
    endif()
endif()

if(ENABLE_UHD AND GNURADIO_UHD_LIBRARIES_gnuradio-uhd)
    set(GNSS_SDR_OPTIONAL_LIBS ${GNSS_SDR_OPTIONAL_LIBS} ${UHD_LIBRARIES} ${GNURADIO_UHD_LIBRARIES})
    set(GNSS_SDR_OPTIONAL_HEADERS ${GNSS_SDR_OPTIONAL_HEADERS} ${UHD_INCLUDE_DIRS})
endif()

if(OPENSSL_FOUND)
    add_definitions(-DUSE_OPENSSL_FALLBACK=1)
endif()

if(ENABLE_CUDA)
    add_definitions(-DCUDA_GPU_ACCEL=1)
    set(GNSS_SDR_OPTIONAL_LIBS ${GNSS_SDR_OPTIONAL_LIBS} ${CUDA_LIBRARIES})
    set(GNSS_SDR_OPTIONAL_HEADERS ${GNSS_SDR_OPTIONAL_HEADERS} ${CUDA_INCLUDE_DIRS})
endif()

if(ORC_FOUND)
    set(GNSS_SDR_OPTIONAL_LIBS ${GNSS_SDR_OPTIONAL_LIBS} ${ORC_LIBRARIES})
    set(GNSS_SDR_OPTIONAL_HEADERS ${GNSS_SDR_OPTIONAL_HEADERS} ${ORC_INCLUDE_DIRS})
endif()

if(OS_IS_MACOSX)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        set(MAC_LIBRARIES "-framework Accelerate -lc++")
    endif()
endif()

add_executable(gnss-sdr ${CMAKE_CURRENT_SOURCE_DIR}/main.cc)

target_include_directories(gnss-sdr PRIVATE
    ${GNSS_SDR_OPTIONAL_HEADERS}
    ${OPT_GNSSSDR_INCLUDE_DIRS}
)

target_compile_definitions(gnss-sdr PUBLIC -DGNSS_SDR_VERSION="${VERSION}")

add_custom_command(TARGET gnss-sdr
    POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:gnss-sdr>
        ${CMAKE_SOURCE_DIR}/install/$<TARGET_FILE_NAME:gnss-sdr>
)

target_link_libraries(gnss-sdr
    PUBLIC
        Boost::filesystem
        Boost::system
        gnss_sdr_flags
        gnss_system_parameters
        gnss_sp_libs
        gnss_rx
        ${MAC_LIBRARIES}
        ${THREAD_LIBRARIES}
        ${GNSS_SDR_OPTIONAL_LIBS}
    PRIVATE
        rtklib_lib
        Gflags::gflags
        Glog::glog
        Armadillo::armadillo
        Volkgnsssdr::volkgnsssdr
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
