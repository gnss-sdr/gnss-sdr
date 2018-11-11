# Copyright (C) 2012-2018  (see AUTHORS file for a list of contributors)
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


if(OPENSSL_FOUND)
    add_definitions( -DUSE_OPENSSL_FALLBACK=1 )
endif(OPENSSL_FOUND)

set(FRONT_END_CAL_SOURCES front_end_cal.cc)
set(FRONT_END_CAL_HEADERS front_end_cal.h)


include_directories(
    ${CMAKE_SOURCE_DIR}/src/core/system_parameters
    ${CMAKE_SOURCE_DIR}/src/core/interfaces
    ${CMAKE_SOURCE_DIR}/src/core/receiver
    ${CMAKE_SOURCE_DIR}/src/core/libs
    ${CMAKE_SOURCE_DIR}/src/core/libs/supl
    ${CMAKE_SOURCE_DIR}/src/core/libs/supl/asn-rrlp
	${CMAKE_SOURCE_DIR}/src/core/libs/supl/asn-supl
    ${CMAKE_SOURCE_DIR}/src/algorithms/acquisition/adapters
    ${CMAKE_SOURCE_DIR}/src/algorithms/acquisition/gnuradio_blocks
    ${CMAKE_SOURCE_DIR}/src/algorithms/acquisition/libs
    ${CMAKE_SOURCE_DIR}/src/algorithms/libs
    ${GLOG_INCLUDE_DIRS}
    ${GFlags_INCLUDE_DIRS}
    ${GNURADIO_RUNTIME_INCLUDE_DIRS}
    ${GNURADIO_BLOCKS_INCLUDE_DIRS}
    ${ARMADILLO_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PUGIXML_INCLUDE_DIR}
    ${VOLK_GNSSSDR_INCLUDE_DIRS}
)

add_library(front_end_cal_lib ${FRONT_END_CAL_SOURCES} ${FRONT_END_CAL_HEADERS})
source_group(Headers FILES ${FRONT_END_CAL_HEADERS})

target_link_libraries(front_end_cal_lib ${MAC_LIBRARIES}
                                        ${Boost_LIBRARIES}
                                        ${GNURADIO_RUNTIME_LIBRARIES}
                                        ${GNURADIO_BLOCKS_LIBRARIES}
                                        ${GNURADIO_FFT_LIBRARIES}
                                        ${GNURADIO_FILTER_LIBRARIES}
                                        ${GFlags_LIBS}
                                        ${GLOG_LIBRARIES}
                                        ${ARMADILLO_LIBRARIES}
                                        ${VOLK_GNSSSDR_LIBRARIES} ${ORC_LIBRARIES}
                                        ${GNSS_SDR_OPTIONAL_LIBS}
                                        rx_core_lib
                                        gnss_sdr_flags
                                        gnss_rx
                                        channel_fsm
)
    
add_dependencies(front_end_cal_lib glog-${glog_RELEASE} armadillo-${armadillo_RELEASE})

add_definitions( -DGNSS_SDR_VERSION="${VERSION}" )
add_definitions( -DGNSSSDR_INSTALL_DIR="${CMAKE_INSTALL_PREFIX}" )

add_executable(front-end-cal ${CMAKE_CURRENT_SOURCE_DIR}/main.cc)

add_custom_command(TARGET front-end-cal POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:front-end-cal>
                                   ${CMAKE_SOURCE_DIR}/install/$<TARGET_FILE_NAME:front-end-cal>)


target_link_libraries(front-end-cal    ${MAC_LIBRARIES}
                                       ${Boost_LIBRARIES}
                                       ${GNURADIO_RUNTIME_LIBRARIES}
                                       ${GNURADIO_BLOCKS_LIBRARIES}
                                       ${GNURADIO_FFT_LIBRARIES}
                                       ${GNURADIO_FILTER_LIBRARIES}
                                       ${GFlags_LIBS}
                                       ${GLOG_LIBRARIES}
                                       ${ARMADILLO_LIBRARIES}
                                       ${VOLK_GNSSSDR_LIBRARIES} ${ORC_LIBRARIES}
                                       ${GNSS_SDR_OPTIONAL_LIBS}
                                       rx_core_lib
                                       gnss_rx
                                       front_end_cal_lib
)

                               
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
endif(NOT GZIP_NOTFOUND)
