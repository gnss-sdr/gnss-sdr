# Copyright (C) 2012-2014  (see AUTHORS file for a list of contributors)
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
# along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
#

set(GNSS_SDR_OPTIONAL_LIBS "")
set(GNSS_SDR_OPTIONAL_HEADERS "")

if(ENABLE_GPERFTOOLS)
    if(GPERFTOOLS_FOUND)
        #set(GNSS_SDR_OPTIONAL_LIBS "${GNSS_SDR_OPTIONAL_LIBS};${GPERFTOOLS_LIBRARIES}")
        set(GNSS_SDR_OPTIONAL_LIBS "${GNSS_SDR_OPTIONAL_LIBS};${GPERFTOOLS_PROFILER};${GPERFTOOLS_TCMALLOC}")
        set(GNSS_SDR_OPTIONAL_HEADERS "${GNSS_SDR_OPTIONAL_HEADERS};${GPERFTOOLS_INCLUDE_DIR}")
    endif(GPERFTOOLS_FOUND)
endif(ENABLE_GPERFTOOLS)

include_directories(
     ${CMAKE_SOURCE_DIR}/src/core/system_parameters
     ${CMAKE_SOURCE_DIR}/src/core/interfaces
     ${CMAKE_SOURCE_DIR}/src/core/receiver
     ${CMAKE_SOURCE_DIR}/src/core/libs
     ${CMAKE_SOURCE_DIR}/src/core/libs/supl
     ${CMAKE_SOURCE_DIR}/src/core/libs/supl/asn-rrlp
     ${CMAKE_SOURCE_DIR}/src/core/libs/supl/asn-supl
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${ARMADILLO_INCLUDE_DIRS}  
     ${Boost_INCLUDE_DIRS}
     ${GNURADIO_RUNTIME_INCLUDE_DIRS}
     ${GNSS_SDR_OPTIONAL_HEADERS}
     ${VOLK_GNSSSDR_INCLUDE_DIRS}
)

add_definitions( -DGNSS_SDR_VERSION="${VERSION}" )

if(OS_IS_MACOSX)
     if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
         set(MAC_LIBRARIES "-stdlib=libc++ -std=c++11 -framework Accelerate")
     endif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
endif(OS_IS_MACOSX)


add_executable(gnss-sdr ${CMAKE_CURRENT_SOURCE_DIR}/main.cc)

add_custom_command(TARGET gnss-sdr POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:gnss-sdr>
                                   ${CMAKE_SOURCE_DIR}/install/$<TARGET_FILE_NAME:gnss-sdr>
                  )

target_link_libraries(gnss-sdr ${MAC_LIBRARIES} 
                               ${Boost_LIBRARIES}
                               ${GNURADIO_RUNTIME_LIBRARIES} 
                               ${GNURADIO_BLOCKS_LIBRARIES} 
                               ${GNURADIO_FFT_LIBRARIES} 
                               ${GNURADIO_FILTER_LIBRARIES} 
                               ${GFlags_LIBS} 
                               ${GLOG_LIBRARIES} 
                               ${ARMADILLO_LIBRARIES} 
                               ${UHD_LIBRARIES}
                               gnss_sp_libs 
                               gnss_rx
                               ${VOLK_GNSSSDR_LIBRARIES} ${ORC_LIBRARIES}
                               ${GNSS_SDR_OPTIONAL_LIBS}
                               )


install(TARGETS gnss-sdr
        RUNTIME DESTINATION bin
        COMPONENT "gnss-sdr"
        )
        
install(DIRECTORY ${CMAKE_SOURCE_DIR}/conf DESTINATION etc/gnss-sdr
        FILES_MATCHING PATTERN "*.conf"
        )
        
install(DIRECTORY DESTINATION etc/gnss-sdr/data/)

install(FILES ${CMAKE_SOURCE_DIR}/conf/gnss-sdr.conf DESTINATION etc/gnss-sdr/conf
        RENAME default.conf)

install(PROGRAMS ${CMAKE_BINARY_DIR}/volk_gnsssdr_module/build/apps/volk_gnsssdr_profile DESTINATION bin)