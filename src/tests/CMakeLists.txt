# Copyright (C) 2010-2013  (see AUTHORS file for a list of contributors)
#
# This file is part of GNSS-SDR.
#
# GNSS-SDR is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# at your option) any later version.
#
# GNSS-SDR is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
#

set(GTEST_DIR $ENV{GTEST_DIR})

if(GTEST_DIR)
     set(GTEST_INCLUDE_DIRECTORIES ${GTEST_DIR}/include ${GTEST_DIR} ${GTEST_DIR}/src)
     add_library(gtest ${GTEST_DIR}/src/gtest-all.cc ${GTEST_DIR}/src/gtest_main.cc)
     include_directories(
          ${GTEST_INCLUDE_DIRECTORIES}
          ${CMAKE_SOURCE_DIR}/src/core/system_parameters
          ${CMAKE_SOURCE_DIR}/src/core/interfaces
          ${CMAKE_SOURCE_DIR}/src/core/receiver
          ${CMAKE_SOURCE_DIR}/src/core/libs   
          ${CMAKE_SOURCE_DIR}/src/algorithms/libs
          ${CMAKE_SOURCE_DIR}/src/algorithms/resampler/gnuradio_blocks
          ${CMAKE_SOURCE_DIR}/src/algorithms/tracking/libs
          ${CMAKE_SOURCE_DIR}/src/algorithms/signal_source/adapters
          ${CMAKE_SOURCE_DIR}/src/algorithms/input_filter/adapters
          ${CMAKE_SOURCE_DIR}/src/algorithms/acquisition/adapters
          ${CMAKE_SOURCE_DIR}/src/algorithms/acquisition/gnuradio_blocks
          ${CMAKE_SOURCE_DIR}/src/algorithms/output_filter/adapters
          ${GLOG_INCLUDE_DIRS}
          ${GFlags_INCLUDE_DIRS}
          ${GNURADIO_CORE_INCLUDE_DIRS}
          ${GNURADIO_GRUEL_INCLUDE_DIRS}
          ${Boost_INCLUDE_DIRS}
     )
     add_executable(run_tests ${CMAKE_CURRENT_SOURCE_DIR}/test_main.cc)
     target_link_libraries(run_tests ${Boost_LIBRARIES} glog gflags gtest gnss_sp_libs gnss_rx)
     add_dependencies(run_tests googletest)
     #add_test(tests_gnss ${CMAKE_SOURCE_DIR}/install/run_tests)
     install(TARGETS run_tests DESTINATION ${CMAKE_SOURCE_DIR}/install)
else()
     message (" ################################ ")
     message (" Googletest not found! ")
     message (" ################################ ")
     message (" GNSS-SDR test system is based on Googletest, which has not been found ")
     message (" Tests will not be build. Please download Googletest source code ")
     message (" and define the environment variable GTEST_DIR pointing to its root folder")
endif(GTEST_DIR)