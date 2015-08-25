# Copyright (C) 2012-2015  (see AUTHORS file for a list of contributors)
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

########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX

add_library(gr-gn3s SHARED gn3s_source_cc.cc gn3s_source.cc gn3s.cc fusb.cc fusb_linux.cc)
target_link_libraries(gr-gn3s ${Boost_LIBRARIES} ${GNURADIO_RUNTIME_LIBRARIES} "usb" ${LIBUSB_LIBRARIES})
set_target_properties(gr-gn3s PROPERTIES DEFINE_SYMBOL "gr_gn3s_EXPORTS")

########################################################################
# Install built library files
########################################################################
install(TARGETS gr-gn3s
    LIBRARY DESTINATION lib${LIB_SUFFIX}
    COMPONENT "gr-gn3s"
)

########################################################################
# Build and register unit test
########################################################################
find_package(Boost COMPONENTS unit_test_framework)

include(GrTest)
set(GR_TEST_TARGET_DEPS gr-gn3s)
#turn each test cpp file into an executable with an int main() function
add_definitions(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

add_executable(qa_gn3s_source_cc qa_gn3s_source_cc.cc)
target_link_libraries(qa_gn3s_source_cc gr-gn3s ${Boost_LIBRARIES} "usb" ${LIBUSB_LIBRARIES})
GR_ADD_TEST(qa_gn3s_source_cc qa_gn3s_source_cc)

