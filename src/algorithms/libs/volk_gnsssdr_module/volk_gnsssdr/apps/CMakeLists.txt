#
# Copyright (C) 2010-2015 (see AUTHORS file for a list of contributors)
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
# Setup profiler
########################################################################
if(Boost_FOUND)

if(MSVC)
    include_directories(${CMAKE_SOURCE_DIR}/cmake/msvc)
endif(MSVC)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}/include
    ${CMAKE_SOURCE_DIR}/lib
    ${CMAKE_BINARY_DIR}/lib
    ${Boost_INCLUDE_DIRS}
)


if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
   set(Clang_required_link "c++")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
   set(Clang_required_link "")
endif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")

# MAKE volk_gnsssdr_profile
add_executable(volk_gnsssdr_profile
    ${CMAKE_CURRENT_SOURCE_DIR}/volk_gnsssdr_profile.cc
    ${CMAKE_SOURCE_DIR}/lib/qa_utils.cc
)

target_link_libraries(volk_gnsssdr_profile volk_gnsssdr ${Boost_LIBRARIES} ${Clang_required_link})
add_dependencies(volk_gnsssdr_profile volk_gnsssdr)

if(ENABLE_STRIP)
    if(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
        set_target_properties(volk_gnsssdr_profile
                              PROPERTIES LINK_FLAGS "-s")
    endif(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
endif(ENABLE_STRIP)

install(
    TARGETS volk_gnsssdr_profile
    DESTINATION bin
    COMPONENT "volk_gnsssdr"
)

# MAKE volk_gnsssdr-config-info
add_executable(volk_gnsssdr-config-info volk_gnsssdr-config-info.cc)
target_link_libraries(volk_gnsssdr-config-info volk_gnsssdr ${Boost_LIBRARIES} ${Clang_required_link})
add_dependencies(volk_gnsssdr-config-info volk_gnsssdr)

install(
    TARGETS volk_gnsssdr-config-info
    DESTINATION bin
    COMPONENT "volk_gnsssdr"
)

endif(Boost_FOUND)
