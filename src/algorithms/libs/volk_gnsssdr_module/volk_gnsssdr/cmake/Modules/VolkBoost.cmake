# Copyright (C) 2015-2018 (see AUTHORS file for a list of contributors)
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

if(DEFINED __INCLUDED_VOLK_BOOST_CMAKE)
    return()
endif()
set(__INCLUDED_VOLK_BOOST_CMAKE TRUE)

########################################################################
# Setup Boost and handle some system specific things
########################################################################

set(BOOST_REQUIRED_COMPONENTS
    filesystem
    system
)

if(UNIX AND NOT BOOST_ROOT AND EXISTS "/usr/lib64")
    list(APPEND BOOST_LIBRARYDIR "/usr/lib64") #fedora 64-bit fix
endif()

if(MSVC)
    set(BOOST_REQUIRED_COMPONENTS ${BOOST_REQUIRED_COMPONENTS} chrono)

    if(NOT DEFINED BOOST_ALL_DYN_LINK)
        set(BOOST_ALL_DYN_LINK TRUE)
    endif()
    set(BOOST_ALL_DYN_LINK "${BOOST_ALL_DYN_LINK}" CACHE BOOL "boost enable dynamic linking")
    if(BOOST_ALL_DYN_LINK)
        add_definitions(-DBOOST_ALL_DYN_LINK) #setup boost auto-linking in msvc
    else()
        unset(BOOST_REQUIRED_COMPONENTS) #empty components list for static link
    endif()
endif()

# This does not allow us to disable specific versions. It is used internally by
# cmake to know the formation of newer versions. No need to increase, not used
# anymore since newer Boost provides its own CMake configuration files.
set(Boost_ADDITIONAL_VERSIONS
    "1.53.0" "1.53" "1.54.0" "1.54"
    "1.55.0" "1.55" "1.56.0" "1.56" "1.57.0" "1.57" "1.58.0" "1.58" "1.59.0" "1.59"
    "1.60.0" "1.60" "1.61.0" "1.61" "1.62.0" "1.62" "1.63.0" "1.63" "1.64.0" "1.64"
    "1.65.0" "1.65" "1.66.0" "1.66" "1.67.0" "1.67" "1.68.0" "1.68" "1.69.0" "1.69"
    "1.70.0" "1.70" "1.71.0" "1.71"
)

find_package(Boost "1.53" COMPONENTS ${BOOST_REQUIRED_COMPONENTS})

if(CMAKE_VERSION VERSION_LESS 3.14)
    set(Boost_VERSION_STRING "${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}.${Boost_SUBMINOR_VERSION}")
endif()
