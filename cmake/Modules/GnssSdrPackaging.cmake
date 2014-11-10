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


if(DEFINED __INCLUDED_GNSS_SDR_PACKAGE_CMAKE)
    return()
endif()
set( __INCLUDED_GNSS_SDR_PACKAGE_CMAKE TRUE)


#set the cpack generator based on the platform type
if(CPACK_GENERATOR)
    #already set by user
elseif(APPLE)
    set(CPACK_GENERATOR PackageMaker)
elseif(WIN32)
    set(CPACK_GENERATOR NSIS)
elseif(DEBIAN)
    set(CPACK_GENERATOR DEB)
elseif(REDHAT)
    set(CPACK_GENERATOR RPM)
else()
    set(CPACK_GENERATOR TGZ)
endif()



########################################################################
# Setup CPack
########################################################################
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "GNSS-SDR - An Open Source GNSS Software Defined Receiver")
set(CPACK_PACKAGE_VENDOR              "Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)")
set(CPACK_PACKAGE_CONTACT             "Carles Fernandez-Prades <carles.fernandez@cttc.cat>")
set(CPACK_PACKAGE_ICON                ${CMAKE_SOURCE_DIR}/docs/doxygen/images/gnss-sdr_logo_round.png)
set(CPACK_PACKAGE_VERSION_MAJOR       ${VERSION_INFO_MAJOR_VERSION}
set(CPACK_PACKAGE_VERSION_MINOR       ${VERSION_INFO_API_COMPAT}
set(CPACK_PACKAGE_VERSION_PATCH       ${VERSION_INFO_MINOR_VERSION}
set(CPACK_RESOURCE_FILE_LICENSE       ${CMAKE_SOURCE_DIR}/COPYING)
set(CPACK_RESOURCE_FILE_README        ${CMAKE_SOURCE_DIR}/README.md)
set(CPACK_RESOURCE_FILE_WELCOME       ${CMAKE_SOURCE_DIR}/README.md)
set(CPACK_SOURCE_GENERATOR            "TGZ;TZ")
set(CPACK_DEBIAN_PACKAGE_SECTION      "Science")
#set(DEBIAN_PACKAGE_BUILDS_DEPENDS     "cmake (>= 2.8.8), libstdc++6 (>= 4.7)" 
set(CPACK_DEBIAN_PACKAGE_DEPENDS      "libboost-dev (>= 1.45),
                                       libstdc++6 (>= 4.7),
                                       libc6 (>= 2.18),
                                       gnuradio (>= 3.7),
                                       libarmadillo-dev (>= 1:4.400.2),
                                       liblapack-dev (>= 3.5),
                                       libopenblas-dev  (>= 0.2),
                                       gfortran (>= 1:4.7),
                                       libssl-dev (>= 1.0),
                                       libgflags-dev (>= 2.0)")

#find_program(LSB_RELEASE_EXECUTABLE lsb_release)

#if((DEBIAN OR REDHAT) AND LSB_RELEASE_EXECUTABLE)
    #extract system information by executing the commands
#    execute_process(
#       COMMAND ${LSB_RELEASE_EXECUTABLE} --short --id
#        OUTPUT_VARIABLE LSB_ID OUTPUT_STRIP_TRAILING_WHITESPACE
#    )
    
#    execute_process(	
#       COMMAND ${LSB_RELEASE_EXECUTABLE} --short --release
#        OUTPUT_VARIABLE LSB_RELEASE OUTPUT_STRIP_TRAILING_WHITESPACE
#    )

    #set a more sensible package name for this system
#    SET(CPACK_PACKAGE_FILE_NAME "gnss-sdr_${CPACK_PACKAGE_VERSION}_${LSB_ID}-${LSB_RELEASE}-${CMAKE_SYSTEM_PROCESSOR}")

    #now try to include the component based dependencies
#    set(package_deps_file "${CMAKE_SOURCE_DIR}/cmake/Packaging/${LSB_ID}-${LSB_RELEASE}.cmake")
#    if (EXISTS ${package_deps_file})
#        include(${package_deps_file})
#    endif()
#endif()


include(CPack)