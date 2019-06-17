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

find_package(GPSTK QUIET)
if(NOT GPSTK_FOUND OR ENABLE_OWN_GPSTK)
    include(GNUInstallDirs)
    #string(REGEX REPLACE /[^/]*$ "" LIBDIR ${CMAKE_INSTALL_LIBDIR})
    set(GPSTK_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install/${CMAKE_INSTALL_LIBDIR}/${CMAKE_FIND_LIBRARY_PREFIXES}gpstk${CMAKE_SHARED_LIBRARY_SUFFIX})
    set(GPSTK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install/include)
endif()


find_package(Boost COMPONENTS iostreams serialization QUIET)
if(CMAKE_VERSION VERSION_LESS 3.5)
    if(NOT TARGET Boost::iostreams)
        add_library(Boost::iostreams IMPORTED SHARED)
        set_property(TARGET Boost::iostreams PROPERTY
            INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR})
        set_property(TARGET Boost::iostreams PROPERTY
            INTERFACE_LINK_LIBRARIES ${Boost_IOSTREAMS_LIBRARIES})
        set_property(TARGET Boost::iostreams PROPERTY
            IMPORTED_LOCATION ${Boost_IOSTREAMS_LIBRARIES})
    endif()
    if(NOT TARGET Boost::serialization)
        add_library(Boost::serialization IMPORTED SHARED)
        set_property(TARGET Boost::serialization PROPERTY
            INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR})
        set_property(TARGET Boost::serialization PROPERTY
            INTERFACE_LINK_LIBRARIES ${Boost_SERIALIZARION_LIBRARIES})
        set_property(TARGET Boost::serialization PROPERTY
            IMPORTED_LOCATION ${Boost_SERIALIZARION_LIBRARIES})
    endif()
endif()


find_program(UNCOMPRESS_EXECUTABLE uncompress
    PATHS /bin
    /usr/bin
    /usr/sbin
)

if(Boost_FOUND)
    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated")
    add_executable(rinex2assist ${CMAKE_CURRENT_SOURCE_DIR}/main.cc)

    target_link_libraries(rinex2assist
        PUBLIC
            Boost::iostreams
            Boost::serialization
            ${GPSTK_LIBRARY}
            Gflags::gflags
            algorithms_libs
            core_receiver
            core_system_parameters
    )

    target_include_directories(rinex2assist
        PUBLIC
            ${GPSTK_INCLUDE_DIR}/gpstk
            ${GPSTK_INCLUDE_DIR}
    )

    if(NOT UNCOMPRESS_EXECUTABLE-NOTFOUND)
        target_compile_definitions(rinex2assist PUBLIC -DUNCOMPRESS_EXECUTABLE="${UNCOMPRESS_EXECUTABLE}")
    else()
        target_compile_definitions(rinex2assist PUBLIC -DUNCOMPRESS_EXECUTABLE="")
    endif()

    if(NOT GPSTK_FOUND OR ENABLE_OWN_GPSTK)
        add_dependencies(rinex2assist gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION})
    endif()

    add_custom_command(TARGET rinex2assist POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:rinex2assist>
            ${CMAKE_SOURCE_DIR}/install/$<TARGET_FILE_NAME:rinex2assist>
    )

    install(TARGETS rinex2assist
        RUNTIME DESTINATION bin
        COMPONENT "rinex2assist"
    )
else()
    message(STATUS "Boost Iostreams library not found.")
    message(STATUS " rinex2assist will not be built.")
endif()

set(Boost_FOUND TRUE) # trick for summary report
