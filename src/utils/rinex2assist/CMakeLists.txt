# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

if(NOT GNSSTK_FOUND AND NOT ENABLE_OWN_GNSSTK)
    find_package(GNSSTK)
endif()

if(NOT GNSSTK_FOUND OR ENABLE_OWN_GNSSTK)
    include(GNUInstallDirs)
    if(GNSSTK_USES_GPSTK_NAMESPACE)
        set(GNSSTK_LIBRARY ${GNSSSDR_BINARY_DIR}/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}/install/${CMAKE_INSTALL_LIBDIR}/${CMAKE_FIND_LIBRARY_PREFIXES}gpstk${CMAKE_STATIC_LIBRARY_SUFFIX})
        set(GNSSTK_INCLUDE_DIR ${GNSSSDR_BINARY_DIR}/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}/install/include)
    else()
        set(GNSSTK_LIBRARY ${GNSSSDR_BINARY_DIR}/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}/install/${CMAKE_INSTALL_LIBDIR}/${CMAKE_FIND_LIBRARY_PREFIXES}gnsstk${CMAKE_STATIC_LIBRARY_SUFFIX})
        set(GNSSTK_INCLUDE_DIR ${GNSSSDR_BINARY_DIR}/gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION}/install/include)
    endif()
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
    message(STATUS "The rinex2assist utility tool will be built when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'")
    if(USE_CMAKE_TARGET_SOURCES)
        add_executable(rinex2assist)
        target_sources(rinex2assist
            PRIVATE
                main.cc
        )
    else()
        add_executable(rinex2assist ${CMAKE_CURRENT_SOURCE_DIR}/main.cc)
    endif()

    set_property(TARGET rinex2assist PROPERTY CXX_STANDARD 14)  # Required by GPSTk
    # Do not show warnings raised by GPSTk v3.0.0
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options(rinex2assist
            PRIVATE
                -Wno-deprecated -Wno-unused-parameter -Wno-sign-compare -Wno-type-limits -Wno-unused-but-set-variable -Wno-deprecated-copy -Wno-extra
        )
    endif()
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(rinex2assist
            PRIVATE
                -Wno-deprecated -Wno-unused-parameter -Wno-sign-compare -Wno-switch -Wno-inconsistent-missing-override
        )
    endif()

    target_link_libraries(rinex2assist
        PRIVATE
            Boost::iostreams
            Boost::serialization
            ${GNSSTK_LIBRARY}
            Gflags::gflags
            Threads::Threads
            core_system_parameters
    )

    if(GNSSTK_USES_GPSTK_NAMESPACE)
        target_compile_definitions(rinex2assist PUBLIC -DGNSSTK_USES_GPSTK_NAMESPACE=1)
        target_include_directories(rinex2assist
            PRIVATE
                ${GNSSTK_INCLUDE_DIR}/gpstk
                ${GNSSTK_INCLUDE_DIR}
        )
    else()
        target_include_directories(rinex2assist
            PRIVATE
                ${GNSSTK_INCLUDE_DIR}/gnsstk
                ${GNSSTK_INCLUDE_DIR}
        )
    endif()
    if(GNSSTK_OLDER_THAN_9)
        target_compile_definitions(rinex2assist PRIVATE -DGNSSTK_OLDER_THAN_9=1)
    endif()

    if(NOT UNCOMPRESS_EXECUTABLE-NOTFOUND)
        target_compile_definitions(rinex2assist PRIVATE -DUNCOMPRESS_EXECUTABLE="${UNCOMPRESS_EXECUTABLE}")
    else()
        target_compile_definitions(rinex2assist PRIVATE -DUNCOMPRESS_EXECUTABLE="")
    endif()

    if(NOT GNSSTK_FOUND OR ENABLE_OWN_GNSSTK)
        add_dependencies(rinex2assist gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION})
    endif()

    if(ENABLE_STRIP)
        set_target_properties(rinex2assist PROPERTIES LINK_FLAGS "-s")
    endif()

    add_custom_command(TARGET rinex2assist POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:rinex2assist>
            ${LOCAL_INSTALL_BASE_DIR}/install/$<TARGET_FILE_NAME:rinex2assist>
    )

    install(TARGETS rinex2assist
        RUNTIME DESTINATION bin
        COMPONENT "rinex2assist"
    )
else()
    message(STATUS "Boost Iostreams library not found.")
    message(STATUS " The rinex2assist utility tool will not be built.")
    if(${CMAKE_SYSTEM_NAME} MATCHES "Linux|kFreeBSD|GNU")
        message(STATUS " You can install the Boost Iostreams library by doing:")
        if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
            message(STATUS " sudo yum install boost-iostreams")
        elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
            message(STATUS " sudo zypper install libboost_iostreams-devel")
        else()
            message(STATUS " sudo apt-get install libboost-iostreams-dev")
        endif()
    endif()
endif()

set(Boost_FOUND TRUE)  # trick for summary report
