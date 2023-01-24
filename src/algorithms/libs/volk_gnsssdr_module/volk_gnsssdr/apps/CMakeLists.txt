# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause
#

########################################################################
# Setup profiler
########################################################################
if(ORC_FOUND)
    set(orc_lib ${ORC_LIBRARIES})
else()
    set(orc_lib "")
endif()

# allow 'large' files in 32 bit builds
if(UNIX)
    add_definitions(-D_LARGEFILE_SOURCE
        -D_FILE_OFFSET_BITS=64
        -D_LARGE_FILES
    )
endif()

# POSIX_MEMALIGN: If we have to fall back to `posix_memalign`.
if(HAVE_POSIX_MEMALIGN)
    message(STATUS "Use `posix_memalign` for aligned malloc!")
    add_definitions(-DHAVE_POSIX_MEMALIGN)
endif()

# MAKE volk_gnsssdr_profile
add_executable(volk_gnsssdr_profile
    ${CMAKE_CURRENT_SOURCE_DIR}/volk_gnsssdr_profile.cc
    ${PROJECT_SOURCE_DIR}/lib/qa_utils.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/volk_gnsssdr_option_helpers.cc
)

if(MSVC)
    target_include_directories(volk_gnsssdr_profile
        PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/cmake/msvc>
    )
endif()

target_include_directories(volk_gnsssdr_profile
    PRIVATE $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
    PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    PRIVATE $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/lib>
    PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/lib>
    PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)

if(NOT FILESYSTEM_FOUND)
    target_include_directories(volk_gnsssdr_profile
        PUBLIC ${Boost_INCLUDE_DIRS}
    )
    target_link_libraries(volk_gnsssdr_profile PRIVATE ${Boost_LIBRARIES})
endif()

if(FILESYSTEM_FOUND)
    add_definitions(-DHAS_STD_FILESYSTEM=1)
    if(${find_experimental})
        add_definitions(-DHAS_STD_FILESYSTEM_EXPERIMENTAL=1)
    endif()
    target_link_libraries(volk_gnsssdr_profile PRIVATE std::filesystem)
endif()

if(ENABLE_STATIC_LIBS)
    target_link_libraries(volk_gnsssdr_profile PRIVATE volk_gnsssdr_static ${orc_lib})
else()
    target_link_libraries(volk_gnsssdr_profile PRIVATE volk_gnsssdr ${orc_lib})
endif()

if(UNIX)
    target_compile_definitions(volk_gnsssdr_profile
        PUBLIC $<$<CONFIG:Release>:_FORTIFY_SOURCE=2>$<$<CONFIG:RelWithDebInfo>:_FORTIFY_SOURCE=2>
    )
endif()

if(ENABLE_STRIP)
    if((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND NOT WIN32)
        set_target_properties(volk_gnsssdr_profile
            PROPERTIES LINK_FLAGS "-s"
        )
    endif()
endif()

install(
    TARGETS volk_gnsssdr_profile
    DESTINATION bin
    COMPONENT "volk_gnsssdr"
)

# MAKE volk_gnsssdr-config-info
add_executable(volk_gnsssdr-config-info volk_gnsssdr-config-info.cc ${CMAKE_CURRENT_SOURCE_DIR}/volk_gnsssdr_option_helpers.cc)

if(ENABLE_STATIC_LIBS)
    target_link_libraries(volk_gnsssdr-config-info volk_gnsssdr_static ${orc_lib})
else()
    target_link_libraries(volk_gnsssdr-config-info volk_gnsssdr ${orc_lib})
endif()

if(UNIX)
    target_compile_definitions(volk_gnsssdr-config-info
        PUBLIC $<$<CONFIG:Release>:_FORTIFY_SOURCE=2>$<$<CONFIG:RelWithDebInfo>:_FORTIFY_SOURCE=2>
    )
endif()

if(ENABLE_STRIP)
    if((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND NOT WIN32)
        set_target_properties(volk_gnsssdr-config-info
            PROPERTIES LINK_FLAGS "-s"
        )
    endif()
endif()

install(
    TARGETS volk_gnsssdr-config-info
    DESTINATION bin
    COMPONENT "volk_gnsssdr"
)

# Launch volk_gnsssdr_profile if requested to do so
if(ENABLE_PROFILING)
    if(DEFINED VOLK_CONFIGPATH)
        set(VOLK_CONFIG_ARG "-p${VOLK_CONFIGPATH}")
        set(VOLK_CONFIG "${VOLK_CONFIGPATH}/volk_gnsssdr_config")
    endif()
    add_custom_command(OUTPUT ${VOLK_CONFIG}
        COMMAND volk_gnsssdr_profile "${VOLK_CONFIG_ARG}"
        DEPENDS volk_gnsssdr_profile
        COMMENT "Launching profiler, this may take a few minutes..."
    )
    add_custom_target(volk-gnsssdr-profile-run ALL DEPENDS ${VOLK_CONFIG})
endif()
