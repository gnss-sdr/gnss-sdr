# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


if("${ARMADILLO_VERSION_STRING}" VERSION_GREATER "9.800" OR (NOT ARMADILLO_FOUND) OR ENABLE_OWN_ARMADILLO)  # requires back(), introduced in Armadillo 9.800
    message(STATUS "The obsdiff utility tool will be built when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'")
    find_package(GPSTK QUIET)
    if(NOT GPSTK_FOUND OR ENABLE_OWN_GPSTK)
        include(GNUInstallDirs)
        set(GPSTK_LIBRARY ${CMAKE_BINARY_DIR}/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gpstk${CMAKE_STATIC_LIBRARY_SUFFIX})
        set(GPSTK_INCLUDE_DIR ${CMAKE_BINARY_DIR}/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install/include)
    endif()

    if(USE_CMAKE_TARGET_SOURCES)
        add_executable(obsdiff)
        target_sources(obsdiff
            PRIVATE
                obsdiff.cc
                obsdiff_flags.h
        )
    else()
        source_group(Headers FILES obsdiff_flags.h)
        add_executable(obsdiff ${CMAKE_CURRENT_SOURCE_DIR}/obsdiff.cc obsdiff_flags.h)
    endif()

    target_include_directories(obsdiff PUBLIC ${CMAKE_SOURCE_DIR}/src/tests/common-files)
    set_property(TARGET obsdiff PROPERTY CXX_STANDARD 14)  # Required by GPSTk v3.0.0
    # Do not show warnings raised by GPSTk v3.0.0
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options(obsdiff
            PRIVATE
                -Wno-deprecated -Wno-unused-parameter -Wno-sign-compare -Wno-reorder -Wno-deprecated-copy -Wno-extra -Wno-unused-but-set-variable -Wno-unknown-pragmas
        )
    endif()
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(obsdiff
            PRIVATE
                -Wno-deprecated -Wno-unused-parameter -Wno-sign-compare -Wno-reorder
        )
    endif()

    if(NOT ARMADILLO_FOUND OR ENABLE_OWN_ARMADILLO)
        add_dependencies(obsdiff armadillo-${armadillo_RELEASE})
    endif()
    if(NOT GFLAGS_FOUND)
        add_dependencies(obsdiff gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION})
    endif()
    if(NOT GPSTK_FOUND OR ENABLE_OWN_GPSTK)
        add_dependencies(obsdiff gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION})
    endif()
    if(NOT MATIO_FOUND OR MATIO_VERSION_STRING VERSION_LESS ${GNSSSDR_MATIO_MIN_VERSION})
        add_dependencies(obsdiff matio-${GNSSSDR_MATIO_LOCAL_VERSION})
    endif()

    if(NOT TARGET Gpstk::gpstk)
        file(MAKE_DIRECTORY ${GPSTK_INCLUDE_DIR}/gpstk)
        add_library(Gpstk::gpstk STATIC IMPORTED)
        add_dependencies(Gpstk::gpstk gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION})
        set_target_properties(Gpstk::gpstk PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
            IMPORTED_LOCATION "${GPSTK_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${GPSTK_INCLUDE_DIR};${GPSTK_INCLUDE_DIR}/gpstk"
            INTERFACE_LINK_LIBRARIES "${GPSTK_LIBRARY}"
        )
    endif()

    target_link_libraries(obsdiff
        PRIVATE
            Armadillo::armadillo
            Threads::Threads
            Gflags::gflags
            Matio::matio
            Gpstk::gpstk
    )

    if(GPSTK_OLDER_THAN_8)
        target_compile_definitions(obsdiff PUBLIC -DOLD_GPSTK=1)
    endif()

    if(ENABLE_STRIP)
        set_target_properties(obsdiff PROPERTIES LINK_FLAGS "-s")
    endif()

    add_custom_command(TARGET obsdiff POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:obsdiff>
        ${LOCAL_INSTALL_BASE_DIR}/install/$<TARGET_FILE_NAME:obsdiff>
    )

    install(TARGETS obsdiff
        RUNTIME DESTINATION bin
        COMPONENT "obsdiff"
    )
else()
    message(STATUS "The Armadillo library version found (${ARMADILLO_VERSION_STRING}) is older than 9.800.")
    message(STATUS " The obsdiff utility tool will not be built.")
    message(STATUS " You could build it by setting -DENABLE_OWN_ARMADILLO=ON")
endif()
