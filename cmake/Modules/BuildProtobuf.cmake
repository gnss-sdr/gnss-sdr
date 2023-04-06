# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2023 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

# Downloads and builds the protoc compiler and static libraries of Protocol
# Buffers >= v22.0 (see https://protobuf.dev/) It requires CMake >= 3.10 and the
# abseil-cpp >= 20230117 libraries (see https://github.com/abseil/abseil-cpp)
# already installed. Zlib is used if found.
#
# Note: requires the patch command if using GCC >= 13 or Clang >= 16
#
# Creates protobuf::libprotobuf and protobuf::protoc imported targets.


if(NOT GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION)
    set(GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION "22.2")
endif()

if(NOT GNSSSDR_BINARY_DIR)
    set(GNSSSDR_BINARY_DIR "${PROJECT_BINARY_DIR}")
endif()

if(NOT ZLIB_FOUND)
    find_package(ZLIB)
    set_package_properties(ZLIB PROPERTIES
        URL "https://www.zlib.net/"
        PURPOSE "Required to build Protocol Buffers."
        TYPE REQUIRED
    )
    if(ZLIB_FOUND AND ZLIB_VERSION_STRING)
        set_package_properties(ZLIB PROPERTIES
            DESCRIPTION "A Massively Spiffy Yet Delicately Unobtrusive Compression Library (found: v${ZLIB_VERSION_STRING})"
        )
    else()
        set_package_properties(ZLIB PROPERTIES
            DESCRIPTION "A Massively Spiffy Yet Delicately Unobtrusive Compression Library"
        )
    endif()
    if(ZLIB_FOUND)
        set(ZLIB_LIBRARIES_ ${ZLIB_LIBRARIES})
        set(USE_ZLIB -Dprotobuf_WITH_ZLIB=ON)
    else()
        set(ZLIB_LIBRARIES_ "")
        set(USE_ZLIB -Dprotobuf_WITH_ZLIB=OFF)
    endif()
else()
    set(ZLIB_LIBRARIES_ ${ZLIB_LIBRARIES})
    set(USE_ZLIB -Dprotobuf_WITH_ZLIB=ON)
endif()

include(GNUInstallDirs)

list(APPEND UTF8_LIBRARIES
    ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/${CMAKE_INSTALL_LIBDIR}/${CMAKE_STATIC_LIBRARY_PREFIX}utf8_validity${CMAKE_STATIC_LIBRARY_SUFFIX}
    ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/${CMAKE_INSTALL_LIBDIR}/${CMAKE_STATIC_LIBRARY_PREFIX}utf8_range${CMAKE_STATIC_LIBRARY_SUFFIX}
)

# Fix for GCC 13 and Clang 16
if(((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "13")) OR
  ((CMAKE_CXX_COMPILER_ID STREQUAL "Clang") AND (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "16")))
    find_program(Patch_EXECUTABLE NAME patch PATHS ENV PATH)
    if(NOT Patch_EXECUTABLE)
        message(FATAL_ERROR "The patch command is not found. It is required to build Protocol Buffers. Please check your OS documentation and install the patch command.")
    endif()
    set(PROTOBUF_PATCH_COMMAND
        cd ${GNSSSDR_BINARY_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/src/google/protobuf/ &&
        ${Patch_EXECUTABLE} ${GNSSSDR_BINARY_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/src/google/protobuf/port.h < ${GNSSSDR_SOURCE_DIR}/src/tests/data/protobuf22.patch
    )
    # Patch only once
    if(EXISTS ${GNSSSDR_BINARY_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/src/google/protobuf/port.h)
        set(PROTOBUF_PATCH_COMMAND "")
    endif()
else()
    set(PROTOBUF_PATCH_COMMAND "")
endif()

ExternalProject_Add(protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
    PREFIX ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
    GIT_REPOSITORY https://github.com/protocolbuffers/protobuf
    GIT_TAG v${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
    GIT_PROGRESS ON
    UPDATE_COMMAND ""
    PATCH_COMMAND ${PROTOBUF_PATCH_COMMAND}
    SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
    BINARY_DIR ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
    CMAKE_ARGS
        -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
        -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
        -DBUILD_SHARED_LIBS=OFF
        -DCMAKE_BUILD_TYPE=$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
        -DCMAKE_CXX_VISIBILITY_PRESET=hidden
        -DCMAKE_VISIBILITY_INLINES_HIDDEN=1
        -DCMAKE_INSTALL_PREFIX=${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
        -Dprotobuf_BUILD_TESTS=OFF
        -Dprotobuf_ABSL_PROVIDER=package
        ${USE_ZLIB}
    BUILD_COMMAND ${CMAKE_COMMAND}
        "--build" "${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}"
        "--config" $<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
        "--target" install
    BUILD_BYPRODUCTS ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/${CMAKE_INSTALL_LIBDIR}/${CMAKE_FIND_LIBRARY_PREFIXES}protobuf$<$<CONFIG:Debug>:d>${CMAKE_STATIC_LIBRARY_SUFFIX}
        ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/bin/protoc
        ${ABSL_LIBRARIES}
        ${UTF8_LIBRARIES}
    INSTALL_COMMAND ""
)

file(MAKE_DIRECTORY ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/include)
file(MAKE_DIRECTORY ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/${CMAKE_INSTALL_LIBDIR})

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    find_library(CoreFoundation CoreFoundation)
else()
    set(CoreFoundation "")
endif()

set(PROTOBUF_ABSL_USED_TARGETS
    absl::absl_check
    absl::absl_log
    absl::algorithm
    absl::base
    absl::bind_front
    absl::bits
    absl::btree
    absl::cleanup
    absl::cord
    absl::core_headers
    absl::debugging
    absl::die_if_null
    absl::dynamic_annotations
    absl::flags
    absl::flat_hash_map
    absl::flat_hash_set
    absl::function_ref
    absl::hash
    absl::layout
    absl::log_initialize
    absl::log_severity
    absl::memory
    absl::node_hash_map
    absl::node_hash_set
    absl::optional
    absl::span
    absl::status
    absl::statusor
    absl::strings
    absl::synchronization
    absl::time
    absl::type_traits
    absl::utility
    absl::variant
)

if(NOT TARGET protobuf::libprotobuf)
    add_library(protobuf::libprotobuf STATIC IMPORTED)
    add_dependencies(protobuf::libprotobuf protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION})
    set_target_properties(protobuf::libprotobuf PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_CONFIGURATIONS "None;Debug;Release;RelWithDebInfo;MinSizeRel"
        MAP_IMPORTED_CONFIG_NOOPTWITHASM Debug
        MAP_IMPORTED_CONFIG_COVERAGE Debug
        MAP_IMPORTED_CONFIG_O2WITHASM RelWithDebInfo
        MAP_IMPORTED_CONFIG_O3WITHASM RelWithDebInfo
        MAP_IMPORTED_CONFIG_ASAN Debug
        IMPORTED_LOCATION_NONE ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/${CMAKE_INSTALL_LIBDIR}/${CMAKE_FIND_LIBRARY_PREFIXES}protobuf${CMAKE_STATIC_LIBRARY_SUFFIX}
        IMPORTED_LOCATION_DEBUG ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/${CMAKE_INSTALL_LIBDIR}/${CMAKE_FIND_LIBRARY_PREFIXES}protobufd${CMAKE_STATIC_LIBRARY_SUFFIX}
        IMPORTED_LOCATION_RELEASE ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/${CMAKE_INSTALL_LIBDIR}/${CMAKE_FIND_LIBRARY_PREFIXES}protobuf${CMAKE_STATIC_LIBRARY_SUFFIX}
        IMPORTED_LOCATION_RELWITHDEBINFO ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/${CMAKE_INSTALL_LIBDIR}/${CMAKE_FIND_LIBRARY_PREFIXES}protobuf${CMAKE_STATIC_LIBRARY_SUFFIX}
        IMPORTED_LOCATION_MINSIZEREL ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/${CMAKE_INSTALL_LIBDIR}/${CMAKE_FIND_LIBRARY_PREFIXES}protobuf${CMAKE_STATIC_LIBRARY_SUFFIX}
        INTERFACE_INCLUDE_DIRECTORIES ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/include
        INTERFACE_LINK_LIBRARIES "${ZLIB_LIBRARIES_};${CoreFoundation};${UTF8_LIBRARIES};${PROTOBUF_ABSL_USED_TARGETS};${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/${CMAKE_INSTALL_LIBDIR}/${CMAKE_FIND_LIBRARY_PREFIXES}protobuf$<$<CONFIG:Debug>:d>${CMAKE_STATIC_LIBRARY_SUFFIX}"
    )
endif()

if(NOT TARGET protobuf::protoc)
    add_executable(protobuf::protoc IMPORTED)
    add_dependencies(protobuf::protoc protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION})
    unset(Protobuf_PROTOC_EXECUTABLE)
    set(PROTOBUF_PROTOC_EXECUTABLE "${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/bin/protoc")
    set_target_properties(protobuf::protoc PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/bin/protoc
        INTERFACE_LINK_LIBRARIES ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/${CMAKE_INSTALL_LIBDIR}/${CMAKE_FIND_LIBRARY_PREFIXES}protoc$<$<CONFIG:Debug>:d>${CMAKE_STATIC_LIBRARY_SUFFIX}
    )
endif()
