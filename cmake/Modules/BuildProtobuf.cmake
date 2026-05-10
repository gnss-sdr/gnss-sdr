# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2023-2026 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

# Downloads and builds the protoc compiler and static libraries of Protocol
# Buffers >= v22.0 (see https://protobuf.dev/) It requires CMake >= 3.10 and the
# abseil-cpp >= 20230117 libraries (see https://github.com/abseil/abseil-cpp)
# already installed. Zlib is used if found.
#
# Creates protobuf::libprotobuf and protobuf::protoc imported targets.

if(NOT COMMAND set_package_properties)
    include(FeatureSummary)
endif()

if(NOT COMMAND ExternalProject_Add)
    include(ExternalProject)
endif()

include(GNUInstallDirs)
if(CMAKE_VERSION VERSION_LESS "3.10")
    message(FATAL_ERROR
        "Building bundled Protocol Buffers requires CMake >= 3.10. "
        "Use a system Protobuf package with this CMake version."
    )
endif()

if(NOT GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION)
    set(GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION "22.0")
endif()
if((NOT GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION VERSION_LESS "30.0") AND
        CMAKE_VERSION VERSION_LESS "3.16")
    message(FATAL_ERROR
        "Building bundled Protocol Buffers ${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION} "
        "requires CMake >= 3.16. Use a system Protobuf package, or select "
        "an older bundled Protobuf version."
    )
endif()

if(NOT GNSSSDR_BINARY_DIR)
    set(GNSSSDR_BINARY_DIR "${PROJECT_BINARY_DIR}")
endif()

if(NOT ZLIB_FOUND)
    find_package(ZLIB)
    set_package_properties(ZLIB PROPERTIES
        URL "https://www.zlib.net/"
        PURPOSE "Enables gzip stream support in Protocol Buffers."
        TYPE RECOMMENDED
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
        set(USE_ZLIB_ -Dprotobuf_WITH_ZLIB=ON)
    else()
        set(ZLIB_LIBRARIES_ "")
        set(USE_ZLIB_ -Dprotobuf_WITH_ZLIB=OFF)
    endif()
else()
    set(ZLIB_LIBRARIES_ ${ZLIB_LIBRARIES})
    set(USE_ZLIB_ -Dprotobuf_WITH_ZLIB=ON)
endif()

set(UTF8_LIBRARIES
    ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/${CMAKE_INSTALL_LIBDIR}/${CMAKE_STATIC_LIBRARY_PREFIX}utf8_validity${CMAKE_STATIC_LIBRARY_SUFFIX}
    ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/${CMAKE_INSTALL_LIBDIR}/${CMAKE_STATIC_LIBRARY_PREFIX}utf8_range${CMAKE_STATIC_LIBRARY_SUFFIX}
)

set(ABSL_DIR_OPTION_ "")
if(DEFINED absl_DIR)
    set(ABSL_DIR_OPTION_ "-Dabsl_DIR=${absl_DIR}")
endif()

# Protobuf < 30.0 uses the protobuf_ABSL_PROVIDER option.  For newer
# releases, request local dependencies only so the external build uses the
# dependencies already found or installed by the main project.
set(GNSSSDR_ABSL_PROVIDER_ "")
if(GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION VERSION_LESS "30.0")
    set(GNSSSDR_ABSL_PROVIDER_ "-Dprotobuf_ABSL_PROVIDER=package")
else()
    set(GNSSSDR_ABSL_PROVIDER_ "-Dprotobuf_LOCAL_DEPENDENCIES_ONLY=ON")
endif()

if(NOT DEFINED PROTOBUF_PATCH_COMMAND)
    set(PROTOBUF_PATCH_COMMAND "")
endif()

set(PROTOBUF_CXX_STANDARD_ "14")
if(NOT GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION VERSION_LESS "30.0")
    set(PROTOBUF_CXX_STANDARD_ "17")
endif()

set(PROTOBUF_LIBPROTOBUF_
    ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/${CMAKE_INSTALL_LIBDIR}/${CMAKE_STATIC_LIBRARY_PREFIX}protobuf${CMAKE_STATIC_LIBRARY_SUFFIX}
)

set(PROTOBUF_LIBPROTOBUF_DEBUG_
    ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/${CMAKE_INSTALL_LIBDIR}/${CMAKE_STATIC_LIBRARY_PREFIX}protobufd${CMAKE_STATIC_LIBRARY_SUFFIX}
)

set(PROTOBUF_PROTOC_EXECUTABLE_
    ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/bin/protoc${CMAKE_EXECUTABLE_SUFFIX}
)

set(PROTOBUF_BUILD_BYPRODUCTS_
    ${PROTOBUF_PROTOC_EXECUTABLE_}
    ${UTF8_LIBRARIES}
)

if(CMAKE_CONFIGURATION_TYPES)
    list(APPEND PROTOBUF_BUILD_BYPRODUCTS_
        ${PROTOBUF_LIBPROTOBUF_}
        ${PROTOBUF_LIBPROTOBUF_DEBUG_}
    )
elseif(CMAKE_BUILD_TYPE MATCHES "^(Debug|NoOptWithASM|Coverage|ASAN)$")
    list(APPEND PROTOBUF_BUILD_BYPRODUCTS_
        ${PROTOBUF_LIBPROTOBUF_DEBUG_}
    )
else()
    list(APPEND PROTOBUF_BUILD_BYPRODUCTS_
        ${PROTOBUF_LIBPROTOBUF_}
    )
endif()

set(CMAKE_PREFIX_PATH_OPTION_ "")
if(CMAKE_PREFIX_PATH)
    string(REPLACE ";" "|" CMAKE_PREFIX_PATH_EXTERNAL_ "${CMAKE_PREFIX_PATH}")
    set(CMAKE_PREFIX_PATH_OPTION_
        "-DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH_EXTERNAL_}"
    )
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
    LIST_SEPARATOR |
    CMAKE_ARGS
        -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
        -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
        -DCMAKE_CXX_STANDARD=${PROTOBUF_CXX_STANDARD_}
        -DCMAKE_CXX_STANDARD_REQUIRED=ON
        -DCMAKE_CXX_EXTENSIONS=OFF
        ${CMAKE_PREFIX_PATH_OPTION_}
        -DBUILD_SHARED_LIBS=OFF
        -DCMAKE_BUILD_TYPE=$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
        -DCMAKE_CXX_VISIBILITY_PRESET=hidden
        -DCMAKE_VISIBILITY_INLINES_HIDDEN=1
        -DCMAKE_INSTALL_PREFIX=${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
        "-DCMAKE_INSTALL_LIBDIR:PATH=${CMAKE_INSTALL_LIBDIR}"
        -Dprotobuf_BUILD_TESTS=OFF
        ${ABSL_DIR_OPTION_}
        ${GNSSSDR_ABSL_PROVIDER_}
        ${USE_ZLIB_}
    BUILD_COMMAND ${CMAKE_COMMAND}
        "--build" "${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}"
        "--config" $<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
        "--target" install
    BUILD_BYPRODUCTS ${PROTOBUF_BUILD_BYPRODUCTS_}
    INSTALL_COMMAND ""
)

if("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
    find_program(CODESIGN_EXECUTABLE codesign)

    if(CODESIGN_EXECUTABLE)
        ExternalProject_Add_Step(
            protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
            codesign_protoc
            COMMAND ${CODESIGN_EXECUTABLE}
                --force
                --sign -
                ${PROTOBUF_PROTOC_EXECUTABLE_}
            DEPENDEES build
            COMMENT "Ad-hoc signing bundled protoc executable"
            VERBATIM
        )
    endif()
endif()

file(MAKE_DIRECTORY ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/include)
file(MAKE_DIRECTORY ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/${CMAKE_INSTALL_LIBDIR})

set(PROTOBUF_EXTRA_LIBRARIES_ "")
if("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
    find_library(GNSSSDR_COREFOUNDATION_LIBRARY CoreFoundation)
    if(GNSSSDR_COREFOUNDATION_LIBRARY)
        list(APPEND PROTOBUF_EXTRA_LIBRARIES_
            ${GNSSSDR_COREFOUNDATION_LIBRARY}
        )
    endif()
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

if(GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION VERSION_LESS "30.0")
    foreach(_gnsssdr_absl_target ${PROTOBUF_ABSL_USED_TARGETS})
        if(NOT TARGET ${_gnsssdr_absl_target})
            message(FATAL_ERROR
                "Required Abseil target ${_gnsssdr_absl_target} was not found. "
                "Install a compatible abseil-cpp package before building bundled "
                "Protocol Buffers ${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}."
            )
        endif()
    endforeach()
    unset(_gnsssdr_absl_target)
endif()

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
        IMPORTED_LOCATION ${PROTOBUF_LIBPROTOBUF_}
        IMPORTED_LOCATION_NONE ${PROTOBUF_LIBPROTOBUF_}
        IMPORTED_LOCATION_DEBUG ${PROTOBUF_LIBPROTOBUF_DEBUG_}
        IMPORTED_LOCATION_RELEASE ${PROTOBUF_LIBPROTOBUF_}
        IMPORTED_LOCATION_RELWITHDEBINFO ${PROTOBUF_LIBPROTOBUF_}
        IMPORTED_LOCATION_MINSIZEREL ${PROTOBUF_LIBPROTOBUF_}
        INTERFACE_INCLUDE_DIRECTORIES ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/include
        INTERFACE_LINK_LIBRARIES "${ZLIB_LIBRARIES_};${PROTOBUF_EXTRA_LIBRARIES_};${UTF8_LIBRARIES};${PROTOBUF_ABSL_USED_TARGETS}"
    )
endif()

if(NOT TARGET protobuf::protoc)
    add_executable(protobuf::protoc IMPORTED)
    add_dependencies(protobuf::protoc protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION})
    set(Protobuf_PROTOC_EXECUTABLE "${PROTOBUF_PROTOC_EXECUTABLE_}")
    set(PROTOBUF_PROTOC_EXECUTABLE "${PROTOBUF_PROTOC_EXECUTABLE_}")
    set_target_properties(protobuf::protoc PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        MAP_IMPORTED_CONFIG_NOOPTWITHASM Debug
        MAP_IMPORTED_CONFIG_COVERAGE Debug
        MAP_IMPORTED_CONFIG_O2WITHASM RelWithDebInfo
        MAP_IMPORTED_CONFIG_O3WITHASM RelWithDebInfo
        MAP_IMPORTED_CONFIG_ASAN Debug
        IMPORTED_LOCATION ${PROTOBUF_PROTOC_EXECUTABLE_}
    )
endif()

unset(GNSSSDR_ABSL_PROVIDER_)
unset(USE_ZLIB_)
unset(ZLIB_LIBRARIES_)
unset(ABSL_DIR_OPTION_)
unset(PROTOBUF_EXTRA_LIBRARIES_)
unset(PROTOBUF_CXX_STANDARD_)
unset(PROTOBUF_LIBPROTOBUF_)
unset(PROTOBUF_LIBPROTOBUF_DEBUG_)
unset(PROTOBUF_PROTOC_EXECUTABLE_)
unset(PROTOBUF_BUILD_BYPRODUCTS_)
unset(CMAKE_PREFIX_PATH_OPTION_)
if(DEFINED CMAKE_PREFIX_PATH_EXTERNAL_)
    unset(CMAKE_PREFIX_PATH_EXTERNAL_)
endif()