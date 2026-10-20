# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2025 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

# Allow building standalone or as a subdirectory of GNSS-SDR.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.14...4.3)
    project(gnss-sdr-tui CXX)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(TUI_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# --- Dependencies ---

find_package(Protobuf REQUIRED QUIET MODULE)
if(Protobuf_VERSION)
    if(${Protobuf_VERSION} VERSION_LESS "3.0.0")
        message(FATAL_ERROR "Fatal error: Protocol Buffers >= v3.0.0 required.")
    endif()
    message(STATUS "Found Protobuf ${Protobuf_VERSION}: ${PROTOBUF_INCLUDE_DIR}")
endif()

# Fix CMake 4.x FindProtobuf missing IMPORTED_LOCATION
if(TARGET protobuf::libprotobuf)
    set_target_properties(protobuf::libprotobuf PROPERTIES
        IMPORTED_LOCATION "${PROTOBUF_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${PROTOBUF_INCLUDE_DIR}"
    )
endif()

# protoc
if(TARGET protobuf::protoc)
    set_target_properties(protobuf::protoc PROPERTIES
        IMPORTED_LOCATION "${Protobuf_PROTOC_EXECUTABLE}"
    )
    set(PROTOC_BINARY protobuf::protoc)
else()
    find_program(PROTOC_BINARY protoc REQUIRED)
endif()

# FTXUI via FetchContent (guard against duplicate declarations)
if(NOT ftxui_POPULATED)
    include(FetchContent)
    FetchContent_Declare(ftxui
        GIT_REPOSITORY https://github.com/ArthurSonzogni/FTXUI
        GIT_TAG v5.0.0
    )
    FetchContent_MakeAvailable(ftxui)
endif()

# --- Proto compilation ---

set(GNSS_PROTO_SRCS
    ${CMAKE_CURRENT_BINARY_DIR}/gnss_synchro.pb.cc
    ${CMAKE_CURRENT_BINARY_DIR}/monitor_pvt.pb.cc
)
set(GNSS_PROTO_HDRS
    ${CMAKE_CURRENT_BINARY_DIR}/gnss_synchro.pb.h
    ${CMAKE_CURRENT_BINARY_DIR}/monitor_pvt.pb.h
)

foreach(PROTO_FILE
    ${TUI_SOURCE_DIR}/proto/gnss_synchro.proto
    ${TUI_SOURCE_DIR}/proto/monitor_pvt.proto
)
    get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.pb.cc
               ${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.pb.h
        COMMAND ${PROTOC_BINARY}
            --cpp_out ${CMAKE_CURRENT_BINARY_DIR}
            -I ${TUI_SOURCE_DIR}/proto
            ${PROTO_FILE}
        DEPENDS ${PROTO_FILE}
        COMMENT "Generating ${PROTO_NAME} protobuf sources"
    )
endforeach()

# --- Libraries ---

add_library(tui_data_store
    ${TUI_SOURCE_DIR}/data_store.cc
)
target_include_directories(tui_data_store
    PUBLIC ${TUI_SOURCE_DIR}
)

set_source_files_properties(${GNSS_PROTO_SRCS} PROPERTIES
    GENERATED TRUE
)

add_library(tui_udp_listener
    ${TUI_SOURCE_DIR}/udp_listener.cc
    ${GNSS_PROTO_SRCS}
)
target_include_directories(tui_udp_listener
    PUBLIC ${TUI_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}
)
target_include_directories(tui_udp_listener
    SYSTEM PUBLIC ${PROTOBUF_INCLUDE_DIR}
)
target_link_libraries(tui_udp_listener
    PUBLIC
        tui_data_store
        ${PROTOBUF_LIBRARIES}
)

add_library(tui_screens
    ${TUI_SOURCE_DIR}/screens/signal_overview.cc
    ${TUI_SOURCE_DIR}/screens/satellite_table.cc
    ${TUI_SOURCE_DIR}/screens/pvt_panel.cc
    ${TUI_SOURCE_DIR}/screens/event_log.cc
    ${TUI_SOURCE_DIR}/screens/help_screen.cc
    ${TUI_SOURCE_DIR}/screens/satellite_detail.cc
)
target_include_directories(tui_screens
    PUBLIC ${TUI_SOURCE_DIR}
)
target_link_libraries(tui_screens
    PUBLIC
        tui_data_store
        ftxui::dom
        ftxui::screen
)

# --- Executable ---

add_executable(gnss_sdr_tui
    ${TUI_SOURCE_DIR}/main.cc
)
target_link_libraries(gnss_sdr_tui
    PRIVATE
        tui_udp_listener
        tui_screens
        ftxui::component
        ftxui::dom
        ftxui::screen
)
target_include_directories(gnss_sdr_tui
    PRIVATE ${TUI_SOURCE_DIR}
)

install(TARGETS gnss_sdr_tui
    RUNTIME DESTINATION bin
)
