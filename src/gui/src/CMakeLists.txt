if(NOT (CMAKE_VERSION VERSION_LESS "2.8.13"))
cmake_policy(SET CMP0043 NEW)
endif(NOT (CMAKE_VERSION VERSION_LESS "2.8.13"))

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

file(GLOB SRC . *.cc)
list(SORT SRC)
file(GLOB HEADERS . *.h)
list(SORT HEADERS)

set(GUI_FILES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../")

add_definitions(-DGUI_FILES_DIR="${GUI_FILES_DIR}")
add_definitions(-DQT_WIDGETS_LIB)
add_definitions(-DQT_CORE_LIB)

if(OS_IS_MACOSX)
    set(MACOSX_BUNDLE_VAR MACOSX_BUNDLE)
endif(OS_IS_MACOSX)

add_executable (gui ${MACOSX_BUNDLE_VAR} ${SRC} ${HEADERS})

qt5_use_modules(gui Widgets)

add_custom_command(TARGET gui POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:gui>
                           ${CMAKE_SOURCE_DIR}/install/$<TARGET_FILE_NAME:gui>
                  )
