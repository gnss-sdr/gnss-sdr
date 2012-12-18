# CMAKE for GPSTK by Javier Arribas 2012
cmake_minimum_required (VERSION 2.6)
project (gnsspvt_project)

include_directories(${gnsspvt_project_SOURCE_DIR}/src)

add_library (kml_printer_gpstk ${gnsspvt_project_SOURCE_DIR}/src/kml_printer_gpstk.cpp)
add_executable(gnsspvt ${gnsspvt_project_SOURCE_DIR}/src/gnsspvt.cpp)

target_link_libraries (gnsspvt kml_printer_gpstk)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeModules/")

find_package(GPSTK REQUIRED)

if ( NOT GPSTK_FOUND )
  message(FATAL_ERROR "GPSTK library not found!")
endif( NOT GPSTK_FOUND )

find_package(GLOG REQUIRED)
if ( NOT GLOG_FOUND )
  message(FATAL_ERROR "GLOG library not found!")
endif( NOT GLOG_FOUND )

include_directories(${GLOG_INCLUDE_DIRS})

# IMPORTANT NOTICE: The GPSTK linking order is critical. First it is required to link agains libprocframe to avoid vtable errors
include_directories(${GPSTK_INCLUDE_DIR}/gpstk ${GEOMATICS_INCLUDE_DIR} ${PROCFRAME_INCLUDE_DIR} ${VDRAW_INCLUDE_DIR} ${VPLOT_INCLUDE_DIR} ${RXIO_INCLUDE_DIR})
# set(LIBS ${LIBS} ${GPSTK_LIBRARIES} ${GEOMATICS_LIBRARIES} ${VDRAW_LIBRARIES} ${VPLOT_LIBRARIES} ${RXIO_LIBRARIES})
set(LIBS ${LIBS} ${PROCFRAME_LIBRARIES} ${GPSTK_LIBRARIES} ${GEOMATICS_LIBRARIES} ${GLOG_LIBRARIES})


target_link_libraries(gnsspvt ${LIBS})

message(STATUS "GPSTK_INCLUDE_DIR="${GLOG_LIBRARIES})
#message(STATUS "GPSTK_LIBRARIES=${GPSTK_LIBRARIES}")
#message(STATUS "LIBS=${LIBS}")

# debug info: print all variables
#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

#get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
#foreach(dir ${dirs})
#  message(STATUS "INCLUDE_DIRECTORIES='${dir}'")
#endforeach()

