# Copyright (C) 2012-2015  (see AUTHORS file for a list of contributors)
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

########################################################################
# Include swig generation macros
########################################################################
find_package(SWIG)
find_package(PythonLibs)
if(NOT SWIG_FOUND OR NOT PYTHONLIBS_FOUND)
    return()
endif()
include(GrSwig)
include(GrPython)

########################################################################
# Setup swig generation
########################################################################
set(GNURADIO_RUNTIME_SWIG_INCLUDE_DIRS ${GNURADIO_RUNTIME_INCLUDE_DIRS}/gnuradio/swig)

set(GR_SWIG_INCLUDE_DIRS
    ${GR_GN3S_INCLUDE_DIRS}
    ${GNURADIO_RUNTIME_SWIG_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

set(GR_SWIG_LIBRARIES gr-gn3s gnuradio-runtime)
set(GR_SWIG_DOC_FILE ${CMAKE_CURRENT_BINARY_DIR}/gn3s_swig_doc.i)
set(GR_SWIG_DOC_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../include)

GR_SWIG_MAKE(gn3s_swig gn3s_swig.i)

########################################################################
# Install the build swig module
########################################################################
GR_SWIG_INSTALL(TARGETS gn3s_swig DESTINATION ${GR_PYTHON_DIR}/gn3s)

########################################################################
# Install swig .i files for development
########################################################################
install(
    FILES
    gn3s_swig.i
    ${CMAKE_CURRENT_BINARY_DIR}/gn3s_swig_doc.i
    DESTINATION ${GNURADIO_RUNTIME_INCLUDE_DIR}/gn3s/swig
)
