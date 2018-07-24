# Copyright (C) 2011-2018 (see AUTHORS file for a list of contributors)
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


########################################################################
# Setup the python interpreter:
# This allows the user to specify a specific interpreter,
# or finds the interpreter via the built-in cmake module.
########################################################################

if (PYTHON_EXECUTABLE)
    message(STATUS "User set python executable ${PYTHON_EXECUTABLE}")
    find_package(PythonInterp ${GNSSSDR_PYTHON_MIN_VERSION} REQUIRED)
else (PYTHON_EXECUTABLE)
    message(STATUS "PYTHON_EXECUTABLE not set - using default python3")
    message(STATUS "Use -DPYTHON_EXECUTABLE=/path/to/python2 to build for python2.")
    find_package(PythonInterp ${GNSSSDR_PYTHON3_MIN_VERSION} REQUIRED)
endif (PYTHON_EXECUTABLE)

if (${PYTHON_VERSION_MAJOR} VERSION_EQUAL 3)
    set(PYTHON3 TRUE)
endif ()

find_package(PythonLibs ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR} EXACT)

if (CMAKE_CROSSCOMPILING)
    set(QA_PYTHON_EXECUTABLE "/usr/bin/python")
else (CMAKE_CROSSCOMPILING)
    set(QA_PYTHON_EXECUTABLE ${PYTHON_EXECUTABLE})
endif(CMAKE_CROSSCOMPILING)

#make the path to the executable appear in the cmake gui
set(PYTHON_EXECUTABLE ${PYTHON_EXECUTABLE} CACHE FILEPATH "python interpreter")
set(QA_PYTHON_EXECUTABLE ${QA_PYTHON_EXECUTABLE} CACHE FILEPATH "python interpreter for QA tests")

########################################################################
# Check for the existence of a python module:
# - desc a string description of the check
# - mod the name of the module to import
# - cmd an additional command to run
# - have the result variable to set
########################################################################
macro(GNSSSDR_PYTHON_CHECK_MODULE_RAW desc python_code have)
    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c "${python_code}"
        OUTPUT_QUIET ERROR_QUIET
        RESULT_VARIABLE return_code
    )
    if(return_code EQUAL 0)
        message(STATUS "Python checking for ${desc} - found")
        set(${have} TRUE)
    else()
        message(STATUS "Python checking for ${desc} - not found")
        set(${have} FALSE)
    endif()
endmacro(GNSSSDR_PYTHON_CHECK_MODULE_RAW)

macro(GNSSSDR_PYTHON_CHECK_MODULE desc mod cmd have)
    GNSSSDR_PYTHON_CHECK_MODULE_RAW(
        "${desc}" "
#########################################
try:
    import ${mod}
    assert ${cmd}
except (ImportError, AssertionError): exit(-1)
except: pass
#########################################"
    "${have}")
endmacro(GNSSSDR_PYTHON_CHECK_MODULE)
