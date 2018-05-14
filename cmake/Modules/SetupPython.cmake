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
#this allows the user to override PYTHON_EXECUTABLE
if(PYTHON_EXECUTABLE)

    set(PYTHONINTERP_FOUND TRUE)

#otherwise if not set, try to automatically find it
else(PYTHON_EXECUTABLE)

    #use the built-in find script
    set(Python_ADDITIONAL_VERSIONS 3.4 3.5 3.6)
    find_package(PythonInterp 2)

    #and if that fails use the find program routine
    if(NOT PYTHONINTERP_FOUND)
        find_program(PYTHON_EXECUTABLE NAMES python python2 python2.7 python3)
        if(PYTHON_EXECUTABLE)
            set(PYTHONINTERP_FOUND TRUE)
        endif(PYTHON_EXECUTABLE)
    endif(NOT PYTHONINTERP_FOUND)

endif(PYTHON_EXECUTABLE)

if (CMAKE_CROSSCOMPILING)
    set(QA_PYTHON_EXECUTABLE "/usr/bin/python")
else (CMAKE_CROSSCOMPILING)
    set(QA_PYTHON_EXECUTABLE ${PYTHON_EXECUTABLE})
endif(CMAKE_CROSSCOMPILING)

#make the path to the executable appear in the cmake gui
set(PYTHON_EXECUTABLE ${PYTHON_EXECUTABLE} CACHE FILEPATH "python interpreter")
set(QA_PYTHON_EXECUTABLE ${QA_PYTHON_EXECUTABLE} CACHE FILEPATH "python interpreter for QA tests")

#make sure we can use -B with python (introduced in 2.6)
if(PYTHON_EXECUTABLE)
    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -B -c ""
        OUTPUT_QUIET ERROR_QUIET
        RESULT_VARIABLE PYTHON_HAS_DASH_B_RESULT
    )
    if(PYTHON_HAS_DASH_B_RESULT EQUAL 0)
        set(PYTHON_DASH_B "-B")
    endif()
endif(PYTHON_EXECUTABLE)

########################################################################
# Check for the existence of a python module:
# - desc a string description of the check
# - mod the name of the module to import
# - cmd an additional command to run
# - have the result variable to set
########################################################################
macro(GNSSSDR_PYTHON_CHECK_MODULE desc mod cmd have)
    message(STATUS "Python checking for ${desc}")
    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c "
#########################################
try: import ${mod}
except:
    try: ${mod}
    except: exit(-1)
try: assert ${cmd}
except: exit(-1)
#########################################"
        RESULT_VARIABLE ${have}
    )
    if(${have} EQUAL 0)
        message(STATUS "Python checking for ${desc} - found")
        set(${have} TRUE)
    else(${have} EQUAL 0)
        message(STATUS "Python checking for ${desc} - not found")
        set(${have} FALSE)
    endif(${have} EQUAL 0)
endmacro(GNSSSDR_PYTHON_CHECK_MODULE)
