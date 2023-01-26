# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


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
        OUTPUT_QUIET ERROR_QUIET
        RESULT_VARIABLE ${have}
    )
    if(${have} EQUAL 0)
        message(STATUS "Python checking for ${desc} - found")
        set(${have} TRUE)
    else()
        message(STATUS "Python checking for ${desc} - not found")
        set(${have} FALSE)
    endif()
endmacro()


########################################################################
# Setup the python interpreter:
# This allows the user to specify a specific interpreter,
# or finds the interpreter via the built-in cmake module.
########################################################################

if(CMAKE_VERSION VERSION_LESS 3.12 OR CMAKE_CROSSCOMPILING)
    if(PYTHON_EXECUTABLE)
        message(STATUS "User set python executable ${PYTHON_EXECUTABLE}")
        string(FIND "${PYTHON_EXECUTABLE}" "python3" IS_PYTHON3)
        if(IS_PYTHON3 EQUAL -1)
            find_package(PythonInterp ${GNSSSDR_PYTHON_MIN_VERSION} REQUIRED)
        else()
            find_package(PythonInterp ${GNSSSDR_PYTHON3_MIN_VERSION} REQUIRED)
        endif()
        gnsssdr_python_check_module("python >= ${GNSSSDR_PYTHON_MIN_VERSION}" sys "sys.version.split()[0] >= '${GNSSSDR_PYTHON_MIN_VERSION}'" PYTHON_MIN_VER_FOUND)
        gnsssdr_python_check_module("mako >= ${GNSSSDR_MAKO_MIN_VERSION}" mako "mako.__version__ >= '${GNSSSDR_MAKO_MIN_VERSION}'" MAKO_FOUND)
        if(IS_PYTHON3 EQUAL -1)
            gnsssdr_python_check_module("six - python 2 and 3 compatibility library" six "True" SIX_FOUND)
        endif()
    else()
        message(STATUS "PYTHON_EXECUTABLE not set - trying by default python3")
        set(Python_ADDITIONAL_VERSIONS 3.4 3.5 3.6 3.7 3.8 3.9)
        find_package(PythonInterp ${GNSSSDR_PYTHON_MIN3_VERSION})
        if(NOT PYTHONINTERP_FOUND)
            message(STATUS "python3 not found - trying with python2.7")
            find_package(PythonInterp ${GNSSSDR_PYTHON_MIN_VERSION} REQUIRED)
        endif()
        gnsssdr_python_check_module("python >= ${GNSSSDR_PYTHON_MIN_VERSION}" sys "sys.version.split()[0] >= '${GNSSSDR_PYTHON_MIN_VERSION}'" PYTHON_MIN_VER_FOUND)
        gnsssdr_python_check_module("mako >= ${GNSSSDR_MAKO_MIN_VERSION}" mako "mako.__version__ >= '${GNSSSDR_MAKO_MIN_VERSION}'" MAKO_FOUND)
        if(PYTHON_VERSION_STRING VERSION_LESS "3.0")
            gnsssdr_python_check_module("six - python 2 and 3 compatibility library" six "True" SIX_FOUND)
        endif()
    endif()
else()
    if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        set(_previous ${CMAKE_FIND_FRAMEWORK})
        set(CMAKE_FIND_FRAMEWORK LAST)
    endif()
    find_package(Python3 COMPONENTS Interpreter)
    if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        set(CMAKE_FIND_FRAMEWORK ${_previous})
    endif()
    if(Python3_FOUND)
        set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
        set(PYTHON_VERSION_MAJOR ${Python3_VERSION_MAJOR})
        gnsssdr_python_check_module("python >= ${GNSSSDR_PYTHON_MIN_VERSION}" sys "sys.version.split()[0] >= '${GNSSSDR_PYTHON_MIN_VERSION}'" PYTHON_MIN_VER_FOUND)
        gnsssdr_python_check_module("mako >= ${GNSSSDR_MAKO_MIN_VERSION}" mako "mako.__version__ >= '${GNSSSDR_MAKO_MIN_VERSION}'" MAKO_FOUND)
    endif()
    if(NOT Python3_FOUND OR NOT MAKO_FOUND)
        find_package(Python2 COMPONENTS Interpreter)
        if(Python2_FOUND)
            set(PYTHON_EXECUTABLE ${Python2_EXECUTABLE})
            set(PYTHON_VERSION_MAJOR ${Python2_VERSION_MAJOR})
            gnsssdr_python_check_module("python >= ${GNSSSDR_PYTHON_MIN_VERSION}" sys "sys.version.split()[0] >= '${GNSSSDR_PYTHON_MIN_VERSION}'" PYTHON_MIN_VER_FOUND)
            gnsssdr_python_check_module("mako >= ${GNSSSDR_MAKO_MIN_VERSION}" mako "mako.__version__ >= '${GNSSSDR_MAKO_MIN_VERSION}'" MAKO_FOUND)
            gnsssdr_python_check_module("six - python 2 and 3 compatibility library" six "True" SIX_FOUND)
        endif()
        if(NOT MAKO_FOUND OR NOT SIX_FOUND)
            unset(PYTHON_EXECUTABLE)
            find_package(PythonInterp ${GNSSSDR_PYTHON_MIN_VERSION})
            gnsssdr_python_check_module("python >= ${GNSSSDR_PYTHON_MIN_VERSION}" sys "sys.version.split()[0] >= '${GNSSSDR_PYTHON_MIN_VERSION}'" PYTHON_MIN_VER_FOUND)
            gnsssdr_python_check_module("mako >= ${GNSSSDR_MAKO_MIN_VERSION}" mako "mako.__version__ >= '${GNSSSDR_MAKO_MIN_VERSION}'" MAKO_FOUND)
            if(PYTHON_VERSION_STRING VERSION_LESS "3.0")
                gnsssdr_python_check_module("six - python 2 and 3 compatibility library" six "True" SIX_FOUND)
            endif()
        endif()
    endif()
endif()

if(${PYTHON_VERSION_MAJOR} VERSION_EQUAL 3)
    set(PYTHON3 TRUE)
endif()

if(CMAKE_CROSSCOMPILING)
    set(QA_PYTHON_EXECUTABLE "/usr/bin/python")
else()
    set(QA_PYTHON_EXECUTABLE ${PYTHON_EXECUTABLE})
endif()

# make the path to the executable appear in the cmake gui
set(PYTHON_EXECUTABLE ${PYTHON_EXECUTABLE} CACHE FILEPATH "python interpreter")
set(QA_PYTHON_EXECUTABLE ${QA_PYTHON_EXECUTABLE} CACHE FILEPATH "python interpreter for QA tests")
