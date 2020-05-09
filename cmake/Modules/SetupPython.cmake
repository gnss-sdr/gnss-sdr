# Copyright (C) 2011-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later


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
endmacro()

macro(GNSSSDR_PYTHON_CHECK_MODULE desc mod cmd have)
    gnsssdr_python_check_module_raw(
        "${desc}" "
#########################################
try:
    import ${mod}
    assert ${cmd}
except (ImportError, AssertionError): exit(-1)
except: pass
#########################################"
    "${have}")
endmacro()


########################################################################
# Setup the python interpreter:
# This allows the user to specify a specific interpreter,
# or finds the interpreter via the built-in cmake module.
########################################################################

if(CMAKE_VERSION VERSION_LESS 3.12)
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
        gnsssdr_python_check_module("six - python 2 and 3 compatibility library" six "True" SIX_FOUND)
    else()
        message(STATUS "PYTHON_EXECUTABLE not set - trying by default python3")
        message(STATUS "Use -DPYTHON_EXECUTABLE=/path/to/python to build for python 2.7")
        set(Python_ADDITIONAL_VERSIONS 3.4 3.5 3.6 3.7 3.8 3.9)
        find_package(PythonInterp ${GNSSSDR_PYTHON_MIN3_VERSION})
        if(NOT PYTHONINTERP_FOUND)
            message(STATUS "python3 not found - trying with python2.7")
            find_package(PythonInterp ${GNSSSDR_PYTHON_MIN_VERSION} REQUIRED)
        endif()
        gnsssdr_python_check_module("python >= ${GNSSSDR_PYTHON_MIN_VERSION}" sys "sys.version.split()[0] >= '${GNSSSDR_PYTHON_MIN_VERSION}'" PYTHON_MIN_VER_FOUND)
        gnsssdr_python_check_module("mako >= ${GNSSSDR_MAKO_MIN_VERSION}" mako "mako.__version__ >= '${GNSSSDR_MAKO_MIN_VERSION}'" MAKO_FOUND)
        gnsssdr_python_check_module("six - python 2 and 3 compatibility library" six "True" SIX_FOUND)
    endif()
else()
    find_package(Python3 COMPONENTS Interpreter)
    if(Python3_FOUND)
        set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
        set(PYTHON_VERSION_MAJOR ${Python3_VERSION_MAJOR})
        gnsssdr_python_check_module("python >= ${GNSSSDR_PYTHON_MIN_VERSION}" sys "sys.version.split()[0] >= '${GNSSSDR_PYTHON_MIN_VERSION}'" PYTHON_MIN_VER_FOUND)
        gnsssdr_python_check_module("mako >= ${GNSSSDR_MAKO_MIN_VERSION}" mako "mako.__version__ >= '${GNSSSDR_MAKO_MIN_VERSION}'" MAKO_FOUND)
        gnsssdr_python_check_module("six - python 2 and 3 compatibility library" six "True" SIX_FOUND)
    endif()
    if(NOT Python3_FOUND OR NOT MAKO_FOUND OR NOT SIX_FOUND)
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
            gnsssdr_python_check_module("six - python 2 and 3 compatibility library" six "True" SIX_FOUND)
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
