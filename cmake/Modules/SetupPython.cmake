# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2026 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


########################################################################
# Internal helper:
# Run a Python snippet with the selected interpreter.
#
# Usage:
#   gnsssdr_python_run("<python code>" result_var)
#
# The Python code must exit with code 0 on success and non-zero on failure.
########################################################################
macro(GNSSSDR_PYTHON_RUN code result_var)
    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c "${code}"
        OUTPUT_QUIET
        ERROR_QUIET
        RESULT_VARIABLE _gnsssdr_python_run_result
    )
    set(${result_var} ${_gnsssdr_python_run_result})
endmacro()


########################################################################
# Internal helper:
# Read Python version information from the selected interpreter.
#
# Sets:
#   PYTHON_VERSION_STRING
#   PYTHON_VERSION_MAJOR
########################################################################
macro(GNSSSDR_PYTHON_GET_VERSION)
    unset(PYTHON_VERSION_STRING)
    unset(PYTHON_VERSION_MAJOR)
    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c
                "import sys; sys.stdout.write(sys.version.split()[0])"
        OUTPUT_VARIABLE PYTHON_VERSION_STRING
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(NOT "${PYTHON_VERSION_STRING}" STREQUAL "")
        string(SUBSTRING "${PYTHON_VERSION_STRING}" 0 1 PYTHON_VERSION_MAJOR)
    endif()
endmacro()


########################################################################
# Check that Python satisfies a minimum version.
#
# Arguments:
#   minver   Minimum version string, e.g. 2.7 or 3.6
#   have     Output variable set to TRUE/FALSE
########################################################################
macro(GNSSSDR_PYTHON_CHECK_MIN_VERSION minver have)
    message(STATUS "Python checking for python >= ${minver}")
    gnsssdr_python_run(
"import sys

def norm(v):
    out = []
    for p in str(v).split('.'):
        n = ''.join([c for c in p if '0' <= c <= '9'])
        out.append(int(n or '0'))
    while len(out) < 3:
        out.append(0)
    return tuple(out)

sys.exit(0 if norm(sys.version.split()[0]) >= norm('${minver}') else 1)"
        _gnsssdr_python_minver_result
    )
    if(_gnsssdr_python_minver_result EQUAL 0)
        message(STATUS "Python checking for python >= ${minver} - found")
        set(${have} TRUE)
    else()
        message(STATUS "Python checking for python >= ${minver} - not found")
        set(${have} FALSE)
    endif()
endmacro()


########################################################################
# Check for a Python module, optionally with a minimum version.
#
# Arguments:
#   desc     Human-readable description
#   mod      Module name to import
#   minver   Minimum module version, or empty string to skip version check
#   have     Output variable set to TRUE/FALSE
#
########################################################################
macro(GNSSSDR_PYTHON_CHECK_MODULE desc mod minver have)
    message(STATUS "Python checking for ${desc}")
    if("${minver}" STREQUAL "")
        gnsssdr_python_run(
"import sys
try:
    import ${mod}
except Exception:
    sys.exit(1)
sys.exit(0)"
            _gnsssdr_python_module_result
        )
    else()
        gnsssdr_python_run(
"import sys
try:
    import ${mod}
except Exception:
    sys.exit(1)

def norm(v):
    out = []
    for p in str(v).split('.'):
        n = ''.join([c for c in p if '0' <= c <= '9'])
        out.append(int(n or '0'))
    while len(out) < 3:
        out.append(0)
    return tuple(out)

try:
    modver = ${mod}.__version__
except Exception:
    sys.exit(1)

sys.exit(0 if norm(modver) >= norm('${minver}') else 1)"
            _gnsssdr_python_module_result
        )
    endif()
    if(_gnsssdr_python_module_result EQUAL 0)
        message(STATUS "Python checking for ${desc} - found")
        set(${have} TRUE)
    else()
        message(STATUS "Python checking for ${desc} - not found")
        set(${have} FALSE)
    endif()
endmacro()


########################################################################
# Internal helper:
# Test whether a given interpreter is suitable.
#
# Conditions:
# - Python version >= GNSSSDR_PYTHON_MIN_VERSION
# - mako >= GNSSSDR_MAKO_MIN_VERSION
# - six present if Python 2
#
# Arguments:
#   candidate   Full path to Python interpreter
#   have        Output variable set to TRUE/FALSE
########################################################################
function(GNSSSDR_PYTHON_EVALUATE_CANDIDATE candidate have)
    set(${have} FALSE PARENT_SCOPE)

    if("${candidate}" STREQUAL "")
        return()
    endif()

    if(NOT EXISTS "${candidate}")
        return()
    endif()

    set(PYTHON_EXECUTABLE "${candidate}")
    gnsssdr_python_get_version()

    if("${PYTHON_VERSION_STRING}" STREQUAL "")
        return()
    endif()

    gnsssdr_python_check_min_version(
        ${GNSSSDR_PYTHON_MIN_VERSION}
        _GNSSSDR_PYTHON_MIN_VER_FOUND
    )
    if(NOT _GNSSSDR_PYTHON_MIN_VER_FOUND)
        return()
    endif()

    gnsssdr_python_check_module(
        "mako >= ${GNSSSDR_MAKO_MIN_VERSION}"
        mako
        ${GNSSSDR_MAKO_MIN_VERSION}
        _GNSSSDR_MAKO_FOUND
    )
    if(NOT _GNSSSDR_MAKO_FOUND)
        return()
    endif()

    if(PYTHON_VERSION_STRING VERSION_LESS "3.0")
        gnsssdr_python_check_module(
            "six - python 2 and 3 compatibility library"
            six
            ""
            _GNSSSDR_SIX_FOUND
        )
        if(NOT _GNSSSDR_SIX_FOUND)
            return()
        endif()
    else()
        set(_GNSSSDR_SIX_FOUND TRUE)
    endif()

    set(_GNSSSDR_CANDIDATE_VERSION_STRING "${PYTHON_VERSION_STRING}"
        PARENT_SCOPE)
    set(_GNSSSDR_CANDIDATE_VERSION_MAJOR "${PYTHON_VERSION_MAJOR}"
        PARENT_SCOPE)
    set(_GNSSSDR_CANDIDATE_PYTHON_MIN_VER_FOUND
        "${_GNSSSDR_PYTHON_MIN_VER_FOUND}" PARENT_SCOPE)
    set(_GNSSSDR_CANDIDATE_MAKO_FOUND "${_GNSSSDR_MAKO_FOUND}"
        PARENT_SCOPE)
    set(_GNSSSDR_CANDIDATE_SIX_FOUND "${_GNSSSDR_SIX_FOUND}"
        PARENT_SCOPE)
    set(${have} TRUE PARENT_SCOPE)
endfunction()


########################################################################
# Select the Python interpreter
#
# Strategy:
# 1) Respect user-provided PYTHON_EXECUTABLE
# 2) On modern CMake, try Python3 then Python2
# 3) On old CMake, collect candidates from PythonInterp and explicit names
# 4) Last resort: find_program()
#
# The selected interpreter must satisfy all required module checks.
########################################################################
set(_GNSSSDR_PYTHON_CANDIDATES "")
set(_GNSSSDR_SELECTED_PYTHON "")
set(_GNSSSDR_USER_PYTHON_EXECUTABLE "")

if(DEFINED PYTHON_EXECUTABLE AND NOT "${PYTHON_EXECUTABLE}" STREQUAL "")
    set(_GNSSSDR_USER_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
endif()

set(PYTHONINTERP_FOUND FALSE)
set(PYTHON_MIN_VER_FOUND FALSE)
set(MAKO_FOUND FALSE)
set(SIX_FOUND FALSE)
unset(PYTHON_VERSION_MAJOR)
unset(PYTHON_VERSION_STRING)
unset(PYTHON3)


########################################################################
# Build candidate list
########################################################################

# 1) Respect user-provided interpreter first
if(NOT "${_GNSSSDR_USER_PYTHON_EXECUTABLE}" STREQUAL "")
    message(STATUS
        "User set python executable ${_GNSSSDR_USER_PYTHON_EXECUTABLE}"
    )
    list(APPEND _GNSSSDR_PYTHON_CANDIDATES
        "${_GNSSSDR_USER_PYTHON_EXECUTABLE}"
    )
endif()

# 2) Modern CMake (>= 3.12): prefer Python3, then Python2
if(NOT CMAKE_VERSION VERSION_LESS "3.12" AND NOT CMAKE_CROSSCOMPILING)
    if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
        set(_GNSSSDR_PREVIOUS_CMAKE_FIND_FRAMEWORK ${CMAKE_FIND_FRAMEWORK})
        set(CMAKE_FIND_FRAMEWORK LAST)
    endif()

    unset(Python3_EXECUTABLE)
    unset(Python2_EXECUTABLE)
    unset(Python3_FOUND)
    unset(Python2_FOUND)

    find_package(Python3 COMPONENTS Interpreter)
    if(Python3_FOUND)
        list(APPEND _GNSSSDR_PYTHON_CANDIDATES "${Python3_EXECUTABLE}")
    else()
        find_package(Python2 COMPONENTS Interpreter)
        if(Python2_FOUND)
            list(APPEND _GNSSSDR_PYTHON_CANDIDATES "${Python2_EXECUTABLE}")
        endif()
    endif()

    if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
        set(CMAKE_FIND_FRAMEWORK ${_GNSSSDR_PREVIOUS_CMAKE_FIND_FRAMEWORK})
    endif()

# 3) Old CMake: use PythonInterp as a candidate source,
#    but still validate the interpreter before accepting it
else()
    set(Python_ADDITIONAL_VERSIONS
        3.11 3.10 3.9 3.8 3.7 3.6 3.5 3.4 2.7
    )
    if(CMAKE_VERSION VERSION_LESS "2.8.12")
        find_package(PythonInterp)
    else()
        find_package(PythonInterp ${GNSSSDR_PYTHON_MIN_VERSION})
    endif()
    if(PYTHONINTERP_FOUND AND NOT "${PYTHON_EXECUTABLE}" STREQUAL "")
        list(APPEND _GNSSSDR_PYTHON_CANDIDATES "${PYTHON_EXECUTABLE}")
    endif()
endif()

# 4) Last-resort program-name fallbacks, in preferred order
find_program(_GNSSSDR_PYTHON3 NAMES python3)
find_program(_GNSSSDR_PYTHON2 NAMES python2.7 python2)
find_program(_GNSSSDR_PYTHON  NAMES python)

foreach(_gnsssdr_python_prog
    ${_GNSSSDR_PYTHON3}
    ${_GNSSSDR_PYTHON2}
    ${_GNSSSDR_PYTHON}
)
    if(NOT "${_gnsssdr_python_prog}" STREQUAL "")
        list(APPEND _GNSSSDR_PYTHON_CANDIDATES "${_gnsssdr_python_prog}")
    endif()
endforeach()

list(REMOVE_DUPLICATES _GNSSSDR_PYTHON_CANDIDATES)


########################################################################
# Evaluate candidates and accept the first fully suitable interpreter
########################################################################
foreach(_GNSSSDR_PYTHON_CANDIDATE IN LISTS _GNSSSDR_PYTHON_CANDIDATES)
    message(STATUS
        "Trying Python candidate: ${_GNSSSDR_PYTHON_CANDIDATE}")
    gnsssdr_python_evaluate_candidate(
        "${_GNSSSDR_PYTHON_CANDIDATE}"
        _GNSSSDR_PYTHON_CANDIDATE_OK
    )
    if(_GNSSSDR_PYTHON_CANDIDATE_OK)
        set(_GNSSSDR_SELECTED_PYTHON "${_GNSSSDR_PYTHON_CANDIDATE}")
        break()
    endif()
endforeach()

if(NOT "${_GNSSSDR_SELECTED_PYTHON}" STREQUAL "")
    set(PYTHON_EXECUTABLE "${_GNSSSDR_SELECTED_PYTHON}")
    set(PYTHONINTERP_FOUND TRUE)
    set(PYTHON_VERSION_STRING "${_GNSSSDR_CANDIDATE_VERSION_STRING}")
    set(PYTHON_VERSION_MAJOR "${_GNSSSDR_CANDIDATE_VERSION_MAJOR}")
    set(PYTHON_MIN_VER_FOUND "${_GNSSSDR_CANDIDATE_PYTHON_MIN_VER_FOUND}")
    set(MAKO_FOUND "${_GNSSSDR_CANDIDATE_MAKO_FOUND}")
    set(SIX_FOUND "${_GNSSSDR_CANDIDATE_SIX_FOUND}")
else()
    set(PYTHONINTERP_FOUND FALSE)
    unset(PYTHON_EXECUTABLE)
    unset(PYTHON_VERSION_MAJOR)
    unset(PYTHON_VERSION_STRING)
    set(PYTHON_MIN_VER_FOUND FALSE)
    set(MAKO_FOUND FALSE)
    set(SIX_FOUND FALSE)
endif()


########################################################################
# Set convenience variables
########################################################################
if(DEFINED PYTHON_VERSION_MAJOR AND "${PYTHON_VERSION_MAJOR}" STREQUAL "3")
    set(PYTHON3 TRUE)
endif()

if(CMAKE_CROSSCOMPILING)
    set(QA_PYTHON_EXECUTABLE "/usr/bin/python")
else()
    set(QA_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
endif()

# Make the path to the executable appear in the CMake GUI
set(PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}" CACHE FILEPATH
    "python interpreter"
)
set(QA_PYTHON_EXECUTABLE "${QA_PYTHON_EXECUTABLE}" CACHE FILEPATH
    "python interpreter for QA tests"
)
