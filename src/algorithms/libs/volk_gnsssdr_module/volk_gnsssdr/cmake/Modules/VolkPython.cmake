# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# Copyright (C) 2015-2026 (see AUTHORS file for a list of contributors)
# SPDX-License-Identifier: BSD-3-Clause

if(DEFINED __INCLUDED_VOLK_PYTHON_CMAKE)
    return()
endif()
set(__INCLUDED_VOLK_PYTHON_CMAKE TRUE)

########################################################################
# Check for the existence of a Python module:
# - desc a string description of the check
# - mod the name of the module to import
# - cmd an additional command to run
# - have the result variable to set
########################################################################
macro(VOLK_PYTHON_CHECK_MODULE desc mod cmd have)
    message(STATUS "Python checking for ${desc}")
    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c
"import sys
try:
    import ${mod}
except Exception:
    try:
        ${mod}
    except Exception:
        sys.exit(1)

try:
    assert ${cmd}
except Exception:
    sys.exit(1)

sys.exit(0)"
        OUTPUT_QUIET
        ERROR_QUIET
        RESULT_VARIABLE _volk_python_check_module_result
    )
    if(_volk_python_check_module_result EQUAL 0)
        message(STATUS "Python checking for ${desc} - found")
        set(${have} TRUE)
    else()
        message(STATUS "Python checking for ${desc} - not found")
        set(${have} FALSE)
    endif()
endmacro()


########################################################################
# Check that Python itself satisfies a minimum version
########################################################################
macro(VOLK_PYTHON_CHECK_MIN_VERSION minver have)
    message(STATUS "Python checking for python >= ${minver}")
    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c
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
        OUTPUT_QUIET
        ERROR_QUIET
        RESULT_VARIABLE _volk_python_minver_result
    )
    if(_volk_python_minver_result EQUAL 0)
        message(STATUS "Python checking for python >= ${minver} - found")
        set(${have} TRUE)
    else()
        message(STATUS "Python checking for python >= ${minver} - not found")
        set(${have} FALSE)
    endif()
endmacro()


########################################################################
# Get version information for the selected interpreter
########################################################################
macro(VOLK_PYTHON_GET_VERSION)
    unset(PYTHON_VERSION_MAJOR)
    unset(PYTHON_VERSION_STRING)

    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c
                "import sys; sys.stdout.write(sys.version.split()[0])"
        OUTPUT_VARIABLE PYTHON_VERSION_STRING
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    if(NOT "${PYTHON_VERSION_STRING}" STREQUAL "")
        string(SUBSTRING "${PYTHON_VERSION_STRING}" 0 1
            PYTHON_VERSION_MAJOR)
    endif()
endmacro()


########################################################################
# Evaluate whether a candidate interpreter is suitable
#
# Conditions:
# - Python version >= VOLK_PYTHON_MIN_VERSION
# - mako >= VOLK_MAKO_MIN_VERSION
# - six present if Python 2
########################################################################
function(VOLK_PYTHON_EVALUATE_CANDIDATE candidate have)
    set(${have} FALSE PARENT_SCOPE)

    if("${candidate}" STREQUAL "")
        return()
    endif()

    if(NOT EXISTS "${candidate}")
        return()
    endif()

    set(PYTHON_EXECUTABLE "${candidate}")
    volk_python_get_version()

    if("${PYTHON_VERSION_STRING}" STREQUAL "")
        return()
    endif()

    volk_python_check_min_version(${VOLK_PYTHON_MIN_VERSION}
        _VOLK_PYTHON_MIN_VER_FOUND)
    if(NOT _VOLK_PYTHON_MIN_VER_FOUND)
        return()
    endif()

    volk_python_check_module(
        "mako >= ${VOLK_MAKO_MIN_VERSION}"
        mako
        "tuple(int(x) for x in mako.__version__.split('.')) >= tuple(int(x) for x in '${VOLK_MAKO_MIN_VERSION}'.split('.'))"
        _VOLK_MAKO_FOUND
    )
    if(NOT _VOLK_MAKO_FOUND)
        return()
    endif()

    if(PYTHON_VERSION_STRING VERSION_LESS "3.0")
        volk_python_check_module(
            "six - python 2 and 3 compatibility library"
            six
            "True"
            _VOLK_SIX_FOUND
        )
        if(NOT _VOLK_SIX_FOUND)
            return()
        endif()
    else()
        set(_VOLK_SIX_FOUND TRUE)
    endif()

    set(_VOLK_CANDIDATE_VERSION_STRING "${PYTHON_VERSION_STRING}"
        PARENT_SCOPE)
    set(_VOLK_CANDIDATE_VERSION_MAJOR "${PYTHON_VERSION_MAJOR}"
        PARENT_SCOPE)
    set(_VOLK_CANDIDATE_PYTHON_MIN_VER_FOUND
        "${_VOLK_PYTHON_MIN_VER_FOUND}" PARENT_SCOPE)
    set(_VOLK_CANDIDATE_MAKO_FOUND "${_VOLK_MAKO_FOUND}"
        PARENT_SCOPE)
    set(_VOLK_CANDIDATE_SIX_FOUND "${_VOLK_SIX_FOUND}"
        PARENT_SCOPE)
    set(${have} TRUE PARENT_SCOPE)
endfunction()


########################################################################
# Setup the Python interpreter
########################################################################
set(VOLK_PYTHON_MIN_VERSION "2.7")
set(VOLK_PYTHON3_MIN_VERSION "3.4")
set(VOLK_MAKO_MIN_VERSION "0.4.2")

set(PYTHONINTERP_FOUND FALSE)
set(PYTHON_MIN_VER_FOUND FALSE)
set(MAKO_FOUND FALSE)
set(SIX_FOUND FALSE)
unset(PYTHON_VERSION_MAJOR)
unset(PYTHON_VERSION_STRING)
unset(PYTHON3)

set(_VOLK_USER_PYTHON_EXECUTABLE "")
set(_VOLK_PYTHON_CANDIDATES "")
set(_VOLK_SELECTED_PYTHON "")

if(DEFINED PYTHON_EXECUTABLE AND NOT "${PYTHON_EXECUTABLE}" STREQUAL "")
    set(_VOLK_USER_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
endif()


########################################################################
# Build candidate list
########################################################################

# 1) User-provided interpreter first
if(NOT "${_VOLK_USER_PYTHON_EXECUTABLE}" STREQUAL "")
    message(STATUS "User set python executable ${_VOLK_USER_PYTHON_EXECUTABLE}")
    list(APPEND _VOLK_PYTHON_CANDIDATES "${_VOLK_USER_PYTHON_EXECUTABLE}")
endif()

# 2) Modern CMake: prefer Python3, then Python2
if(NOT CMAKE_VERSION VERSION_LESS "3.12" AND NOT CMAKE_CROSSCOMPILING)
    if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
        set(_volk_previous_cmake_find_framework ${CMAKE_FIND_FRAMEWORK})
        set(CMAKE_FIND_FRAMEWORK LAST)
    endif()

    find_package(Python3 COMPONENTS Interpreter)
    if(Python3_FOUND)
        list(APPEND _VOLK_PYTHON_CANDIDATES "${Python3_EXECUTABLE}")
    else()
        find_package(Python2 COMPONENTS Interpreter)
        if(Python2_FOUND)
            list(APPEND _VOLK_PYTHON_CANDIDATES "${Python2_EXECUTABLE}")
        endif()
    endif()

    if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
        set(CMAKE_FIND_FRAMEWORK ${_volk_previous_cmake_find_framework})
    endif()

# 3) Old CMake or cross-compiling: use PythonInterp as a candidate source,
#    but still validate the interpreter before accepting it
else()
    set(Python_ADDITIONAL_VERSIONS
        3.11 3.10 3.9 3.8 3.7 3.6 3.5 3.4 2.7
    )

    if(CMAKE_VERSION VERSION_LESS "2.8.12")
        find_package(PythonInterp)
    else()
        find_package(PythonInterp ${VOLK_PYTHON_MIN_VERSION})
    endif()

    if(PYTHONINTERP_FOUND AND NOT "${PYTHON_EXECUTABLE}" STREQUAL "")
        list(APPEND _VOLK_PYTHON_CANDIDATES "${PYTHON_EXECUTABLE}")
    endif()
endif()

# 4) Generic last-resort program-name fallbacks
find_program(_VOLK_PYTHON3 NAMES python3)
find_program(_VOLK_PYTHON2 NAMES python2.7 python2)
find_program(_VOLK_PYTHON NAMES python)

foreach(_volk_python_prog
    ${_VOLK_PYTHON3}
    ${_VOLK_PYTHON2}
    ${_VOLK_PYTHON}
)
    if(NOT "${_volk_python_prog}" STREQUAL "")
        list(APPEND _VOLK_PYTHON_CANDIDATES "${_volk_python_prog}")
    endif()
endforeach()

list(REMOVE_DUPLICATES _VOLK_PYTHON_CANDIDATES)


########################################################################
# Evaluate candidates and accept the first fully suitable interpreter
########################################################################
foreach(_VOLK_PYTHON_CANDIDATE IN LISTS _VOLK_PYTHON_CANDIDATES)
    message(STATUS "Trying Python candidate: ${_VOLK_PYTHON_CANDIDATE}")
    volk_python_evaluate_candidate("${_VOLK_PYTHON_CANDIDATE}"
        _VOLK_PYTHON_CANDIDATE_OK)
    if(_VOLK_PYTHON_CANDIDATE_OK)
        set(_VOLK_SELECTED_PYTHON "${_VOLK_PYTHON_CANDIDATE}")
        break()
    endif()
endforeach()

if(NOT "${_VOLK_SELECTED_PYTHON}" STREQUAL "")
    set(PYTHON_EXECUTABLE "${_VOLK_SELECTED_PYTHON}")
    set(PYTHONINTERP_FOUND TRUE)
    set(PYTHON_VERSION_STRING "${_VOLK_CANDIDATE_VERSION_STRING}")
    set(PYTHON_VERSION_MAJOR "${_VOLK_CANDIDATE_VERSION_MAJOR}")
    set(PYTHON_MIN_VER_FOUND "${_VOLK_CANDIDATE_PYTHON_MIN_VER_FOUND}")
    set(MAKO_FOUND "${_VOLK_CANDIDATE_MAKO_FOUND}")
    set(SIX_FOUND "${_VOLK_CANDIDATE_SIX_FOUND}")
else()
    set(PYTHONINTERP_FOUND FALSE)
    unset(PYTHON_EXECUTABLE)
    unset(PYTHON_VERSION_MAJOR)
    unset(PYTHON_VERSION_STRING)
    set(PYTHON_MIN_VER_FOUND FALSE)
    set(MAKO_FOUND FALSE)
    set(SIX_FOUND FALSE)
endif()

if("${PYTHON_VERSION_MAJOR}" STREQUAL "3")
    set(PYTHON3 TRUE)
endif()


########################################################################
# Sets the Python installation directory VOLK_PYTHON_DIR
########################################################################
if(NOT DEFINED VOLK_PYTHON_DIR)
    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c
"import os
import sys
if os.name == 'posix':
    print(os.path.join(
        'lib',
        'python{0}.{1}'.format(sys.version_info[0], sys.version_info[1]),
        'dist-packages'))
elif os.name == 'nt':
    print(os.path.join('Lib', 'site-packages'))"
        OUTPUT_VARIABLE VOLK_PYTHON_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
endif()
file(TO_CMAKE_PATH "${VOLK_PYTHON_DIR}" VOLK_PYTHON_DIR)


########################################################################
# Create an always-built target with a unique name
# Usage: volk_unique_target(<description> <dependencies list>)
########################################################################
function(VOLK_UNIQUE_TARGET desc)
    file(RELATIVE_PATH reldir "${PROJECT_BINARY_DIR}"
        "${CMAKE_CURRENT_BINARY_DIR}")
    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c
"import re, hashlib
unique = hashlib.sha256(b'${reldir}${ARGN}').hexdigest()[:5]
print(re.sub(r'\\W', '_', '${desc} ${reldir} ' + unique))"
        OUTPUT_VARIABLE _target
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    add_custom_target(${_target} ALL DEPENDS ${ARGN})
endfunction()


########################################################################
# Install Python sources (also builds and installs byte-compiled Python)
########################################################################
function(VOLK_PYTHON_INSTALL)
    include(CMakeParseArgumentsCopy)
    cmake_parse_arguments(
        VOLK_PYTHON_INSTALL
        ""
        "DESTINATION;COMPONENT"
        "FILES;PROGRAMS"
        ${ARGN}
    )

    if(VOLK_PYTHON_INSTALL_FILES)
        install(${ARGN})

        unset(pysrcfiles)
        unset(pycfiles)
        unset(pyofiles)

        foreach(pyfile ${VOLK_PYTHON_INSTALL_FILES})
            get_filename_component(pyfile "${pyfile}" ABSOLUTE)
            list(APPEND pysrcfiles "${pyfile}")

            file(RELATIVE_PATH source_rel_path
                "${CMAKE_CURRENT_SOURCE_DIR}" "${pyfile}")
            string(LENGTH "${source_rel_path}" source_rel_path_len)

            file(RELATIVE_PATH binary_rel_path
                "${CMAKE_CURRENT_BINARY_DIR}" "${pyfile}")
            string(LENGTH "${binary_rel_path}" binary_rel_path_len)

            if(${source_rel_path_len} GREATER ${binary_rel_path_len})
                set(pygenfile "${CMAKE_CURRENT_BINARY_DIR}/${binary_rel_path}")
            else()
                set(pygenfile "${CMAKE_CURRENT_BINARY_DIR}/${source_rel_path}")
            endif()

            list(APPEND pycfiles "${pygenfile}c")
            list(APPEND pyofiles "${pygenfile}o")

            get_filename_component(pygen_path "${pygenfile}" PATH)
            file(MAKE_DIRECTORY "${pygen_path}")
        endforeach()

        add_custom_command(
            DEPENDS ${pysrcfiles}
            OUTPUT ${pycfiles}
            COMMAND ${PYTHON_EXECUTABLE}
                    ${PROJECT_BINARY_DIR}/python_compile_helper.py
                    ${pysrcfiles} ${pycfiles}
        )

        add_custom_command(
            DEPENDS ${pysrcfiles}
            OUTPUT ${pyofiles}
            COMMAND ${PYTHON_EXECUTABLE} -O
                    ${PROJECT_BINARY_DIR}/python_compile_helper.py
                    ${pysrcfiles} ${pyofiles}
        )

        set(python_install_gen_targets ${pycfiles} ${pyofiles})

        install(FILES ${python_install_gen_targets}
            DESTINATION ${VOLK_PYTHON_INSTALL_DESTINATION}
            COMPONENT ${VOLK_PYTHON_INSTALL_COMPONENT}
        )

    elseif(VOLK_PYTHON_INSTALL_PROGRAMS)
        file(TO_NATIVE_PATH "${PYTHON_EXECUTABLE}" pyexe_native)

        if(CMAKE_CROSSCOMPILING)
            set(pyexe_native "/usr/bin/env python")
        endif()

        foreach(pyfile ${VOLK_PYTHON_INSTALL_PROGRAMS})
            get_filename_component(pyfile_name "${pyfile}" NAME)
            get_filename_component(pyfile "${pyfile}" ABSOLUTE)

            string(REPLACE "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}"
                pyexefile "${pyfile}.exe")
            list(APPEND python_install_gen_targets "${pyexefile}")

            get_filename_component(pyexefile_path "${pyexefile}" PATH)
            file(MAKE_DIRECTORY "${pyexefile_path}")

            add_custom_command(
                OUTPUT ${pyexefile}
                DEPENDS ${pyfile}
                COMMAND ${PYTHON_EXECUTABLE} -c
                        "open('${pyexefile}','w').write(r'\#!${pyexe_native}'+'\\n'+open('${pyfile}').read())"
                COMMENT "Shebangin ${pyfile_name}"
                VERBATIM
            )

            get_filename_component(pyfile_ext "${pyfile}" EXT)
            if(WIN32 AND NOT pyfile_ext)
                set(pyfile_name "${pyfile_name}.py")
            endif()

            install(PROGRAMS ${pyexefile}
                RENAME ${pyfile_name}
                DESTINATION ${VOLK_PYTHON_INSTALL_DESTINATION}
                COMPONENT ${VOLK_PYTHON_INSTALL_COMPONENT}
            )
        endforeach()
    endif()

    volk_unique_target("pygen" ${python_install_gen_targets})
endfunction()


########################################################################
# Write the Python helper script that generates byte code files
########################################################################
file(WRITE ${PROJECT_BINARY_DIR}/python_compile_helper.py
"import sys, py_compile
files = sys.argv[1:]
srcs, gens = files[:len(files)//2], files[len(files)//2:]
for src, gen in zip(srcs, gens):
    py_compile.compile(file=src, cfile=gen, doraise=True)
"
)
