# Copyright (C) 2015-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later

if(DEFINED __INCLUDED_VOLK_PYTHON_CMAKE)
    return()
endif()
set(__INCLUDED_VOLK_PYTHON_CMAKE TRUE)

########################################################################
# Check for the existence of a python module:
# - desc a string description of the check
# - mod the name of the module to import
# - cmd an additional command to run
# - have the result variable to set
########################################################################
macro(VOLK_PYTHON_CHECK_MODULE_RAW desc python_code have)
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

macro(VOLK_PYTHON_CHECK_MODULE desc mod cmd have)
    volk_python_check_module_raw(
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
set(VOLK_PYTHON_MIN_VERSION "2.7")
set(VOLK_PYTHON3_MIN_VERSION "3.4")

if(CMAKE_VERSION VERSION_LESS 3.12)
    if(PYTHON_EXECUTABLE)
        message(STATUS "User set python executable ${PYTHON_EXECUTABLE}")
        find_package(PythonInterp ${VOLK_PYTHON_MIN_VERSION} REQUIRED)
    else()
        message(STATUS "PYTHON_EXECUTABLE not set - trying by default python3")
        message(STATUS "Use -DPYTHON_EXECUTABLE=/path/to/python to build for python 2.7")
        set(Python_ADDITIONAL_VERSIONS 3.4 3.5 3.6 3.7 3.8 3.9)
        find_package(PythonInterp ${VOLK_PYTHON_MIN3_VERSION})
        if(NOT PYTHONINTERP_FOUND)
            message(STATUS "python3 not found - trying with python2.7")
            find_package(PythonInterp ${VOLK_PYTHON_MIN_VERSION} REQUIRED)
        endif()
    endif()
else()
    if(PYTHON_EXECUTABLE)
        message(STATUS "User set python executable ${PYTHON_EXECUTABLE}")
        find_package(PythonInterp ${VOLK_PYTHON_MIN_VERSION} REQUIRED)
    else()
        find_package(Python3 COMPONENTS Interpreter)
        if(Python3_FOUND)
            set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
            set(PYTHON_VERSION_MAJOR ${Python3_VERSION_MAJOR})
            volk_python_check_module("mako >= 0.4.2" mako "mako.__version__ >= '0.4.2'" MAKO_FOUND)
            volk_python_check_module("six - python 2 and 3 compatibility library" six "True" SIX_FOUND)
        endif()
        if(NOT Python3_FOUND OR NOT MAKO_FOUND OR NOT SIX_FOUND)
            find_package(Python2 COMPONENTS Interpreter)
            if(Python2_FOUND)
                set(PYTHON_EXECUTABLE ${Python2_EXECUTABLE})
                set(PYTHON_VERSION_MAJOR ${Python2_VERSION_MAJOR})
                volk_python_check_module("mako >= 0.4.2" mako "mako.__version__ >= '0.4.2'" MAKO_FOUND)
                volk_python_check_module("six - python 2 and 3 compatibility library" six "True" SIX_FOUND)
            endif()
            if(NOT MAKO_FOUND OR NOT SIX_FOUND)
                unset(PYTHON_EXECUTABLE)
                find_package(PythonInterp ${VOLK_PYTHON_MIN_VERSION})
            endif()
        endif()
    endif()
endif()

if(${PYTHON_VERSION_MAJOR} VERSION_EQUAL 3)
    set(PYTHON3 TRUE)
endif()


########################################################################
# Sets the python installation directory VOLK_PYTHON_DIR
########################################################################
if(NOT DEFINED VOLK_PYTHON_DIR)
execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "
import os
import sys
if os.name == 'posix':
    print(os.path.join('lib', 'python' + sys.version[:3], 'dist-packages'))
if os.name == 'nt':
    print(os.path.join('Lib', 'site-packages'))
" OUTPUT_VARIABLE VOLK_PYTHON_DIR OUTPUT_STRIP_TRAILING_WHITESPACE
)
endif()
file(TO_CMAKE_PATH ${VOLK_PYTHON_DIR} VOLK_PYTHON_DIR)

########################################################################
# Create an always-built target with a unique name
# Usage: VOLK_UNIQUE_TARGET(<description> <dependencies list>)
########################################################################
function(VOLK_UNIQUE_TARGET desc)
    file(RELATIVE_PATH reldir ${PROJECT_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR})
    execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import re, hashlib
unique = hashlib.sha256(b'${reldir}${ARGN}').hexdigest()[:5]
print(re.sub('\\W', '_', '${desc} ${reldir} ' + unique))"
    OUTPUT_VARIABLE _target OUTPUT_STRIP_TRAILING_WHITESPACE)
    add_custom_target(${_target} ALL DEPENDS ${ARGN})
endfunction()

########################################################################
# Install python sources (also builds and installs byte-compiled python)
########################################################################
function(VOLK_PYTHON_INSTALL)
    include(CMakeParseArgumentsCopy)
    cmake_parse_arguments(VOLK_PYTHON_INSTALL "" "DESTINATION;COMPONENT" "FILES;PROGRAMS" ${ARGN})

    ####################################################################
    if(VOLK_PYTHON_INSTALL_FILES)
    ####################################################################
        install(${ARGN}) #installs regular python files

        #create a list of all generated files
        unset(pysrcfiles)
        unset(pycfiles)
        unset(pyofiles)
        foreach(pyfile ${VOLK_PYTHON_INSTALL_FILES})
            get_filename_component(pyfile ${pyfile} ABSOLUTE)
            list(APPEND pysrcfiles ${pyfile})

            #determine if this file is in the source or binary directory
            file(RELATIVE_PATH source_rel_path ${CMAKE_CURRENT_SOURCE_DIR} ${pyfile})
            string(LENGTH "${source_rel_path}" source_rel_path_len)
            file(RELATIVE_PATH binary_rel_path ${CMAKE_CURRENT_BINARY_DIR} ${pyfile})
            string(LENGTH "${binary_rel_path}" binary_rel_path_len)

            #and set the generated path appropriately
            if(${source_rel_path_len} GREATER ${binary_rel_path_len})
                set(pygenfile ${CMAKE_CURRENT_BINARY_DIR}/${binary_rel_path})
            else()
                set(pygenfile ${CMAKE_CURRENT_BINARY_DIR}/${source_rel_path})
            endif()
            list(APPEND pycfiles ${pygenfile}c)
            list(APPEND pyofiles ${pygenfile}o)

            #ensure generation path exists
            get_filename_component(pygen_path ${pygenfile} PATH)
            file(MAKE_DIRECTORY ${pygen_path})

        endforeach()

        #the command to generate the pyc files
        add_custom_command(
            DEPENDS ${pysrcfiles} OUTPUT ${pycfiles}
            COMMAND ${PYTHON_EXECUTABLE} ${PROJECT_BINARY_DIR}/python_compile_helper.py ${pysrcfiles} ${pycfiles}
        )

        #the command to generate the pyo files
        add_custom_command(
            DEPENDS ${pysrcfiles} OUTPUT ${pyofiles}
            COMMAND ${PYTHON_EXECUTABLE} -O ${PROJECT_BINARY_DIR}/python_compile_helper.py ${pysrcfiles} ${pyofiles}
        )

        #create install rule and add generated files to target list
        set(python_install_gen_targets ${pycfiles} ${pyofiles})
        install(FILES ${python_install_gen_targets}
            DESTINATION ${VOLK_PYTHON_INSTALL_DESTINATION}
            COMPONENT ${VOLK_PYTHON_INSTALL_COMPONENT}
        )


    ####################################################################
    elseif(VOLK_PYTHON_INSTALL_PROGRAMS)
    ####################################################################
        file(TO_NATIVE_PATH ${PYTHON_EXECUTABLE} pyexe_native)

        if(CMAKE_CROSSCOMPILING)
            set(pyexe_native "/usr/bin/env python")
        endif()

        foreach(pyfile ${VOLK_PYTHON_INSTALL_PROGRAMS})
            get_filename_component(pyfile_name ${pyfile} NAME)
            get_filename_component(pyfile ${pyfile} ABSOLUTE)
            string(REPLACE "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}" pyexefile "${pyfile}.exe")
            list(APPEND python_install_gen_targets ${pyexefile})

            get_filename_component(pyexefile_path ${pyexefile} PATH)
            file(MAKE_DIRECTORY ${pyexefile_path})

            add_custom_command(
                OUTPUT ${pyexefile} DEPENDS ${pyfile}
                COMMAND ${PYTHON_EXECUTABLE} -c
                "open('${pyexefile}','w').write(r'\#!${pyexe_native}'+'\\n'+open('${pyfile}').read())"
                COMMENT "Shebangin ${pyfile_name}"
                VERBATIM
            )

            #on windows, python files need an extension to execute
            get_filename_component(pyfile_ext ${pyfile} EXT)
            if(WIN32 AND NOT pyfile_ext)
                set(pyfile_name "${pyfile_name}.py")
            endif()

            install(PROGRAMS ${pyexefile} RENAME ${pyfile_name}
                DESTINATION ${VOLK_PYTHON_INSTALL_DESTINATION}
                COMPONENT ${VOLK_PYTHON_INSTALL_COMPONENT}
            )
        endforeach()

    endif()

    volk_unique_target("pygen" ${python_install_gen_targets})

endfunction()

########################################################################
# Write the python helper script that generates byte code files
########################################################################
file(WRITE ${PROJECT_BINARY_DIR}/python_compile_helper.py "
import sys, py_compile
files = sys.argv[1:]
srcs, gens = files[:len(files)//2], files[len(files)//2:]
for src, gen in zip(srcs, gens):
    py_compile.compile(file=src, cfile=gen, doraise=True)
")
