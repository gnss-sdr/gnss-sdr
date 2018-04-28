# Copyright 2015 Free Software Foundation, Inc.
#
# This file is part of Volk
#
# Volk is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# Volk is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
# License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Volk; see the file COPYING.  If not, write to the Free
# Software Foundation, Inc., 51 Franklin Street, Boston, MA
# 02110-1301, USA.

if(DEFINED __INCLUDED_VOLK_ADD_TEST)
    return()
endif()
set(__INCLUDED_VOLK_ADD_TEST TRUE)

########################################################################
# Generate a test executable which can be used in ADD_TEST to call
# various subtests.
#
# SOURCES        - sources for the test
# TARGET_DEPS    - build target dependencies (e.g., libraries)
########################################################################

function(VOLK_GEN_TEST executable_name)
    include(CMakeParseArgumentsCopy)
    CMAKE_PARSE_ARGUMENTS(VOLK_TEST "" "" "SOURCES;TARGET_DEPS;EXTRA_LIB_DIRS;ENVIRONS;ARGS" ${ARGN})
    add_executable(${executable_name} ${VOLK_TEST_SOURCES})
    target_link_libraries(${executable_name} ${VOLK_TEST_TARGET_DEPS})
endfunction()

########################################################################
# Add a unit test and setup the environment for it.
# Encloses ADD_TEST, with additional functionality to create a shell
# script that sets the environment to gain access to in-build binaries
# properly. The following variables are used to pass in settings:
# A test executable has to be generated with VOLK_GEN_TEST beforehand.
# The executable name has to be passed as argument.
#
# NAME           - the test name
# TARGET_DEPS    - build target dependencies (e.g., libraries)
# EXTRA_LIB_DIRS - other directories for the library path
# ENVIRONS       - other environment key/value pairs
# ARGS           - arguments for the test
########################################################################
function(VOLK_ADD_TEST test_name executable_name)

  #parse the arguments for component names
  include(CMakeParseArgumentsCopy)
  CMAKE_PARSE_ARGUMENTS(VOLK_TEST "" "" "TARGET_DEPS;EXTRA_LIB_DIRS;ENVIRONS;ARGS" ${ARGN})

  #set the initial environs to use
  set(environs ${VOLK_TEST_ENVIRONS})

  #create the initial library path
  file(TO_NATIVE_PATH "${VOLK_TEST_EXTRA_LIB_DIRS}" libpath)

  #set the source directory, which is mostly FYI
  file(TO_NATIVE_PATH ${CMAKE_CURRENT_SOURCE_DIR} srcdir)
  list(APPEND environs "srcdir=\"${srcdir}\"")

  #http://www.cmake.org/pipermail/cmake/2009-May/029464.html
  #Replaced this add test + set environs code with the shell script generation.
  #Its nicer to be able to manually run the shell script to diagnose problems.
  if(UNIX)
    if(APPLE)
      set(LD_PATH_VAR "DYLD_LIBRARY_PATH")
    else()
      set(LD_PATH_VAR "LD_LIBRARY_PATH")
    endif()

    #create a list of target directories to be determined by the
    #"add_test" command, via the $<FOO:BAR> operator; make sure the
    #test's directory is first, since it ($1) is prepended to PATH.
    unset(TARGET_DIR_LIST)
    foreach(target ${executable_name} ${VOLK_TEST_TARGET_DEPS})
      list(APPEND TARGET_DIR_LIST "\$<TARGET_FILE_DIR:${target}>")
    endforeach()

    #augment the PATH to start with the directory of the test
    set(binpath "\"$1:\$PATH\"")
    list(APPEND environs "PATH=${binpath}")

    #set the shell to use
    if(CMAKE_CROSSCOMPILING)
      set(SHELL "/bin/sh")
    else()
      find_program(SHELL sh)
    endif()

    #check to see if the shell supports "$*" expansion with IFS
    if(NOT TESTED_SHELL_SUPPORTS_IFS)
      set(TESTED_SHELL_SUPPORTS_IFS TRUE CACHE BOOL "")
      set(sh_file ${CMAKE_CURRENT_BINARY_DIR}/ifs_test.sh)
      file(WRITE ${sh_file} "#!${SHELL}\n")
      file(APPEND ${sh_file} "export IFS=:\n")
      file(APPEND ${sh_file} "echo \"$*\"\n")
      #make the shell file executable
      execute_process(COMMAND chmod +x ${sh_file})

      #execute the shell script
      execute_process(COMMAND ${sh_file} "a" "b" "c"
        OUTPUT_VARIABLE output OUTPUT_STRIP_TRAILING_WHITESPACE
      )

      #check the output to see if it is correct
      string(COMPARE EQUAL ${output} "a:b:c" SHELL_SUPPORTS_IFS)
      set(SHELL_SUPPORTS_IFS ${SHELL_SUPPORTS_IFS} CACHE BOOL
        "Set this value to TRUE if the shell supports IFS argument expansion"
      )
    endif()
    unset(testlibpath)
    if(SHELL_SUPPORTS_IFS)
      #"$*" expands in the shell into a list of all of the arguments
      #to the shell script, concatenated using the character provided
      #in ${IFS}.
      list(APPEND testlibpath "$*")
    else()
      #shell does not support IFS expansion; use a loop instead
      list(APPEND testlibpath "\${LL}")
    endif()

    #finally: add in the current library path variable
    list(INSERT libpath 0 ${testlibpath})
    list(APPEND libpath "$${LD_PATH_VAR}")

    #replace list separator with the path separator
    string(REPLACE ";" ":" libpath "${libpath}")
    list(APPEND environs "${LD_PATH_VAR}=\"${libpath}\"")

    #generate a shell script file that sets the environment and runs the test
    set(sh_file ${CMAKE_CURRENT_BINARY_DIR}/${test_name}_test.sh)
    file(WRITE ${sh_file} "#!${SHELL}\n")
    if(SHELL_SUPPORTS_IFS)
      file(APPEND ${sh_file} "export IFS=:\n")
    else()
      file(APPEND ${sh_file} "LL=\"$1\" && for tf in \"\$@\"; do LL=\"\${LL}:\${tf}\"; done\n")
    endif()

    #each line sets an environment variable
    foreach(environ ${environs})
      file(APPEND ${sh_file} "export ${environ}\n")
    endforeach(environ)

    set(VOLK_TEST_ARGS "${test_name}")

    #redo the test args to have a space between each
    string(REPLACE ";" " " VOLK_TEST_ARGS "${VOLK_TEST_ARGS}")

    #finally: append the test name to execute
    file(APPEND ${sh_file} "${CMAKE_CROSSCOMPILING_EMULATOR} ${executable_name} ${VOLK_TEST_ARGS}\n")

    #make the shell file executable
    execute_process(COMMAND chmod +x ${sh_file})

    #add the shell file as the test to execute;
    #use the form that allows for $<FOO:BAR> substitutions,
    #then combine the script arguments inside the script.
    add_test(NAME qa_${test_name}
      COMMAND ${SHELL} ${sh_file} ${TARGET_DIR_LIST}
    )

  endif(UNIX)

  if(WIN32)
    #In the land of windows, all libraries must be in the PATH.  Since
    #the dependent libraries are not yet installed, we must manually
    #set them in the PATH to run tests.  The following appends the
    #path of a target dependency.
    #
    #create a list of target directories to be determined by the
    #"add_test" command, via the $<FOO:BAR> operator; make sure the
    #test's directory is first, since it ($1) is prepended to PATH.
    unset(TARGET_DIR_LIST)
    foreach(target ${executable_name} ${VOLK_TEST_TARGET_DEPS})
      list(APPEND TARGET_DIR_LIST "$<TARGET_FILE_DIR:${target}>")
    endforeach()
    #replace list separator with the path separator (escaped)
    string(REPLACE ";" "\\\\;" TARGET_DIR_LIST "${TARGET_DIR_LIST}")

    #add command line argument (TARGET_DIR_LIST) to path and append current path
    list(INSERT libpath 0 "%1")
    list(APPEND libpath "%PATH%")

    #replace list separator with the path separator (escaped)
    string(REPLACE ";" "\\;" libpath "${libpath}")
    list(APPEND environs "PATH=${libpath}")

    #generate a bat file that sets the environment and runs the test
    set(bat_file ${CMAKE_CURRENT_BINARY_DIR}/${test_name}_test.bat)
    file(WRITE ${bat_file} "@echo off\n")

    #each line sets an environment variable
    foreach(environ ${environs})
      file(APPEND ${bat_file} "SET ${environ}\n")
    endforeach(environ)

    set(VOLK_TEST_ARGS "${test_name}")

    #redo the test args to have a space between each
    string(REPLACE ";" " " VOLK_TEST_ARGS "${VOLK_TEST_ARGS}")

    #finally: append the test name to execute
    file(APPEND ${bat_file} "${executable_name} ${VOLK_TEST_ARGS}\n")
    file(APPEND ${bat_file} "\n")

    add_test(NAME qa_${test_name}
        COMMAND ${bat_file} ${TARGET_DIR_LIST}
    )
  endif(WIN32)

endfunction(VOLK_ADD_TEST)

