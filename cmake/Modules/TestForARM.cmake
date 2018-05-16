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

##############################################################################
# check if the compiler defines the architecture as ARM  and set the 
# version, if found.
#
# - Anthony Arnold
##############################################################################

if (__TEST_FOR_ARM_INCLUDED)
  return ()
endif()
set(__TEST_FOR_ARM_INCLUDED TRUE)

# Function checks if the input string defines ARM version and sets the
# output variable if found.
function(check_arm_version ppdef input_string version output_var)
  string(REGEX MATCH "${ppdef}"  _VERSION_MATCH "${input_string}")
  if (NOT _VERSION_MATCH STREQUAL "")
    set(${output_var} "${version}" PARENT_SCOPE)
  endif(NOT _VERSION_MATCH STREQUAL "")
endfunction()

message(STATUS "Checking for ARM")

set (IS_ARM NO)
set (ARM_VERSION "")

if (CMAKE_COMPILER_IS_GNUCXX)
  execute_process(COMMAND echo "int main(){}"
                  COMMAND ${CMAKE_CXX_COMPILER} ${CMAKE_CXX_COMPILER_ARG1} -dM -E -
                  OUTPUT_VARIABLE TEST_FOR_ARM_RESULTS)

  string(REGEX MATCH "__arm" ARM_FOUND "${TEST_FOR_ARM_RESULTS}")
  if(ARM_FOUND STREQUAL "")
     string(REGEX MATCH "__aarch64" ARM_FOUND "${TEST_FOR_ARM_RESULTS}")
  endif(ARM_FOUND STREQUAL "")

  if (NOT ARM_FOUND STREQUAL "")
    set(IS_ARM YES)
    message(STATUS "ARM system detected")

    # detect the version
    check_arm_version("__ARM_ARCH_2__" ${TEST_FOR_ARM_RESULTS} "armv2" ARM_VERSION)
    check_arm_version("__ARM_ARCH_2A__" ${TEST_FOR_ARM_RESULTS} "armv2a" ARM_VERSION)
    check_arm_version("__ARM_ARCH_3__" ${TEST_FOR_ARM_RESULTS} "armv3" ARM_VERSION)
    check_arm_version("__ARM_ARCH_3M__" ${TEST_FOR_ARM_RESULTS} "armv3m" ARM_VERSION)
    check_arm_version("__ARM_ARCH_4__" ${TEST_FOR_ARM_RESULTS} "armv4" ARM_VERSION)
    check_arm_version("__ARM_ARCH_4T__" ${TEST_FOR_ARM_RESULTS} "armv4t" ARM_VERSION)
    check_arm_version("__ARM_ARCH_5__" ${TEST_FOR_ARM_RESULTS} "armv5" ARM_VERSION)
    check_arm_version("__ARM_ARCH_5T__" ${TEST_FOR_ARM_RESULTS} "armv5t" ARM_VERSION)
    check_arm_version("__ARM_ARCH_5E__" ${TEST_FOR_ARM_RESULTS} "armv5e" ARM_VERSION)
    check_arm_version("__ARM_ARCH_5TE__" ${TEST_FOR_ARM_RESULTS} "armv5te" ARM_VERSION)
    check_arm_version("__ARM_ARCH_6__" ${TEST_FOR_ARM_RESULTS} "armv6" ARM_VERSION)
    check_arm_version("__ARM_ARCH_6J__" ${TEST_FOR_ARM_RESULTS} "armv6j" ARM_VERSION)
    check_arm_version("__ARM_ARCH_6K__" ${TEST_FOR_ARM_RESULTS} "armv6k" ARM_VERSION)
    check_arm_version("__ARM_ARCH_6T2__" ${TEST_FOR_ARM_RESULTS} "armv6t2" ARM_VERSION)
    check_arm_version("__ARM_ARCH_6Z__" ${TEST_FOR_ARM_RESULTS} "armv6z" ARM_VERSION)
    check_arm_version("__ARM_ARCH_6ZK__" ${TEST_FOR_ARM_RESULTS} "armv6zk" ARM_VERSION)
    check_arm_version("__ARM_ARCH_6M__" ${TEST_FOR_ARM_RESULTS} "armv6-m" ARM_VERSION)
    check_arm_version("__ARM_ARCH_7__" ${TEST_FOR_ARM_RESULTS} "armv7" ARM_VERSION)
    check_arm_version("__ARM_ARCH_7A__" ${TEST_FOR_ARM_RESULTS} "armv7-a" ARM_VERSION)
    check_arm_version("__ARM_ARCH_7M__" ${TEST_FOR_ARM_RESULTS} "armv7-m" ARM_VERSION)
    check_arm_version("__ARM_ARCH_7R__" ${TEST_FOR_ARM_RESULTS} "armv7-r" ARM_VERSION)
    check_arm_version("__ARM_ARCH_7EM_" ${TEST_FOR_ARM_RESULTS} "armv7e-m" ARM_VERSION)
    check_arm_version("__ARM_ARCH_7VE__" ${TEST_FOR_ARM_RESULTS} "armv7ve" ARM_VERSION)
    check_arm_version("__ARM_ARCH_8A__" ${TEST_FOR_ARM_RESULTS} "armv8-a" ARM_VERSION)
    check_arm_version("__ARM_ARCH_8A" ${TEST_FOR_ARM_RESULTS} "armv8-a" ARM_VERSION)

    # anything else just define as arm
    if (ARM_VERSION STREQUAL "")
      message(STATUS "Couldn't detect ARM version. Setting to 'arm'")
      set(ARM_VERSION "arm")
    else (ARM_VERSION STREQUAL "")
      message(STATUS "ARM version ${ARM_VERSION} detected")
    endif (ARM_VERSION STREQUAL "")
    
  else (NOT ARM_FOUND STREQUAL "")
    message(STATUS "System is not ARM")  
  endif(NOT ARM_FOUND STREQUAL "")

else (CMAKE_COMPILE_IS_GNUCXX)
  # TODO: Other compilers
  message(STATUS "Not detecting ARM on non-GNUCXX compiler. Defaulting to false")
  message(STATUS "If you are compiling for ARM, set IS_ARM=ON manually")
endif(CMAKE_COMPILER_IS_GNUCXX)

set(IS_ARM ${IS_ARM} CACHE BOOL "Compiling for ARM")
set(ARM_VERSION ${ARM_VERSION} CACHE STRING "ARM version")
