# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

##############################################################################
# check if the compiler defines the architecture as ARM  and set the
# version, if found.
#
# - Anthony Arnold
##############################################################################

if(__TEST_FOR_ARM_INCLUDED)
    return()
endif()
set(__TEST_FOR_ARM_INCLUDED TRUE)

# Function checks if the input string defines ARM version and sets the
# output variable if found.
function(check_arm_version ppdef input_string version output_var)
    string(REGEX MATCH "${ppdef}"  _VERSION_MATCH "${input_string}")
    if(NOT _VERSION_MATCH STREQUAL "")
        set(${output_var} "${version}" PARENT_SCOPE)
    endif()
endfunction()

message(STATUS "Checking for ARM")

set(IS_ARM FALSE)
set(ARM_VERSION "")

if(ENABLE_PACKAGING)
    set(VERBOSE_BUILDING "-v")
else()
    set(VERBOSE_BUILDING "")
endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    execute_process(COMMAND echo "int main(){}"
        COMMAND ${CMAKE_CXX_COMPILER} ${CMAKE_CXX_COMPILER_ARG1} -dM ${VERBOSE_BUILDING} -E -
        OUTPUT_VARIABLE TEST_FOR_ARM_RESULTS
    )

    string(REGEX MATCH "__arm" ARM_FOUND "${TEST_FOR_ARM_RESULTS}")
    if(ARM_FOUND STREQUAL "")
        string(REGEX MATCH "__aarch64" ARM_FOUND "${TEST_FOR_ARM_RESULTS}")
    endif()
    if(ARM_FOUND STREQUAL "")
        if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "arm")
            set(ARM_FOUND ${CMAKE_HOST_SYSTEM_PROCESSOR})
        endif()
    endif()

    if(NOT ARM_FOUND STREQUAL "")
        set(IS_ARM TRUE)
        message(STATUS "ARM system detected.")

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

        # anything else just define as unknown
        if(ARM_VERSION STREQUAL "")
            message(STATUS "Couldn't detect ARM version.")
            set(ARM_VERSION "unknown")
        else()
            message(STATUS "ARM version ${ARM_VERSION} detected.")
        endif()
    else()
        message(STATUS "System is not ARM.")
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "arm")
        set(IS_ARM TRUE)
        set(ARM_VERSION ${CMAKE_HOST_SYSTEM_PROCESSOR})
        message(STATUS "ARM version ${ARM_VERSION} detected.")
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "ARMClang")
    set(IS_ARM TRUE)
    set(ARM_VERSION "arm")
    if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "arm")
        set(ARM_VERSION ${CMAKE_HOST_SYSTEM_PROCESSOR})
        message(STATUS "ARM version ${ARM_VERSION} detected.")
    endif()
else()
    message(STATUS "Not detecting ARM on non-GNUCXX or non-Clang compiler. Defaulting to false.")
    message(STATUS "If you are compiling for ARM, set -DIS_ARM=ON manually.")
endif()

set(IS_ARM ${IS_ARM} CACHE BOOL "Compiling for ARM")
set(ARM_VERSION ${ARM_VERSION} CACHE STRING "ARM version")
