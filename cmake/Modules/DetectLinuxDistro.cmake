# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(ARCHITECTURE_STRING "(64 bits)")
else()
    set(ARCHITECTURE_STRING "(32 bits)")
endif()

if(EXISTS "/etc/lsb-release")
    execute_process(COMMAND cat /etc/lsb-release
        COMMAND grep DISTRIB_ID
        COMMAND awk -F= "{ print $2 }"
        COMMAND tr "\n" " "
        COMMAND sed "s/ //"
        OUTPUT_VARIABLE LINUX_DISTRIBUTION
        RESULT_VARIABLE LINUX_ID_RESULT
    )
    execute_process(COMMAND cat /etc/lsb-release
        COMMAND grep DISTRIB_RELEASE
        COMMAND awk -F= "{ print $2 }"
        COMMAND tr "\n" " "
        COMMAND sed "s/ //"
        OUTPUT_VARIABLE LINUX_VER
        RESULT_VARIABLE LINUX_VER_RESULT
    )
endif()

if(NOT LINUX_DISTRIBUTION)
    if(EXISTS "/etc/linuxmint/info")
        set(LINUX_DISTRIBUTION "LinuxMint")
        execute_process(COMMAND cat /etc/linuxmint/info
            COMMAND grep -m1 RELEASE
            COMMAND awk -F= "{ print $2 }"
            COMMAND tr "\n" " "
            COMMAND sed "s/ //"
            OUTPUT_VARIABLE LINUX_VER
            RESULT_VARIABLE LINUX_VER_RESULT
        )
    endif()
endif()

if(NOT LINUX_DISTRIBUTION)
    if(EXISTS "/etc/os-release")
        execute_process(COMMAND cat /etc/os-release
            COMMAND grep -m1 NAME
            COMMAND awk -F= "{ print $2 }"
            COMMAND tr "\n" " "
            COMMAND sed "s/ //"
            OUTPUT_VARIABLE LINUX_DISTRIBUTION_
            RESULT_VARIABLE LINUX_ID_RESULT
        )
        execute_process(COMMAND cat /etc/os-release
            COMMAND grep VERSION_ID
            COMMAND awk -F= "{ print $2 }"
            COMMAND tr "\n" " "
            COMMAND sed "s/ //"
            OUTPUT_VARIABLE LINUX_VER_
            RESULT_VARIABLE LINUX_VER_RESULT
        )
        if(LINUX_DISTRIBUTION_)
            string(REPLACE "\"" "" LINUX_DISTRIBUTION__ ${LINUX_DISTRIBUTION_})
            string(STRIP ${LINUX_DISTRIBUTION__} LINUX_DISTRIBUTION)
        endif()
        if(LINUX_VER_)
            string(REPLACE "\"" "" LINUX_VER ${LINUX_VER_})
        endif()
        if(${LINUX_DISTRIBUTION} MATCHES "Debian")
            set(LINUX_DISTRIBUTION "Debian")
            file(READ /etc/debian_version LINUX_VER_)
            string(REPLACE "\n" "" LINUX_VER ${LINUX_VER_})
        endif()
    endif()
endif()

if(NOT LINUX_DISTRIBUTION)
    if(EXISTS "/etc/redhat-release")
        set(LINUX_DISTRIBUTION "Red Hat")
        file(READ /etc/redhat-release LINUX_VER_)
        string(REPLACE "\n" "" LINUX_VER ${LINUX_VER_})
    endif()
endif()

if(NOT LINUX_DISTRIBUTION)
    if(EXISTS "/etc/debian_version")
        set(LINUX_DISTRIBUTION "Debian")
        file(READ /etc/debian_version LINUX_VER_)
        string(REPLACE "\n" "" LINUX_VER ${LINUX_VER_})
    endif()
endif()

if(NOT LINUX_DISTRIBUTION)
    if(EXISTS "/etc/arch-release")
        set(LINUX_DISTRIBUTION "Arch Linux")
        set(LINUX_VER "")
    endif()
endif()

if(NOT LINUX_DISTRIBUTION)
    set(LINUX_DISTRIBUTION "Generic")
    set(LINUX_VER "Unknown")
endif()
