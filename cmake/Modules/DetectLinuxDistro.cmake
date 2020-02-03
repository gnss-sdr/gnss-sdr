# Copyright (C) 2020 (see AUTHORS file for a list of contributors)
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
            string(REPLACE "\"" "" LINUX_DISTRIBUTION ${LINUX_DISTRIBUTION_})
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
