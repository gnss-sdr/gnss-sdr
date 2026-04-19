# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

if(CMAKE_VERSION VERSION_LESS 3.19)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(ARCHITECTURE_STRING "(64 bits)")
    else()
        set(ARCHITECTURE_STRING "(32 bits)")
    endif()
else()
    set(ARCHITECTURE_STRING "(${CMAKE_HOST_SYSTEM_PROCESSOR})")
endif()

if(EXISTS "/etc/lsb-release")
    file(STRINGS "/etc/lsb-release" _lsb_distrib_id_line
        REGEX "^DISTRIB_ID="
        LIMIT_COUNT 1
    )
    file(STRINGS "/etc/lsb-release" _lsb_distrib_release_line
        REGEX "^DISTRIB_RELEASE="
        LIMIT_COUNT 1
    )
    if(_lsb_distrib_id_line)
        string(REGEX REPLACE "^DISTRIB_ID=" "" LINUX_DISTRIBUTION "${_lsb_distrib_id_line}")
        string(REPLACE "\"" "" LINUX_DISTRIBUTION "${LINUX_DISTRIBUTION}")
        string(STRIP "${LINUX_DISTRIBUTION}" LINUX_DISTRIBUTION)
        set(LINUX_ID_RESULT 0)
    else()
        set(LINUX_ID_RESULT 1)
    endif()
    if(_lsb_distrib_release_line)
        string(REGEX REPLACE "^DISTRIB_RELEASE=" "" LINUX_VER "${_lsb_distrib_release_line}")
        string(REPLACE "\"" "" LINUX_VER "${LINUX_VER}")
        string(STRIP "${LINUX_VER}" LINUX_VER)
        set(LINUX_VER_RESULT 0)
    else()
        set(LINUX_VER_RESULT 1)
    endif()
    unset(_lsb_distrib_id_line)
    unset(_lsb_distrib_release_line)
endif()


if(NOT LINUX_DISTRIBUTION)
    if(EXISTS "/etc/linuxmint/info")
        set(LINUX_DISTRIBUTION "LinuxMint")
        file(STRINGS "/etc/linuxmint/info" _linuxmint_release_line
            REGEX "^RELEASE="
            LIMIT_COUNT 1
        )
        if(_linuxmint_release_line)
            string(REGEX REPLACE "^RELEASE=" "" LINUX_VER "${_linuxmint_release_line}")
            string(REPLACE "\"" "" LINUX_VER "${LINUX_VER}")
            string(STRIP "${LINUX_VER}" LINUX_VER)
            set(LINUX_VER_RESULT 0)
        else()
            set(LINUX_VER_RESULT 1)
        endif()
        unset(_linuxmint_release_line)
    endif()
endif()

if(NOT LINUX_DISTRIBUTION)
    if(EXISTS "/etc/os-release")
        file(STRINGS "/etc/os-release" _os_release_name_line REGEX "^NAME=" LIMIT_COUNT 1)
        file(STRINGS "/etc/os-release" _os_release_version_line REGEX "^VERSION_ID=" LIMIT_COUNT 1)

        if(_os_release_name_line)
            string(REGEX REPLACE "^NAME=" "" LINUX_DISTRIBUTION "${_os_release_name_line}")
            string(REPLACE "\"" "" LINUX_DISTRIBUTION "${LINUX_DISTRIBUTION}")
            string(STRIP "${LINUX_DISTRIBUTION}" LINUX_DISTRIBUTION)
        endif()

        if(_os_release_version_line)
            string(REGEX REPLACE "^VERSION_ID=" "" LINUX_VER "${_os_release_version_line}")
            string(REPLACE "\"" "" LINUX_VER "${LINUX_VER}")
            string(STRIP "${LINUX_VER}" LINUX_VER)
        endif()

        if("${LINUX_DISTRIBUTION}" STREQUAL "Debian" AND EXISTS "/etc/debian_version")
            file(READ "/etc/debian_version" LINUX_VER_)
            string(STRIP "${LINUX_VER_}" LINUX_VER)
        endif()

        unset(_os_release_name_line)
        unset(_os_release_version_line)
    endif()
endif()

if(NOT LINUX_DISTRIBUTION)
    if(EXISTS "/etc/redhat-release")
        set(LINUX_DISTRIBUTION "Red Hat")
        file(READ /etc/redhat-release LINUX_VER_)
        string(REPLACE "\n" "" LINUX_VER "${LINUX_VER_}")
    endif()
endif()

if(NOT LINUX_DISTRIBUTION)
    if(EXISTS "/etc/debian_version")
        set(LINUX_DISTRIBUTION "Debian")
        file(READ /etc/debian_version LINUX_VER_)
        string(REPLACE "\n" "" LINUX_VER "${LINUX_VER_}")
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
