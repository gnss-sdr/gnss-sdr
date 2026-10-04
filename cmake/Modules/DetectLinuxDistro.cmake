# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2026 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

# Output variables
#
#   ARCHITECTURE_STRING
#       Human-readable host architecture suffix used in status messages.
#       For older CMake versions this is "(64 bits)" or "(32 bits)";
#       for newer CMake versions this is "(${CMAKE_HOST_SYSTEM_PROCESSOR})".
#
#   LINUX_DISTRIBUTION
#       Human-readable Linux distribution name, for example "Ubuntu",
#       "Debian", "Fedora", "LinuxMint", "Alpine Linux", or "Generic".
#
#   LINUX_VER
#       Distribution version string when available. This may be numeric
#       such as "24.04", "12.5", or "9.3", or a rolling/testing label
#       such as "edge" or "trixie/sid".
#
#   LINUX_ID
#       Normalized lower-case distribution identifier when available,
#       usually matching /etc/os-release ID, for example "ubuntu",
#       "debian", "fedora", "rhel", "linuxmint", "alpine", or "arch".
#
#   LINUX_PRETTY_NAME
#       Pretty distribution name from /etc/os-release PRETTY_NAME when
#       available. This is informational and may be empty.
#
#   LINUX_VERSION_CODENAME
#       Distribution codename when available, for example "jammy",
#       "noble", "bookworm", or "trixie". This is informational and
#       may be empty.
#
#   LINUX_ID_RESULT
#       Detection status for the distribution identity. Set to 0 when a
#       distribution was positively identified; left as 1 for the generic
#       fallback.
#
#   LINUX_VER_RESULT
#       Detection status for the distribution version. Set to 0 when a
#       version string was found; left as 1 when no version was detected.


# text file at `filename` and store it in `output_variable` in the
# caller's scope.
# Both quoted ("value") and unquoted values are supported. Leading/trailing
# whitespace is stripped. If the file does not exist or the key is absent,
# `output_variable` is set to the empty string.
function(_gnsssdr_read_key_value_file filename key output_variable)
    set(${output_variable} "" PARENT_SCOPE)

    if(NOT EXISTS "${filename}")
        return()
    endif()

    file(STRINGS "${filename}" _gnsssdr_key_value_line
        REGEX "^${key}="
        LIMIT_COUNT 1
    )

    if(NOT "${_gnsssdr_key_value_line}" STREQUAL "")
        string(REGEX REPLACE
            "^[^=]*="
            ""
            _gnsssdr_key_value
            "${_gnsssdr_key_value_line}"
        )
        string(REGEX REPLACE
            "^\"(.*)\"$"
            "\\1"
            _gnsssdr_key_value
            "${_gnsssdr_key_value}"
        )
        string(REGEX REPLACE
            "^'(.*)'$"
            "\\1"
            _gnsssdr_key_value
            "${_gnsssdr_key_value}"
        )
        string(STRIP "${_gnsssdr_key_value}" _gnsssdr_key_value)
        set(${output_variable} "${_gnsssdr_key_value}" PARENT_SCOPE)
    endif()
endfunction()


# ---------------------------------------------------------------------------
# Architecture string
# ---------------------------------------------------------------------------
if(CMAKE_VERSION VERSION_LESS 3.19)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(ARCHITECTURE_STRING "(64 bits)")
    else()
        set(ARCHITECTURE_STRING "(32 bits)")
    endif()
else()
    set(ARCHITECTURE_STRING "(${CMAKE_HOST_SYSTEM_PROCESSOR})")
endif()


# ---------------------------------------------------------------------------
# Initialise output variables
# ---------------------------------------------------------------------------
set(LINUX_DISTRIBUTION "")
set(LINUX_VER "")
set(LINUX_ID "")
set(LINUX_PRETTY_NAME "")
set(LINUX_VERSION_CODENAME "")
set(LINUX_ID_RESULT 1)
set(LINUX_VER_RESULT 1)


# ---------------------------------------------------------------------------
# 1. /etc/os-release -- the systemd-era standard; present on virtually all
#    modern distributions.
# ---------------------------------------------------------------------------
if(EXISTS "/etc/os-release")
    _gnsssdr_read_key_value_file(
        "/etc/os-release" "ID" _gnsssdr_os_id)
    _gnsssdr_read_key_value_file(
        "/etc/os-release" "NAME" _gnsssdr_os_name)
    _gnsssdr_read_key_value_file(
        "/etc/os-release" "PRETTY_NAME" LINUX_PRETTY_NAME)
    _gnsssdr_read_key_value_file(
        "/etc/os-release" "VERSION_ID" _gnsssdr_os_version_id)
    _gnsssdr_read_key_value_file(
        "/etc/os-release" "VERSION_CODENAME" LINUX_VERSION_CODENAME)

    if(NOT "${_gnsssdr_os_id}" STREQUAL "")
        set(LINUX_ID "${_gnsssdr_os_id}")
    endif()

    if(NOT "${_gnsssdr_os_name}" STREQUAL "")
        if("${LINUX_ID}" STREQUAL "debian")
            set(LINUX_DISTRIBUTION "Debian")
        elseif("${LINUX_ID}" STREQUAL "linuxmint")
            set(LINUX_DISTRIBUTION "LinuxMint")
        else()
            set(LINUX_DISTRIBUTION "${_gnsssdr_os_name}")
        endif()
        set(LINUX_ID_RESULT 0)
    endif()

    if("${LINUX_DISTRIBUTION}" STREQUAL "" AND NOT "${LINUX_ID}" STREQUAL "")
        set(LINUX_DISTRIBUTION "${LINUX_ID}")
        set(LINUX_ID_RESULT 0)
    endif()

    if(NOT "${_gnsssdr_os_version_id}" STREQUAL "")
        set(LINUX_VER "${_gnsssdr_os_version_id}")
        set(LINUX_VER_RESULT 0)
    endif()
endif()


# ---------------------------------------------------------------------------
# 2. /etc/lsb-release -- fallback for older distributions where os-release may
#    be absent or incomplete.
# ---------------------------------------------------------------------------
if(("${LINUX_DISTRIBUTION}" STREQUAL "" OR "${LINUX_VER}" STREQUAL "") AND
        EXISTS "/etc/lsb-release")
    _gnsssdr_read_key_value_file(
        "/etc/lsb-release" "DISTRIB_ID" _gnsssdr_lsb_distrib_id)
    _gnsssdr_read_key_value_file(
        "/etc/lsb-release" "DISTRIB_RELEASE" _gnsssdr_lsb_distrib_release)
    _gnsssdr_read_key_value_file(
        "/etc/lsb-release" "DISTRIB_CODENAME" _gnsssdr_lsb_distrib_codename)

    if("${LINUX_ID}" STREQUAL "" AND
            NOT "${_gnsssdr_lsb_distrib_id}" STREQUAL "")
        string(TOLOWER "${_gnsssdr_lsb_distrib_id}" LINUX_ID)
        # Collapse spaces so e.g. "Linux Mint" becomes "linuxmint", matching
        # the canonical ID used in os-release on modern Mint installations.
        string(REPLACE " " "" LINUX_ID "${LINUX_ID}")
    endif()

    if("${LINUX_DISTRIBUTION}" STREQUAL "" AND
            NOT "${_gnsssdr_lsb_distrib_id}" STREQUAL "")
        if("${LINUX_ID}" STREQUAL "linuxmint")
            set(LINUX_DISTRIBUTION "LinuxMint")
        else()
            set(LINUX_DISTRIBUTION "${_gnsssdr_lsb_distrib_id}")
        endif()
        set(LINUX_ID_RESULT 0)
    endif()

    if("${LINUX_VER}" STREQUAL "" AND
            NOT "${_gnsssdr_lsb_distrib_release}" STREQUAL "")
        set(LINUX_VER "${_gnsssdr_lsb_distrib_release}")
        set(LINUX_VER_RESULT 0)
    endif()

    # Populate the codename if os-release did not already provide it.
    if("${LINUX_VERSION_CODENAME}" STREQUAL "" AND
            NOT "${_gnsssdr_lsb_distrib_codename}" STREQUAL "")
        set(LINUX_VERSION_CODENAME "${_gnsssdr_lsb_distrib_codename}")
    endif()
    unset(_gnsssdr_lsb_distrib_id)
    unset(_gnsssdr_lsb_distrib_release)
    unset(_gnsssdr_lsb_distrib_codename)
endif()


# ---------------------------------------------------------------------------
# 3. /etc/debian_version -- Debian's own file often carries a more precise
#    point-release string (e.g. "12.5") than VERSION_ID in os-release.
#    Applied only for Debian itself, not derivatives.
# ---------------------------------------------------------------------------
if("${LINUX_ID}" STREQUAL "debian" AND EXISTS "/etc/debian_version")
    file(READ "/etc/debian_version" _gnsssdr_debian_version)
    string(STRIP "${_gnsssdr_debian_version}" _gnsssdr_debian_version)
    string(REGEX MATCH "^[0-9]" _gnsssdr_debian_is_numeric
        "${_gnsssdr_debian_version}")
    if(NOT "${_gnsssdr_debian_is_numeric}" STREQUAL "")
        # Numeric: always prefer -- more precise than os-release VERSION_ID.
        set(LINUX_VER "${_gnsssdr_debian_version}")
        set(LINUX_VER_RESULT 0)
    elseif("${LINUX_VER}" STREQUAL "" AND
            NOT "${_gnsssdr_debian_version}" STREQUAL "")
        # Non-numeric (e.g. "trixie/sid"): use only as a last resort.
        set(LINUX_VER "${_gnsssdr_debian_version}")
        set(LINUX_VER_RESULT 0)
    endif()
    unset(_gnsssdr_debian_is_numeric)
    unset(_gnsssdr_debian_version)
endif()


# ---------------------------------------------------------------------------
# 4. /etc/redhat-release -- Red Hat, CentOS, CentOS Stream, Fedora, Rocky,
#    AlmaLinux, Oracle Linux, and other RHEL-family distributions.
# ---------------------------------------------------------------------------
if(("${LINUX_DISTRIBUTION}" STREQUAL "" OR
        "${LINUX_VER}" STREQUAL "" OR
        "${LINUX_ID}" STREQUAL "") AND
        EXISTS "/etc/redhat-release")
    file(READ "/etc/redhat-release" _gnsssdr_redhat_release)
    string(STRIP "${_gnsssdr_redhat_release}" _gnsssdr_redhat_release)

    if("${LINUX_DISTRIBUTION}" STREQUAL "")
        # Extract the human-readable name: everything up to the keyword
        # "release" or the first digit, whichever comes first.
        string(REGEX MATCH
            "^([A-Za-z][A-Za-z0-9 ]*[A-Za-z])[ ]+(release|[0-9])"
            _gnsssdr_rh_name_match
            "${_gnsssdr_redhat_release}"
        )

        if(NOT "${_gnsssdr_rh_name_match}" STREQUAL "")
            string(REGEX REPLACE
                "^([A-Za-z][A-Za-z0-9 ]*[A-Za-z])[ ]+(release|[0-9]).*$"
                "\\1"
                _gnsssdr_rh_name
                "${_gnsssdr_redhat_release}"
            )
            string(STRIP "${_gnsssdr_rh_name}" _gnsssdr_rh_name)
        else()
            # Fallback: take every leading alphabetic word.
            string(REGEX MATCH "^[A-Za-z ]+" _gnsssdr_rh_name
                "${_gnsssdr_redhat_release}")
            string(STRIP "${_gnsssdr_rh_name}" _gnsssdr_rh_name)
        endif()

        if(NOT "${_gnsssdr_rh_name}" STREQUAL "")
            set(LINUX_DISTRIBUTION "${_gnsssdr_rh_name}")
        else()
            set(LINUX_DISTRIBUTION "Red Hat")
        endif()

        set(LINUX_ID_RESULT 0)
        unset(_gnsssdr_rh_name)
        unset(_gnsssdr_rh_name_match)
    endif()

    # Derive a compact, lower-case ID token (e.g. "rhel", "centos",
    # "fedora", "rocky", "almalinux", "ol") when os-release did not
    # already provide one.
    if("${LINUX_ID}" STREQUAL "" AND
            NOT "${LINUX_DISTRIBUTION}" STREQUAL "")
        string(TOLOWER "${LINUX_DISTRIBUTION}" _gnsssdr_rh_id)

        # Map well-known verbose names to their canonical short IDs.
        if("${_gnsssdr_rh_id}" MATCHES "red hat")
            set(LINUX_ID "rhel")
        elseif("${_gnsssdr_rh_id}" MATCHES "centos stream")
            set(LINUX_ID "centos-stream")
        elseif("${_gnsssdr_rh_id}" MATCHES "centos")
            set(LINUX_ID "centos")
        elseif("${_gnsssdr_rh_id}" MATCHES "fedora")
            set(LINUX_ID "fedora")
        elseif("${_gnsssdr_rh_id}" MATCHES "rocky")
            set(LINUX_ID "rocky")
        elseif("${_gnsssdr_rh_id}" MATCHES "alma")
            set(LINUX_ID "almalinux")
        elseif("${_gnsssdr_rh_id}" MATCHES "oracle")
            set(LINUX_ID "ol")
        else()
            # Replace spaces with hyphens for unknown names, e.g.
            # "some linux" -> "some-linux".
            string(REPLACE " " "-" _gnsssdr_rh_id "${_gnsssdr_rh_id}")
            set(LINUX_ID "${_gnsssdr_rh_id}")
        endif()
        unset(_gnsssdr_rh_id)
    endif()

    if("${LINUX_VER}" STREQUAL "")
        # Extract the numeric X.Y version; fall back to a bare integer
        # (e.g. Fedora uses a single number).
        string(REGEX MATCH "[0-9]+\\.[0-9]+" LINUX_VER
            "${_gnsssdr_redhat_release}")
        if("${LINUX_VER}" STREQUAL "")
            string(REGEX MATCH "[0-9]+" LINUX_VER
                "${_gnsssdr_redhat_release}")
        endif()
        if(NOT "${LINUX_VER}" STREQUAL "")
            set(LINUX_VER_RESULT 0)
        endif()
    endif()
    unset(_gnsssdr_redhat_release)
endif()


# ---------------------------------------------------------------------------
# 5. /etc/debian_version without a prior ID -- Debian-based system that
#    lacked os-release entirely.
# ---------------------------------------------------------------------------
if("${LINUX_DISTRIBUTION}" STREQUAL "" AND EXISTS "/etc/debian_version")
    set(LINUX_DISTRIBUTION "Debian")
    set(LINUX_ID "debian")
    set(LINUX_ID_RESULT 0)
    file(READ "/etc/debian_version" _gnsssdr_debian_version)
    string(STRIP "${_gnsssdr_debian_version}" _gnsssdr_debian_version)
    if(NOT "${_gnsssdr_debian_version}" STREQUAL "")
        set(LINUX_VER "${_gnsssdr_debian_version}")
        set(LINUX_VER_RESULT 0)
    endif()
    unset(_gnsssdr_debian_version)
endif()


# ---------------------------------------------------------------------------
# 6. /etc/alpine-release -- Alpine Linux.
#    This file contains only a bare version string, e.g. "3.19.1" or "edge".
# ---------------------------------------------------------------------------
if(("${LINUX_DISTRIBUTION}" STREQUAL "" OR
        "${LINUX_VER}" STREQUAL "" OR
        "${LINUX_ID}" STREQUAL "") AND
        EXISTS "/etc/alpine-release")
    if("${LINUX_DISTRIBUTION}" STREQUAL "")
        set(LINUX_DISTRIBUTION "Alpine Linux")
        set(LINUX_ID_RESULT 0)
    endif()

    if("${LINUX_ID}" STREQUAL "")
        set(LINUX_ID "alpine")
    endif()

    if("${LINUX_VER}" STREQUAL "")
        file(READ "/etc/alpine-release" _gnsssdr_alpine_version)
        string(STRIP "${_gnsssdr_alpine_version}" _gnsssdr_alpine_version)
        if(NOT "${_gnsssdr_alpine_version}" STREQUAL "")
            set(LINUX_VER "${_gnsssdr_alpine_version}")
            set(LINUX_VER_RESULT 0)
        endif()
        unset(_gnsssdr_alpine_version)
    endif()
endif()


# ---------------------------------------------------------------------------
# 7. /etc/arch-release -- Arch Linux (rolling; no meaningful version number).
# ---------------------------------------------------------------------------
if("${LINUX_DISTRIBUTION}" STREQUAL "" AND EXISTS "/etc/arch-release")
    set(LINUX_DISTRIBUTION "Arch Linux")
    set(LINUX_ID "arch")
    set(LINUX_VER "")
    set(LINUX_ID_RESULT 0)
endif()


# ---------------------------------------------------------------------------
# 8. Last-resort generic fallback.
# ---------------------------------------------------------------------------
if("${LINUX_DISTRIBUTION}" STREQUAL "")
    set(LINUX_DISTRIBUTION "Generic")
    set(LINUX_VER "Unknown")
endif()


# ---------------------------------------------------------------------------
# Clean up intermediate variables used at include scope.
# ---------------------------------------------------------------------------
unset(_gnsssdr_os_id)
unset(_gnsssdr_os_name)
unset(_gnsssdr_os_version_id)
