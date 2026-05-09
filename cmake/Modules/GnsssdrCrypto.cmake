# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2024-2026 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

# Finds and configures a TLS/crypto backend for gnss-sdr (OpenSSL or GnuTLS).
#
# OpenSSL is preferred. GnuTLS with OpenSSL compatibility is used as a
# fallback when OpenSSL is not found or when ENABLE_GNUTLS is set.
# If neither library is available, a fatal error is raised.
#
# Input variables:
#   ENABLE_GNUTLS            - If set, skip OpenSSL and search for GnuTLS only
#   OPENSSL_ROOT_DIR         - Optional hint for the OpenSSL installation root
#   GNUTLS_ROOT_DIR          - Optional hint for the GnuTLS installation root
#
# Output variables:
#   GNSSSDR_OPENSSL_FOUND         - TRUE if a usable OpenSSL installation was found
#   GNSSSDR_OPENSSL_VERSION       - OpenSSL version string (e.g. "3.2.1")
#   GNSSSDR_GNUTLS_FOUND          - TRUE if a usable GnuTLS installation was found
#   GNSSSDR_GNUTLS_VERSION        - GnuTLS version string
#   GNSSSDR_GNUTLS_INCLUDE_DIR    - Path to the GnuTLS include directory
#
# Provided function:
#   link_to_crypto_dependencies(<target>)
#     Links <target> against the selected crypto backend and sets the
#     appropriate compile definitions:
#       USE_OPENSSL_3=1            (OpenSSL >= 3.0.0)
#       USE_OPENSSL_111=1          (OpenSSL >= 1.1.1, < 3.0.0)
#       USE_GNUTLS_FALLBACK=1      (GnuTLS backend)
#     When using GnuTLS, additional HAVE_GNUTLS_* definitions are set
#     based on the features detected in the installed headers.

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT DEFINED GNSSSDR_LIB_PATHS)
    include(GnsssdrFindPaths)
endif()

set(GNSSSDR_OPENSSL_FOUND FALSE)
set(GNSSSDR_OPENSSL_VERSION "")
set(GNSSSDR_OPENSSL_VERSION_NUMBER "")
set(GNSSSDR_GNUTLS_FOUND FALSE)
set(GNSSSDR_GNUTLS_VERSION "")
set(GNSSSDR_GNUTLS_INCLUDE_DIR "")

################################################################################
# OpenSSL https://www.openssl.org/
################################################################################
if("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
    if((NOT DEFINED OPENSSL_ROOT_DIR) OR ("${OPENSSL_ROOT_DIR}" STREQUAL ""))
        foreach(_gnsssdr_openssl_root_candidate
            /opt/homebrew/opt/openssl@3
            /usr/local/opt/openssl@3
            /opt/homebrew/opt/openssl
            /usr/local/opt/openssl
            /opt/local
        )
            if(EXISTS "${_gnsssdr_openssl_root_candidate}/include/openssl/ssl.h")
                set(OPENSSL_ROOT_DIR "${_gnsssdr_openssl_root_candidate}")
                break()
            endif()
        endforeach()
        unset(_gnsssdr_openssl_root_candidate)
    endif()
endif()

if(NOT ENABLE_GNUTLS)
    if(CMAKE_VERSION VERSION_LESS "3.12")
        find_package(OpenSSL)
    else()
        find_package(OpenSSL COMPONENTS SSL)
    endif()

    if(OpenSSL_FOUND OR OPENSSL_FOUND)
        set(GNSSSDR_OPENSSL_FOUND TRUE)
    endif()

    # CMake < 3.12 did not support OpenSSL components.  Check that libssl was
    # actually found, because gnss-sdr needs both SSL/TLS and crypto support.
    if(GNSSSDR_OPENSSL_FOUND AND CMAKE_VERSION VERSION_LESS "3.12")
        if((NOT TARGET OpenSSL::SSL) AND
                (NOT OPENSSL_SSL_LIBRARY) AND
                (NOT OPENSSL_SSL_LIBRARIES))
            set(GNSSSDR_OPENSSL_FOUND FALSE)
        endif()
    endif()

    if(OpenSSL_VERSION)
        set(GNSSSDR_OPENSSL_VERSION "${OpenSSL_VERSION}")
    elseif(OPENSSL_VERSION)
        set(GNSSSDR_OPENSSL_VERSION "${OPENSSL_VERSION}")
    endif()

    if(GNSSSDR_OPENSSL_VERSION)
        string(REGEX MATCH "^[0-9]+(\\.[0-9]+)*"
            GNSSSDR_OPENSSL_VERSION_NUMBER
            "${GNSSSDR_OPENSSL_VERSION}"
        )
    endif()

    if(GNSSSDR_OPENSSL_FOUND)
        if(GNSSSDR_OPENSSL_VERSION)
            set_package_properties(OpenSSL
                PROPERTIES
                    URL "https://www.openssl.org"
                    DESCRIPTION "Cryptography and SSL/TLS Toolkit (found: v${GNSSSDR_OPENSSL_VERSION})"
                    PURPOSE "Used for the OSNMA and SUPL protocol implementations."
                    TYPE REQUIRED
            )
        else()
            set_package_properties(OpenSSL
                PROPERTIES
                    URL "https://www.openssl.org"
                    DESCRIPTION "Cryptography and SSL/TLS Toolkit"
                    PURPOSE "Used for the OSNMA and SUPL protocol implementations."
                    TYPE REQUIRED
            )
        endif()
    else()
        set_package_properties(OpenSSL
            PROPERTIES
                URL "https://www.openssl.org"
                DESCRIPTION "OpenSSL has not been found, but GnuTLS with OpenSSL compatibility can replace it."
                PURPOSE "Used for the OSNMA and SUPL protocol implementations."
                TYPE RECOMMENDED
        )
    endif()
endif()


################################################################################
# GnuTLS https://www.gnutls.org/
################################################################################
if(NOT GNSSSDR_OPENSSL_FOUND)
    find_package(GnuTLS)

    if(GnuTLS_FOUND OR GNUTLS_FOUND)
        set(GNSSSDR_GNUTLS_FOUND TRUE)
    endif()

    if(GnuTLS_VERSION)
        set(GNSSSDR_GNUTLS_VERSION "${GnuTLS_VERSION}")
    elseif(GNUTLS_VERSION)
        set(GNSSSDR_GNUTLS_VERSION "${GNUTLS_VERSION}")
    elseif(GNUTLS_VERSION_STRING)
        set(GNSSSDR_GNUTLS_VERSION "${GNUTLS_VERSION_STRING}")
    endif()

    if(GNSSSDR_GNUTLS_VERSION)
        set_package_properties(GnuTLS
            PROPERTIES
                URL "https://www.gnutls.org/"
                DESCRIPTION "Transport Layer Security Library (found: v${GNSSSDR_GNUTLS_VERSION})"
                PURPOSE "Used for the OSNMA and SUPL protocol implementations."
                TYPE REQUIRED
        )
    else()
        set_package_properties(GnuTLS
            PROPERTIES
                URL "https://www.gnutls.org/"
                DESCRIPTION "Transport Layer Security Library"
                PURPOSE "Used for the OSNMA and SUPL protocol implementations."
                TYPE REQUIRED
        )
    endif()

    set(_gnsssdr_gnutls_library_paths ${GNSSSDR_LIB_PATHS})
    if(GNUTLS_ROOT_DIR)
        list(APPEND _gnsssdr_gnutls_library_paths
            "${GNUTLS_ROOT_DIR}/lib"
            "${GNUTLS_ROOT_DIR}/lib64"
        )
    endif()

    find_library(GNUTLS_OPENSSL_LIBRARY
        NAMES gnutls-openssl libgnutls-openssl.so.27
        PATHS ${_gnsssdr_gnutls_library_paths}
    )
    unset(_gnsssdr_gnutls_library_paths)

    set(_gnsssdr_gnutls_include_paths ${GNSSSDR_INCLUDE_PATHS})
    if(GNUTLS_ROOT_DIR)
        list(APPEND _gnsssdr_gnutls_include_paths
            "${GNUTLS_ROOT_DIR}/include"
        )
    endif()

    find_path(GNUTLS_INCLUDE_DIR
        NAMES gnutls/gnutls.h
        PATHS ${_gnsssdr_gnutls_include_paths}
    )
    unset(_gnsssdr_gnutls_include_paths)

    if(GNUTLS_INCLUDE_DIR)
        set(GNSSSDR_GNUTLS_INCLUDE_DIR "${GNUTLS_INCLUDE_DIR}")
    elseif(GNUTLS_INCLUDE_DIRS)
        set(_gnsssdr_gnutls_include_dirs ${GNUTLS_INCLUDE_DIRS})
        list(GET _gnsssdr_gnutls_include_dirs 0 GNSSSDR_GNUTLS_INCLUDE_DIR)
        unset(_gnsssdr_gnutls_include_dirs)
    endif()

    if((NOT GNSSSDR_GNUTLS_FOUND) OR
            (NOT GNUTLS_OPENSSL_LIBRARY) OR
            (NOT GNSSSDR_GNUTLS_INCLUDE_DIR))
        message(STATUS "The GnuTLS library with OpenSSL compatibility has not been found.")
        message(STATUS "You can try to install the required libraries by typing:")

        if("${CMAKE_SYSTEM_NAME}" MATCHES "Linux|kFreeBSD|GNU")
            if("${LINUX_DISTRIBUTION}" MATCHES "Fedora" OR
                    "${LINUX_DISTRIBUTION}" MATCHES "Red Hat")
                message(STATUS " sudo yum install openssl-devel gnutls-devel")
            else()
                message(STATUS " sudo apt install libssl-dev libgnutls28-dev")
            endif()
        endif()

        if("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
            message(STATUS " sudo port install openssl3, if you are using MacPorts, or")
            message(STATUS " brew install openssl@3, if you are using Homebrew.")
        endif()

        message(FATAL_ERROR
            "OpenSSL or GnuTLS with OpenSSL compatibility is required "
            "to build gnss-sdr"
        )
    endif()

    set(GNUTLS_SIGN_ECDSA_SHA256 FALSE)
    set(GNUTLS_SIGN_ECDSA_SHA512 FALSE)
    set(GNUTLS_DIG_SHA3_256 FALSE)
    set(GNUTLS_HMAC_INIT_WITH_DIGEST FALSE)
    set(GNUTLS_MAC_AES_CMAC_128 FALSE)
    set(GNUTLS_PUBKEY_EXPORT2 FALSE)

    set(_gnsssdr_gnutls_header
        "${GNSSSDR_GNUTLS_INCLUDE_DIR}/gnutls/gnutls.h"
    )
    set(_gnsssdr_gnutls_abstract_header
        "${GNSSSDR_GNUTLS_INCLUDE_DIR}/gnutls/abstract.h"
    )

    if(EXISTS "${_gnsssdr_gnutls_header}")
        file(READ "${_gnsssdr_gnutls_header}" gnutls_gnutls_file_contents)

        if("${gnutls_gnutls_file_contents}" MATCHES "GNUTLS_SIGN_ECDSA_SHA256")
            set(GNUTLS_SIGN_ECDSA_SHA256 TRUE)
        endif()

        if("${gnutls_gnutls_file_contents}" MATCHES "GNUTLS_SIGN_ECDSA_SHA512")
            set(GNUTLS_SIGN_ECDSA_SHA512 TRUE)
        endif()

        if("${gnutls_gnutls_file_contents}" MATCHES "GNUTLS_DIG_SHA3_256")
            set(GNUTLS_DIG_SHA3_256 TRUE)
        endif()

        set(_gnsssdr_gnutls_hmac_old FALSE)
        if("${gnutls_gnutls_file_contents}" MATCHES "#define GNUTLS_VERSION_MAJOR 2")
            set(_gnsssdr_gnutls_hmac_old TRUE)
        elseif("${gnutls_gnutls_file_contents}" MATCHES "#define GNUTLS_VERSION_MAJOR 3")
            if("${gnutls_gnutls_file_contents}" MATCHES "#define GNUTLS_VERSION_MINOR 0")
                if(NOT "${gnutls_gnutls_file_contents}" MATCHES "#define GNUTLS_VERSION_MINOR 0[0-9]")
                    set(_gnsssdr_gnutls_hmac_old TRUE)
                endif()
            endif()
        endif()
        if(_gnsssdr_gnutls_hmac_old)
            set(GNUTLS_HMAC_INIT_WITH_DIGEST TRUE)
        endif()
        unset(_gnsssdr_gnutls_hmac_old)

        if("${gnutls_gnutls_file_contents}" MATCHES "GNUTLS_MAC_AES_CMAC_128")
            set(GNUTLS_MAC_AES_CMAC_128 TRUE)
        endif()
    else()
        message(FATAL_ERROR
            "The GnuTLS header ${_gnsssdr_gnutls_header} has not been found"
        )
    endif()

    if(EXISTS "${_gnsssdr_gnutls_abstract_header}")
        file(READ
            "${_gnsssdr_gnutls_abstract_header}"
            gnutls_abstract_file_contents
        )

        if("${gnutls_abstract_file_contents}" MATCHES "gnutls_pubkey_export2")
            set(GNUTLS_PUBKEY_EXPORT2 TRUE)
        endif()
    endif()

    unset(_gnsssdr_gnutls_header)
    unset(_gnsssdr_gnutls_abstract_header)

    find_package(GMP)
    set_package_properties(GMP
        PROPERTIES
            URL "https://gmplib.org/"
            PURPOSE "Required to decompress cryptographic keys."
            TYPE REQUIRED
    )

    if(NOT GMP_FOUND)
        message(FATAL_ERROR
            "GMP is required by gnss-sdr when linking against GnuTLS"
        )
    endif()
endif()


################################################################################


function(link_to_crypto_dependencies target)
    if("${target}" STREQUAL "")
        message(FATAL_ERROR "link_to_crypto_dependencies called without a target")
    endif()

    if(NOT TARGET ${target})
        message(FATAL_ERROR
            "link_to_crypto_dependencies called for unknown target: ${target}"
        )
    endif()

    if(GNSSSDR_OPENSSL_FOUND)
        if(TARGET OpenSSL::SSL)
            target_link_libraries(${target}
                PUBLIC
                    OpenSSL::SSL
            )

            if(MSVC AND TARGET OpenSSL::applink)
                target_link_libraries(${target}
                    PRIVATE
                        OpenSSL::applink
                )
            endif()
        else()
            set(_gnsssdr_openssl_libraries ${OPENSSL_LIBRARIES})

            if(NOT _gnsssdr_openssl_libraries)
                list(APPEND _gnsssdr_openssl_libraries
                    ${OPENSSL_SSL_LIBRARIES}
                    ${OPENSSL_CRYPTO_LIBRARIES}
                    ${OPENSSL_SSL_LIBRARY}
                    ${OPENSSL_CRYPTO_LIBRARY}
                )
            endif()

            if(_gnsssdr_openssl_libraries)
                list(REMOVE_DUPLICATES _gnsssdr_openssl_libraries)
                target_link_libraries(${target}
                    PUBLIC
                        ${_gnsssdr_openssl_libraries}
                )
            endif()

            if(OPENSSL_INCLUDE_DIR)
                target_include_directories(${target}
                    PUBLIC
                        ${OPENSSL_INCLUDE_DIR}
                )
            elseif(OPENSSL_INCLUDE_DIRS)
                target_include_directories(${target}
                    PUBLIC
                        ${OPENSSL_INCLUDE_DIRS}
                )
            endif()

            unset(_gnsssdr_openssl_libraries)
        endif()

        if(GNSSSDR_OPENSSL_VERSION_NUMBER)
            if(NOT "${GNSSSDR_OPENSSL_VERSION_NUMBER}" VERSION_LESS "3.0.0")
                target_compile_definitions(${target}
                    PUBLIC
                        USE_OPENSSL_3=1
                )
            elseif(NOT "${GNSSSDR_OPENSSL_VERSION_NUMBER}" VERSION_LESS "1.1.1")
                target_compile_definitions(${target}
                    PUBLIC
                        USE_OPENSSL_111=1
                )
            endif()
        endif()
    else()
        if(TARGET GnuTLS::GnuTLS)
            target_link_libraries(${target}
                PUBLIC
                    GnuTLS::GnuTLS
                    ${GNUTLS_OPENSSL_LIBRARY}
            )
        else()
            target_link_libraries(${target}
                PUBLIC
                    ${GNUTLS_LIBRARIES}
                    ${GNUTLS_OPENSSL_LIBRARY}
            )

            if(GNUTLS_INCLUDE_DIRS)
                target_include_directories(${target}
                    PUBLIC
                        ${GNUTLS_INCLUDE_DIRS}
                )
            elseif(GNSSSDR_GNUTLS_INCLUDE_DIR)
                target_include_directories(${target}
                    PUBLIC
                        ${GNSSSDR_GNUTLS_INCLUDE_DIR}
                )
            endif()
        endif()

        if(TARGET Gmp::gmp)
            target_link_libraries(${target}
                PRIVATE
                    Gmp::gmp
            )
        else()
            if(GMP_LIBRARIES)
                target_link_libraries(${target}
                    PRIVATE
                        ${GMP_LIBRARIES}
                )
            endif()

            if(GMP_INCLUDE_DIR)
                target_include_directories(${target}
                    PRIVATE
                        ${GMP_INCLUDE_DIR}
                )
            elseif(GMP_INCLUDE_DIRS)
                target_include_directories(${target}
                    PRIVATE
                        ${GMP_INCLUDE_DIRS}
                )
            endif()
        endif()

        if(GNUTLS_DEFINITIONS)
            target_compile_options(${target}
                PUBLIC
                    ${GNUTLS_DEFINITIONS}
            )
        endif()

        target_compile_definitions(${target}
            PUBLIC
                USE_GNUTLS_FALLBACK=1
        )

        if(GNUTLS_SIGN_ECDSA_SHA256)
            target_compile_definitions(${target}
                PRIVATE
                    HAVE_GNUTLS_SIGN_ECDSA_SHA256=1
            )
        endif()

        if(GNUTLS_SIGN_ECDSA_SHA512)
            target_compile_definitions(${target}
                PRIVATE
                    HAVE_GNUTLS_SIGN_ECDSA_SHA512=1
            )
        endif()

        if(GNUTLS_DIG_SHA3_256)
            target_compile_definitions(${target}
                PRIVATE
                    HAVE_GNUTLS_DIG_SHA3_256=1
            )
        endif()

        if(GNUTLS_PUBKEY_EXPORT2)
            target_compile_definitions(${target}
                PRIVATE
                    HAVE_GNUTLS_PUBKEY_EXPORT2=1
            )
        endif()

        if(GNUTLS_HMAC_INIT_WITH_DIGEST)
            target_compile_definitions(${target}
                PRIVATE
                    HAVE_GNUTLS_HMAC_INIT_WITH_DIGEST=1
            )
        endif()

        if(GNUTLS_MAC_AES_CMAC_128)
            target_compile_definitions(${target}
                PRIVATE
                    HAVE_GNUTLS_MAC_AES_CMAC_128=1
            )
        endif()
    endif()
endfunction()