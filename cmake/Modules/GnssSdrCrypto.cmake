# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2024 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


################################################################################
# OpenSSL https://www.openssl.org/
################################################################################
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(OPENSSL_ROOT_DIR /usr/local/opt/openssl) # Trick for Homebrew
endif()
find_package(OpenSSL)
set_package_properties(OpenSSL
    PROPERTIES
        URL "https://www.openssl.org"
        PURPOSE "Used for the OSNMA and SUPL protocol implementations."
        TYPE REQUIRED
)
if(OPENSSL_FOUND)
    set_package_properties(OpenSSL
        PROPERTIES
            DESCRIPTION "Cryptography and SSL/TLS Toolkit (found: v${OPENSSL_VERSION})"
    )
else()
    set_package_properties(OpenSSL
        PROPERTIES
            DESCRIPTION "OpenSSL has not been found, but GnuTLS with openssl compatibility can replace it"
    )
    ################################################################################
    # GnuTLS - https://www.gnutls.org/
    ################################################################################
    find_package(GnuTLS)
    set_package_properties(GnuTLS PROPERTIES
        URL "https://www.gnutls.org/"
        PURPOSE "Used for the OSNMA and SUPL protocol implementations."
        TYPE REQUIRED
    )
    if(GnuTLS_FOUND AND GNUTLS_VERSION_STRING)
        set_package_properties(GnuTLS PROPERTIES
            DESCRIPTION "Transport Layer Security Library (found: v${GNUTLS_VERSION_STRING})"
        )
    else()
        set_package_properties(GnuTLS PROPERTIES
            DESCRIPTION "Transport Layer Security Library"
        )
    endif()
    find_library(GNUTLS_OPENSSL_LIBRARY
        NAMES gnutls-openssl libgnutls-openssl.so.27
        PATHS
            /usr/lib
            /usr/lib64
            /usr/lib/x86_64-linux-gnu
            /usr/lib/aarch64-linux-gnu
            /usr/lib/arm-linux-gnueabihf
            /usr/lib/arm-linux-gnueabi
            /usr/lib/i386-linux-gnu
            /usr/lib/alpha-linux-gnu
            /usr/lib/hppa-linux-gnu
            /usr/lib/i386-gnu
            /usr/lib/i686-gnu
            /usr/lib/i686-linux-gnu
            /usr/lib/x86_64-kfreebsd-gnu
            /usr/lib/i686-kfreebsd-gnu
            /usr/lib/m68k-linux-gnu
            /usr/lib/mips-linux-gnu
            /usr/lib/mips64el-linux-gnuabi64
            /usr/lib/mipsel-linux-gnu
            /usr/lib/powerpc-linux-gnu
            /usr/lib/powerpc-linux-gnuspe
            /usr/lib/powerpc64-linux-gnu
            /usr/lib/powerpc64le-linux-gnu
            /usr/lib/s390x-linux-gnu
            /usr/lib/riscv64-linux-gnu
            /usr/lib/sparc64-linux-gnu
            /usr/lib/x86_64-linux-gnux32
            /usr/lib/sh4-linux-gnu
            /usr/lib/loongarch64-linux-gnu
            /usr/local/lib
            /usr/local/lib64
            /opt/local/lib
    )

    if(NOT GNUTLS_OPENSSL_LIBRARY)
        message(" The GnuTLS library with openssl compatibility enabled has not been found.")
        message(" You can try to install the required libraries by typing:")
        if(${CMAKE_SYSTEM_NAME} MATCHES "Linux|kFreeBSD|GNU")
            if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                message(" sudo yum install openssl-devel")
            else()
                message(" sudo apt-get install libgnutls28-dev")
            endif()
        endif()
        if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
            message(" 'sudo port install openssl3', if you are using Macports, or")
            message(" 'brew install openssl', if you are using Homebrew.")
        endif()
        message(FATAL_ERROR "OpenSSL or the GnuTLS libraries with openssl compatibility are required to build gnss-sdr")
    endif()
endif()

################################################################################

function(link_to_crypto_dependencies target)
    if(OPENSSL_FOUND)
        if(TARGET OpenSSL::SSL)
            target_link_libraries(${target}
                PUBLIC
                    OpenSSL::SSL
            )
            if(TARGET OpenSSL::Crypto)
                target_link_libraries(${target}
                    PUBLIC
                        OpenSSL::Crypto
                )
            endif()
        else()
            target_link_libraries(${target}
                PUBLIC
                    ${OPENSSL_LIBRARIES}
                    "${OPENSSL_CRYPTO_LIBRARIES}"
            )
            target_include_directories(${target}
                PUBLIC
                    ${OPENSSL_INCLUDE_DIR}
            )
        endif()
        if(OPENSSL_VERSION)
            if(OPENSSL_VERSION VERSION_GREATER "3.0.0")
                target_compile_definitions(${target} PUBLIC -DUSE_OPENSSL_3=1)
            else()
                if(NOT OPENSSL_VERSION VERSION_LESS "1.1.1")
                    target_compile_definitions(${target} PUBLIC -DUSE_OPENSSL_111=1)
                endif()
            endif()
        else()
        endif()
    else()  # GnuTLS
        target_link_libraries(${target}
            PUBLIC
                ${GNUTLS_LIBRARIES}
                ${GNUTLS_OPENSSL_LIBRARY}
        )
        target_include_directories(${target}
            PUBLIC
                ${GNUTLS_INCLUDE_DIR}
        )
        target_compile_definitions(${target} PUBLIC -DUSE_GNUTLS_FALLBACK=1)
    endif()
endfunction()
