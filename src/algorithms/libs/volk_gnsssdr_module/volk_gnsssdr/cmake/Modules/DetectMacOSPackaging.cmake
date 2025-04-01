# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2025 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

set(MACOS_PACKAGES_PREFIX "")
# Detect if MacPorts is installed on this system; if so, return base path and version
execute_process(COMMAND which port RESULT_VARIABLE DETECT_MACPORTS OUTPUT_VARIABLE MACPORTS_PREFIX ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
if(${DETECT_MACPORTS} EQUAL 0)
    # "/opt/local/bin/port", so we get the parent directory
    get_filename_component(MACPORTS_PREFIX ${MACPORTS_PREFIX} DIRECTORY)
    # "/opt/local/bin", so we get the parent directory
    get_filename_component(MACPORTS_PREFIX ${MACPORTS_PREFIX} DIRECTORY)
    execute_process(COMMAND port version RESULT_VARIABLE DETECT_MACPORTS_VERSION OUTPUT_VARIABLE MACPORTS_VERSION ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" MACPORTS_VERSION "${MACPORTS_VERSION}")
    set(MACOS_PACKAGES_PREFIX ${MACPORTS_PREFIX})
endif()

# Detect if Homebrew is installed on this system; if so, return base path and version
execute_process(COMMAND brew --prefix RESULT_VARIABLE DETECT_HOMEBREW OUTPUT_VARIABLE HOMEBREW_PREFIX ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
if(${DETECT_HOMEBREW} EQUAL 0)
    execute_process(COMMAND brew --version RESULT_VARIABLE DETECT_HOMEBREW_VERSION OUTPUT_VARIABLE HOMEBREW_VERSION ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" HOMEBREW_VERSION "${HOMEBREW_VERSION}")
    set(MACOS_PACKAGES_PREFIX ${HOMEBREW_PREFIX})
endif()
