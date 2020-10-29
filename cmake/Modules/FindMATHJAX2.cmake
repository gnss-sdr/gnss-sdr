# Copyright (C) 2011-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
#
# Find MathJax package (version 2).
#
# A hint can be provided by defining MATHJAX2_ROOT
# (will succeed if ${MATHJAX2_ROOT}/MathJax.js is found).
#
# Alternatively, a path can be provided in MATHJAX2_USE_ROOT
# so that ${MATHJAX2_USE_ROOT}/MathJax.js is used without
# checking its existence.
# This path could be a URL, an absolute local path or
# a path relative to the generated HTML folder.
#
# Note that version 2 and 3 are incompatible
# and doxygen requires version 2.
# See: https://github.com/doxygen/doxygen/issues/7346
#
# Defined variables:
#  - MATHJAX2_FOUND     - True if MathJax found
#  - MATHJAX2_JS_PATH   - Path to MathJax.js file
#  - MATHJAX2_PATH      - Path to the MathJax root directory


if(CMAKE_VERSION VERSION_LESS 3.11)
    if(__INCLUDED_MATHJAX2)
        return()
    endif()
    set(__INCLUDED_MATHJAX2 TRUE)
else()
    include_guard()
endif()

if(DEFINED MATHJAX2_USE_ROOT)
    set(MATHJAX2_FOUND TRUE)
    set(MATHJAX2_PATH "${MATHJAX2_USE_ROOT}/")
    set(MATHJAX2_JS_PATH "${MATHJAX2_USE_ROOT}/MathJax.js")
else()
    find_file(MATHJAX2_JS_PATH
        NAMES
            MathJax.js
        PATHS
            "${MATHJAX2_ROOT}"
            /usr/share/mathjax2/
            /usr/share/javascript/mathjax/
            /usr/local/share/javascript/mathjax/
    )

    get_filename_component(MATHJAX2_PATH ${MATHJAX2_JS_PATH} DIRECTORY)

    find_package_handle_standard_args(MATHJAX2 DEFAULT_MSG MATHJAX2_JS_PATH)
endif()

mark_as_advanced(MATHJAX2_JS_PATH)
