# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause
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
endif()
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MATHJAX2 DEFAULT_MSG MATHJAX2_JS_PATH)

mark_as_advanced(MATHJAX2_JS_PATH)
