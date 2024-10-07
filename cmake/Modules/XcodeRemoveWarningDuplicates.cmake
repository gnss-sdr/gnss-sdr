# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2024 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

if(DEFINED __INCLUDED_XCODE_REMOVE_WARNING_DUPLICATES_CMAKE)
    return()
endif()
set(__INCLUDED_XCODE_REMOVE_WARNING_DUPLICATES_CMAKE TRUE)

function(xcode_remove_warning_duplicates target)
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
        if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "15.0.0")
            # A bug in Xcode 15 adds duplicate flags to the linker. In addition, the
            # `-warn_duplicate_libraries` is now enabled by default which may result
            # in several 'duplicate libraries warning'.
            #   - https://gitlab.kitware.com/cmake/cmake/-/issues/25297 and
            #   - https://indiestack.com/2023/10/xcode-15-duplicate-library-linker-warnings/
            target_link_options(${target} PUBLIC "LINKER:-no_warn_duplicate_libraries")
        endif()
    endif()
endfunction()