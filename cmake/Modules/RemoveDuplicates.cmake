# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2024 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

# Usage:
# include(RemoveDuplicates)
# remove_duplicate_linked_libraries(my_target)

if(DEFINED __INCLUDED_REMOVE_DUPLICATE_LINKED_LIBRARIES_MODULE)
    return()
endif()
set(__INCLUDED_REMOVE_DUPLICATE_LINKED_LIBRARIES_MODULE TRUE)

function(remove_duplicate_linked_libraries target_name)
    get_target_property(LINK_LIBRARIES ${target_name} LINK_LIBRARIES)
    list(REMOVE_DUPLICATES LINK_LIBRARIES)
    set_target_properties(${target_name} PROPERTIES LINK_LIBRARIES "${LINK_LIBRARIES}")
endfunction()