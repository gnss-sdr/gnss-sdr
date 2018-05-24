/*
 * Copyright (C) 2010-2018 (see AUTHORS file for a list of contributors)
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef INCLUDED_LIBVOLK_GNSSSDR_MACHINES_H
#define INCLUDED_LIBVOLK_GNSSSDR_MACHINES_H

#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <volk_gnsssdr/volk_gnsssdr_typedefs.h>
#include <stdbool.h>
#include <stdlib.h>

__VOLK_DECL_BEGIN

// clang-format off
struct volk_gnsssdr_machine {
    const unsigned int caps; //capabilities (i.e., archs compiled into this machine, in the volk_gnsssdr_get_lvarch format)
    const char *name;
    const size_t alignment; //the maximum byte alignment required for functions in this library
    %for kern in kernels:
    const char *${kern.name}_name;
    const char *${kern.name}_impl_names[<%len_archs=len(archs)%>${len_archs}];
    const int ${kern.name}_impl_deps[${len_archs}];
    const bool ${kern.name}_impl_alignment[${len_archs}];
    const ${kern.pname} ${kern.name}_impls[${len_archs}];
    const size_t ${kern.name}_n_impls;
    %endfor
};

%for machine in machines:
#ifdef LV_MACHINE_${machine.name.upper()}
extern struct volk_gnsssdr_machine volk_gnsssdr_machine_${machine.name};
#endif
%endfor

__VOLK_DECL_END
// clang-format on

#endif  //INCLUDED_LIBVOLK_GNSSSDR_MACHINES_H
