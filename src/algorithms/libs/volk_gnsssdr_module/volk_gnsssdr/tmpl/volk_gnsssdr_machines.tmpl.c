/* Copyright (C) 2010-2015 (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 */

#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <volk_gnsssdr/volk_gnsssdr_typedefs.h>
#include "volk_gnsssdr_machines.h"

struct volk_gnsssdr_machine *volk_gnsssdr_machines[] = {
%for machine in machines:
#ifdef LV_MACHINE_${machine.name.upper()}
&volk_gnsssdr_machine_${machine.name},
#endif
%endfor
};

unsigned int n_volk_gnsssdr_machines = sizeof(volk_gnsssdr_machines)/sizeof(*volk_gnsssdr_machines);
