/*
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2019 (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

// clang-format off
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
// clang-format on
unsigned int n_volk_gnsssdr_machines = sizeof(volk_gnsssdr_machines) / sizeof(*volk_gnsssdr_machines);
