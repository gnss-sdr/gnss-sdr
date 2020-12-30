/*
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2019 (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifndef INCLUDED_VOLK_GNSSSDR_CPU_H
#define INCLUDED_VOLK_GNSSSDR_CPU_H

#include <volk_gnsssdr/volk_gnsssdr_common.h>

__VOLK_DECL_BEGIN

// clang-format off
struct VOLK_CPU {
    %for arch in archs:
    int (*has_${arch.name}) ();
    %endfor
};
// clang-format on

extern struct VOLK_CPU volk_gnsssdr_cpu;

void volk_gnsssdr_cpu_init();
unsigned int volk_gnsssdr_get_lvarch();

__VOLK_DECL_END

#endif /* INCLUDED_VOLK_GNSSSDR_CPU_H */
