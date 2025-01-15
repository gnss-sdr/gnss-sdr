/*
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2019 (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifndef INCLUDED_VOLK_GNSSSDR_CONFIG_FIXED_H
#define INCLUDED_VOLK_GNSSSDR_CONFIG_FIXED_H

// clang-format off
%for i, arch in enumerate(archs):
#ifndef LV_${arch.name.upper()}
#define LV_${arch.name.upper()} ${i}
#endif
%endfor
// clang-format on
#endif  // INCLUDED_VOLK_GNSSSDR_CONFIG_FIXED
