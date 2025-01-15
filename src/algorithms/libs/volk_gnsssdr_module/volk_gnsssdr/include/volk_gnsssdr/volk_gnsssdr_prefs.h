/*!
 * \file volk_gnsssdr_prefs.h
 * \brief  Gets path to volk_gnsssdr_config profiling info and loads
 * prefs into global prefs struct
 * \author Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifndef INCLUDED_VOLK_GNSSSDR_PREFS_H
#define INCLUDED_VOLK_GNSSSDR_PREFS_H

#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <stdbool.h>
#include <stdlib.h>

__VOLK_DECL_BEGIN

typedef struct volk_gnsssdr_arch_pref
{
    char name[128];    // name of the kernel
    char impl_a[128];  // best aligned impl
    char impl_u[128];  // best unaligned impl
} volk_gnsssdr_arch_pref_t;

////////////////////////////////////////////////////////////////////////
// get path to volk_gnsssdr_config profiling info; second arguments specifies
// if config file should be tested on existence for reading.
// returns \0 in the argument on failure.
////////////////////////////////////////////////////////////////////////
VOLK_API void volk_gnsssdr_get_config_path(char *, bool);

////////////////////////////////////////////////////////////////////////
// load prefs into global prefs struct
////////////////////////////////////////////////////////////////////////
VOLK_API size_t volk_gnsssdr_load_preferences(volk_gnsssdr_arch_pref_t **);

__VOLK_DECL_END

#endif /* INCLUDED_VOLK_GNSSSDR_PREFS_H */
