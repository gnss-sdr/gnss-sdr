/*!
 * \file volk_gnsssdr_prefs.h
 * \brief  Gets path to volk_gnsssdr_config profiling info and loads
 * prefs into global prefs struct
 * \author Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *
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

#ifndef INCLUDED_VOLK_GNSSSDR_PREFS_H
#define INCLUDED_VOLK_GNSSSDR_PREFS_H

#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <stdlib.h>

__VOLK_DECL_BEGIN

typedef struct volk_gnsssdr_arch_pref
{
    char name[128];    //name of the kernel
    char impl_a[128];  //best aligned impl
    char impl_u[128];  //best unaligned impl
} volk_gnsssdr_arch_pref_t;

////////////////////////////////////////////////////////////////////////
// get path to volk_gnsssdr_config profiling info;
// returns \0 in the argument on failure.
////////////////////////////////////////////////////////////////////////
VOLK_API void volk_gnsssdr_get_config_path(char *);

////////////////////////////////////////////////////////////////////////
// load prefs into global prefs struct
////////////////////////////////////////////////////////////////////////
VOLK_API size_t volk_gnsssdr_load_preferences(volk_gnsssdr_arch_pref_t **);

__VOLK_DECL_END

#endif /* INCLUDED_VOLK_GNSSSDR_PREFS_H */
