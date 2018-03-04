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

#ifndef INCLUDED_VOLK_GNSSSDR_CONFIG_FIXED_H
#define INCLUDED_VOLK_GNSSSDR_CONFIG_FIXED_H

// clang-format off
%for i, arch in enumerate(archs):
//#ifndef LV_${arch.name.upper()}
#define LV_${arch.name.upper()} ${i}
//#endif
%endfor
// clang-format on
#endif /*INCLUDED_VOLK_GNSSSDR_CONFIG_FIXED*/
