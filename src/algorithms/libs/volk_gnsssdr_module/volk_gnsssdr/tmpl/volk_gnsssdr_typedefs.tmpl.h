/* Copyright (C) 2010-2018 (see AUTHORS file for a list of contributors)
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

#ifndef INCLUDED_VOLK_GNSSSDR_TYPEDEFS
#define INCLUDED_VOLK_GNSSSDR_TYPEDEFS

#include <inttypes.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>

// clang-format off
%for kern in kernels:
typedef void (*${kern.pname})(${kern.arglist_types});
%endfor
// clang-format on

#endif /*INCLUDED_VOLK_GNSSSDR_TYPEDEFS*/
