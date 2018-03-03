/*!
 * \file saturation_arithmetic.h
 * \brief Defines addition of 16-bit integers with saturation
 * \author Javier Arribas, 2015. javier.arribas(at)cttc.es
 *
 * Copyright (C) 2010-2015 (see AUTHORS file for a list of contributors)
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


#ifndef INCLUDED_VOLK_GNSSSDR_SATURATION_ARITHMETIC_H_
#define INCLUDED_VOLK_GNSSSDR_SATURATION_ARITHMETIC_H_

#include <limits.h>

static inline int16_t sat_adds16i(int16_t x, int16_t y)
{
    int32_t res = (int32_t)x + (int32_t)y;

    if (res < SHRT_MIN) res = SHRT_MIN;
    if (res > SHRT_MAX) res = SHRT_MAX;

    return res;
}

static inline int16_t sat_muls16i(int16_t x, int16_t y)
{
    int32_t res = (int32_t)x * (int32_t)y;

    if (res < SHRT_MIN) res = SHRT_MIN;
    if (res > SHRT_MAX) res = SHRT_MAX;

    return res;
}

#endif /* INCLUDED_VOLK_GNSSSDR_SATURATION_ARITHMETIC_H_ */
