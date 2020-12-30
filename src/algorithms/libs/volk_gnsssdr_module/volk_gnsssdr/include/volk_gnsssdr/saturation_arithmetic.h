/*!
 * \file saturation_arithmetic.h
 * \brief Defines addition of 16-bit integers with saturation
 * \author Javier Arribas, 2015. javier.arribas(at)cttc.es
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 */


#ifndef INCLUDED_VOLK_GNSSSDR_SATURATION_ARITHMETIC_H
#define INCLUDED_VOLK_GNSSSDR_SATURATION_ARITHMETIC_H

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
