/*
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

#ifndef _MSC_VER  // [
#error "Use this header only with Microsoft Visual C++ compilers!"
#endif  // _MSC_VER ]

#ifndef _MSC_CONFIG_H_  // [
#define _MSC_CONFIG_H_

////////////////////////////////////////////////////////////////////////
// enable inline functions for C code
////////////////////////////////////////////////////////////////////////
#ifndef __cplusplus
#define inline __inline
#endif

////////////////////////////////////////////////////////////////////////
// signed size_t
////////////////////////////////////////////////////////////////////////
#include <stddef.h>
typedef ptrdiff_t ssize_t;

////////////////////////////////////////////////////////////////////////
// rint functions
////////////////////////////////////////////////////////////////////////
#if _MSC_VER < 1800
#include <math.h>
static inline long lrint(double x)
{
    return (long)(x > 0.0 ? x + 0.5 : x - 0.5);
}
static inline long lrintf(float x) { return (long)(x > 0.0f ? x + 0.5f : x - 0.5f); }
static inline long long llrint(double x) { return (long long)(x > 0.0 ? x + 0.5 : x - 0.5); }
static inline long long llrintf(float x) { return (long long)(x > 0.0f ? x + 0.5f : x - 0.5f); }
static inline double rint(double x) { return (x > 0.0) ? floor(x + 0.5) : ceil(x - 0.5); }
static inline float rintf(float x) { return (x > 0.0f) ? floorf(x + 0.5f) : ceilf(x - 0.5f); }
#endif

////////////////////////////////////////////////////////////////////////
// math constants
////////////////////////////////////////////////////////////////////////
#if _MSC_VER < 1800
#include <math.h>
#define INFINITY HUGE_VAL
#endif

////////////////////////////////////////////////////////////////////////
// random and srandom
////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
static inline long int random(void)
{
    return rand();
}
static inline void srandom(unsigned int seed) { srand(seed); }

#endif  // _MSC_CONFIG_H_ ]
