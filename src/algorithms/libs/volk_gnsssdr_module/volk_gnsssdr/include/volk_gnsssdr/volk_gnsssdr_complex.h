/*!
 * \file volk_gnsssdr_complex.h
 * \brief Provide typedefs and operators for all complex types in C and C++.
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

#ifndef INCLUDE_VOLK_COMPLEX_H
#define INCLUDE_VOLK_COMPLEX_H

#ifndef INCLUDED_VOLK_COMPLEX_H
#define INCLUDED_VOLK_COMPLEX_H

/*!
 * \brief Provide typedefs and operators for all complex types in C and C++.
 *
 * The typedefs encompass all signed integer and floating point types.
 * Each operator function is intended to work across all data types.
 * Under C++, these operators are defined as inline templates.
 * Under C, these operators are defined as preprocessor macros.
 * The use of macros makes the operators agnostic to the type.
 *
 * The following operator functions are defined:
 * - lv_cmake - make a complex type from components
 * - lv_creal - get the real part of the complex number
 * - lv_cimag - get the imaginary part of the complex number
 * - lv_conj - take the conjugate of the complex number
 */

#ifdef __cplusplus

#include <complex>
#include <stdint.h>

typedef std::complex<int8_t> lv_8sc_t;
typedef std::complex<int16_t> lv_16sc_t;
typedef std::complex<int32_t> lv_32sc_t;
typedef std::complex<int64_t> lv_64sc_t;
typedef std::complex<float> lv_32fc_t;
typedef std::complex<double> lv_64fc_t;

template <typename T>
inline std::complex<T> lv_cmake(const T &r, const T &i)
{
    return std::complex<T>(r, i);
}

template <typename T>
inline typename T::value_type lv_creal(const T &x)
{
    return x.real();
}

template <typename T>
inline typename T::value_type lv_cimag(const T &x)
{
    return x.imag();
}

template <typename T>
inline T lv_conj(const T &x)
{
    return std::conj(x);
}

#else /* __cplusplus */

#if __STDC_VERSION__ >= 199901L /* C99 check */
/* this allows us to conj in lv_conj without the double detour for single-precision floats */
#include <tgmath.h>
#endif /* C99 check */

#include <complex.h>

typedef char complex lv_8sc_t;
typedef short complex lv_16sc_t;
typedef long complex lv_32sc_t;
typedef long long complex lv_64sc_t;
typedef float complex lv_32fc_t;
typedef double complex lv_64fc_t;

#define lv_cmake(r, i) ((r) + _Complex_I * (i))

// When GNUC is available, use the complex extensions.
// The extensions always return the correct value type.
// http://gcc.gnu.org/onlinedocs/gcc/Complex.html
#ifdef __GNUC__

#define lv_creal(x) (__real__(x))

#define lv_cimag(x) (__imag__(x))

#define lv_conj(x) (~(x))

// When not available, use the c99 complex function family,
// which always returns double regardless of the input type,
// unless we have C99 and thus tgmath.h overriding functions
// with type-generic versions.
#else /* __GNUC__ */

#define lv_creal(x) (creal(x))

#define lv_cimag(x) (cimag(x))

#define lv_conj(x) (conj(x))

#endif /* __GNUC__ */

#endif /* __cplusplus */


#endif /* INCLUDED_VOLK_COMPLEX_H */

#endif /* INCLUDE_VOLK_COMPLEX_H */
