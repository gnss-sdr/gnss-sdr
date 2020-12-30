/*!
 * \file gnss_sdr_make_unique.h
 * \brief This file implements std::make_unique for C++11
 *
 * \author Carles Fernandez-Prades, 2020. cfernandez(at)cttc.es
 *
 * Based on https://stackoverflow.com/a/17902439
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GNSS_SDR_MAKE_UNIQUE_H
#define GNSS_SDR_GNSS_SDR_MAKE_UNIQUE_H

#if __cplusplus == 201103L

#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */


namespace std
{
template <class T>
struct _Unique_if
{
    typedef unique_ptr<T> _Single_object;
};

template <class T>
struct _Unique_if<T[]>
{
    typedef unique_ptr<T[]> _Unknown_bound;
};

template <class T, size_t N>
struct _Unique_if<T[N]>
{
    typedef void _Known_bound;
};

template <class T, class... Args>
typename _Unique_if<T>::_Single_object
make_unique(Args&&... args)
{
    return unique_ptr<T>(new T(std::forward<Args>(args)...));
}

template <class T>
typename _Unique_if<T>::_Unknown_bound
make_unique(size_t n)
{
    typedef typename remove_extent<T>::type U;
    return unique_ptr<T>(new U[n]());
}

template <class T, class... Args>
typename _Unique_if<T>::_Known_bound
make_unique(Args&&...) = delete;
}  // namespace std

#endif  // __cplusplus == 201103L


/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_SDR_MAKE_UNIQUE_H
