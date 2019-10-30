/*!
 * \file volk_gnsssdr_alloc.h
 * \author Carles Fernandez, 2019. cfernandez(at)cttc.es
 * \brief C++11 allocator using volk_gnsssdr_malloc and volk_gnsssdr_free.
 * Based on https://github.com/gnuradio/volk/pull/284/ by @hcab14
 *
 * Copyright (C) 2010-2019 (see AUTHORS file for a list of contributors)
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

#ifndef INCLUDED_VOLK_GNSSSDR_ALLOC_H
#define INCLUDED_VOLK_GNSSSDR_ALLOC_H

#include <volk_gnsssdr/volk_gnsssdr.h>
#include <cstdlib>
#include <limits>
#include <new>
#include <vector>

namespace volk_gnsssdr
{
/*!
 * \brief C++11 allocator using volk_gnsssdr_malloc and volk_gnsssdr_free
 *
 * \details
 *   adapted from https://en.cppreference.com/w/cpp/named_req/Alloc
 */
template <class T>
struct alloc
{
    typedef T value_type;

    alloc() = default;

    template <class U>
    constexpr alloc(alloc<U> const&) noexcept {}

    T* allocate(std::size_t n)
    {
        if (n > std::numeric_limits<std::size_t>::max() / sizeof(T)) throw std::bad_alloc();

        if (auto p = static_cast<T*>(volk_gnsssdr_malloc(n * sizeof(T), volk_gnsssdr_get_alignment())))
            return p;

        throw std::bad_alloc();
    }

    void deallocate(T* p, std::size_t) noexcept { volk_gnsssdr_free(p); }
};

template <class T, class U>
bool operator==(alloc<T> const&, alloc<U> const&) { return true; }

template <class T, class U>
bool operator!=(alloc<T> const&, alloc<U> const&) { return false; }


/*!
 * \brief type alias for std::vector using volk_gnsssdr::alloc
 *
 * \details
 * example code:
 *   volk_gnsssdr::vector<float> v(100); // vector using volk_gnsssdr_malloc, volk_gnsssdr_free
 */
template <class T>
using vector = std::vector<T, alloc<T> >;

}  // namespace volk_gnsssdr

#endif  // INCLUDED_VOLK_GNSSSDR_ALLOC_H
