/*!
 * \file volk_gnsssdr_malloc.h
 * \brief The volk_gnsssdr_malloc function behaves like malloc in that it
 * returns a pointer to the allocated memory.
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

#ifndef INCLUDED_VOLK_GNSSSDR_MALLOC_H
#define INCLUDED_VOLK_GNSSSDR_MALLOC_H

#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <stdlib.h>

__VOLK_DECL_BEGIN

/*!
 * \brief Allocate \p size bytes of data aligned to \p alignment.
 *
 * \details
 * Because we don't have a standard method to allocate buffers in
 * memory that are guaranteed to be on an alignment, VOLK handles this
 * itself. The volk_gnsssdr_malloc function behaves like malloc in that it
 * returns a pointer to the allocated memory. However, it also takes
 * in an alignment specification, which is usually something like 16 or
 * 32 to ensure that the aligned memory is located on a particular
 * byte boundary for use with SIMD.
 *
 * Internally, the volk_gnsssdr_malloc first checks if the compiler is C11
 * compliant and uses the new aligned_alloc method. If not, it checks
 * if the system is POSIX compliant and uses posix_memalign. If that
 * fails, volk_gnsssdr_malloc handles the memory allocation and alignment
 * internally.
 *
 * Because of the ways in which volk_gnsssdr_malloc may allocate memory, it is
 * important to always free volk_gnsssdr_malloc pointers using volk_gnsssdr_free.
 *
 * \param size The number of bytes to allocate.
 * \param alignment The byte alignment of the allocated memory.
 * \return pointer to aligned memory.
 */
VOLK_API void *volk_gnsssdr_malloc(size_t size, size_t alignment);

/*!
 * \brief Free's memory allocated by volk_gnsssdr_malloc.
 * \param aptr The aligned pointer allocaed by volk_gnsssdr_malloc.
 */
VOLK_API void volk_gnsssdr_free(void *aptr);

__VOLK_DECL_END

#endif /* INCLUDED_VOLK_GNSSSDR_MALLOC_H */
