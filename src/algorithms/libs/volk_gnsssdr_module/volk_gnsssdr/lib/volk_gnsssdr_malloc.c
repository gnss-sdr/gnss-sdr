/*
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2019 (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#include <volk_gnsssdr/volk_gnsssdr_malloc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * C11 features:
 * see: https://en.cppreference.com/w/c/memory/aligned_alloc
 *
 * MSVC is broken
 * see: https://docs.microsoft.com/en-us/cpp/overview/visual-cpp-language-conformance?view=vs-2019
 * This section:
 * C11 The Universal CRT implemented the parts of the
 * C11 Standard Library that are required by C++17,
 * with the exception of C99 strftime() E/O alternative
 * conversion specifiers, C11 fopen() exclusive mode,
 * and C11 aligned_alloc(). The latter is unlikely to
 * be implemented, because C11 specified aligned_alloc()
 * in a way that's incompatible with the Microsoft
 * implementation of free():
 * namely, that free() must be able to handle highly aligned allocations.
 *
 * We must work around this problem because MSVC is non-compliant!
 */

void *volk_gnsssdr_malloc(size_t size, size_t alignment)
{
    if ((size == 0) || (alignment == 0))
        {
            fprintf(stderr, "VOLK_GNSSSDR: Error allocating memory: either size or alignment is 0");
            return NULL;
        }
    // Tweak size to satisfy ASAN (the GCC address sanitizer).
    // Calling 'volk_gnsssdr_malloc' might therefor result in the allocation of more memory than
    // requested for correct alignment. Any allocation size change here will in general not
    // impact the end result since initial size alignment is required either way.
    if (size % alignment)
        {
            size += alignment - (size % alignment);
        }
#if HAVE_POSIX_MEMALIGN
    // quoting posix_memalign() man page:
    // "alignment must be a power of two and a multiple of sizeof(void *)"
    // volk_gnsssdr_get_alignment() could return 1 for some machines (e.g. generic_orc)
    if (alignment == 1)
        {
            return malloc(size);
        }
    void *ptr;
    int err = posix_memalign(&ptr, alignment, size);
    if (err != 0)
        {
            ptr = NULL;
            fprintf(stderr,
                "VOLK_GNSSSDR: Error allocating memory "
                "(posix_memalign: error %d: %s)\n",
                err, strerror(err));
        }
#elif defined(_MSC_VER) || defined(__MINGW32__)
    void *ptr = _aligned_malloc(size, alignment);
#else
    void *ptr = aligned_alloc(alignment, size);
#endif
    if (ptr == NULL)
        {
            fprintf(stderr, "VOLK_GNSSSDR: Error allocating memory (aligned_alloc/_aligned_malloc)\n");
        }
    return ptr;
}

void volk_gnsssdr_free(void *ptr)
{
#if defined(_MSC_VER) || defined(__MINGW32__)
    _aligned_free(ptr);
#else
    free(ptr);
#endif
}
