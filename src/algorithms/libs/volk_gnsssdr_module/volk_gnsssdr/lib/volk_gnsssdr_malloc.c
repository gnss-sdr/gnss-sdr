/*
 * Copyright (C) 2010-2019 (see AUTHORS file for a list of contributors)
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "volk_gnsssdr/volk_gnsssdr_malloc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * For #defines used to determine support for allocation functions,
 * see: http://linux.die.net/man/3/aligned_alloc
 */

// Otherwise, test if we are a POSIX or X/Open system
// This only has a restriction that alignment be a power of 2and a
// multiple of sizeof(void *).
#if _POSIX_C_SOURCE >= 200112L || _XOPEN_SOURCE >= 600 || HAVE_POSIX_MEMALIGN

void *volk_gnsssdr_malloc(size_t size, size_t alignment)
{
    void *ptr;

    // quoting posix_memalign() man page:
    // "alignment must be a power of two and a multiple of sizeof(void *)"
    // volk_get_alignment() could return 1 for some machines (e.g. generic_orc)
    if (alignment == 1)
        return malloc(size);

    int err = posix_memalign(&ptr, alignment, size);
    if (err == 0)
        {
            return ptr;
        }
    else
        {
            fprintf(stderr,
                "VOLK_GNSSSDR: Error allocating memory "
                "(posix_memalign: error %d: %s)\n",
                err, strerror(err));
            return NULL;
        }
}

void volk_gnsssdr_free(void *ptr)
{
    free(ptr);
}

// _aligned_malloc has no restriction on size,
// available on Windows since Visual C++ 2005
#elif _MSC_VER >= 1400

void *volk_gnsssdr_malloc(size_t size, size_t alignment)
{
    void *ptr = _aligned_malloc(size, alignment);
    if (ptr == NULL)
        {
            fprintf(stderr, "VOLK_GNSSSDR: Error allocating memory (_aligned_malloc)\n");
        }
    return ptr;
}

void volk_gnsssdr_free(void *ptr)
{
    _aligned_free(ptr);
}

// No standard handlers; we'll do it ourselves.
#else  // _POSIX_C_SOURCE >= 200112L || _XOPEN_SOURCE >= 600 || HAVE_POSIX_MEMALIGN

struct block_info
{
    void *real;
};

void *
volk_gnsssdr_malloc(size_t size, size_t alignment)
{
    void *real, *user;
    struct block_info *info;

    /* At least align to sizeof our struct */
    if (alignment < sizeof(struct block_info))
        alignment = sizeof(struct block_info);

    /* Alloc */
    real = malloc(size + (2 * alignment - 1));

    /* Get pointer to the various zones */
    user = (void *)((((uintptr_t)real) + sizeof(struct block_info) + alignment - 1) & ~(alignment - 1));
    info = (struct block_info *)(((uintptr_t)user) - sizeof(struct block_info));

    /* Store the info for the free */
    info->real = real;

    /* Return pointer to user */
    return user;
}

void volk_gnsssdr_free(void *ptr)
{
    struct block_info *info;

    /* Get the real pointer */
    info = (struct block_info *)(((uintptr_t)ptr) - sizeof(struct block_info));

    /* Release real pointer */
    free(info->real);
}

#endif  // _POSIX_C_SOURCE >= 200112L || _XOPEN_SOURCE >= 600 || HAVE_POSIX_MEMALIGN

//#endif  //_ISOC11_SOURCE
