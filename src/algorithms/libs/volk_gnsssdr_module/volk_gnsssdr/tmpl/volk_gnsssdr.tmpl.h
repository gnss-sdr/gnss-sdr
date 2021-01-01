/*
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2019 (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifndef INCLUDED_VOLK_GNSSSDR_RUNTIME
#define INCLUDED_VOLK_GNSSSDR_RUNTIME

#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <volk_gnsssdr/volk_gnsssdr_config_fixed.h>
#include <volk_gnsssdr/volk_gnsssdr_malloc.h>
#include <volk_gnsssdr/volk_gnsssdr_typedefs.h>
#include <stdbool.h>
#include <stdlib.h>

__VOLK_DECL_BEGIN

typedef struct volk_gnsssdr_func_desc
{
    const char **impl_names;
    const int *impl_deps;
    const bool *impl_alignment;
    size_t n_impls;
} volk_gnsssdr_func_desc_t;

//! Prints a list of machines available
VOLK_API void volk_gnsssdr_list_machines(void);

//! Returns the name of the machine this instance will use
VOLK_API const char *volk_gnsssdr_get_machine(void);

//! Get the machine alignment in bytes
VOLK_API size_t volk_gnsssdr_get_alignment(void);

/*!
 * The VOLK_OR_PTR macro is a convenience macro
 * for checking the alignment of a set of pointers.
 * Example usage:
 * volk_gnsssdr_is_aligned(VOLK_OR_PTR((VOLK_OR_PTR(p0, p1), p2)))
 */
#define VOLK_OR_PTR(ptr0, ptr1) \
    (const void *)(((intptr_t)(ptr0)) | ((intptr_t)(ptr1)))

/*!
 * Is the pointer on a machine alignment boundary?
 *
 * Note: for performance reasons, this function
 * is not usable until another volk_gnsssdr API call is made
 * which will perform certain initialization tasks.
 *
 * \param ptr the pointer to some memory buffer
 * \return 1 for alignment boundary, else 0
 */
VOLK_API bool volk_gnsssdr_is_aligned(const void *ptr);

// clang-format off
%for kern in kernels:

//! A function pointer to the dispatcher implementation
extern VOLK_API ${kern.pname} ${kern.name};

//! A function pointer to the fastest aligned implementation
extern VOLK_API ${kern.pname} ${kern.name}_a;

//! A function pointer to the fastest unaligned implementation
extern VOLK_API ${kern.pname} ${kern.name}_u;

//! Call into a specific implementation given by name
extern VOLK_API void ${kern.name}_manual(${kern.arglist_full}, const char* impl_name);

//! Get description parameters for this kernel
extern VOLK_API volk_gnsssdr_func_desc_t ${kern.name}_get_func_desc(void);
%endfor
__VOLK_DECL_END
// clang-format on

#endif /* INCLUDED_VOLK_GNSSSDR_RUNTIME */
