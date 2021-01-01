/*
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2019 (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifndef GNSS_SDR_VOLK_GNSSSDR_RANK_ARCHS_H
#define GNSS_SDR_VOLK_GNSSSDR_RANK_ARCHS_H

#include <stdbool.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C"
{
#endif

    int volk_gnsssdr_get_index(
        const char *impl_names[],  // list of implementations by name
        const size_t n_impls,      // number of implementations available
        const char *impl_name      // the implementation name to find
    );

    int volk_gnsssdr_rank_archs(
        const char *kern_name,     // name of the kernel to rank
        const char *impl_names[],  // list of implementations by name
        const int *impl_deps,      // requirement mask per implementation
        const bool *alignment,     // alignment status of each implementation
        size_t n_impls,            // number of implementations available
        const bool align           // if false, filter aligned implementations
    );

#ifdef __cplusplus
}
#endif
#endif /* GNSS_SDR_VOLK_GNSSSDR_RANK_ARCHS_H */
