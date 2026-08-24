/*!
 * \file rtklib_secure_wipe.h
 * \brief Single declaration of the credential-memory wipe primitive
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * SPDX-FileCopyrightText: 2026 (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_RTKLIB_SECURE_WIPE_H
#define GNSS_SDR_RTKLIB_SECURE_WIPE_H

#include <cstddef>

/** \addtogroup PVT
 * \{ */
/** \addtogroup RTKLIB
 * \{ */

/* Overwrites credential-bearing memory with volatile stores so the wipe is
 * not optimized away (a memset before free is a legal elision target).
 * Single wipe primitive behind every credential scrub in the code base,
 * defined in rtklib_stream.cc. This header carries the one declaration so
 * light-weight consumers (secure_string.h) need not pull the heavy rtklib
 * headers. */
void secure_wipe(void *data, std::size_t size);

/** \} */
/** \} */
#endif  // GNSS_SDR_RTKLIB_SECURE_WIPE_H
