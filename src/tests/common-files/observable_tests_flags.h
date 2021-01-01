/*!
 * \file observable_tests_flags.h
 * \brief Helper file for unit testing
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_OBSERVABLE_TESTS_FLAGS_H
#define GNSS_SDR_OBSERVABLE_TESTS_FLAGS_H

#include <gflags/gflags.h>
#include <limits>

DEFINE_double(skip_obs_transitory_s, 30.0, "Skip the initial observable outputs to avoid transitory results [s]");
DEFINE_bool(compute_single_diffs, false, "Compute also the single difference errors for Accumulated Carrier Phase and Carrier Doppler (requires LO synchronization between receivers)");
DEFINE_bool(compare_with_5X, false, "Compare the E5a Doppler and Carrier Phases with the E5 full bw in RINEX (expect discrepancy due to the center frequencies differences");
DEFINE_bool(duplicated_satellites_test, false, "Enable special observable test mode where the scenario contains duplicated satellite orbits");
DEFINE_string(duplicated_satellites_prns, "1,2,3,4", "List of duplicated satellites PRN pairs (i.e. 1,2,3,4 indicates that the PRNs 1,2 share the same orbit. The same applies for PRNs 3,4)");
#endif
