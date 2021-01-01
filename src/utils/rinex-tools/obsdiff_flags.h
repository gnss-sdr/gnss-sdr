/*!
 * \file obsdiff_flags.h
 * \brief Helper file for unit testing
 * \author Javier Arribas, 2020. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_OBSDIFF_FLAGS_H
#define GNSS_SDR_OBSDIFF_FLAGS_H

#include <gflags/gflags.h>

DEFINE_double(skip_obs_transitory_s, 30.0, "Skip the initial observable outputs to avoid transitory results [s]");
DEFINE_double(skip_obs_ends_s, 5.0, "Skip the lasts observable outputs to avoid transitory results [s]");
DEFINE_bool(single_diffs, false, "Compute also the single difference errors for Accumulated Carrier Phase and Carrier Doppler (requires LO synchronization between receivers)");
DEFINE_bool(compare_with_5X, false, "Compare the E5a Doppler and Carrier Phases with the E5 full bw in RINEX (expect discrepancy due to the center frequencies difference)");
DEFINE_bool(dupli_sat, false, "Enable special observable test mode where the scenario contains duplicated satellite orbits");
DEFINE_bool(single_diff, false, "Enable special observable test mode using only rover observables");
DEFINE_string(dupli_sat_prns, "1,2,3,4", "List of duplicated satellites PRN pairs (i.e. 1,2,3,4 indicates that the PRNs 1,2 share the same orbit. The same applies for PRNs 3,4)");
DEFINE_string(base_rinex_obs, "base.obs", "Filename of reference RINEX observation file");
DEFINE_string(rinex_nav, "base.nav", "Filename of reference RINEX navigation file");
DEFINE_string(rover_rinex_obs, "base.obs", "Filename of test RINEX observation file");
DEFINE_string(system, "G", "GNSS satellite system: G for GPS, E for Galileo");
DEFINE_string(signal, "1C", "GNSS signal: 1C for GPS L1 CA, 1B for Galileo E1");
DEFINE_bool(remove_rx_clock_error, false, "Compute and remove the receivers clock error prior to compute observable differences (requires a valid RINEX nav file for both receivers)");

#endif
