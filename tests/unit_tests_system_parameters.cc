/*!
 * \file unit_tests_system_parameters.cc
 * \brief Unit tests for system parameters (navigation messages, FEC, ephemeris, ...).
 * \author Carles Fernandez-Prades, 2026 cfernandez(at)cttc.es
 *
 * This translation unit #includes the test sources of its category so that
 * they are compiled together into the run_tests executable. Splitting the
 * tests into several aggregate units (instead of a single one) keeps the
 * per-unit size, and hence the compiler's memory and time, bounded.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "unit_tests_common.h"

#if !UNIT_TESTING_MINIMAL

#include "unit-tests/system-parameters/beidou_b1c_pvt_helpers_test.cc"
#include "unit-tests/system-parameters/beidou_b2a_pvt_helpers_test.cc"
#include "unit-tests/system-parameters/beidou_bdgim_test.cc"
#include "unit-tests/system-parameters/beidou_cnav1_ldpc_test.cc"
#include "unit-tests/system-parameters/beidou_cnav1_navigation_message_test.cc"
#include "unit-tests/system-parameters/beidou_cnav2_ldpc_test.cc"
#include "unit-tests/system-parameters/beidou_cnav2_navigation_message_test.cc"
#include "unit-tests/system-parameters/beidou_dnav_navigation_message_test.cc"
#include "unit-tests/system-parameters/galileo_e1b_reed_solomon_test.cc"
#include "unit-tests/system-parameters/galileo_e6b_reed_solomon_test.cc"
#include "unit-tests/system-parameters/galileo_ism_test.cc"
#include "unit-tests/system-parameters/glonass_gnav_ephemeris_test.cc"
#include "unit-tests/system-parameters/glonass_gnav_nav_message_test.cc"
#include "unit-tests/system-parameters/gnss_ephemeris_posvel_test.cc"
#include "unit-tests/system-parameters/gps_cnav_navigation_message_test.cc"
#include "unit-tests/system-parameters/has_decoding_test.cc"
#include "unit-tests/system-parameters/qzss_code_generation_test.cc"
#include "unit-tests/system-parameters/qzss_lnav_navigation_message_test.cc"

#endif  // !UNIT_TESTING_MINIMAL
