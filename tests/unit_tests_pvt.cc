/*!
 * \file unit_tests_pvt.cc
 * \brief Unit tests for PVT blocks and libraries (RTKLIB, RINEX/RTCM/NMEA printers, ...).
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

#include "unit-tests/signal-processing-blocks/pvt/bds_tgd_iono_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/galileo_e1_bgd_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/galileo_nav_fallback_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/geohash_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/gps_tgd_isc_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/nmea_printer_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/ntrip_rtcm_client_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/rinex_printer_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/rtcm_printer_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/rtcm_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/rtklib_detslp_dop_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/rtklib_fixed_base_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/rtklib_pvt_ntrip_configuration_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/rtklib_tls_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/rtklib_udpos_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/sbas_rtklib_corrections_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/serdes_monitor_pvt_test.cc"

#if EXTRA_TESTS
// #include "unit-tests/signal-processing-blocks/pvt/rtklib_solver_test.cc"
#endif  // EXTRA_TESTS

#endif  // !UNIT_TESTING_MINIMAL
