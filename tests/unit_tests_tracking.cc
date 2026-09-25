/*!
 * \file unit_tests_tracking.cc
 * \brief Unit tests for tracking blocks.
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

#include "unit-tests/signal-processing-blocks/tracking/bit_synchronizer_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/cpu_multicorrelator_real_codes_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/cpu_multicorrelator_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/discriminator_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/frequency_error_reduction_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/galileo_e5a_tracking_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/galileo_e5b_dll_pll_tracking_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/tracking_loop_filter_test.cc"

#ifndef EXCLUDE_TESTS_REQUIRING_BINARIES
#include "unit-tests/signal-processing-blocks/tracking/galileo_e1_dll_pll_veml_tracking_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/glonass_l1_ca_dll_pll_tracking_test.cc"
#endif

#include "unit-tests/signal-processing-blocks/tracking/bayesian_estimation_test.cc"
#if ARMADILLO_HAVE_MVNRND
#include "unit-tests/signal-processing-blocks/tracking/cubature_filter_test.cc"
// #include "unit-tests/signal-processing-blocks/tracking/unscented_filter_test.cc"
#endif

#if CUDA_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/tracking/gpu_multicorrelator_test.cc"
#endif

#if EXTRA_TESTS
#include "unit-tests/signal-processing-blocks/tracking/beidou_b1c_dll_pll_veml_tracking_test.cc"
#ifndef EXCLUDE_TESTS_REQUIRING_BINARIES
#include "unit-tests/signal-processing-blocks/tracking/gps_l2_m_dll_pll_tracking_test.cc"
#endif
#endif  // EXTRA_TESTS

#endif  // !UNIT_TESTING_MINIMAL
