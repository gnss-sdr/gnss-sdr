/*!
 * \file unit_tests_common_flags.cc
 * \brief Unit tests that use the shared command-line flags of tests/common-files/
 * \author Carles Fernandez-Prades, 2026 cfernandez(at)cttc.es
 *
 * The headers test_flags.h, signal_generator_flags.h, tracking_tests_flags.h,
 * observable_tests_flags.h and gnuplot_i.h *define* their flags (and, for
 * gnuplot_i.h, static data members) instead of only declaring them, so they
 * can be included from one translation unit only. Every test that includes
 * any of them must therefore be listed here, and not in another aggregate
 * unit_tests_*.cc, otherwise the linker will report duplicate symbols.
 *
 * This translation unit #includes the test sources so that they are compiled
 * together into the run_tests executable. Splitting the tests into several
 * aggregate units (instead of a single one) keeps the per-unit size, and
 * hence the compiler's memory and time, bounded.
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

#if UNIT_TESTING_MINIMAL

#if EXTRA_TESTS
#include "unit-tests/signal-processing-blocks/acquisition/acq_performance_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/tracking_pull-in_test.cc"
#if FPGA_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/tracking/tracking_pull-in_test_fpga.cc"
#endif  // FPGA_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/observables/hybrid_observables_test.cc"
#endif  // EXTRA_TESTS

#else  // UNIT_TESTING_MINIMAL

#include "unit-tests/arithmetic/fft_length_test.cc"

#ifndef EXCLUDE_TESTS_REQUIRING_BINARIES
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_ambiguous_acquisition_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/gps_l1_ca_pcps_acquisition_test.cc"
#endif

#if CUDA_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/acquisition/gps_l1_ca_pcps_acquisition_cuda_test.cc"
#endif

#if FPGA_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/tracking/gps_l1_ca_dll_pll_tracking_test_fpga.cc"
#endif

#if EXTRA_TESTS
#include "unit-tests/signal-processing-blocks/acquisition/acq_performance_test.cc"
// #include "unit-tests/signal-processing-blocks/acquisition/beidou_b1i_pcps_acquisition_test.cc"
// #include "unit-tests/signal-processing-blocks/acquisition/beidou_b3i_pcps_acquisition_test.cc"
#ifndef EXCLUDE_TESTS_REQUIRING_BINARIES
#include "unit-tests/signal-processing-blocks/acquisition/gps_l2_m_pcps_acquisition_test.cc"
#endif
#include "unit-tests/signal-processing-blocks/tracking/gps_l1_ca_dll_pll_tracking_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/gps_l1_ca_gaussian_tracking_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/tracking_pull-in_test.cc"
#if FPGA_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/tracking/tracking_pull-in_test_fpga.cc"
#endif  // FPGA_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/observables/hybrid_observables_test.cc"
#if FPGA_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/observables/hybrid_observables_test_fpga.cc"
#endif  // FPGA_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/telemetry_decoder/gps_l1_ca_telemetry_decoder_test.cc"
#endif  // EXTRA_TESTS

#endif  // UNIT_TESTING_MINIMAL
