/*!
 * \file unit_tests_acquisition.cc
 * \brief Unit tests for acquisition blocks.
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

#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_8ms_ambiguous_acquisition_gsoc2013_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_ambiguous_acquisition_gsoc2013_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_cccwsr_ambiguous_acquisition_gsoc2013_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_quicksync_ambiguous_acquisition_gsoc2014_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_tong_ambiguous_acquisition_gsoc2013_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e5a_pcps_acquisition_gsoc2014_gensource_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e5b_pcps_acquisition_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e6_pcps_acquisition_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/glonass_l1_ca_pcps_acquisition_gsoc2017_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/gps_l1_ca_pcps_acquisition_gsoc2013_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/gps_l1_ca_pcps_quicksync_acquisition_gsoc2014_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/gps_l1_ca_pcps_tong_acquisition_gsoc2013_test.cc"

#ifndef EXCLUDE_TESTS_REQUIRING_BINARIES
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_ambiguous_acquisition_gsoc_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/pcps_acquisition_doppler_narrowing_test.cc"
#endif

#if OPENCL_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/acquisition/gps_l1_ca_pcps_opencl_acquisition_gsoc2013_test.cc"
#endif

#if CUDA_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/acquisition/cuda_pcps_engine_test.cc"
#endif

#if FPGA_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_ambiguous_acquisition_test_fpga.cc"
#include "unit-tests/signal-processing-blocks/acquisition/gps_l1_ca_pcps_acquisition_test_fpga.cc"
#endif

#if EXTRA_TESTS
#include "unit-tests/signal-processing-blocks/acquisition/beidou_b1c_pcps_acquisition_test.cc"
#ifndef EXCLUDE_TESTS_REQUIRING_BINARIES
#include "unit-tests/signal-processing-blocks/acquisition/glonass_l1_ca_pcps_acquisition_test.cc"
#endif
#endif  // EXTRA_TESTS

#endif  // !UNIT_TESTING_MINIMAL
