/*!
 * \file unit_tests_osnma.cc
 * \brief Unit tests for Galileo OSNMA.
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

#include "unit-tests/signal-processing-blocks/osnma/gnss_crypto_test.cc"
#include "unit-tests/signal-processing-blocks/osnma/osnma_msg_receiver_test.cc"

#if EXTRA_TESTS
#ifndef EXCLUDE_TESTS_REQUIRING_BINARIES
#include "unit-tests/signal-processing-blocks/osnma/osnma_test_vectors.cc"
#endif
#endif  // EXTRA_TESTS

#endif  // !UNIT_TESTING_MINIMAL
