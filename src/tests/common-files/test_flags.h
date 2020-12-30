/*!
 * \file test_flags.h
 * \brief Helper file for unit testing
 * \author Carles Fernandez-Prades, 2017. cfernandez(at)cttc.es
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

#ifndef GNSS_SDR_TEST_FLAGS_H
#define GNSS_SDR_TEST_FLAGS_H

#include <gflags/gflags.h>
#include <string>

#if defined GNUPLOT_EXECUTABLE
DEFINE_string(gnuplot_executable, std::string(GNUPLOT_EXECUTABLE), "Gnuplot binary path");
#elif !defined GNUPLOT_EXECUTABLE
DEFINE_string(gnuplot_executable, "", "Gnuplot binary path");
#endif

DEFINE_bool(plot_acq_grid, false, "Plots acquisition grid with gnuplot");
DEFINE_int32(plot_decimate, 1, "Decimate plots");

#endif
