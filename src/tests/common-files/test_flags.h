/*!
 * \file test_flags.h
 * \brief Helper file for unit testing
 * \author Carles Fernandez-Prades, 2017. cfernandez(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_TEST_FLAGS_H_
#define GNSS_SDR_TEST_FLAGS_H_

#include <gflags/gflags.h>

#if defined GNUPLOT_EXECUTABLE
DEFINE_string(gnuplot_executable, std::string(GNUPLOT_EXECUTABLE), "Gnuplot binary path");
#elif !defined GNUPLOT_EXECUTABLE
DEFINE_string(gnuplot_executable, "", "Gnuplot binary path");
#endif

DEFINE_bool(plot_acq_grid, false, "Plots acquisition grid with gnuplot");
DEFINE_int32(plot_decimate, 1, "Decimate plots");

#endif
