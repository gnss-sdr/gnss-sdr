/*!
 * \file signal_generator_flags.h
 * \brief Helper file for unit testing
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_POSITION_TEST_FLAGS_H_
#define GNSS_SDR_POSITION_TEST_FLAGS_H_

#include <gflags/gflags.h>
#include <limits>

DEFINE_string(config_file_ptest, std::string(""), "File containing the configuration parameters for the position test.");
DEFINE_bool(plot_position_test, false, "Plots results of with gnuplot");
DEFINE_bool(static_scenario, true, "Compute figures of merit for static user position (DRMS, CEP, etc..)");
DEFINE_bool(use_pvt_solver_dump, false, "Use PVT solver binary dump or fall back to KML PVT file (contains only position information)");
DEFINE_bool(use_ref_motion_file, false, "Enable or disable the use of a reference file containing the true receiver position, velocity and acceleration.");
DEFINE_int32(ref_motion_file_type, 1, "Type of reference motion file: 1- Spirent CSV motion file");
DEFINE_string(ref_motion_filename, std::string("motion.csv"), "Path and filename for the reference motion file");
DEFINE_string(pvt_solver_dump_filename, std::string("PVT.dat"), "Path and filename for the PVT solver binary dump file");
DEFINE_double(static_2D_error_m, 2.0, "Static scenario 2D (East, North) positioning error threshold [meters]");
DEFINE_double(static_3D_error_m, 5.0, "Static scenario 3D (East, North, Up) positioning error threshold [meters]");
DEFINE_double(accuracy_CEP, 2.0, "Static scenario 2D (East, North) accuracy Circular Error Position (CEP) threshold [meters]");
DEFINE_double(precision_SEP, 10.0, "Static scenario 3D (East, North, Up) precision Spherical Error Position (SEP) threshold [meters]");
DEFINE_double(dynamic_3D_position_RMSE, 10.0, "Dynamic scenario 3D (ECEF) accuracy RMSE threshold [meters]");
DEFINE_double(dynamic_3D_velocity_RMSE, 5.0, "Dynamic scenario 3D (ECEF) accuracy RMSE threshold [meters/second]");
#endif
