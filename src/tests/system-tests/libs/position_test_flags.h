/*!
 * \file position_test_flags.h
 * \brief Helper file for unit testing
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_POSITION_TEST_FLAGS_H
#define GNSS_SDR_POSITION_TEST_FLAGS_H

#include <gflags/gflags.h>
#include <limits>
#include <string>

DEFINE_string(config_file_ptest, std::string(""), "File containing the configuration parameters for the position test.");
DEFINE_bool(plot_position_test, false, "Plots results of with gnuplot");
DEFINE_bool(static_scenario, true, "Compute figures of merit for static user position (DRMS, CEP, etc..)");
DEFINE_bool(use_ref_motion_file, false, "Enable or disable the use of a reference file containing the true receiver position, velocity and acceleration.");
DEFINE_int32(ref_motion_file_type, 1, "Type of reference motion file: 1- Spirent CSV motion file");
DEFINE_string(ref_motion_filename, std::string("motion.csv"), "Path and filename for the reference motion file");
DEFINE_string(pvt_solver_dump_filename, std::string("PVT.dat"), "Path and filename for the PVT solver binary dump file");
DEFINE_double(static_2D_error_m, 2.0, "Static scenario 2D (East, North) positioning error threshold [meters]");
DEFINE_double(static_3D_error_m, 5.0, "Static scenario 3D (East, North, Up) positioning error threshold [meters]");
DEFINE_double(accuracy_CEP, 3.0, "Static scenario 2D (East, North) accuracy Circular Error Position (CEP) threshold [meters]");
DEFINE_double(precision_SEP, 15.0, "Static scenario 3D (East, North, Up) precision Spherical Error Position (SEP) threshold [meters]");
DEFINE_double(dynamic_3D_position_RMSE, 10.0, "Dynamic scenario 3D (ECEF) accuracy RMSE threshold [meters]");
DEFINE_double(dynamic_3D_velocity_RMSE, 5.0, "Dynamic scenario 3D (ECEF) velocity accuracy RMSE threshold [meters/second]");
DEFINE_bool(enable_carrier_smoothing, false, "Activates carrier smoothing of pseudoranges");

#endif
