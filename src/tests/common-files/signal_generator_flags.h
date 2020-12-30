/*!
 * \file signal_generator_flags.h
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

#ifndef GNSS_SDR_SIGNAL_GENERATOR_FLAGS_H
#define GNSS_SDR_SIGNAL_GENERATOR_FLAGS_H

#include <gflags/gflags.h>
#include <limits>
#include <string>

DEFINE_bool(disable_generator, false, "Disable the signal generator (a external signal file must be available for the test)");
DEFINE_string(generator_binary, std::string(SW_GENERATOR_BIN), "Path of software-defined signal generator binary");
DEFINE_string(rinex_nav_file, std::string(DEFAULT_RINEX_NAV), "Input RINEX navigation file");
DEFINE_int32(duration, 100, "Duration of the experiment [in seconds, max = 300]");
DEFINE_string(static_position, "30.286502,120.032669,100", "Static receiver position [latitude,longitude,height]");
DEFINE_string(dynamic_position, "", "Observer positions file, in .csv or .nmea format");
DEFINE_string(filename_rinex_obs, "sim.16o", "Filename of output RINEX navigation file");
DEFINE_string(filename_raw_data, "signal_out.bin", "Filename of output raw data file");
DEFINE_int32(fs_gen_sps, 2600000, "Sampling frequency [sps]");
DEFINE_int32(test_satellite_PRN, 1, "PRN of the satellite under test (must be visible during the observation time)");
DEFINE_int32(test_satellite_PRN2, 2, "PRN of the satellite under test (must be visible during the observation time)");
DEFINE_string(test_satellite_PRN_list, "1,2,3,6,9,10,12,17,20,23,28", "List of PRN of the satellites under test (must be visible during the observation time)");
DEFINE_double(CN0_dBHz, std::numeric_limits<double>::infinity(), "Enable noise generator and set the CN0 [dB-Hz]");

#endif
