/*!
 * \file tracking_tests_flags.h
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

#ifndef GNSS_SDR_TRACKING_TESTS_FLAGS_H_
#define GNSS_SDR_TRACKING_TESTS_FLAGS_H_

#include <gflags/gflags.h>
#include <limits>


DEFINE_string(trk_test_implementation, std::string("GPS_L1_CA_DLL_PLL_Tracking"), "Tracking block implementation under test, defaults to GPS_L1_CA_DLL_PLL_Tracking");
// Input signal configuration
DEFINE_bool(enable_external_signal_file, false, "Use an external signal file capture instead of the software-defined signal generator");
DEFINE_double(external_signal_acquisition_threshold, 2.5, "Threshold for satellite acquisition when external file is used");
DEFINE_int32(external_signal_acquisition_dwells, 5, "Maximum dwells count for satellite acquisition when external file is used");
DEFINE_double(external_signal_acquisition_doppler_max_hz, 5000.0, "Doppler max for satellite acquisition when external file is used");
DEFINE_double(external_signal_acquisition_doppler_step_hz, 125, "Doppler step for satellite acquisition when external file is used");

DEFINE_string(signal_file, std::string("signal_out.bin"), "Path of the external signal capture file");
DEFINE_double(CN0_dBHz_start, std::numeric_limits<double>::infinity(), "Enable noise generator and set the CN0 start sweep value [dB-Hz]");
DEFINE_double(CN0_dBHz_stop, std::numeric_limits<double>::infinity(), "Enable noise generator and set the CN0 stop sweep value [dB-Hz]");
DEFINE_double(CN0_dB_step, 3.0, "Noise generator CN0 sweep step value [dB]");

DEFINE_double(PLL_bw_hz_start, 20.0, "PLL Wide configuration start sweep value [Hz]");
DEFINE_double(PLL_bw_hz_stop, 20.0, "PLL Wide configuration stop sweep value [Hz]");
DEFINE_double(PLL_bw_hz_step, 5.0, "PLL Wide configuration sweep step value [Hz]");

DEFINE_double(DLL_bw_hz_start, 1.0, "DLL Wide configuration start sweep value [Hz]");
DEFINE_double(DLL_bw_hz_stop, 1.0, "DLL Wide configuration stop sweep value [Hz]");
DEFINE_double(DLL_bw_hz_step, 0.25, "DLL Wide configuration sweep step value [Hz]");

DEFINE_double(PLL_narrow_bw_hz, 5.0, "PLL Narrow configuration value [Hz]");
DEFINE_double(DLL_narrow_bw_hz, 0.75, "DLL Narrow configuration value [Hz]");

DEFINE_double(acq_Doppler_error_hz_start, 1000.0, "Acquisition Doppler error start sweep value [Hz]");
DEFINE_double(acq_Doppler_error_hz_stop, -1000.0, "Acquisition Doppler error stop sweep value [Hz]");
DEFINE_double(acq_Doppler_error_hz_step, -50.0, "Acquisition Doppler error sweep step value [Hz]");

DEFINE_double(acq_Delay_error_chips_start, 2.0, "Acquisition Code Delay error start sweep value [Chips]");
DEFINE_double(acq_Delay_error_chips_stop, -2.0, "Acquisition Code Delay error stop sweep value [Chips]");
DEFINE_double(acq_Delay_error_chips_step, -0.1, "Acquisition Code Delay error sweep step value [Chips]");

DEFINE_int64(skip_samples, 0, "Skip an initial transitory in the processed signal file capture [samples]");

DEFINE_int32(plot_detail_level, 0, "Specify the desired plot detail (0,1,2): 0 - Minimum plots (default) 2 - Plot all tracking parameters");

DEFINE_double(skip_trk_transitory_s, 1.0, "Skip the initial tracking output signal to avoid transitory results [s]");

//Emulated acquisition configuration

//Tracking configuration
DEFINE_int32(extend_correlation_symbols, 1, "Set the tracking coherent correlation to N symbols (up to 20 for GPS L1 C/A)");

//Test output configuration
DEFINE_bool(plot_gps_l1_tracking_test, false, "Plots results of GpsL1CADllPllTrackingTest with gnuplot");


#endif
