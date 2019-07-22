/*!
 * \file dll_pll_conf_fpga.cc
 * \brief Class that contains all the configuration parameters for generic
 * tracking block based on a DLL and a PLL for the FPGA.
 * \author Marc Majoral, 2019. mmajoral(at)cttc.cat
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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


#include "dll_pll_conf_fpga.h"
#include "gnss_sdr_flags.h"
#include <cstring>

Dll_Pll_Conf_Fpga::Dll_Pll_Conf_Fpga()
{
    /* DLL/PLL tracking configuration */
    high_dyn = false;
    smoother_length = 10;
    fs_in = 0.0;
    vector_length = 0U;
    dump = false;
    dump_mat = true;
    dump_filename = std::string("./dll_pll_dump.dat");
    enable_fll_pull_in = false;
    enable_fll_steady_state = false;
    pull_in_time_s = 10;
    bit_synchronization_time_limit_s = pull_in_time_s + 60;
    fll_filter_order = 1;
    pll_filter_order = 3;
    dll_filter_order = 2;
    fll_bw_hz = 35.0;
    pll_pull_in_bw_hz = 50.0;
    dll_pull_in_bw_hz = 3.0;
    pll_bw_hz = 35.0;
    dll_bw_hz = 2.0;
    pll_bw_narrow_hz = 5.0;
    dll_bw_narrow_hz = 0.75;
    early_late_space_chips = 0.5;
    very_early_late_space_chips = 0.5;
    early_late_space_narrow_chips = 0.1;
    very_early_late_space_narrow_chips = 0.1;
    extend_correlation_symbols = 5;
    cn0_samples = FLAGS_cn0_samples;
    cn0_min = FLAGS_cn0_min;
    max_carrier_lock_fail = FLAGS_max_carrier_lock_fail;
    max_code_lock_fail = FLAGS_max_lock_fail;
    carrier_lock_th = FLAGS_carrier_lock_th;
    //max_lock_fail = 50;
    enable_doppler_correction = false;
    track_pilot = false;
    system = 'G';
    signal[0] = '1';
    signal[1] = 'C';
    signal[2] = '\0';
    device_name = "/dev/uio";
    device_base = 1U;
    code_length_chips = 0U;
    code_samples_per_chip = 0U;
    ca_codes = nullptr;
    data_codes = nullptr;
    extended_correlation_in_fpga = false;
    extend_fpga_integration_periods = 1;
    fpga_integration_period = 0;
}
