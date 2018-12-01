/*!
 * \file dll_pll_conf.cc
 * \brief Class that contains all the configuration parameters for generic
 * tracking block based on a DLL and a PLL.
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


#include "dll_pll_conf_fpga.h"
#include <cstring>

Dll_Pll_Conf_Fpga::Dll_Pll_Conf_Fpga()
{
    /* DLL/PLL tracking configuration */
    fs_in = 0.0;
    vector_length = 0U;
    dump = false;
    dump_mat = true;
    dump_filename = std::string("./dll_pll_dump.dat");
    pll_bw_hz = 40.0;
    dll_bw_hz = 2.0;
    pll_bw_narrow_hz = 5.0;
    dll_bw_narrow_hz = 0.75;
    early_late_space_chips = 0.5;
    very_early_late_space_chips = 0.5;
    early_late_space_narrow_chips = 0.1;
    very_early_late_space_narrow_chips = 0.1;
    extend_correlation_symbols = 5;
    cn0_samples = 20;
    cn0_min = 25;
    max_lock_fail = 50;
    carrier_lock_th = 0.85;
    track_pilot = false;
    system = 'G';
    char sig_[3] = "1C";
    std::memcpy(signal, sig_, 3);
    device_name = "/dev/uio";
    device_base = 1U;
    multicorr_type = 0U;
    code_length_chips = 0U;
    code_samples_per_chip = 0U;
    ca_codes = nullptr;
    data_codes = nullptr;
}
