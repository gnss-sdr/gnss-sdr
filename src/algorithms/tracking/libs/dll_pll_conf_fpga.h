/*!
 * \file dll_pll_conf.h
 * \brief Class that contains all the configuration parameters for generic tracking block based on a DLL and a PLL.
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
 *
 * Class that contains all the configuration parameters for generic tracking block based on a DLL and a PLL.
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

#ifndef GNSS_SDR_DLL_PLL_CONF_FPGA_H_
#define GNSS_SDR_DLL_PLL_CONF_FPGA_H_

#include <cstdint>
#include <string>

class Dll_Pll_Conf_Fpga
{
public:
    /* DLL/PLL tracking configuration */
    double fs_in;
    uint32_t vector_length;
    bool dump;
    bool dump_mat;
    std::string dump_filename;
    float pll_bw_hz;
    float dll_bw_hz;
    float pll_bw_narrow_hz;
    float dll_bw_narrow_hz;
    float early_late_space_chips;
    float very_early_late_space_chips;
    float early_late_space_narrow_chips;
    float very_early_late_space_narrow_chips;
    int32_t extend_correlation_symbols;
    int32_t cn0_samples;
    int32_t cn0_min;
    int32_t max_lock_fail;
    double carrier_lock_th;
    bool track_pilot;
    char system;
    char signal[3];
    std::string device_name;
    uint32_t device_base;
    uint32_t multicorr_type;
    uint32_t code_length_chips;
    uint32_t code_samples_per_chip;
    int32_t* ca_codes;
    int32_t* data_codes;

    Dll_Pll_Conf_Fpga();
};

#endif
