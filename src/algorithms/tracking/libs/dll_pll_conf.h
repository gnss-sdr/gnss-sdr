/*!
 * \file dll_pll_conf.h
 * \brief Class that contains all the configuration parameters for generic tracking block based on a DLL and a PLL.
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
 *
 * Class that contains all the configuration parameters for generic tracking block based on a DLL and a PLL.
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

#ifndef GNSS_SDR_DLL_PLL_CONF_H_
#define GNSS_SDR_DLL_PLL_CONF_H_

#include <cstdint>
#include <string>

class Dll_Pll_Conf
{
public:
    /* DLL/PLL tracking configuration */

    int fll_filter_order;
    bool enable_fll_pull_in;
    bool enable_fll_steady_state;
    unsigned int pull_in_time_s;
    unsigned int bit_synchronization_time_limit_s;
    int pll_filter_order;
    int dll_filter_order;

    double fs_in;
    uint32_t vector_length;
    bool dump;
    bool dump_mat;
    std::string dump_filename;
    float pll_pull_in_bw_hz;
    float dll_pull_in_bw_hz;
    float fll_bw_hz;
    float pll_bw_hz;
    float dll_bw_hz;
    float pll_bw_narrow_hz;
    float dll_bw_narrow_hz;
    float early_late_space_chips;
    float very_early_late_space_chips;
    float early_late_space_narrow_chips;
    float very_early_late_space_narrow_chips;
    int32_t extend_correlation_symbols;
    bool high_dyn;
    int32_t cn0_samples;
    int32_t carrier_lock_det_mav_samples;
    int32_t cn0_min;
    int32_t max_code_lock_fail;
    int32_t max_carrier_lock_fail;
    uint32_t smoother_length;
    double carrier_lock_th;
    bool track_pilot;
    bool enable_doppler_correction;
    char system;
    char signal[3]{};

    Dll_Pll_Conf();
};

#endif
