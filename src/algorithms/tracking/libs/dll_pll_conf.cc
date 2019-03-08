/*!
 * \file dll_pll_conf.cc
 * \brief Class that contains all the configuration parameters for generic
 * tracking block based on a DLL and a PLL.
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


#include "dll_pll_conf.h"
#include "gnss_sdr_flags.h"
#include "item_type_helpers.h"
#include <glog/logging.h>
#include <cstring>

Dll_Pll_Conf::Dll_Pll_Conf()
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
    cn0_smoother_samples = 200;
    cn0_smoother_alpha = 0.002;
    carrier_lock_test_smoother_alpha = 0.002;
    carrier_lock_test_smoother_samples = 25;
    cn0_min = FLAGS_cn0_min;
    max_carrier_lock_fail = FLAGS_max_carrier_lock_fail;
    max_code_lock_fail = FLAGS_max_lock_fail;
    carrier_lock_th = FLAGS_carrier_lock_th;
    enable_doppler_correction = false;
    item_type = "gr_complex";
    local_replica_item_type = "float";
    track_pilot = false;
    system = 'G';
    signal[0] = '1';
    signal[1] = 'C';
    signal[2] = '\0';
}
    
void Dll_Pll_Conf::SetFromConfiguration( ConfigurationInterface *configuration,
        const std::string &role)
{
    item_type = configuration->property(role + ".item_type", item_type);
    local_replica_item_type = configuration->property(role + ".local_replica_item_type", local_replica_item_type);

    item_type = external_item_type_to_internal( item_type );
    local_replica_item_type = external_item_type_to_internal( local_replica_item_type );
    if( !item_type_valid( item_type ) )
    {
        throw std::invalid_argument( "Unknown item type: " + item_type );
    }

    if( !item_type_valid( local_replica_item_type ) )
    {
        throw std::invalid_argument( "Unknown item type: " + local_replica_item_type );
    }

    int fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", fs_in);
    fs_in = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    high_dyn = configuration->property(role + ".high_dyn", high_dyn);
    dump = configuration->property(role + ".dump", dump);
    dump_filename = configuration->property(role + ".dump_filename", dump_filename);
    dump_mat = configuration->property(role + ".dump_mat", dump_mat);
    pll_bw_hz = configuration->property(role + ".pll_bw_hz", pll_bw_hz);
    pll_bw_narrow_hz = configuration->property(role + ".pll_bw_narrow_hz", pll_bw_narrow_hz);
    dll_bw_narrow_hz = configuration->property(role + ".dll_bw_narrow_hz", dll_bw_narrow_hz);
    dll_bw_hz = configuration->property(role + ".dll_bw_hz", dll_bw_hz);
    dll_filter_order = configuration->property(role + ".dll_filter_order", dll_filter_order);
    pll_filter_order = configuration->property(role + ".pll_filter_order", pll_filter_order);
    if (dll_filter_order < 1)
        {
            LOG(WARNING) << "dll_filter_order parameter must be 1, 2 or 3. Set to 1.";
            dll_filter_order = 1;
        }
    if (dll_filter_order > 3)
        {
            LOG(WARNING) << "dll_filter_order parameter must be 1, 2 or 3. Set to 3.";
            dll_filter_order = 3;
        }
    if (pll_filter_order < 2)
        {
            LOG(WARNING) << "pll_filter_order parameter must be 2 or 3. Set to 2.";
            pll_filter_order = 2;
        }
    if (pll_filter_order > 3)
        {
            LOG(WARNING) << "pll_filter_order parameter must be 2 or 3. Set to 3.";
            pll_filter_order = 3;
        }

    if (pll_filter_order == 2)
        {
            fll_filter_order = 1;
        }
    if (pll_filter_order == 3)
        {
            fll_filter_order = 2;
        }

    enable_fll_pull_in = configuration->property(role + ".enable_fll_pull_in", enable_fll_pull_in);
    enable_fll_steady_state = configuration->property(role + ".enable_fll_steady_state", enable_fll_steady_state);
    fll_bw_hz = configuration->property(role + ".fll_bw_hz", fll_bw_hz);

    early_late_space_chips = configuration->property(role + ".early_late_space_chips", early_late_space_chips);
    early_late_space_narrow_chips = configuration->property(role + ".early_late_space_narrow_chips", early_late_space_narrow_chips);
    very_early_late_space_chips = configuration->property(role + ".very_early_late_space_chips", very_early_late_space_chips);
    very_early_late_space_narrow_chips = configuration->property(role + ".very_early_late_space_narrow_chips", very_early_late_space_narrow_chips);
    extend_correlation_symbols = configuration->property(role + ".extend_correlation_symbols", extend_correlation_symbols);
    track_pilot = configuration->property(role + ".track_pilot", track_pilot);
    cn0_samples = configuration->property(role + ".cn0_samples", cn0_samples);
    cn0_min = configuration->property(role + ".cn0_min", cn0_min);
    max_code_lock_fail = configuration->property(role + ".max_lock_fail", max_code_lock_fail);
    max_carrier_lock_fail = configuration->property(role + ".max_carrier_lock_fail", max_carrier_lock_fail);
    carrier_lock_th = configuration->property(role + ".carrier_lock_th", carrier_lock_th);

    // tracking lock tests smoother parameters
    cn0_smoother_samples = configuration->property(role + ".cn0_smoother_samples", cn0_smoother_samples);
    cn0_smoother_alpha = configuration->property(role + ".cn0_smoother_alpha", cn0_smoother_alpha);
    carrier_lock_test_smoother_samples = configuration->property(role + ".carrier_lock_test_smoother_samples", carrier_lock_test_smoother_samples);
    carrier_lock_test_smoother_alpha = configuration->property(role + ".carrier_lock_test_smoother_alpha", carrier_lock_test_smoother_alpha);

}
