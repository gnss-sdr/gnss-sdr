/*!
 * \file dll_pll_conf.cc
 * \brief Class that contains all the configuration parameters for generic
 * tracking block based on a DLL and a PLL.
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

#include "dll_pll_conf.h"
#include "gnss_sdr_flags.h"
#include "item_type_helpers.h"
#include <glog/logging.h>


Dll_Pll_Conf::Dll_Pll_Conf() : carrier_lock_th(FLAGS_carrier_lock_th),
                               cn0_samples(FLAGS_cn0_samples),
                               cn0_min(FLAGS_cn0_min),
                               max_code_lock_fail(FLAGS_max_lock_fail),
                               max_carrier_lock_fail(FLAGS_max_carrier_lock_fail)
{
    signal[0] = '1';
    signal[1] = 'C';
    signal[2] = '\0';
}


void Dll_Pll_Conf::SetFromConfiguration(const ConfigurationInterface *configuration,
    const std::string &role)
{
    item_type = configuration->property(role + ".item_type", item_type);
    if (!item_type_valid(item_type))
        {
            LOG(WARNING) << "Unknown item type: " + item_type << ". Set to gr_complex";
            item_type = "gr_complex";
        }

    double fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", fs_in);
    fs_in = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    high_dyn = configuration->property(role + ".high_dyn", high_dyn);
    dump = configuration->property(role + ".dump", dump);
    dump_filename = configuration->property(role + ".dump_filename", dump_filename);
    dump_mat = configuration->property(role + ".dump_mat", dump_mat);
    pll_bw_hz = configuration->property(role + ".pll_bw_hz", pll_bw_hz);
    if (FLAGS_pll_bw_hz != 0.0)
        {
            pll_bw_hz = static_cast<float>(FLAGS_pll_bw_hz);
        }
    pll_bw_narrow_hz = configuration->property(role + ".pll_bw_narrow_hz", pll_bw_narrow_hz);
    dll_bw_narrow_hz = configuration->property(role + ".dll_bw_narrow_hz", dll_bw_narrow_hz);
    dll_bw_hz = configuration->property(role + ".dll_bw_hz", dll_bw_hz);
    if (FLAGS_dll_bw_hz != 0.0)
        {
            dll_bw_hz = static_cast<float>(FLAGS_dll_bw_hz);
        }

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
    pull_in_time_s = configuration->property(role + ".pull_in_time_s", pull_in_time_s);
    bit_synchronization_time_limit_s = configuration->property(role + ".bit_synchronization_time_limit_s", bit_synchronization_time_limit_s);
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
    carrier_aiding = configuration->property(role + ".carrier_aiding", carrier_aiding);

    // tracking lock tests smoother parameters
    cn0_smoother_samples = configuration->property(role + ".cn0_smoother_samples", cn0_smoother_samples);
    cn0_smoother_alpha = configuration->property(role + ".cn0_smoother_alpha", cn0_smoother_alpha);
    smoother_length = configuration->property(role + ".smoother_length", smoother_length);
    if (smoother_length < 1)
        {
            smoother_length = 1;
            LOG(WARNING) << "smoother_length must be bigger than 0. It has been set to 1";
        }
    carrier_lock_test_smoother_samples = configuration->property(role + ".carrier_lock_test_smoother_samples", carrier_lock_test_smoother_samples);
    carrier_lock_test_smoother_alpha = configuration->property(role + ".carrier_lock_test_smoother_alpha", carrier_lock_test_smoother_alpha);
}
