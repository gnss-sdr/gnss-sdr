/*!
 * \file Kf_conf.cc
 * \brief Class that contains all the configuration parameters for generic
 * tracking block based on a DLL and a PLL.
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "kf_conf.h"
#include "gnss_sdr_flags.h"
#include "item_type_helpers.h"
#include <glog/logging.h>


Kf_Conf::Kf_Conf() : item_type("gr_complex"),
                     dump_filename("./Kf_dump.dat"),
                     fs_in(2000000.0),
                     carrier_lock_th(FLAGS_carrier_lock_th),
                     code_disc_sd_chips(0.2),
                     carrier_disc_sd_rads(0.3),
                     code_phase_sd_chips(0.15),
                     carrier_phase_sd_rad(0.25),
                     carrier_freq_sd_hz(0.6),
                     carrier_freq_rate_sd_hz_s(0.01),
                     init_code_phase_sd_chips(0.5),
                     init_carrier_phase_sd_rad(0.7),
                     init_carrier_freq_sd_hz(5),
                     init_carrier_freq_rate_sd_hz_s(1),
                     early_late_space_chips(0.25),
                     very_early_late_space_chips(0.5),
                     early_late_space_narrow_chips(0.15),
                     very_early_late_space_narrow_chips(0.5),
                     slope(1.0),
                     spc(0.5),
                     y_intercept(1.0),
                     cn0_smoother_alpha(0.002),
                     carrier_lock_test_smoother_alpha(0.002),
                     pull_in_time_s(10),
                     bit_synchronization_time_limit_s(70),
                     vector_length(0U),
                     smoother_length(10),
                     extend_correlation_symbols(1),
                     cn0_samples(FLAGS_cn0_samples),
                     cn0_smoother_samples(200),
                     carrier_lock_test_smoother_samples(25),
                     cn0_min(FLAGS_cn0_min),
                     max_code_lock_fail(FLAGS_max_lock_fail),
                     max_carrier_lock_fail(FLAGS_max_carrier_lock_fail),
                     system('G'),
                     track_pilot(true),
                     enable_doppler_correction(false),
                     high_dyn(false),
                     dump(false),
                     dump_mat(true)
{
    signal[0] = '1';
    signal[1] = 'C';
    signal[2] = '\0';
}


void Kf_Conf::SetFromConfiguration(const ConfigurationInterface *configuration,
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

    pull_in_time_s = configuration->property(role + ".pull_in_time_s", pull_in_time_s);
    bit_synchronization_time_limit_s = pull_in_time_s + 60;
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
    smoother_length = configuration->property(role + ".smoother_length", smoother_length);
    if (smoother_length < 1)
        {
            smoother_length = 1;
            LOG(WARNING) << "smoother_length must be bigger than 0. It has been set to 1";
        }
    carrier_lock_test_smoother_samples = configuration->property(role + ".carrier_lock_test_smoother_samples", carrier_lock_test_smoother_samples);
    carrier_lock_test_smoother_alpha = configuration->property(role + ".carrier_lock_test_smoother_alpha", carrier_lock_test_smoother_alpha);

    // Kalman filter covariances

    // Measurement covariances (R)
    code_disc_sd_chips = configuration->property(role + ".code_disc_sd_chips", code_disc_sd_chips);
    carrier_disc_sd_rads = configuration->property(role + ".carrier_disc_sd_rads", carrier_disc_sd_rads);

    // System covariances (Q)
    code_phase_sd_chips = configuration->property(role + ".code_phase_sd_chips", code_phase_sd_chips);
    carrier_phase_sd_rad = configuration->property(role + ".carrier_phase_sd_rad", carrier_phase_sd_rad);
    carrier_freq_sd_hz = configuration->property(role + ".carrier_freq_sd_hz", carrier_freq_sd_hz);
    carrier_freq_rate_sd_hz_s = configuration->property(role + ".carrier_freq_rate_sd_hz_s", carrier_freq_rate_sd_hz_s);

    // initial Kalman covariance matrix (P)
    init_code_phase_sd_chips = configuration->property(role + ".init_code_phase_sd_chips", init_code_phase_sd_chips);
    init_carrier_phase_sd_rad = configuration->property(role + ".init_carrier_phase_sd_rad", init_carrier_phase_sd_rad);
    init_carrier_freq_sd_hz = configuration->property(role + ".init_carrier_freq_sd_hz", init_carrier_freq_sd_hz);
    init_carrier_freq_rate_sd_hz_s = configuration->property(role + ".init_carrier_freq_rate_sd_hz_s", init_carrier_freq_rate_sd_hz_s);
}
