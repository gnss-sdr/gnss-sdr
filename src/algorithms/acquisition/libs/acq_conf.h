/*!
 * \file acq_conf.h
 * \brief Class that contains all the configuration parameters for generic
 * acquisition block based on the PCPS algorithm.
 * \author Carles Fernandez, 2018. cfernandez(at)cttc.es
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

#ifndef GNSS_SDR_ACQ_CONF_H
#define GNSS_SDR_ACQ_CONF_H

#include "configuration_interface.h"
#include <gnuradio/gr_complex.h>
#include <cstdint>
#include <string>

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup acquisition_libs acquisition_libs
 * Library with utilities for GNSS signal acquisition
 * \{ */


class Acq_Conf
{
public:
    Acq_Conf() = default;

    void SetFromConfiguration(const ConfigurationInterface *configuration, const std::string &role, double chip_rate, double opt_freq);

    /* PCPS Acquisition configuration */
    std::string item_type{"gr_complex"};
    std::string dump_filename;

    int64_t fs_in{4000000LL};
    int64_t resampled_fs{0LL};

    size_t it_size{sizeof(gr_complex)};

    float samples_per_ms{0.0};
    float doppler_step2{125.0};
    float threshold{0.0};
    float pfa{0.0};
    float pfa2{0.0};
    float samples_per_code{0.0};
    float resampler_ratio{1.0};

    uint32_t sampled_ms{1U};
    uint32_t ms_per_code{1U};
    uint32_t samples_per_chip{2U};
    uint32_t chips_per_second{1023000U};
    uint32_t max_dwells{1U};
    uint32_t num_doppler_bins_step2{4U};
    uint32_t resampler_latency_samples{0U};
    uint32_t dump_channel{0U};
    int32_t doppler_max{5000};
    int32_t doppler_min{-5000};
    int32_t doppler_step{500};

    bool bit_transition_flag{false};
    bool use_CFAR_algorithm_flag{true};
    bool dump{false};
    bool blocking{true};
    bool blocking_on_standby{false};  // enable it only for unit testing to avoid sample consume on idle status
    bool make_2_steps{false};
    bool use_automatic_resampler{false};
    bool enable_monitor_output{false};
    // When Doppler is assisted (doppler_uncertainty == 0), collapse the search to
    // the known bin + one reference bin instead of the full grid. Off by default;
    // enable per-implementation in the .conf (e.g.
    // Acquisition_5X.enable_doppler_narrowing = true).
    //
    // Applies to any acquisition implementation built on pcps_acquisition (the vast
    // majority of them); FPGA-offloaded acquisitions use a separate implementation
    // that never calls set_doppler_uncertainty(), so this has no effect there.
    //
    // Only takes effect when the caller also passes doppler_uncertainty == 0 to
    // set_doppler_uncertainty() -- in practice, this means
    // GNSS-SDR.assist_dual_frequency_acq must also be enabled and a Doppler
    // projection from the satellite's already-tracked primary frequency must have
    // succeeded (see GNSSFlowgraph::acquisition_manager()). With
    // assist_dual_frequency_acq off, or when no projection is available yet, this
    // flag has no effect and the full configured Doppler grid is always searched.
    bool enable_doppler_narrowing{false};

    // Specific to some implementations
    bool acquire_pilot{false};
    bool acquire_iq{false};
    bool cboc{false};
    bool qmboc{false};
    int zero_padding{0};
    uint32_t folding_factor{0};

    // Not part of the configuration interface
    uint32_t num_codes{0};
    uint32_t code_length{0};
    uint32_t vector_length{0};

private:
    void SetDerivedParams();

    void ConfigureAutomaticResampler(double opt_freq);
};


/** \} */
/** \} */
#endif  // GNSS_SDR_ACQ_CONF_H
