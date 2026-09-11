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
    // When Doppler is assisted (doppler_num_bins == 1), collapse the search to
    // doppler_narrowing_num_bins candidate bins (plus one reference bin, see
    // below) instead of the full grid. Opt-in (new, still-experimental feature):
    // off by default, enable per-implementation in the .conf (e.g.
    // Acquisition_5X.enable_assisted_doppler_narrowing = true).
    bool enable_assisted_doppler_narrowing{false};
    // How many candidate Doppler bins (spaced doppler_step apart, centered on
    // the assisted Doppler estimate) the narrowed search above actually tests,
    // to absorb residual assist error (receiver dynamics, clock drift
    // uncertainty) instead of requiring the assist to be exact. Must be odd
    // (a center bin plus a symmetric number of +/- steps); 1 (default) is the
    // original assisted-search behavior -- exactly the assisted Doppler,
    // no margin. Only takes effect together with enable_assisted_doppler_narrowing.
    uint32_t doppler_narrowing_num_bins{1U};
    // Target number of correlation sidelobes (in Doppler) the CFAR noise-floor
    // reference bin should clear from the search grid's own candidate span, used
    // only to decide WHETHER a plain full grid needs a dedicated extra reference
    // row instead of reusing an in-grid candidate (see
    // d_full_grid_reference_needs_extra_row in pcps_acquisition.h) -- never to
    // place that row (or narrowed mode's own reference row) beyond the configured
    // doppler_max. doppler_max is the receiver-validated edge of the search/filter
    // passband the rest of the acquisition chain is designed for; searching beyond
    // it to chase a theoretical sidelobe target risks sampling "noise" from a
    // region the decimation/anti-alias response is no longer flat, corrupting the
    // estimate instead of cleaning it up (this cost real satellites at hot start
    // in an earlier version that did push beyond doppler_max -- see
    // update_grid_doppler_wipeoffs()'s narrowed-branch comment). Sidelobe spacing
    // is set by the coherent integration time (~1/sampled_ms), not by doppler_step
    // or the bin count, so at a small enough grid or doppler_max, this target
    // simply won't be met -- accepted, not something to fix by exceeding
    // doppler_max. Only takes effect when use_CFAR_algorithm_flag is set (the
    // non-CFAR peak-ratio statistic never uses a Doppler-domain reference).
    uint32_t reference_bin_min_sidelobes{4U};

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
