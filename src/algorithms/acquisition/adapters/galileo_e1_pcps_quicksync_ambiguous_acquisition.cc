/*!
 * \file galileo_e1_pcps_quicksync_ambiguous_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E1 Signals using the QuickSync Algorithm
 * \author Damian Miralles, 2014. dmiralles2009@gmail.com
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

#include "galileo_e1_pcps_quicksync_ambiguous_acquisition.h"
#include "Galileo_E1.h"
#include "configuration_interface.h"
#include "galileo_e1_signal_replica.h"
#include "pcps_quicksync_acquisition_cc.h"
#include <boost/math/distributions/exponential.hpp>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

namespace
{
uint32_t get_folding_factor(const ConfigurationInterface* configuration, const std::string& role)
{
    /*  Calculate the folding factor value based on the formula described in the paper.
        This may be a bug, but acquisition also work by variying the folding factor at va-
        lues different that the expressed in the paper. In addition, it is important to point
        out that by making the folding factor smaller we were able to get QuickSync work with
        Galileo. Future work should be directed to test this assumption statistically. */

    // return static_cast<unsigned int>(ceil(sqrt(log2(code_length_))));
    return configuration->property(role + ".folding_factor", 2);
}
}  // namespace

GalileoE1PcpsQuickSyncAmbiguousAcquisition::GalileoE1PcpsQuickSyncAmbiguousAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BasePcpsAcquisitionCustom(
          configuration,
          role,
          in_streams,
          out_streams,
          GALILEO_E1_CODE_CHIP_RATE_CPS,
          GALILEO_E1_B_CODE_LENGTH_CHIPS,
          GALILEO_E1_CODE_PERIOD_MS * get_folding_factor(configuration, role),
          true,
          true),
      folding_factor_(get_folding_factor(configuration, role)),
      cboc_(configuration->property(role + ".cboc", false))
{
    if (is_type_gr_complex())
        {
            // const auto samples_per_ms = static_cast<int>(round(code_length_ / acq_parameters_.sampled_ms));
            const unsigned int max_dwells = acq_parameters_.bit_transition_flag ? 2 : acq_parameters_.max_dwells;

            acquisition_cc_ = pcps_quicksync_make_acquisition_cc(acq_parameters_, folding_factor_, vector_length_, max_dwells, code_length_);

            DLOG(INFO) << "acquisition_quicksync(" << acquisition_cc_->unique_id() << ")";
        }
}


void GalileoE1PcpsQuickSyncAmbiguousAcquisition::code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq)
{
    std::array<char, 3> Signal_{};
    Signal_[0] = gnss_synchro_->Signal[0];
    Signal_[1] = gnss_synchro_->Signal[1];
    Signal_[2] = '\0';

    galileo_e1_code_gen_complex_sampled(dest, Signal_, cboc_, prn, sampling_freq, 0, false);
}


float GalileoE1PcpsQuickSyncAmbiguousAcquisition::calculate_threshold(float pfa) const
{
    unsigned int frequency_bins = 0;
    for (int doppler = -acq_parameters_.doppler_max; doppler <= acq_parameters_.doppler_max; doppler += acq_parameters_.doppler_step)
        {
            frequency_bins++;
        }

    DLOG(INFO) << "Channel " << channel_ << "  Pfa = " << pfa;

    unsigned int ncells = code_length_ / folding_factor_ * frequency_bins;
    double exponent = 1.0 / static_cast<double>(ncells);
    double val = pow(1.0 - pfa, exponent);
    double lambda = static_cast<double>(code_length_) / static_cast<double>(folding_factor_);
    boost::math::exponential_distribution<double> mydist(lambda);
    auto threshold = static_cast<float>(quantile(mydist, val));

    return threshold;
}
