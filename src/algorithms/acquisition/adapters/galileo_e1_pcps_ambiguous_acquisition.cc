/*!
 * \file galileo_e1_pcps_ambiguous_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E1 Signals
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
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

#include "galileo_e1_pcps_ambiguous_acquisition.h"
#include "Galileo_E1.h"
#include "galileo_e1_signal_replica.h"


GalileoE1PcpsAmbiguousAcquisition::GalileoE1PcpsAmbiguousAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BasePcpsAcquisition(configuration,
          role,
          in_streams,
          out_streams,
          GALILEO_E1_CODE_CHIP_RATE_CPS,
          GALILEO_E1_OPT_ACQ_FS_SPS,
          GALILEO_E1_B_CODE_LENGTH_CHIPS,
          GALILEO_E1_CODE_PERIOD_MS),
      acquire_pilot_(configuration->property(role + ".acquire_pilot", false)),
      cboc_(configuration->property(role + ".cboc", false))
{
}


void GalileoE1PcpsAmbiguousAcquisition::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    gnss_synchro_ = p_gnss_synchro;
    BasePcpsAcquisition::set_gnss_synchro(p_gnss_synchro);
}


void GalileoE1PcpsAmbiguousAcquisition::code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq)
{
    if (acquire_pilot_ == true)
        {
            // set local signal generator to Galileo E1 pilot component (1C)
            const std::array<char, 3> pilot_signal = {{'1', 'C', '\0'}};
            galileo_e1_code_gen_complex_sampled(dest, pilot_signal, cboc_, prn, sampling_freq, 0, false);
        }
    else
        {
            std::array<char, 3> Signal_{};
            Signal_[0] = gnss_synchro_->Signal[0];
            Signal_[1] = gnss_synchro_->Signal[1];
            Signal_[2] = '\0';
            galileo_e1_code_gen_complex_sampled(dest, Signal_, cboc_, prn, sampling_freq, 0, false);
        }
}
