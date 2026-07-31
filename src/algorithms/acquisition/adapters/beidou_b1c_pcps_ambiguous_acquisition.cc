/*!
 * \file beidou_b1c_pcps_ambiguous_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  BeiDou B1C Signals
 * \author GNSS-SDR contributors
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

#include "beidou_b1c_pcps_ambiguous_acquisition.h"
#include "Beidou_B1C.h"
#include "beidou_b1c_signal_replica.h"

BeidouB1cPcpsAmbiguousAcquisition::BeidouB1cPcpsAmbiguousAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BasePcpsAcquisition(configuration,
          role,
          in_streams,
          out_streams,
          BEIDOU_B1C_CODE_RATE_CPS,
          BEIDOU_B1C_OPT_ACQ_FS_SPS,
          BEIDOU_B1C_CODE_LENGTH_CHIPS,
          BEIDOU_B1C_CODE_PERIOD_MS),
      acquire_pilot_(configuration->property(role + ".acquire_pilot", true)),
      qmboc_(configuration->property(role + ".qmboc", true)),
      gnss_synchro_(nullptr)
{
}


void BeidouB1cPcpsAmbiguousAcquisition::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    gnss_synchro_ = p_gnss_synchro;
    BasePcpsAcquisition::set_gnss_synchro(p_gnss_synchro);
}


void BeidouB1cPcpsAmbiguousAcquisition::code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq)
{
    if (acquire_pilot_)
        {
            const std::array<char, 3> pilot_signal = {{'1', 'P', '\0'}};
            beidou_b1c_code_gen_complex_sampled(dest, pilot_signal, qmboc_, prn, sampling_freq, 0, false);
        }
    else
        {
            std::array<char, 3> signal_id{};
            signal_id[0] = gnss_synchro_->Signal[0];
            signal_id[1] = gnss_synchro_->Signal[1];
            signal_id[2] = '\0';
            beidou_b1c_code_gen_complex_sampled(dest, signal_id, qmboc_, prn, sampling_freq, 0, false);
        }
}
