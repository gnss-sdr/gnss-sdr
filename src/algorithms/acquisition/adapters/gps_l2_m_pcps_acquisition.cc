/*!
 * \file gps_l2_m_pcps_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  GPS L2 M signals
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
 *          </ul>
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

#include "gps_l2_m_pcps_acquisition.h"
#include "GPS_L2C.h"
#include "gps_l2c_signal_replica.h"


GpsL2MPcpsAcquisition::GpsL2MPcpsAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BasePcpsAcquisition(configuration,
          role,
          in_streams,
          out_streams,
          GPS_L2_M_CODE_RATE_CPS,
          GPS_L2C_OPT_ACQ_FS_SPS,
          GPS_L2_M_CODE_LENGTH_CHIPS,
          GPS_L2_M_CODE_PERIOD_MS)
{
}


void GpsL2MPcpsAcquisition::code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq)
{
    gps_l2c_m_code_gen_complex_sampled(dest, prn, sampling_freq);
}
