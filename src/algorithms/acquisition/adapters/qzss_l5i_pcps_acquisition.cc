/*!
 * \file qzss_l5i_pcps_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  QZSS L5I signals
 * \authors <ul>
 *          <li> Carles Fernandez, 2026. cfernandez(at)cttc.es
 *          </ul>
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "qzss_l5i_pcps_acquisition.h"
#include "qzss.h"
#include "qzss_signal_replica.h"


QzssL5iPcpsAcquisition::QzssL5iPcpsAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BasePcpsAcquisition(configuration,
          role,
          in_streams,
          out_streams,
          QZSS_L5_CHIP_RATE,
          QZSS_L5_OPT_ACQ_FS_SPS,
          QZSS_L5_CODE_LENGTH,
          QZSS_L5I_PERIOD_MS)
{
}


void QzssL5iPcpsAcquisition::code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq)
{
    qzss_l5i_code_gen_complex_sampled(dest, prn, sampling_freq);
}