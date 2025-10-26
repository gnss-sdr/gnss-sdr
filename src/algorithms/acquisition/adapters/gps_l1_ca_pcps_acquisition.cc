/*!
 * \file gps_l1_ca_pcps_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  GPS L1 C/A signals
 * \authors <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          <li> Marc Molina, 2013. marc.molina.pena(at)gmail.com
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

#include "gps_l1_ca_pcps_acquisition.h"
#include "GPS_L1_CA.h"
#include "gps_sdr_signal_replica.h"


GpsL1CaPcpsAcquisition::GpsL1CaPcpsAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : BasePcpsAcquisition(configuration, role, in_streams, out_streams, GPS_L1_CA_CODE_RATE_CPS, GPS_L1_CA_OPT_ACQ_FS_SPS, GPS_L1_CA_CODE_LENGTH_CHIPS, 1)
{
}


void GpsL1CaPcpsAcquisition::code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq)
{
    gps_l1_ca_code_gen_complex_sampled(dest, prn, sampling_freq, 0);
}
