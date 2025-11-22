/*!
 * \file gps_l1_ca_pcps_acquisition_fine_doppler.h
 * \brief Adapts a PCPS acquisition block with fine Doppler estimation to an AcquisitionInterface for
 *  GPS L1 C/A signals
 * \authors <ul>
 *          <li> Javier Arribas, 2013. jarribas(at)cttc.es
 *          </ul> *
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

#ifndef GNSS_SDR_GPS_L1_CA_PCPS_ACQUISITION_FINE_DOPPLER_H
#define GNSS_SDR_GPS_L1_CA_PCPS_ACQUISITION_FINE_DOPPLER_H

#include "base_pcps_acquisition_custom.h"

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_adapters
 * \{ */


/*!
 * \brief This class Adapts a PCPS acquisition block with fine Doppler estimation to an AcquisitionInterface for
 *  GPS L1 C/A signals
 */
class GpsL1CaPcpsAcquisitionFineDoppler : public BasePcpsAcquisitionCustom
{
public:
    GpsL1CaPcpsAcquisitionFineDoppler(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~GpsL1CaPcpsAcquisitionFineDoppler() = default;

    /*!
     * \brief Returns "GPS_L1_CA_PCPS_Acquisition_Fine_Doppler"
     */
    inline std::string implementation() override
    {
        return "GPS_L1_CA_PCPS_Acquisition_Fine_Doppler";
    }

private:
    void code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq) override;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L1_CA_PCPS_ACQUISITION_FINE_DOPPLER_H
