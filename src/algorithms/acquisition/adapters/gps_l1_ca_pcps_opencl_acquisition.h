/*!
 * \file gps_l1_ca_pcps_opencl_acquisition.h
 * \brief Adapts an OpenCL PCPS acquisition block to an
 *  AcquisitionInterface for GPS L1 C/A signals
 * \author Marc Molina, 2013. marc.molina.pena(at)gmail.com
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

#ifndef GNSS_SDR_GPS_L1_CA_PCPS_OPENCL_ACQUISITION_H
#define GNSS_SDR_GPS_L1_CA_PCPS_OPENCL_ACQUISITION_H

#include "base_pcps_acquisition_custom.h"

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_adapters
 * \{ */

/*!
 * \brief This class adapts an OpenCL PCPS acquisition block to an
 *  AcquisitionInterface for GPS L1 C/A signals
 */
class GpsL1CaPcpsOpenClAcquisition : public BasePcpsAcquisitionCustom
{
public:
    GpsL1CaPcpsOpenClAcquisition(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~GpsL1CaPcpsOpenClAcquisition() = default;

    /*!
     * \brief Returns "GPS_L1_CA_PCPS_OpenCl_Acquisition"
     */
    inline std::string implementation() override
    {
        return "GPS_L1_CA_PCPS_OpenCl_Acquisition";
    }

    inline bool opencl_ready() const
    {
        return opencl_ready_;
    }

private:
    void code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq) override;

    bool opencl_ready_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L1_CA_PCPS_OPENCL_ACQUISITION_H
