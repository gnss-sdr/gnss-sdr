/*!
 * \file gps_l1_ca_pcps_quicksync_acquisition.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for GPS L1 C/A signals implementing the QuickSync Algorithm.
 * \date June, 2014
 * \author Damian Miralles Sanchez. dmiralles2009@gmail.com
 *
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

#ifndef GNSS_SDR_GPS_L1_CA_PCPS_QUICKSYNC_ACQUISITION_H
#define GNSS_SDR_GPS_L1_CA_PCPS_QUICKSYNC_ACQUISITION_H

#include "base_pcps_acquisition_custom.h"

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_adapters
 * \{ */

/*!
 * \brief This class adapts a PCPS acquisition block to an AcquisitionInterface
 *  for GPS L1 C/A signals
 */
class GpsL1CaPcpsQuickSyncAcquisition : public BasePcpsAcquisitionCustom
{
public:
    GpsL1CaPcpsQuickSyncAcquisition(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~GpsL1CaPcpsQuickSyncAcquisition() = default;

    /*!
     * \brief Returns "GPS_L1_CA_PCPS_QuickSync_Acquisition"
     */
    inline std::string implementation() override
    {
        return "GPS_L1_CA_PCPS_QuickSync_Acquisition";
    }

private:
    float calculate_threshold(float pfa) const override;
    void code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq) override;

    const unsigned int folding_factor_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L1_CA_PCPS_QUICKSYNC_ACQUISITION_H
