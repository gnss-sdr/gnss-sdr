/*!
 * \file gps_l1_ca_kf_tracking.h
 * \brief  Interface of an adapter of a code + carrier Kalman Filter tracking
 * loop with VTL capabilities block
 * for GPS L1 C/A to a TrackingInterface
 * \author  Javier Arribas, 2020. jarribas(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GPS_L1_CA_KF_TRACKING_H
#define GNSS_SDR_GPS_L1_CA_KF_TRACKING_H

#include "kf_tracking.h"
#include "tracking_interface.h"
#include <string>

class ConfigurationInterface;

/*!
 * \brief This class implements a code + carrier Kalman Filter tracking loop
 * with VTL capabilities
 */
class GpsL1CaKfTracking : public TrackingInterface
{
public:
    GpsL1CaKfTracking(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~GpsL1CaKfTracking() = default;

    inline std::string role() override
    {
        return role_;
    }

    //! Returns "GPS_L1_CA_KF_Tracking"
    inline std::string implementation() override
    {
        return "GPS_L1_CA_KF_Tracking";
    }

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    /*!
     * \brief Set tracking channel unique ID
     */
    void set_channel(unsigned int channel) override;

    /*!
     * \brief Set acquisition/tracking common Gnss_Synchro object pointer
     * to efficiently exchange synchronization data between acquisition
     * and tracking blocks
     */
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override;

    void start_tracking() override;

    /*!
     * \brief Stop running tracking
     */
    void stop_tracking() override;

private:
    kf_tracking_sptr tracking_sptr_;
    std::string role_;
    size_t item_size_;
    unsigned int channel_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif  // GNSS_SDR_GPS_L1_CA_KF_TRACKING_H
