/*!
 * \file gps_l1_ca_kf_tracking.h
 * \brief Interface of an adapter of a DLL + Kalman carrier
 * tracking loop block for GPS L1 C/A signals
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
 * \author Jordi Vila-Valls 2018. jvila(at)cttc.es
 * \author Carles Fernandez-Prades 2018. cfernandez(at)cttc.es
 *
 * Reference:
 * J. Vila-Valls, P. Closas, M. Navarro and C. Fernandez-Prades,
 * "Are PLLs Dead? A Tutorial on Kalman Filter-based Techniques for Digital
 * Carrier Synchronization", IEEE Aerospace and Electronic Systems Magazine,
 * Vol. 32, No. 7, pp. 28–45, July 2017. DOI: 10.1109/MAES.2017.150260
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GPS_L1_CA_KF_TRACKING_H_
#define GNSS_SDR_GPS_L1_CA_KF_TRACKING_H_

#include "gps_l1_ca_kf_tracking_cc.h"
#include "tracking_interface.h"
#include <string>

class ConfigurationInterface;

/*!
 * \brief This class implements a code DLL + carrier PLL tracking loop
 */
class GpsL1CaKfTracking : public TrackingInterface
{
public:
    GpsL1CaKfTracking(ConfigurationInterface* configuration,
        std::string role,
        unsigned int in_streams,
        unsigned int out_streams);

    virtual ~GpsL1CaKfTracking();

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
     * to efficiently exchange synchronization data between acquisition and tracking blocks
     */
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override;

    void start_tracking() override;

    /*!
     * \brief Stop running tracking
     */
    void stop_tracking() override;

private:
    gps_l1_ca_kf_tracking_cc_sptr tracking_;
    size_t item_size_;
    unsigned int channel_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif  // GNSS_SDR_GPS_L1_CA_KF_TRACKING_H_
