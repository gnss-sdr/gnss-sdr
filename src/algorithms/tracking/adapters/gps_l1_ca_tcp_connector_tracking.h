/*!
 * \file gps_l1_ca_tcp_connector_tracking.h
 * \brief  Interface of an adapter of a TCP connector block based on code DLL + carrier PLL
 * for GPS L1 C/A to a TrackingInterface
 * \author David Pubill, 2012. dpubill(at)cttc.es
 *         Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkhauser, 2007
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

#ifndef GNSS_SDR_GPS_L1_CA_TCP_CONNECTOR_TRACKING_H
#define GNSS_SDR_GPS_L1_CA_TCP_CONNECTOR_TRACKING_H

#include "gps_l1_ca_tcp_connector_tracking_cc.h"
#include "tracking_interface.h"
#include <string>

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class implements a code DLL + carrier PLL tracking loop
 */
class GpsL1CaTcpConnectorTracking : public TrackingInterface
{
public:
    GpsL1CaTcpConnectorTracking(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~GpsL1CaTcpConnectorTracking() = default;

    inline std::string role() override
    {
        return role_;
    }

    //! Returns "GPS_L1_CA_TCP_CONNECTOR_Tracking"
    inline std::string implementation() override
    {
        return "GPS_L1_CA_TCP_CONNECTOR_Tracking";
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
    gps_l1_ca_tcp_connector_tracking_cc_sptr tracking_sptr_;
    std::string role_;
    size_t item_size_;
    unsigned int channel_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L1_CA_TCP_CONNECTOR_TRACKING_H
