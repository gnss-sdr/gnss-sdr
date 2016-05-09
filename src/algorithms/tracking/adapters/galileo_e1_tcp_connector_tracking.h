/*!
 * \file galileo_e1_tcp_connector_tracking.h
 * \brief  Interface of an adapter of a TCP connector block based on code DLL + carrier PLL
 * for Galileo E1 to a TrackingInterface
 * \author David Pubill, 2012. dpubill(at)cttc.es
 *         Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *         Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkhauser, 2007
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012-2015  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_GALILEO_E1_TCP_CONNECTOR_TRACKING_H_
#define GNSS_SDR_GALILEO_E1_TCP_CONNECTOR_TRACKING_H_

#include <string>
#include "tracking_interface.h"
#include "galileo_e1_tcp_connector_tracking_cc.h"


class ConfigurationInterface;

/*!
 * \brief This class implements a code DLL + carrier PLL tracking loop
 */
class GalileoE1TcpConnectorTracking : public TrackingInterface
{
public:
    GalileoE1TcpConnectorTracking(ConfigurationInterface* configuration,
            std::string role,
            unsigned int in_streams,
            unsigned int out_streams);

    virtual ~GalileoE1TcpConnectorTracking();

    std::string role()
    {
        return role_;
    }

    //! Returns "Galileo_E1_TCP_CONNECTOR_Tracking"
    std::string implementation()
    {
        return "Galileo_E1_TCP_CONNECTOR_Tracking";
    }
    size_t item_size()
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block);
    void disconnect(gr::top_block_sptr top_block);
    gr::basic_block_sptr get_left_block();
    gr::basic_block_sptr get_right_block();


    /*!
     * \brief Set tracking channel unique ID
     */
    void set_channel(unsigned int channel);

    /*!
     * \brief Set acquisition/tracking common Gnss_Synchro object pointer
     * to efficiently exchange synchronization data between acquisition and
     *  tracking blocks
     */
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro);


    void start_tracking();

private:
    galileo_e1_tcp_connector_tracking_cc_sptr tracking_;
    size_t item_size_;
    unsigned int channel_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif // GNSS_SDR_GALILEO_E1_TCP_CONNECTOR_TRACKING_H_
