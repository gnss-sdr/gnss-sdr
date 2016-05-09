/*!
 * \file galileo_e5a_dll_pll_tracking.h
 * \brief Adapts a code DLL + carrier PLL
 *  tracking block to a TrackingInterface for Galileo E5a signals
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E5a data and pilot Signals
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \based on work from:
 *          <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          </ul>
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_GALILEO_E5A_DLL_PLL_TRACKING_H_
#define GNSS_SDR_GALILEO_E5A_DLL_PLL_TRACKING_H_

#include <string>
#include "tracking_interface.h"
#include "galileo_e5a_dll_pll_tracking_cc.h"


class ConfigurationInterface;

/*!
 * \brief This class implements a code DLL + carrier PLL tracking loop
 */
class GalileoE5aDllPllTracking : public TrackingInterface
{
public:
    GalileoE5aDllPllTracking(ConfigurationInterface* configuration,
            std::string role,
            unsigned int in_streams,
            unsigned int out_streams);

    virtual ~GalileoE5aDllPllTracking();

    std::string role()
    {
        return role_;
    }

    //! Returns "Galileo_E5a_DLL_PLL_Tracking"
    std::string implementation()
    {
        return "Galileo_E5a_DLL_PLL_Tracking";
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
     * to efficiently exchange synchronization data between acquisition and tracking blocks
     */
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro);


    void start_tracking();

private:
    galileo_e5a_dll_pll_tracking_cc_sptr tracking_;
    size_t item_size_;
    unsigned int channel_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif /* GNSS_SDR_GALILEO_E5A_DLL_PLL_TRACKING_H_ */
