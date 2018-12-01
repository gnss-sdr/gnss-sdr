/*!
 * \file glonass_l2_ca_dll_pll_c_aid_tracking.h
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for Glonass L2 C/A to a TrackingInterface
 * \author Damian Miralles, 2018. dmiralles2009(at)gmail.com
 *
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkha user, 2007
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GLONASS_L2_CA_DLL_PLL_C_AID_TRACKING_H_
#define GNSS_SDR_GLONASS_L2_CA_DLL_PLL_C_AID_TRACKING_H_

#include "tracking_interface.h"
#include "glonass_l2_ca_dll_pll_c_aid_tracking_cc.h"
#include "glonass_l2_ca_dll_pll_c_aid_tracking_sc.h"
#include <string>

class ConfigurationInterface;

/*!
 * \brief This class implements a code DLL + carrier PLL tracking loop
 */
class GlonassL2CaDllPllCAidTracking : public TrackingInterface
{
public:
    GlonassL2CaDllPllCAidTracking(ConfigurationInterface* configuration,
        std::string role,
        unsigned int in_streams,
        unsigned int out_streams);

    virtual ~GlonassL2CaDllPllCAidTracking();

    inline std::string role() override
    {
        return role_;
    }

    //! Returns "GLONASS_L2_CA_DLL_PLL_C_Aid_Tracking"
    inline std::string implementation() override
    {
        return "GLONASS_L2_CA_DLL_PLL_C_Aid_Tracking";
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
    glonass_l2_ca_dll_pll_c_aid_tracking_cc_sptr tracking_cc;
    glonass_l2_ca_dll_pll_c_aid_tracking_sc_sptr tracking_sc;
    size_t item_size_;
    std::string item_type_;
    unsigned int channel_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif  // GNSS_SDR_GLONASS_L2_CA_DLL_PLL_C_AID_TRACKING_H_
