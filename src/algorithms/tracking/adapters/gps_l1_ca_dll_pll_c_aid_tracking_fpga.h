/*!
 * \file gps_l1_ca_dll_pll_c_aid_tracking_fpga.h
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for GPS L1 C/A to a TrackingInterface
 * \author Marc Majoral, 2017. mmajoral(at)cttc.cat
 *         Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkhauser, 2007
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_GPS_L1_CA_DLL_PLL_C_AID_TRACKING_FPGA__H_
#define GNSS_SDR_GPS_L1_CA_DLL_PLL_C_AID_TRACKING_FPGA__H_

#include <string>
#include "tracking_interface.h"
#include "gps_l1_ca_dll_pll_c_aid_tracking_fpga_sc.h"

class ConfigurationInterface;

/*!
 * \brief This class implements a code DLL + carrier PLL tracking loop
 */
class GpsL1CaDllPllCAidTrackingFpga : public TrackingInterface
{
public:
    GpsL1CaDllPllCAidTrackingFpga(ConfigurationInterface* configuration,
            std::string role, unsigned int in_streams,
            unsigned int out_streams);

    virtual ~GpsL1CaDllPllCAidTrackingFpga();

    inline std::string role() override
    {
        return role_;
    }

    //! Returns "GPS_L1_CA_DLL_PLL_C_Aid_Tracking_Fpga"
    inline std::string implementation() override
    {
        return "GPS_L1_CA_DLL_PLL_C_Aid_Tracking_Fpga";
    }

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    // CONVERT TO SOURCE
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

    void reset(void);

private:
    gps_l1_ca_dll_pll_c_aid_tracking_fpga_sc_sptr tracking_fpga_sc;
    size_t item_size_;
    std::string item_type_;
    unsigned int channel_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif // GNSS_SDR_GPS_L1_CA_DLL_PLL_C_AID_TRACKING_FPGA__H_
