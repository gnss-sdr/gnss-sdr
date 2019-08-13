/*!
 * \file galileo_e1_dll_pll_veml_tracking_fpga.h
 * \brief Adapts a DLL+PLL VEML (Very Early Minus Late) tracking loop block
 *  to a TrackingInterface for Galileo E1 signals for the FPGA
 * \author Marc Majoral, 2019. mmajoral(at)cttc.cat
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkhauser, 2007
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_GALILEO_E1_DLL_PLL_VEML_TRACKING_FPGA_H_
#define GNSS_SDR_GALILEO_E1_DLL_PLL_VEML_TRACKING_FPGA_H_

#include "dll_pll_veml_tracking_fpga.h"
#include "tracking_interface.h"
#include <string>


class ConfigurationInterface;

/*!
 * \brief This class Adapts a DLL+PLL VEML (Very Early Minus Late) tracking
 * loop block to a TrackingInterface for Galileo E1 signals
 */
class GalileoE1DllPllVemlTrackingFpga : public TrackingInterface
{
public:
    /*!
     * \brief Constructor
     */
    GalileoE1DllPllVemlTrackingFpga(ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    /*!
     * \brief Destructor
     */
    virtual ~GalileoE1DllPllVemlTrackingFpga();

    /*!
     * \brief Role
     */
    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "Galileo_E1_DLL_PLL_VEML_Tracking_Fpga"
     */
    inline std::string implementation() override
    {
        return "Galileo_E1_DLL_PLL_VEML_Tracking_Fpga";
    }

    /*!
     * \brief Returns size of lv_16sc_t
     */
    size_t item_size() override
    {
        return sizeof(int16_t);
    }

    /*!
     * \brief Connect
     */
    void connect(gr::top_block_sptr top_block) override;

    /*!
     * \brief Disconnect
     */
    void disconnect(gr::top_block_sptr top_block) override;

    /*!
     * \brief Get left block
     */
    gr::basic_block_sptr get_left_block() override;

    /*!
     * \brief Get right block
     */
    gr::basic_block_sptr get_right_block() override;

    /*!
     * \brief Set tracking channel unique ID
     */
    void set_channel(unsigned int channel) override;

    /*!
     * \brief Set acquisition/tracking common Gnss_Synchro object pointer
     * to efficiently exchange synchronization data between acquisition and
     * tracking blocks
     */
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override;

    /*!
     * \brief Start the tracking process in the FPGA
     */
    void start_tracking() override;

    /*!
     * \brief Stop the tracking process in the FPGA
     */
    void stop_tracking() override;

private:
    // the following flags are FPGA-specific and they are using arrange the values of the local code in the way the FPGA
    // expects. This arrangement is done in the initialisation to avoid consuming unnecessary clock cycles during tracking.
    static const int32_t LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY = 0x0C000000;      // flag that enables WE (Write Enable) of the local code FPGA
    static const int32_t LOCAL_CODE_FPGA_CORRELATOR_SELECT_COUNT = 0x20000000;  // flag that selects the writing of the pilot code in the FPGA (as opposed to the data code)

    dll_pll_veml_tracking_fpga_sptr tracking_fpga_sc;
    uint32_t channel_;
    std::string role_;
    uint32_t in_streams_;
    uint32_t out_streams_;
    int32_t* d_ca_codes;
    int32_t* d_data_codes;
    bool d_track_pilot;
};


#endif  // GNSS_SDR_GALILEO_E1_DLL_PLL_VEML_TRACKING_FPGA_H_
