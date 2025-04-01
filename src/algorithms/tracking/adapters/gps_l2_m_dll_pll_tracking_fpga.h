/*!
 * \file gps_l2_m_dll_pll_tracking_fpga.h
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for GPS L2C to a TrackingInterface for the FPGA
 * \author Marc Majoral, 2019, mmajoral(at)cttc.es
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

#ifndef GNSS_SDR_GPS_L2_M_DLL_PLL_TRACKING_FPGA_H
#define GNSS_SDR_GPS_L2_M_DLL_PLL_TRACKING_FPGA_H

#include "dll_pll_veml_tracking_fpga.h"
#include "tracking_interface.h"
#include <gnuradio/runtime_types.h>
#include <cstddef>
#include <string>

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_adapters
 * \{ */


class Gnss_Synchro;
class ConfigurationInterface;

/*!
 * \brief This class implements a code DLL + carrier PLL tracking loop
 */
class GpsL2MDllPllTrackingFpga : public TrackingInterface
{
public:
    GpsL2MDllPllTrackingFpga(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    virtual ~GpsL2MDllPllTrackingFpga();

    inline std::string role() override
    {
        return role_;
    }

    //! Returns "GPS_L2_M_DLL_PLL_Tracking_FPGA"
    inline std::string implementation() override
    {
        return "GPS_L2_M_DLL_PLL_Tracking_FPGA";
    }

    inline size_t item_size() override
    {
        return sizeof(int);
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
    const std::string default_device_name_GPS_L2 = "multicorrelator_resampler_S00_AXI";  // UIO device name
    static const uint32_t NUM_PRNs = 32;

    dll_pll_veml_tracking_fpga_sptr tracking_fpga_sc_sptr_;
    std::string role_;
    std::string device_name_;
    int* prn_codes_ptr_;
    uint32_t num_prev_assigned_ch_;
    unsigned int channel_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L2_M_DLL_PLL_TRACKING_FPGA_H
