/*!
 * \file gps_l5_dll_pll_tracking_fpga.h
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for GPS L5 to a TrackingInterface for the FPGA
 * \author Marc Majoral, 2019. mmajoral(at)cttc.cat
 *         Javier Arribas, 2019. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_GPS_L5_DLL_PLL_TRACKING_FPGA_H
#define GNSS_SDR_GPS_L5_DLL_PLL_TRACKING_FPGA_H

#include "dll_pll_veml_tracking_fpga.h"
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
class GpsL5DllPllTrackingFpga : public TrackingInterface
{
public:
    /*!
     * \brief Constructor
     */
    GpsL5DllPllTrackingFpga(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    /*!
     * \brief Destructor
     */
    virtual ~GpsL5DllPllTrackingFpga();

    /*!
     * \brief Role
     */
    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "GPS_L5_DLL_PLL_Tracking_FPGA"
     */
    inline std::string implementation() override
    {
        return "GPS_L5_DLL_PLL_Tracking_FPGA";
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
     * to efficiently exchange synchronization data between acquisition and tracking blocks
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
    const std::string default_device_name_GPS_L5_ = "multicorrelator_resampler_3_1_AXI";  // UIO device name

    static const uint32_t NUM_PRNs = 32;  // total number of PRNs

    // the following flags are FPGA-specific and they are using arrange the values of the local code in the way the FPGA
    // expects. This arrangement is done in the initialisation to avoid consuming unnecessary clock cycles during tracking.
    static const int32_t LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY = 0x0C000000;      // flag that enables WE (Write Enable) of the local code FPGA
    static const int32_t LOCAL_CODE_FPGA_CORRELATOR_SELECT_COUNT = 0x20000000;  // flag that selects the writing of the pilot code in the FPGA (as opposed to the data code)

    dll_pll_veml_tracking_fpga_sptr tracking_fpga_sc_sptr_;
    std::string role_;
    std::string device_name_;
    int32_t* prn_codes_ptr_;
    int32_t* data_codes_ptr_;
    uint32_t channel_;
    uint32_t num_prev_assigned_ch_;
    uint32_t in_streams_;
    uint32_t out_streams_;
    bool track_pilot_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L5_DLL_PLL_TRACKING_FPGA_H
