/*!
 * \file base_dll_pll_tracking_fpga.h
 * \brief Base class providing shared logic for DLL+PLL VEML tracking adapters
 * for FPGA-based devices
 * \authors Carles Fernandez, 2025. carles.fernandez(at)cttc.cat
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2025  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_BASE_DLL_PLL_TRACKING_FPGA_H
#define GNSS_SDR_BASE_DLL_PLL_TRACKING_FPGA_H

#include "dll_pll_conf_fpga.h"
#include "dll_pll_veml_tracking_fpga.h"
#include "tracking_interface.h"
#include <cstddef>
#include <cstdint>
#include <string>

// /** \addtogroup Tracking
//  * \{ */
// /** \addtogroup Tracking_adapters
//  * \{ */

class ConfigurationInterface;

class BaseDllPllTrackingFpga : public TrackingInterface
{
public:
    /*!
     * \brief Base constructor of FPGA-based Tracking block adapters
     */
    BaseDllPllTrackingFpga(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);
    /*!
     * \brief Base destructor of FPGA-based Tracking block adapters
     */
    virtual ~BaseDllPllTrackingFpga() = default;

    // Common TrackingInterface overrides

    /*!
     * \brief Get role from the Tracking block adapter
     */
    std::string role() override { return role_; }

    /*!
     * \brief Get item_size from the Tracking block adapter
     */
    size_t item_size() override { return sizeof(int16_t); }

    /*!
     * \brief Connect the Tracking block adapter
     */
    void connect(gr::top_block_sptr top_block) override;

    /*!
     * \brief Disconnect the Tracking block adapter
     */
    void disconnect(gr::top_block_sptr top_block) override;

    /*!
     * \brief Get left block from the Tracking block adapter
     */
    gr::basic_block_sptr get_left_block() override;

    /*!
     * \brief Get right block from the Tracking block adapter
     */
    gr::basic_block_sptr get_right_block() override;

    /*!
     * \brief Start the tracking process in the FPGA
     */
    void start_tracking() override;

    /*!
     * \brief Stop the tracking process in the FPGA
     */
    void stop_tracking() override;

    /*!
     * \brief Set tracking channel unique ID
     */
    void set_channel(unsigned int channel) override;

    /*!
     * \brief Set acquisition/tracking common Gnss_Synchro object pointer
     * to efficiently exchange synchronization data between acquisition and tracking blocks
     */
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override;

protected:
    // Can be used by each derived class
    static const int32_t LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY = 0x0C000000;                    // flag that enables WE (Write Enable) of the local code FPGA
    static const int32_t LOCAL_CODE_FPGA_CORRELATOR_SELECT_COUNT = 0x20000000;                // flag that selects the writting of the pilot code in the FPGA (as opposed to the data code)
    const std::string default_device_name_GPS_L1 = "multicorrelator_resampler_S00_AXI";       // Default FPGA device name for GPS L1
    const std::string default_device_name_Galileo_E1 = "multicorrelator_resampler_5_1_AXI";   // Default FPGA device name for Galileo L1
    const std::string default_device_name_GPS_L5 = "multicorrelator_resampler_3_1_AXI";       // Default FPGA device name for GPS L5
    const std::string default_device_name_GPS_L2 = "multicorrelator_resampler_S00_AXI";       // Default FPGA device name for GPS L2
    const std::string default_device_name_Galileo_E5a = "multicorrelator_resampler_3_1_AXI";  // Default FPGA device name for Galieo E5a

    inline Dll_Pll_Conf_Fpga& config_params_fpga() { return trk_params_; }
    inline const Dll_Pll_Conf_Fpga& config_params_fpga() const { return trk_params_; }
    inline void set_num_prev_assigned_ch(uint32_t num_prev_assigned_ch) { num_prev_assigned_ch_ = num_prev_assigned_ch; };
    inline uint32_t get_channel() { return channel_; };
    inline uint32_t get_num_prev_assigned_ch() { return num_prev_assigned_ch_; };

    // Can be overriden by derived classes
    bool find_alternative_device(std::string& device_io_name [[maybe_unused]]) { return false; };

    // Must be set by each derived class
    dll_pll_veml_tracking_fpga_sptr tracking_fpga_sc_sptr_;
    std::string device_name_;

private:
    Dll_Pll_Conf_Fpga trk_params_;
    const std::string role_;
    uint32_t channel_;
    uint32_t num_prev_assigned_ch_;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_BASE_DLL_PLL_TRACKING_FPGA_H
