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
#include <map>
#include <mutex>
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
     * \brief configure FPGA tracking channel mapping
     */
    void configure_fpga_tracking_channel_mapping(std::string signal);

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
    static const int32_t LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY = 0x0C000000;      // flag that enables WE (Write Enable) of the local code FPGA
    static const int32_t LOCAL_CODE_FPGA_CORRELATOR_SELECT_COUNT = 0x20000000;  // flag that selects the writing of the pilot code in the FPGA (as opposed to the data code)

    inline Dll_Pll_Conf_Fpga& config_params_fpga() { return trk_params_; }
    inline const Dll_Pll_Conf_Fpga& config_params_fpga() const { return trk_params_; }

    // Must be set by each derived class
    dll_pll_veml_tracking_fpga_sptr tracking_fpga_sc_sptr_;

private:
    // Mapping of GNSS signals to FPGA hardware multicorrelator names
    inline static const std::map<std::string, std::string> signal_to_device_ = {
        {"1C", "multicorrelator_resampler_S00_AXI"},
        {"2S", "multicorrelator_resampler_S00_AXI"},
        {"L5", "multicorrelator_resampler_3_1_AXI"},
        {"1B", "multicorrelator_resampler_5_1_AXI"},
        {"5X", "multicorrelator_resampler_3_1_AXI"},
    };
    // Mapping of GNSS signals to alternative FPGA multicorrelator tracking names
    inline static const std::map<std::string, std::string> signal_to_alternative_device_ = {
        {"1C", "multicorrelator_resampler_5_1_AXI"}};

    // Number of channels per signal supported by the FPGA
    inline static std::map<std::string, int> channel_counts_;

    void set_signal_channel_base_index_locked_();          // Compute the base channel index for signal_ based on the channel initialization order in the receiver and the number of channels assigned to each signal. Requires: channel_counts_mtx_ is held by the caller.
    uint32_t get_num_alternative_devices_locked_() const;  // Return the number of FPGA tracking multicorrelator devices mapped to signal_ that are also assigned as alternative for other signals. Requires: channel_counts_mtx_ is held by the caller.

    Dll_Pll_Conf_Fpga trk_params_;
    const std::string role_;
    std::string signal_;       // GNSS signal type (1C, 2S, L5, 1B, 5X ...)
    std::string device_name_;  // FPGA multicorrelator name

    inline static std::mutex channel_counts_mtx_;  // Protects access to channel_counts_

    uint32_t channel_;
    uint32_t signal_base_channel_index_;  // Channel index within the signal type (GPS L1 C/A, GPS L2, GPS L5, Galileo E1, Galileo E5a ...)
};

/** \} */
/** \} */
#endif  // GNSS_SDR_BASE_DLL_PLL_TRACKING_FPGA_H
