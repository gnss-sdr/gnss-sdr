/*!
 * \file dll_pll_tracking_adapter_fpga.h
 * \brief Adapts an FPGA-offloaded DLL/PLL tracking block to a TrackingInterface
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


#ifndef GNSS_SDR_DLL_PLL_TRACKING_ADAPTER_FPGA_H
#define GNSS_SDR_DLL_PLL_TRACKING_ADAPTER_FPGA_H

#include "dll_pll_conf_fpga.h"
#include "dll_pll_veml_tracking_fpga.h"
#include "signal_flag.h"
#include "tracking_interface.h"
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>

class ConfigurationInterface;

class DllPllTrackingAdapterFpga : public TrackingInterface
{
public:
    DllPllTrackingAdapterFpga(const ConfigurationInterface* configuration,
        const std::string& role,
        const std::string& implementation,
        unsigned int in_streams,
        unsigned int out_streams,
        signal_flag sig_flag);

    ~DllPllTrackingAdapterFpga() override;

    inline std::string role() override { return role_; }
    inline std::string implementation() override { return implementation_; }
    inline size_t item_size() override { return sizeof(int16_t); }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    void start_tracking() override;
    void stop_tracking() override;
    void set_channel(unsigned int channel) override;
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override;

private:
    void configure_fpga_tracking_channel_mapping(const std::string& signal);
    void set_signal_channel_base_index_locked();
    uint32_t get_num_alternative_devices_locked() const;
    void configure_signal_parameters(const ConfigurationInterface* configuration);
    void generate_prn_codes();

    inline static const std::map<std::string, std::string> signal_to_device_ = {
        {"1C", "multicorrelator_resampler_S00_AXI"},
        {"2S", "multicorrelator_resampler_S00_AXI"},
        {"L5", "multicorrelator_resampler_3_1_AXI"},
        {"1B", "multicorrelator_resampler_5_1_AXI"},
        {"5X", "multicorrelator_resampler_3_1_AXI"},
    };
    inline static const std::map<std::string, std::string> signal_to_alternative_device_ = {
        {"1C", "multicorrelator_resampler_5_1_AXI"}};
    inline static std::map<std::string, int> channel_counts_;
    inline static std::mutex channel_counts_mtx_;

    Dll_Pll_Conf_Fpga trk_params_;
    dll_pll_veml_tracking_fpga_sptr tracking_fpga_sc_sptr_;
    int32_t* prn_codes_ptr_;
    int32_t* data_codes_ptr_;
    const signal_flag sig_flag_;
    const std::string role_;
    const std::string implementation_;
    std::string signal_;
    std::string device_name_;
    uint32_t channel_;
    uint32_t signal_base_channel_index_;
};

#endif  // GNSS_SDR_DLL_PLL_TRACKING_ADAPTER_FPGA_H
