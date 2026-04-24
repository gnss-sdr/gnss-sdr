/*!
 * \file pcps_acquisition_adapter_fpga.h
 * \brief Adapts an FPGA-offloaded PCPS acquisition block to an AcquisitionInterface
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


#ifndef GNSS_SDR_PCPS_ACQUISITION_ADAPTER_FPGA_H
#define GNSS_SDR_PCPS_ACQUISITION_ADAPTER_FPGA_H

#include "acquisition_interface.h"
#include "acq_conf_fpga.h"
#include "channel_fsm.h"
#include "gnss_synchro.h"
#include "pcps_acquisition_fpga.h"
#include "signal_flag.h"
#include <gnuradio/runtime_types.h>
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

class ConfigurationInterface;

class PcpsAcquisitionAdapterFpga : public AcquisitionInterface
{
public:
    PcpsAcquisitionAdapterFpga(
        const ConfigurationInterface* configuration,
        const std::string& role,
        const std::string& implementation,
        unsigned int in_streams,
        unsigned int out_streams,
        signal_flag sig_flag);

    ~PcpsAcquisitionAdapterFpga() override = default;

    inline std::string role() override { return role_; }
    inline std::string implementation() override { return implementation_; }
    inline size_t item_size() override { return sizeof(int16_t); }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    signed int mag() override;
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override;
    void set_channel(unsigned int channel) override;
    void set_channel_fsm(std::weak_ptr<ChannelFsm> channel_fsm) override;
    void set_doppler_center(int doppler_center) override;
    void reset() override;
    void stop_acquisition() override;
    void set_resampler_latency(uint32_t latency_samples __attribute__((unused))) override {}
    void set_local_code() override;

private:
    void generate_prn_codes(const ConfigurationInterface* configuration);
    void init();

    volk_gnsssdr::vector<uint32_t> d_all_fft_codes_;
    Acq_Conf_Fpga acq_parameters_;
    pcps_acquisition_fpga_sptr acquisition_fpga_;
    const signal_flag sig_flag_;
    const std::string role_;
    const std::string implementation_;
};

#endif  // GNSS_SDR_PCPS_ACQUISITION_ADAPTER_FPGA_H
