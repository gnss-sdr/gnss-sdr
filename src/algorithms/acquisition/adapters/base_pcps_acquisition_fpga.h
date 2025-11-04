/*!
 * \file base_pcps_acquisition_fpga.h
 * \brief Shared implementation for FPGA-based PCPS acquisition adapters
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


#ifndef GNSS_SDR_BASE_PCPS_ACQUISITION_FPGA_H
#define GNSS_SDR_BASE_PCPS_ACQUISITION_FPGA_H

#include "acq_conf_fpga.h"
#include "channel_fsm.h"
#include "gnss_synchro.h"
#include "pcps_acquisition_fpga.h"
#include <gnuradio/runtime_types.h>  // for basic_block_sptr, top_block_sptr
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

/** \addtogroup Acquisition
 * Classes for GNSS signal acquisition
 * \{ */
/** \addtogroup Acq_adapters acquisition_adapters
 * Wrap GNU Radio acquisition blocks with an AcquisitionInterface
 * \{ */

class ConfigurationInterface;

/*!
 * \brief Base class providing shared logic for FPGA-based GPS PCPS acquisition adapters.
 */
class BasePcpsAcquisitionFpga : public AcquisitionInterface
{
public:
    BasePcpsAcquisitionFpga(const ConfigurationInterface* configuration,
        std::string role,
        double code_rate_cps,
        double code_length_chips,
        uint32_t opt_acq_fs_sps,
        uint32_t default_fpga_blk_exp,
        uint32_t acq_buff,
        unsigned int in_streams,
        unsigned int out_streams);

    ~BasePcpsAcquisitionFpga() override = default;

    inline std::string role() override final { return role_; }

    inline size_t item_size() override { return sizeof(int16_t); }

    // Common AcquisitionInterface overrides
    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    signed int mag() override;
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override;
    void set_channel(unsigned int channel) override;
    void set_channel_fsm(std::weak_ptr<ChannelFsm> channel_fsm) override;
    void set_threshold(float threshold) override;
    void set_doppler_step(unsigned int doppler_step) override;
    void set_doppler_center(int doppler_center) override;
    void set_state(int state) override;
    void reset() override;
    void stop_acquisition() override;
    void init() override;
    void set_resampler_latency(uint32_t latency_samples __attribute__((unused))) override {}
    void set_local_code() override;

protected:
    // Members subclasses can use
    static const uint32_t QUANT_BITS_LOCAL_CODE = 16;
    static const uint32_t SELECT_LSBITS = 0x0000FFFF;         // Select the 10 LSbits out of a 20-bit word
    static const uint32_t SELECT_MSBITS = 0xFFFF0000;         // Select the 10 MSbits out of a 20-bit word
    static const uint32_t SELECT_ALL_CODE_BITS = 0xFFFFFFFF;  // Select a 20 bit word
    static const uint32_t SHL_CODE_BITS = 65536;              // shift left by 10 bits
    static const uint32_t ACQ_BUFF_0 = 0;                     // FPGA Acquisition IP buffer containing L1/E1 frequency band samples by default.
    static const uint32_t ACQ_BUFF_1 = 1;                     // FPGA Acquisition IP buffer containing L2 or L5/E5 frequency band samples by default.

    // Members subclasses must set
    volk_gnsssdr::vector<uint32_t> d_all_fft_codes_;
    Acq_Conf_Fpga acq_parameters_;

private:
    // Managed entirely by the base class
    pcps_acquisition_fpga_sptr acquisition_fpga_;
    const std::string role_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_BASE_PCPS_ACQUISITION_FPGA_H