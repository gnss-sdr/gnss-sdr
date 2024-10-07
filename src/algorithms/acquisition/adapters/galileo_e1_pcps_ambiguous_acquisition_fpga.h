/*!
 * \file galileo_e1_pcps_ambiguous_acquisition_fpga.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E1 Signals for the FPGA
 * \author Marc Majoral, 2019. mmajoral(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GALILEO_E1_PCPS_AMBIGUOUS_ACQUISITION_FPGA_H
#define GNSS_SDR_GALILEO_E1_PCPS_AMBIGUOUS_ACQUISITION_FPGA_H

#include "acq_conf_fpga.h"
#include "channel_fsm.h"
#include "gnss_synchro.h"
#include "pcps_acquisition_fpga.h"
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class adapts a PCPS acquisition block off-loaded on an FPGA
 * to an AcquisitionInterface for Galileo E1 Signals
 */
class GalileoE1PcpsAmbiguousAcquisitionFpga : public AcquisitionInterface
{
public:
    /*!
     * \brief Constructor
     */
    GalileoE1PcpsAmbiguousAcquisitionFpga(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    /*!
     * \brief Destructor
     */
    ~GalileoE1PcpsAmbiguousAcquisitionFpga() = default;

    /*!
     * \brief Role
     */
    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "Galileo_E1_PCPS_Ambiguous_Acquisition_FPGA"
     */
    inline std::string implementation() override
    {
        return "Galileo_E1_PCPS_Ambiguous_Acquisition_FPGA";
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
     * \brief Set acquisition/tracking common Gnss_Synchro object pointer
     * to efficiently exchange synchronization data between acquisition and
     *  tracking blocks
     */
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override;

    /*!
     * \brief Set acquisition channel unique ID
     */
    inline void set_channel(unsigned int channel) override
    {
        channel_ = channel;
        acquisition_fpga_->set_channel(channel_);
    }

    /*!
     * \brief Set channel fsm associated to this acquisition instance
     */
    inline void set_channel_fsm(std::weak_ptr<ChannelFsm> channel_fsm) override
    {
        channel_fsm_ = std::move(channel_fsm);
        acquisition_fpga_->set_channel_fsm(channel_fsm_);
    }

    /*!
     * \brief Set statistics threshold of PCPS algorithm
     */
    void set_threshold(float threshold) override;

    /*!
     * \brief Set maximum Doppler off grid search
     */
    void set_doppler_max(unsigned int doppler_max) override;

    /*!
     * \brief Set Doppler steps for the grid search
     */
    void set_doppler_step(unsigned int doppler_step) override;

    /*!
     * \brief Set Doppler center for the grid search
     */
    void set_doppler_center(int doppler_center) override;

    /*!
     * \brief Initializes acquisition algorithm.
     */
    void init() override;

    /*!
     * \brief Sets local code for Galileo E1 PCPS acquisition algorithm.
     */
    void set_local_code() override;

    /*!
     * \brief Returns the maximum peak of grid search
     */
    signed int mag() override;

    /*!
     * \brief Restart acquisition algorithm
     */
    void reset() override;

    /*!
     * \brief If state = 1, it forces the block to start acquiring from the first sample
     */
    void set_state(int state) override;

    /*!
     * \brief Stop running acquisition
     */
    void stop_acquisition() override;

    /*!
     * \brief Set resampler latency
     */
    void set_resampler_latency(uint32_t latency_samples __attribute__((unused))) override{};

private:
    static const uint32_t downsampling_factor_default = 4;
    static const uint32_t fpga_buff_num = 0;  // L1/E1 band
    static const uint32_t fpga_blk_exp = 13;  // default block exponent

    // the following flags are FPGA-specific and they are using arrange the values of the fft of the local code in the way the FPGA
    // expects. This arrangement is done in the initialisation to avoid consuming unnecessary clock cycles during tracking.
    static const uint32_t quant_bits_local_code = 16;
    static const uint32_t select_lsbits = 0x0000FFFF;         // Select the 10 LSbits out of a 20-bit word
    static const uint32_t select_msbits = 0xFFFF0000;         // Select the 10 MSbits out of a 20-bit word
    static const uint32_t select_all_code_bits = 0xFFFFFFFF;  // Select a 20 bit word
    static const uint32_t shl_code_bits = 65536;              // shift left by 10 bits

    pcps_acquisition_fpga_sptr acquisition_fpga_;
    volk_gnsssdr::vector<uint32_t> d_all_fft_codes_;  // memory that contains all the code ffts
    std::weak_ptr<ChannelFsm> channel_fsm_;
    Gnss_Synchro* gnss_synchro_;
    Acq_Conf_Fpga acq_parameters_;
    std::string role_;
    int64_t fs_in_;
    int32_t doppler_center_;
    uint32_t channel_;
    uint32_t doppler_max_;
    uint32_t doppler_step_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    bool acquire_pilot_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_E1_PCPS_AMBIGUOUS_ACQUISITION_FPGA_H
