/*!
 * \file gps_l2_m_pcps_acquisition_fpga.h
 * \brief Adapts an FPGA-offloaded PCPS acquisition block
 * to an AcquisitionInterface for GPS L2 M signals
 * \authors <ul>
 *          <li> Javier Arribas, 2019. jarribas(at)cttc.es
 *          </ul>
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

#ifndef GNSS_SDR_GPS_L2_M_PCPS_ACQUISITION_FPGA_H
#define GNSS_SDR_GPS_L2_M_PCPS_ACQUISITION_FPGA_H

#include "acq_conf_fpga.h"
#include "channel_fsm.h"
#include "gnss_synchro.h"
#include "pcps_acquisition_fpga.h"
#include <gnuradio/runtime_types.h>  // for basic_block_sptr, top_block_sptr
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <cstddef>  // for size_t
#include <memory>   // for weak_ptr
#include <string>   // for string
#include <utility>

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class adapts a PCPS acquisition block off-loaded on an FPGA
 * to an AcquisitionInterface for GPS L2 M signals
 */
class GpsL2MPcpsAcquisitionFpga : public AcquisitionInterface
{
public:
    GpsL2MPcpsAcquisitionFpga(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~GpsL2MPcpsAcquisitionFpga() = default;

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "GPS_L2_M_PCPS_Acquisition_FPGA"
     */
    inline std::string implementation() override
    {
        return "GPS_L2_M_PCPS_Acquisition_FPGA";
    }

    inline size_t item_size() override
    {
        return sizeof(float);
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    /*!
     * \brief Set acquisition/tracking common Gnss_Synchro object pointer
     * to efficiently exchange synchronization data between acquisition and
     * tracking blocks
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
     * \brief Initializes acquisition algorithm.
     */
    void init() override;

    /*!
     * \brief Sets local code for GPS L2/M PCPS acquisition algorithm.
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

    void set_resampler_latency(uint32_t latency_samples __attribute__((unused))) override{};

private:
    static const uint32_t downsampling_factor_default = 1;
    static const uint32_t fpga_buff_num = 0;  // L2 band
    static const uint32_t fpga_blk_exp = 13;  // default block exponent

    static const uint32_t NUM_PRNs = 32;
    static const uint32_t QUANT_BITS_LOCAL_CODE = 16;
    static const uint32_t SELECT_LSBits = 0x0000FFFF;         // Select the 10 LSbits out of a 20-bit word
    static const uint32_t SELECT_MSBbits = 0xFFFF0000;        // Select the 10 MSbits out of a 20-bit word
    static const uint32_t SELECT_ALL_CODE_BITS = 0xFFFFFFFF;  // Select a 20 bit word
    static const uint32_t SHL_CODE_BITS = 65536;              // shift left by 10 bits

    pcps_acquisition_fpga_sptr acquisition_fpga_;
    volk_gnsssdr::vector<uint32_t> d_all_fft_codes_;  // memory that contains all the code ffts
    std::weak_ptr<ChannelFsm> channel_fsm_;
    Gnss_Synchro* gnss_synchro_;
    Acq_Conf_Fpga acq_parameters_;
    std::string role_;
    int64_t fs_in_;
    float threshold_;
    unsigned int channel_;
    unsigned int doppler_max_;
    unsigned int doppler_step_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L2_M_PCPS_ACQUISITION_FPGA_H
