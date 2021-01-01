/*!
 * \file gps_l1_ca_pcps_quicksync_acquisition.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for GPS L1 C/A signals implementing the QuickSync Algorithm.
 * \date June, 2014
 * \author Damian Miralles Sanchez. dmiralles2009@gmail.com
 *
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

#ifndef GNSS_SDR_GPS_L1_CA_PCPS_QUICKSYNC_ACQUISITION_H
#define GNSS_SDR_GPS_L1_CA_PCPS_QUICKSYNC_ACQUISITION_H

#include "channel_fsm.h"
#include "configuration_interface.h"
#include "gnss_synchro.h"
#include "pcps_quicksync_acquisition_cc.h"
#include <gnuradio/blocks/stream_to_vector.h>
#include <memory>
#include <string>
#include <vector>

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class adapts a PCPS acquisition block to an AcquisitionInterface
 *  for GPS L1 C/A signals
 */
class GpsL1CaPcpsQuickSyncAcquisition : public AcquisitionInterface
{
public:
    GpsL1CaPcpsQuickSyncAcquisition(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~GpsL1CaPcpsQuickSyncAcquisition() = default;

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "GPS_L1_CA_PCPS_QuickSync_Acquisition"
     */
    inline std::string implementation() override
    {
        return "GPS_L1_CA_PCPS_QuickSync_Acquisition";
    }

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
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
        acquisition_cc_->set_channel(channel_);
    }

    /*!
     * \brief Set channel fsm associated to this acquisition instance
     */
    inline void set_channel_fsm(std::weak_ptr<ChannelFsm> channel_fsm) override
    {
        channel_fsm_ = channel_fsm;
        acquisition_cc_->set_channel_fsm(channel_fsm);
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
     * \brief Sets local code for GPS L1/CA PCPS acquisition algorithm.
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
    float calculate_threshold(float pfa) const;

    const ConfigurationInterface* configuration_;
    pcps_quicksync_acquisition_cc_sptr acquisition_cc_;
    std::weak_ptr<ChannelFsm> channel_fsm_;

    gr::blocks::stream_to_vector::sptr stream_to_vector_;
    std::vector<std::complex<float>> code_;
    std::string item_type_;
    std::string dump_filename_;
    std::string role_;

    Gnss_Synchro* gnss_synchro_;

    int64_t fs_in_;
    size_t item_size_;
    float threshold_;
    unsigned int vector_length_;
    unsigned int code_length_;
    unsigned int channel_;
    unsigned int doppler_max_;
    unsigned int doppler_step_;
    unsigned int sampled_ms_;
    unsigned int max_dwells_;
    unsigned int folding_factor_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    bool bit_transition_flag_;
    bool dump_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L1_CA_PCPS_QUICKSYNC_ACQUISITION_H
