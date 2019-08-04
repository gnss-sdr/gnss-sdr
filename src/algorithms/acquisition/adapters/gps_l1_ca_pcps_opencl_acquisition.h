/*!
 * \file gps_l1_ca_pcps_opencl_acquisition.h
 * \brief Adapts an OpenCL PCPS acquisition block to an
 *  AcquisitionInterface for GPS L1 C/A signals
 * \author Marc Molina, 2013. marc.molina.pena(at)gmail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GPS_L1_CA_PCPS_OPENCL_ACQUISITION_H_
#define GNSS_SDR_GPS_L1_CA_PCPS_OPENCL_ACQUISITION_H_

#include "channel_fsm.h"
#include "gnss_synchro.h"
#include "pcps_opencl_acquisition_cc.h"
#include <gnuradio/blocks/stream_to_vector.h>
#include <memory>
#include <string>
#include <vector>

class ConfigurationInterface;

/*!
 * \brief This class adapts an OpenCL PCPS acquisition block to an
 *  AcquisitionInterface for GPS L1 C/A signals
 */
class GpsL1CaPcpsOpenClAcquisition : public AcquisitionInterface
{
public:
    GpsL1CaPcpsOpenClAcquisition(ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~GpsL1CaPcpsOpenClAcquisition() = default;

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "GPS_L1_CA_PCPS_OpenCl_Acquisition"
     */
    inline std::string implementation() override
    {
        return "GPS_L1_CA_PCPS_OpenCl_Acquisition";
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
    void set_state(int state __attribute__((unused))) override{};

    /*!
     * \brief Stop running acquisition
     */
    void stop_acquisition() override;

    void set_resampler_latency(uint32_t latency_samples __attribute__((unused))) override{};

    inline bool opencl_ready() const
    {
        bool ready = this->acquisition_cc_->opencl_ready();
        return ready;
    }

private:
    ConfigurationInterface* configuration_;
    pcps_opencl_acquisition_cc_sptr acquisition_cc_;
    gr::blocks::stream_to_vector::sptr stream_to_vector_;
    size_t item_size_;
    std::string item_type_;
    unsigned int vector_length_;
    unsigned int code_length_;
    bool bit_transition_flag_;
    unsigned int channel_;
    std::weak_ptr<ChannelFsm> channel_fsm_;
    float threshold_;
    unsigned int doppler_max_;
    unsigned int doppler_step_;
    unsigned int sampled_ms_;
    unsigned int max_dwells_;
    int64_t fs_in_;
    bool dump_;
    std::string dump_filename_;
    std::vector<std::complex<float>> code_;
    Gnss_Synchro* gnss_synchro_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    float calculate_threshold(float pfa);
};

#endif /* GNSS_SDR_GPS_L1_CA_PCPS_OPENCL_ACQUISITION_H_ */
