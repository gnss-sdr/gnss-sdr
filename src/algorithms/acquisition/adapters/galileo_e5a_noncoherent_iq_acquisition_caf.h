/*!
 * \file galileo_e5a_noncoherent_iq_acquisition_caf.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E5a data and pilot Signals
  * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \based on work from:
 *          <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          <li> Marc Molina, 2013. marc.molina.pena@gmail.com
 *          </ul>
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

#ifndef GALILEO_E5A_NONCOHERENT_IQ_ACQUISITION_CAF_H_
#define GALILEO_E5A_NONCOHERENT_IQ_ACQUISITION_CAF_H_

#include "channel_fsm.h"
#include "galileo_e5a_noncoherent_iq_acquisition_caf_cc.h"
#include "gnss_synchro.h"
#include <memory>
#include <string>
#include <vector>

class ConfigurationInterface;

class GalileoE5aNoncoherentIQAcquisitionCaf : public AcquisitionInterface
{
public:
    GalileoE5aNoncoherentIQAcquisitionCaf(ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~GalileoE5aNoncoherentIQAcquisitionCaf() = default;

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "Galileo_E5a_Noncoherent_IQ_Acquisition_CAF"
     */
    inline std::string implementation() override
    {
        return "Galileo_E5a_Noncoherent_IQ_Acquisition_CAF";
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
     * \brief Sets local Galileo E5a code for PCPS acquisition algorithm.
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
     * \brief If set to 1, ensures that acquisition starts at the
     * first available sample.
     * \param state - int=1 forces start of acquisition
     */
    void set_state(int state) override;

    /*!
     * \brief Stop running acquisition
     */
    void stop_acquisition() override;

    void set_resampler_latency(uint32_t latency_samples __attribute__((unused))) override{};

private:
    ConfigurationInterface* configuration_;
    galileo_e5a_noncoherentIQ_acquisition_caf_cc_sptr acquisition_cc_;
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
    int Zero_padding;
    int CAF_window_hz_;
    std::vector<std::complex<float>> codeI_;
    std::vector<std::complex<float>> codeQ_;
    bool both_signal_components;
    Gnss_Synchro* gnss_synchro_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    float calculate_threshold(float pfa);
};
#endif /* GALILEO_E5A_NONCOHERENT_IQ_ACQUISITION_CAF_H_ */
