/*!
 * \file galileo_e5a_pcps_acquisition.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E5a data and pilot Signals
 * \author Antonio Ramos, 2018. antonio.ramos(at)cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GALILEO_E5A_PCPS_ACQUISITION_H_
#define GALILEO_E5A_PCPS_ACQUISITION_H_


#include "acquisition_interface.h"
#include "gnss_synchro.h"
#include "pcps_acquisition.h"
#include <gnuradio/blocks/stream_to_vector.h>
#include <string>

class ConfigurationInterface;

class GalileoE5aPcpsAcquisition: public AcquisitionInterface
{
public:
    GalileoE5aPcpsAcquisition(ConfigurationInterface* configuration,
            std::string role, unsigned int in_streams,
            unsigned int out_streams);

    virtual ~GalileoE5aPcpsAcquisition();

    inline std::string role() override
    {
        return role_;
    }

    inline std::string implementation() override
    {
        return "Galileo_E5a_Pcps_Acquisition";
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
    void set_channel(unsigned int channel) override;

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
    void set_state(int state);

private:

    float calculate_threshold(float pfa);

    ConfigurationInterface* configuration_;

    pcps_acquisition_sptr acquisition_;
    gr::blocks::stream_to_vector::sptr stream_to_vector_;

    size_t item_size_;

    std::string item_type_;
    std::string dump_filename_;
    std::string role_;

    bool bit_transition_flag_;
    bool dump_;
    bool acq_pilot_;
    bool use_CFAR_;
    bool blocking_;
    bool acq_iq_;

    unsigned int vector_length_;
    unsigned int code_length_;
    unsigned int channel_;
    unsigned int doppler_max_;
    unsigned int doppler_step_;
    unsigned int sampled_ms_;
    unsigned int max_dwells_;
    unsigned int in_streams_;
    unsigned int out_streams_;

    long fs_in_;

    float threshold_;

    /*
    std::complex<float>* codeI_;
    std::complex<float>* codeQ_;
    */

    gr_complex* code_;

    Gnss_Synchro* gnss_synchro_;

};
#endif /* GALILEO_E5A_PCPS_ACQUISITION_H_ */
