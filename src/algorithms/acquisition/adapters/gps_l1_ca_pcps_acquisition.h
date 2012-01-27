/*!
 * \file gps_l1_ca_pcps_acquisition.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for GPS L1 C/A
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *         Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

#ifndef GNSS_SDR_GPS_L1_CA_PCPS_ACQUISITION_H_
#define GNSS_SDR_GPS_L1_CA_PCPS_ACQUISITION_H_

#include "acquisition_interface.h"
#include "gps_l1_ca_pcps_acquisition_cc.h"
#include <gnuradio/gr_msg_queue.h>

class ConfigurationInterface;

class GpsL1CaPcpsAcquisition: public AcquisitionInterface
{

public:

    GpsL1CaPcpsAcquisition(ConfigurationInterface* configuration,
            std::string role, unsigned int in_streams,
            unsigned int out_streams, gr_msg_queue_sptr queue);

    virtual ~GpsL1CaPcpsAcquisition();

    std::string role()
    {
        return role_;
    }
    std::string implementation()
    {
        return "Acquisition";
    }
    size_t item_size()
    {
        return item_size_;
    }

    void connect(gr_top_block_sptr top_block);
    void disconnect(gr_top_block_sptr top_block);
    gr_basic_block_sptr get_left_block();
    gr_basic_block_sptr get_right_block();

    void set_synchro(Gnss_Synchro p_gnss_synchro);
    void set_satellite(Gnss_Satellite satellite);
    void set_channel(unsigned int channel);
    void set_threshold(float threshold);
    void set_doppler_max(unsigned int doppler_max);
    void set_doppler_step(unsigned int doppler_step);
    void set_channel_queue(concurrent_queue<int> *channel_internal_queue);
    signed int prn_code_phase();
    float doppler_freq_shift();

    unsigned long int get_sample_stamp();

    signed int mag();
    void reset();

private:

    gps_l1_ca_pcps_acquisition_cc_sptr acquisition_cc_;
    gr_block_sptr stream_to_vector_;
    gr_block_sptr complex_to_short_;
    gr_block_sptr short_to_complex_;
    size_t item_size_;
    std::string item_type_;
    unsigned int vector_length_;
    Gnss_Synchro* gnss_synchro_;
    Gnss_Satellite gnss_satellite_;
    //unsigned int satellite_;
    unsigned int channel_;
    float threshold_;
    unsigned int doppler_max_;
    unsigned int doppler_step_;
    unsigned int shift_resolution_;
    unsigned int sampled_ms_;
    long fs_in_;
    long if_;
    bool dump_;
    std::string dump_filename_;

    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    gr_msg_queue_sptr queue_;
    concurrent_queue<int> *channel_internal_queue_;

};

#endif /* GNSS_SDR_GPS_L1_CA_PCPS_ACQUISITION_H_ */
