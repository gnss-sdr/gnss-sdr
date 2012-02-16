/*!
 * \file gps_l1_ca_gps_sdr_acquisition.h
 * \brief Interface of an adapter of an acquisition module based
 * on the method in Gregory Heckler's GPS-SDR (see http://github.com/gps-sdr/gps-sdr)
 * to an AcquisitionInterface
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2011. luis(at)epsilon-formacion.com
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

#ifndef GNSS_SDR_GPS_L1_CA_GPS_SDR_ACQUISITION_H_
#define GNSS_SDR_GPS_L1_CA_GPS_SDR_ACQUISITION_H_

#include "acquisition_interface.h"
#include "gps_l1_ca_gps_sdr_acquisition_cc.h"
//#include "gps_l1_ca_gps_sdr_acquisition_ss.h"
#include <gnuradio/gr_msg_queue.h>

class ConfigurationInterface;

/*!
 * \brief Adapts the GPS-SDR acquisition implementation to
 * an Acquisition Interface
 *
 * Derived from AcquisitionInterface, this class wraps the implementation
 * of the acquisition algorithm proposed by Gregory Heckler at https://github.com/gps-sdr/gps-sdr
 *
 */
class GpsL1CaGpsSdrAcquisition: public AcquisitionInterface
{

public:

    GpsL1CaGpsSdrAcquisition(ConfigurationInterface* configuration,
            std::string role, unsigned int in_streams,
            unsigned int out_streams, gr_msg_queue_sptr queue);

    virtual ~GpsL1CaGpsSdrAcquisition();

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

    void set_satellite(Gnss_Satellite gnss_satellite);
    void set_channel(unsigned int channel);
    void set_threshold(float threshold);
    void set_doppler_max(unsigned int doppler_max);
    void set_channel_queue(concurrent_queue<int> *channel_internal_queue);
    signed int prn_code_phase();
    float doppler_freq_shift();
    signed int mag();
    void reset();
    unsigned long int get_sample_stamp();

    //Not used in this implementation
    void set_doppler_step(unsigned int doppler_step)
    {};

private:

    gps_l1_ca_gps_sdr_acquisition_cc_sptr acquisition_cc_;
    //gps_l1_ca_gps_sdr_acquisition_ss_sptr acquisition_ss_;
    gr_block_sptr stream_to_vector_;
    gr_block_sptr complex_to_short_;
    gr_block_sptr short_to_complex_;
    size_t item_size_;
    std::string item_type_;
    unsigned int vector_length_;
    Gnss_Satellite gnss_satellite_;
    unsigned int channel_;
    float threshold_;
    unsigned int doppler_max_;
    unsigned int acquisition_ms_;
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

#endif
