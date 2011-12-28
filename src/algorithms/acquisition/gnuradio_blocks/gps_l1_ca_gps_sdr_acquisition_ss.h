/*!
 * \file gps_l1_ca_gps_sdr_acquisition_ss.h
 * \brief Brief description of the file here
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
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

#ifndef GNSS_SDR_GPS_L1_CA_GPS_SDR_ACQUISITION_SS_H
#define	GNSS_SDR_GPS_L1_CA_GPS_SDR_ACQUISITION_SS_H

#include <fstream>
#include <gnuradio/gr_block.h>
#include <gnuradio/gr_msg_queue.h>
#include "gps_sdr_signal_processing.h"
#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include "concurrent_queue.h"


class FFT;

class gps_l1_ca_gps_sdr_acquisition_ss;
typedef boost::shared_ptr<gps_l1_ca_gps_sdr_acquisition_ss>
        gps_l1_ca_gps_sdr_acquisition_ss_sptr;
gps_l1_ca_gps_sdr_acquisition_ss_sptr
gps_l1_ca_gps_sdr_make_acquisition_ss(unsigned int sampled_ms, long freq,
        long fs_in, int samples_per_ms, gr_msg_queue_sptr queue, bool dump,
        std::string dump_filename);

class gps_l1_ca_gps_sdr_acquisition_ss: public gr_block
{

private:

    friend gps_l1_ca_gps_sdr_acquisition_ss_sptr
    gps_l1_ca_gps_sdr_make_acquisition_ss(unsigned int sampled_ms, long freq,
            long fs_in, int samples_per_ms, gr_msg_queue_sptr queue,
            bool dump, std::string dump_filename);

    gps_l1_ca_gps_sdr_acquisition_ss(unsigned int sampled_ms, long freq,
            long fs_in, int samples_per_ms, gr_msg_queue_sptr queue,
            bool dump, std::string dump_filename);

    long d_fs_in;
    long d_freq;
    int d_samples_per_ms;
    unsigned int d_satellite;
    float d_threshold;
    unsigned int d_doppler_max;
    unsigned int d_doppler_resolution;
    unsigned int d_sampled_ms;
    unsigned int d_fft_size;
    unsigned long int d_sample_counter;
    unsigned long int d_acq_sample_stamp;

    CPX* d_baseband_signal;
    CPX* d_baseband_signal_shift;
    CPX* d_sine_if;
    CPX* d_sine_250;
    CPX* d_sine_500;
    CPX* d_sine_750;
    FFT* d_pFFT;
    FFT* d_piFFT;
    CPX* d_fft_codes[32];

    signed int d_prn_code_phase;
    float d_doppler_freq_shift;
    unsigned int d_mag;

    gr_msg_queue_sptr d_queue;
    concurrent_queue<int> *d_channel_internal_queue;
    std::ofstream d_dump_file;

    bool d_active;
    bool d_dump;
    unsigned int d_channel;
    std::string d_dump_filename;

public:

    ~gps_l1_ca_gps_sdr_acquisition_ss();

    signed int prn_code_phase();
    float doppler_freq_phase()
    {
        return d_doppler_freq_shift;
    }
    unsigned int mag()
    {
        return d_mag;
    }

    unsigned long int get_sample_stamp()
    {
        return d_acq_sample_stamp;
    }

    void set_satellite(unsigned int satellite);
    void set_active(bool active)
    {
        d_active = active;
    }
    void set_channel(unsigned int channel)
    {
        d_channel = channel;
    }
    void set_threshold(float threshold)
    {
        d_threshold = threshold;
    }
    void set_doppler_max(unsigned int doppler_max)
    {
        d_doppler_max = doppler_max;
    }

    void set_channel_queue(concurrent_queue<int> *channel_internal_queue)
    {
        d_channel_internal_queue = channel_internal_queue;
    }

    int general_work(int noutput_items, gr_vector_int &ninput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items);
};

#endif /* GNSS_SDR_GPS_L1_CA_GPS_SDR_ACQUISITION_SS_H */
