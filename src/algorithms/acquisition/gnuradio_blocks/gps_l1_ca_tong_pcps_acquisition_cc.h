/*!
 * \file gps_l1_ca_tong_pcps_acquisition_cc.h
 * \brief Brief description of the file here
 * \author Luis Esteve, 2011. luis(at)epsilon-formacion.com
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

#ifndef GNSS_SDR_GPS_L1_CA_TONG_PCPS_ACQUISITION_CC_H
#define GNSS_SDR_GPS_L1_CA_TONG_PCPS_ACQUISITION_CC_H

#include <fstream>
#include <gnuradio/gr_block.h>
#include <gnuradio/gr_msg_queue.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/gri_fft.h>
#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include "concurrent_queue.h"

class gps_l1_ca_tong_pcps_acquisition_cc;

typedef boost::shared_ptr<gps_l1_ca_tong_pcps_acquisition_cc>
 gps_l1_ca_tong_pcps_acquisition_cc_sptr;

gps_l1_ca_tong_pcps_acquisition_cc_sptr
 gps_l1_ca_tong_pcps_make_acquisition_cc(unsigned int sampled_ms,
        unsigned int doppler_max, long freq, long fs_in, int samples_per_ms,
        gr_msg_queue_sptr queue, bool dump, std::string dump_filename);

class gps_l1_ca_tong_pcps_acquisition_cc: public gr_block
{

private:

    friend gps_l1_ca_tong_pcps_acquisition_cc_sptr
    gps_l1_ca_tong_pcps_make_acquisition_cc(unsigned int sampled_ms,
            unsigned int doppler_max, long freq, long fs_in,
            int samples_per_ms, gr_msg_queue_sptr queue, bool dump,
            std::string dump_filename);

    gps_l1_ca_tong_pcps_acquisition_cc(unsigned int sampled_ms,
            unsigned int doppler_max, long freq, long fs_in,
            int samples_per_ms, gr_msg_queue_sptr queue, bool dump,
            std::string dump_filename);

    long d_fs_in;
    long d_freq;
    long d_doppler;
    int d_samples_per_ms;
    unsigned int d_doppler_resolution;
    unsigned int d_satellite;
    std::string d_satellite_str;
    unsigned int d_doppler_max;
    unsigned int d_doppler_step;
    unsigned int d_sampled_ms;
    unsigned int d_samples;
    unsigned long int d_sample_counter;
    unsigned long int d_acq_sample_stamp;
    unsigned int d_fbins;
    unsigned int d_indext;
    unsigned d_dwells;
    float d_pfa;
    unsigned int d_A;
    unsigned int d_B;
    unsigned int d_max_dwells;
    unsigned int d_K;

    gr_complex* d_if_sin;

    gr_complex* d_fft_codes;

    gri_fft_complex* d_fft_if;

    gri_fft_complex* d_ifft;

    gr_complex* d_ca_codes;

    gr_complex* d_aux_ca_code;

    unsigned int d_code_phase;
    float d_doppler_freq;
    float d_mag;
    float d_noise_power;
    float d_test_statistics;

    gr_msg_queue_sptr d_queue;
    concurrent_queue<int> *d_channel_internal_queue;
    std::ofstream d_dump_file;

    bool d_active;
    bool d_dump;
    unsigned int d_channel;
    std::string d_dump_filename;

public:

    ~gps_l1_ca_tong_pcps_acquisition_cc();

    signed int prn_code_phase();

    float doppler_freq()
    {
        return d_doppler_freq;
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

    void set_doppler_max(unsigned int doppler_max)
    {
        d_doppler_max = doppler_max;
    }

    void set_doppler_step(unsigned int doppler_step)
    {
        d_doppler_step = doppler_step;
    }

    void set_doppler(unsigned int doppler)
    {
        d_doppler = doppler;
    }

    void set_dwells(unsigned int dwells)
    {
        d_dwells = dwells;
    }

    float calculate_envelope(const gr_complex* _input_signal, std::complex<
            float>* _local_code, std::complex<float>* _local_if_sin);

    void set_channel_queue(concurrent_queue<int> *channel_internal_queue)
    {
        d_channel_internal_queue = channel_internal_queue;
    }

    int general_work(int noutput_items, gr_vector_int &ninput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items);
};

#endif /* GNSS_SDR_GPS_L1_CA_TONG_PCPS_ACQUISITION_CC_H*/
