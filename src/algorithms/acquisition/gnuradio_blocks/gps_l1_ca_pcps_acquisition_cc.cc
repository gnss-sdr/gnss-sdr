/*!
 * \file gps_l1_ca_pcps_acquisition_cc.h
 * \brief Brief description of the file here
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

#include "gps_l1_ca_pcps_acquisition_cc.h"
#include "gps_sdr_signal_processing.h"
#include "control_message_factory.h"
#include "gps_sdr_x86.h"
#include <gnuradio/gr_io_signature.h>
#include <sstream>
#include <glog/log_severity.h>
#include <glog/logging.h>

using google::LogMessage;

gps_l1_ca_pcps_acquisition_cc_sptr gps_l1_ca_pcps_make_acquisition_cc(
        unsigned int sampled_ms, unsigned int doppler_max, long freq,
        long fs_in, int samples_per_ms, gr_msg_queue_sptr queue, bool dump,
        std::string dump_filename)
{

    return gps_l1_ca_pcps_acquisition_cc_sptr(
            new gps_l1_ca_pcps_acquisition_cc(sampled_ms, doppler_max, freq,
                    fs_in, samples_per_ms, queue, dump, dump_filename));
}

gps_l1_ca_pcps_acquisition_cc::gps_l1_ca_pcps_acquisition_cc(
        unsigned int sampled_ms, unsigned int doppler_max, long freq,
        long fs_in, int samples_per_ms, gr_msg_queue_sptr queue, bool dump,
        std::string dump_filename) :
    gr_block("gps_l1_ca_pcps_acquisition_cc", gr_make_io_signature(1, 1,
            sizeof(gr_complex) * samples_per_ms), gr_make_io_signature(0, 0,
            sizeof(gr_complex) * samples_per_ms))
{

    // SAMPLE COUNTER
    d_sample_counter = 0;

    d_active = false;
    d_dump = dump;
    d_queue = queue;
    d_dump_filename = dump_filename;

    d_freq = freq;
    d_fs_in = fs_in;

    d_samples_per_ms = samples_per_ms;
    d_sampled_ms = sampled_ms;

    d_doppler_max = doppler_max;

    d_satellite = 0;

    d_fft_size = d_sampled_ms * d_samples_per_ms;

    d_doppler_freq = 0.0;
    d_code_phase = 0;
    d_mag = 0;
    d_input_power = 0.0;

    d_sine_if = new gr_complex[d_fft_size];

    d_fft_codes = (gr_complex*)malloc(sizeof(gr_complex) * d_samples_per_ms);

    // Direct FFT
    d_fft_if = new gri_fft_complex(d_fft_size, true);

    // Inverse FFT
    d_ifft = new gri_fft_complex(d_fft_size, false);

    DLOG(INFO) << "fs in " << d_fs_in;
    DLOG(INFO) << "samples per ms " << d_samples_per_ms;
    DLOG(INFO) << "doppler max " << d_doppler_max;
    DLOG(INFO) << "freq " << d_freq;
    DLOG(INFO) << "satellite " << d_satellite;
    DLOG(INFO) << "sampled_ms " << d_sampled_ms;
    DLOG(INFO) << "fft_size " << d_fft_size;
    DLOG(INFO) << "dump filename " << d_dump_filename;
    DLOG(INFO) << "dump " << d_dump;
}

gps_l1_ca_pcps_acquisition_cc::~gps_l1_ca_pcps_acquisition_cc()
{
	delete[] d_sine_if;
	delete[] d_fft_codes;
	delete d_ifft;
	delete d_fft_if;

    if (d_dump)
    {
        d_dump_file.close();
    }
}

void gps_l1_ca_pcps_acquisition_cc::set_satellite(unsigned int satellite)
{
    d_satellite = satellite;
    d_code_phase = 0;
    d_doppler_freq = 0;
    d_mag = 0.0;
    d_input_power = 0.0;

    // Now the GPS codes are generated on the fly using a custom version of the GPS code generator
    code_gen_complex_sampled(d_fft_if->get_inbuf(), satellite, d_fs_in, 0);
    d_fft_if->execute(); // We need the FFT of GPS C/A code
    //Conjugate the local code
    //TODO Optimize it ! try conj()
    for (unsigned int i = 0; i < d_fft_size; i++)
    {
        d_fft_codes[i] = std::complex<float>(
                d_fft_if->get_outbuf()[i].real(),
                -d_fft_if->get_outbuf()[i].imag());
        d_fft_codes[i] = d_fft_codes[i] / (float)d_fft_size; //to correct the scale factor introduced by FFTW
    }
    //memcpy(d_fft_codes,d_fft_if->get_outbuf(),sizeof(gr_complex)*d_samples_per_ms);

    //      std::stringstream filename;
    //      std::streamsize n = 2*sizeof(float)*(d_fft_size); // complex file write
    //      filename.str("");
    //      filename << "./data/code.dat";
    //      std::cout<<filename.str().c_str();
    //      std::cout<<".\n";
    //      d_dump_file.open(filename.str().c_str(), std::ios::out | std::ios::binary);
    //
    //
    //      d_dump_file.write((char*)d_ifft->get_inbuf(), n); //write directly |abs(·)|^2 in this Doppler bin
    //      //d_dump_file.write((char*)d_sine_if, n); //to be read with read_complex_binary() -> test OK
    //      d_dump_file.close();
}
signed int gps_l1_ca_pcps_acquisition_cc::prn_code_phase()
{
    return d_code_phase;
}

int gps_l1_ca_pcps_acquisition_cc::general_work(int noutput_items,
        gr_vector_int &ninput_items, gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
{

    /*!
     * By J.Arribas
     * Acquisition A strategy (Kay Borre book + CFAR threshold):
     * 1. Compute the input signal power estimation
     * 2. Doppler serial search loop
     * 3. Perform the FFT-based circular convolution (parallel time search)
     * 4. Record the maximum peak and the associated synchronization parameters
     * 5. Compute the test statistics and compare to the threshold
     * 6. Declare positive or negative acquisition using a message queue
     */

    if (!d_active)
    {
        d_sample_counter += d_fft_size * noutput_items; // sample counter
        consume_each(noutput_items);
    }
    else
    {
        d_sample_counter += d_fft_size; // sample counter

        //restart acquisition variables

        d_code_phase = 0;
        d_doppler_freq = 0;
        d_mag = 0.0;
        d_input_power = 0.0;

        // initialize acquisition algorithm

        int doppler;
        unsigned int indext = 0;
        float magt = 0.0;
        float tmp_magt = 0.0;
        const gr_complex *in = (const gr_complex *)input_items[0]; //Get the input samples pointer
        bool positive_acquisition = false;
        int acquisition_message = -1; //0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL

        //aux vars
        unsigned int i;
        std::stringstream filename;
        std::streamsize n = 2 * sizeof(float) * (d_fft_size); // complex file write

        LOG_AT_LEVEL(INFO) << "Channel: " << d_channel
                << " , doing acquisition of satellite: " << d_satellite
                << " ,sample stamp: " << d_sample_counter << ", threshold: "
                << d_threshold << ", doppler_max: " << d_doppler_max
                << ", doppler_step: " << d_doppler_step;

        // 1- Compute the input signal power estimation
        //! \TODO try norm()
        for (i = 0; i < d_fft_size; i++)
        {
            d_input_power += std::abs(in[i]) * std::abs(in[i]);
        }

        d_input_power = d_input_power / (float)d_fft_size;

        // 2- Doppler frequency search loop
        for (doppler = (int)(-d_doppler_max); doppler < (int)d_doppler_max; doppler
                += d_doppler_step)
        { // doppler search steps
            //Perform the carrier wipe-off
            sine_gen_complex(d_sine_if, d_freq + doppler, d_fs_in, d_fft_size);
            for (i = 0; i < d_fft_size; i++)
            {
                d_fft_if->get_inbuf()[i] = in[i] * d_sine_if[i];
            }

            //3- Perform the FFT-based circular convolution (parallel time search)
            d_fft_if->execute();

            //TODO Optimize me
            for (i = 0; i < d_fft_size; i++)
            {
                d_ifft->get_inbuf()[i] = (d_fft_if->get_outbuf()[i]
                        * d_fft_codes[i]) / (float)d_fft_size;
            }
            d_ifft->execute();

            // old version
            //x86_gr_complex_mag(d_ifft->get_outbuf(), d_fft_size); // d_ifft->get_outbuf()=|abs(·)|^2 and the array is converted from CPX->Float
            //x86_float_max((float*)d_ifft->get_outbuf(), &indext, &magt,
            //        d_fft_size); // find max of |abs(·)|^2 -> index and magt

            // C++ version
            indext=0;
            magt=0;
            for (i = 0; i < d_fft_size; i++)
            {
				tmp_magt=std::norm(d_ifft->get_outbuf()[i]);
				if (tmp_magt>magt){
					magt=tmp_magt;
					indext=i;
				}
            }

            // Record results to files
            if (d_dump)
            {
                filename.str("");
                filename << "./data/fft_" << doppler << "_.dat";
                std::cout << filename.str().c_str();
                std::cout << ".\n";
                d_dump_file.open(filename.str().c_str(), std::ios::out
                        | std::ios::binary);
                d_dump_file.write((char*)d_ifft->get_outbuf(), n); //write directly |abs(·)|^2 in this Doppler bin
                d_dump_file.close();
            }
            // 4- record the maximum peak and the associated synchronization parameters
            if (d_mag < magt)
            {
                d_mag = magt;
                d_code_phase = indext;
                d_doppler_freq = doppler;
            }
        }

        // 5- Compute the test statistics and compare to the threshold
        d_test_statistics = 2*d_fft_size*d_mag / d_input_power;
        // 6- Declare positive or negative acquisition using a message queue

        if (d_test_statistics > d_threshold)
        { //0.004
            positive_acquisition = true;
            d_acq_sample_stamp = d_sample_counter;
            LOG_AT_LEVEL(INFO) << "positive acquisition";
            LOG_AT_LEVEL(INFO) << "satellite " << d_satellite;
            LOG_AT_LEVEL(INFO) << "sample_stamp " << d_sample_counter;
            LOG_AT_LEVEL(INFO) << "test statistics value "
                    << d_test_statistics;
            LOG_AT_LEVEL(INFO) << "test statistics threshold " << d_threshold;
            LOG_AT_LEVEL(INFO) << "code phase " << d_code_phase;
            LOG_AT_LEVEL(INFO) << "doppler " << d_doppler_freq;
            LOG_AT_LEVEL(INFO) << "magnitude " << d_mag;
            LOG_AT_LEVEL(INFO) << "input signal power " << d_input_power;

        }
        else
        {
            LOG_AT_LEVEL(INFO) << "negative acquisition";
            LOG_AT_LEVEL(INFO) << "satellite " << d_satellite;
            LOG_AT_LEVEL(INFO) << "sample_stamp " << d_sample_counter;
            LOG_AT_LEVEL(INFO) << "test statistics value "
                    << d_test_statistics;
            LOG_AT_LEVEL(INFO) << "test statistics threshold " << d_threshold;
            LOG_AT_LEVEL(INFO) << "code phase " << d_code_phase;
            LOG_AT_LEVEL(INFO) << "doppler " << d_doppler_freq;
            LOG_AT_LEVEL(INFO) << "magnitude " << d_mag;
            LOG_AT_LEVEL(INFO) << "input signal power " << d_input_power;

        }
        d_active = false;

        if (positive_acquisition)
        {
            acquisition_message = 1;
        }
        else
        {
            acquisition_message = 2;
        }

        d_channel_internal_queue->push(acquisition_message);

        consume_each(1);
    }
    return 0;
}
