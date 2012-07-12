/*!
 * \file pcps_acquisition_cc.cc
 * \brief This class implements a Parallell Code Phase Search Acquisition
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *         Luis Esteve, 2012. luis(at)epsilon-formacion.com
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

#include "pcps_acquisition_cc.h"
#include "gnss_signal_processing.h"
#include "control_message_factory.h"
#include <gnuradio/gr_io_signature.h>
#include <sstream>
#include <glog/log_severity.h>
#include <glog/logging.h>

using google::LogMessage;

pcps_acquisition_cc_sptr pcps_make_acquisition_cc(
        unsigned int sampled_ms, unsigned int doppler_max, long freq,
        long fs_in, int samples_per_ms, gr_msg_queue_sptr queue, bool dump,
        std::string dump_filename)
{

    return pcps_acquisition_cc_sptr(
            new pcps_acquisition_cc(sampled_ms, doppler_max, freq,
                    fs_in, samples_per_ms, queue, dump, dump_filename));
}

pcps_acquisition_cc::pcps_acquisition_cc(
        unsigned int sampled_ms, unsigned int doppler_max, long freq,
        long fs_in, int samples_per_ms, gr_msg_queue_sptr queue, bool dump,
        std::string dump_filename) :
    gr_block("pcps_acquisition_cc", gr_make_io_signature(1, 1,
            sizeof(gr_complex) * sampled_ms *samples_per_ms), gr_make_io_signature(0, 0,
            sizeof(gr_complex) * sampled_ms *samples_per_ms))
{
    d_sample_counter = 0;    // SAMPLE COUNTER
    d_active = false;
    d_queue = queue;
    d_freq = freq;
    d_fs_in = fs_in;
    d_samples_per_ms = samples_per_ms;
    d_sampled_ms = sampled_ms;
    d_doppler_max = doppler_max;
    d_fft_size = d_sampled_ms * d_samples_per_ms;
    d_mag = 0;
    d_input_power = 0.0;

    d_sine_if = new gr_complex[d_fft_size];

    d_fft_codes = (gr_complex*)malloc(sizeof(gr_complex) * d_fft_size);

    // Direct FFT
    d_fft_if = new gri_fft_complex(d_fft_size, true);

    // Inverse FFT
    d_ifft = new gri_fft_complex(d_fft_size, false);

    d_dump = dump;
    d_dump_filename = dump_filename;

}

pcps_acquisition_cc::~pcps_acquisition_cc()
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

void pcps_acquisition_cc::set_local_code(std::complex<float> * code)
{
	memcpy(d_fft_if->get_inbuf(),code,sizeof(gr_complex)*d_fft_size);
}


void pcps_acquisition_cc::init()
{

    d_gnss_synchro->Acq_delay_samples=0.0;
    d_gnss_synchro->Acq_doppler_hz=0.0;
    d_gnss_synchro->Acq_samplestamp_samples=0;


    d_mag = 0.0;
    d_input_power = 0.0;

    d_fft_if->execute(); // We need the FFT of local code

    //Conjugate the local code
    for (unsigned int i = 0; i < d_fft_size; i++)
    {
        d_fft_codes[i] = std::complex<float>(conj(d_fft_if->get_outbuf()[i]));
        d_fft_codes[i] = d_fft_codes[i] / (float)d_fft_size; //to correct the scale factor introduced by FFTW
    }

}



int pcps_acquisition_cc::general_work(int noutput_items,
        gr_vector_int &ninput_items, gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
{

    /*
     * By J.Arribas and L.Esteve
     * Acquisition strategy (Kay Borre book + CFAR threshold):
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
            d_gnss_synchro->Acq_delay_samples=0.0;
            d_gnss_synchro->Acq_doppler_hz=0.0;
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

            DLOG(INFO) << "Channel: " << d_channel
                    << " , doing acquisition of satellite: " << d_gnss_synchro->System << " "<< d_gnss_synchro->PRN
                    << " ,sample stamp: " << d_sample_counter << ", threshold: "
                    << d_threshold << ", doppler_max: " << d_doppler_max
                    << ", doppler_step: " << d_doppler_step;

            // 1- Compute the input signal power estimation
            for (i = 0; i < d_fft_size; i++)
                {
                    d_input_power += std::norm(in[i]);
                }

            d_input_power = d_input_power / (float)d_fft_size;

            // 2- Doppler frequency search loop
            for (doppler = (int)(-d_doppler_max); doppler < (int)d_doppler_max; doppler += d_doppler_step)
                {
                    //doppler search steps
                    //Perform the carrier wipe-off
                    complex_exp_gen(d_sine_if, d_freq + doppler, d_fs_in, d_fft_size);
                    for (i = 0; i < d_fft_size; i++)
                        {
                            d_fft_if->get_inbuf()[i] = in[i] * d_sine_if[i];
                        }

                    //3- Perform the FFT-based circular convolution (parallel time search)
                    d_fft_if->execute();
                    for (i = 0; i < d_fft_size; i++)
                        {
                            d_ifft->get_inbuf()[i] = (d_fft_if->get_outbuf()[i]
                                      * d_fft_codes[i]) / (float)d_fft_size;
                        }
                    d_ifft->execute();

                    // Search maximum
                    indext = 0;
                    magt = 0;
                    for (i = 0; i < d_fft_size; i++)
                        {
                            tmp_magt = std::norm(d_ifft->get_outbuf()[i]);
                            if (tmp_magt > magt)
                                {
                                    magt = tmp_magt;
                                    indext = i;
                                }
                        }

                    // Record results to files
                    if (d_dump)
                        {
                            std::stringstream filename;
                            std::streamsize n = 2 * sizeof(float) * (d_fft_size); // complex file write
                            filename.str("");
                            filename << "../data/fft_galileo_e1_sat_" << d_gnss_synchro->PRN << "_doppler_"<<  doppler << ".dat";
                            //std::cout << filename.str().c_str();
                            //std::cout << ".\n";
                            d_dump_file.open(filename.str().c_str(), std::ios::out
                                    | std::ios::binary);
                            d_dump_file.write((char*)d_ifft->get_outbuf(), n); //write directly |abs(x)|^2 in this Doppler bin?
                            d_dump_file.close();
                        }

                    // 4- record the maximum peak and the associated synchronization parameters
                    if (d_mag < magt)
                        {
                            d_mag = magt;
                            d_gnss_synchro->Acq_delay_samples= (double)indext;
                            d_gnss_synchro->Acq_doppler_hz= (double)doppler;
                            //d_code_phase = indext;
                            //d_doppler_freq = doppler;
                        }
                }

            // 5- Compute the test statistics and compare to the threshold
            d_test_statistics = 2 * d_fft_size * d_mag / d_input_power;


            // 6- Declare positive or negative acquisition using a message queue
            if (d_test_statistics > d_threshold)
                {
                    positive_acquisition = true;
                    d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter;

                    DLOG(INFO) << "positive acquisition";
                    DLOG(INFO) << "satellite " << d_gnss_synchro->System << " "<< d_gnss_synchro->PRN;
                    DLOG(INFO) << "sample_stamp " << d_sample_counter;
                    DLOG(INFO) << "test statistics value " << d_test_statistics;
                    DLOG(INFO) << "test statistics threshold " << d_threshold;
                    DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
                    DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
                    DLOG(INFO) << "magnitude " << d_mag;
                    DLOG(INFO) << "input signal power " << d_input_power;
                }
            else
                {
                    DLOG(INFO) << "negative acquisition";
                    DLOG(INFO) << "satellite " << d_gnss_synchro->System << " "<< d_gnss_synchro->PRN;
                    DLOG(INFO) << "sample_stamp " << d_sample_counter;
                    DLOG(INFO) << "test statistics value " << d_test_statistics;
                    DLOG(INFO) << "test statistics threshold " << d_threshold;
                    DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
                    DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
                    DLOG(INFO) << "magnitude " << d_mag;
                    DLOG(INFO) << "input signal power " << d_input_power;
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
