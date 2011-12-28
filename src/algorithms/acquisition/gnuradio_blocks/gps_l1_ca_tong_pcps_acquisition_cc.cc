/*!
 * \file gps_l1_ca_tong_pcps_acquisition_cc.cc
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

#include "gps_l1_ca_tong_pcps_acquisition_cc.h"
#include "gps_sdr_signal_processing.h"
#include "control_message_factory.h"
#include "gps_sdr_x86.h"
#include <gnuradio/gr_io_signature.h>
#include <sstream>
#include <glog/log_severity.h>
#include <glog/logging.h>

using google::LogMessage;

gps_l1_ca_tong_pcps_acquisition_cc_sptr gps_l1_ca_tong_pcps_make_acquisition_cc(
        unsigned int sampled_ms, unsigned int doppler_max, long freq,
        long fs_in, int samples_per_ms, gr_msg_queue_sptr queue, bool dump,
        std::string dump_filename)
{

    return gps_l1_ca_tong_pcps_acquisition_cc_sptr(
            new gps_l1_ca_tong_pcps_acquisition_cc(sampled_ms, doppler_max,
                    freq, fs_in, samples_per_ms, queue, dump, dump_filename));
}

gps_l1_ca_tong_pcps_acquisition_cc::gps_l1_ca_tong_pcps_acquisition_cc(
        unsigned int sampled_ms, unsigned int doppler_max, long freq,
        long fs_in, int samples_per_ms, gr_msg_queue_sptr queue, bool dump,
        std::string dump_filename) :
        gr_block("gps_l1_ca_tong_pcps_acquisition_cc", gr_make_io_signature(1, 1,
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

    d_samples = d_sampled_ms * d_samples_per_ms;

    d_doppler_freq = 0.0;
    d_code_phase = 0;
    d_mag = 0.0;
    d_noise_power = 0.0;
    d_fbins = 0;
    d_doppler = 0;
    d_pfa = 0.2;
    d_A = 8;
    d_B = 1;
    d_max_dwells = 15;
    d_K = d_B;

    d_if_sin = new gr_complex[d_samples];

    d_fft_codes = (gr_complex*)malloc(sizeof(gr_complex) * d_samples_per_ms);

    // Direct FFT
    d_fft_if = new gri_fft_complex(d_samples, true);

    // Inverse FFT
    d_ifft = new gri_fft_complex(d_samples, false);

    d_ca_codes = new gr_complex[d_samples];

    d_aux_ca_code = new gr_complex[d_samples];

    //generates a unused PRN code to calculate the noise envelope
    code_gen_complex_sampled(d_aux_ca_code, 33, d_fs_in, 0);

    DLOG(INFO) << "fs in " << d_fs_in;
    DLOG(INFO) << "samples per ms " << d_samples_per_ms;
    DLOG(INFO) << "doppler max " << d_doppler_max;
    DLOG(INFO) << "freq " << d_freq;
    DLOG(INFO) << "satellite " << d_satellite;
    DLOG(INFO) << "sampled_ms " << d_sampled_ms;
    DLOG(INFO) << "Samples_for_processing " << d_samples;
    DLOG(INFO) << "dump filename " << d_dump_filename;
    DLOG(INFO) << "dump " << d_dump;
}

gps_l1_ca_tong_pcps_acquisition_cc::~gps_l1_ca_tong_pcps_acquisition_cc()
{
    delete[] d_if_sin;
    delete[] d_ca_codes;
    delete[] d_aux_ca_code;
    delete d_fft_if;
    delete d_ifft;

    if (d_dump)
        {
            d_dump_file.close();
        }
}

void gps_l1_ca_tong_pcps_acquisition_cc::set_satellite(unsigned int satellite)
{
    d_satellite = satellite;
    d_code_phase = 0;
    d_doppler_freq = 0;
    d_mag = 0.0;
    d_noise_power = 0.0;

    // The GPS codes are generated on the fly using a custom version of the GPS code generator
    //! \TODO In-memory codes instead of generated on the fly
    code_gen_complex_sampled(d_fft_if->get_inbuf(), satellite, d_fs_in, 0);

    d_fft_if->execute(); // We need the FFT of GPS C/A code
    //Conjugate the local code
    //! \TODO Optimize it ! Try conj() or Armadillo
    for (unsigned int i = 0; i < d_samples; i++)
        {
            d_fft_codes[i] = std::complex<float>(
                    d_fft_if->get_outbuf()[i].real(),
                    -d_fft_if->get_outbuf()[i].imag());
        }
}
signed int gps_l1_ca_tong_pcps_acquisition_cc::prn_code_phase()
{
    return d_code_phase;
}

int gps_l1_ca_tong_pcps_acquisition_cc::general_work(int noutput_items,
        gr_vector_int &ninput_items, gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
{

    if (!d_active)
        {
            // sample counter
            d_sample_counter += d_samples * noutput_items;
            consume_each(noutput_items);
        }
    else
        {
            d_sample_counter += d_samples;

            // initialize acquisition algorithm

            bool positive_acquisition = false;
            int acquisition_message = -1; //0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL

            //float noise_envelope = 0.0;
            float vt = 20000;
            //float peak = 0.0;
            float magt = 0.0;
            unsigned int max_freq_step = 2 * (unsigned int)(d_doppler_max
                    / d_doppler_step);
            unsigned int indext = 0;

            // Get the input samples pointer
            const gr_complex *in = (const gr_complex *)input_items[0];

            // aux vars
            std::stringstream filename;
            //unsigned int consume_items = 1;

            // complex file write
            //            std::streamsize n = 2 * sizeof(float) * (d_samples);

            // 1 - Compute the input noise envelope estimation and the threshold vt

            //            sine_gen_complex( d_if_sin, d_freq + doppler, d_fs_in, d_samples );
            //
            //            noise_envelope = calculate_envelope( in, d_aux_ca_code, d_if_sin );
            //            vt = noise_envelope * sqrt( -2 * log( d_pfa ) );


            // 1- Compute the input signal power estimation
            for (unsigned int i = 0; i < d_samples; i++)
                {
                    d_noise_power += std::abs(in[i]);
                }
            d_noise_power = sqrt(d_noise_power / (float)d_samples);

            //2. Perform the carrier wipe-off
            sine_gen_complex(d_if_sin, d_freq + d_doppler, d_fs_in, d_samples);
            for (unsigned int i = 0; i < d_samples; i++)
                {
                    d_fft_if->get_inbuf()[i] = in[i] * d_if_sin[i];
                }

            //3- Perform the FFT-based circular convolution (parallel time search)
            d_fft_if->execute();

            //TODO Optimize me: use Armadillo!
            for (unsigned int i = 0; i < d_samples; i++)
                {
                    d_ifft->get_inbuf()[i] = d_fft_if->get_outbuf()[i]
                                                                    * d_fft_codes[i];
                }

            d_ifft->execute();

            x86_gr_complex_mag(d_ifft->get_outbuf(), d_samples); // d_ifft->get_outbuf()=|abs(·)|^2 and the array is converted from CPX->Float
            x86_float_max((float*)d_ifft->get_outbuf(), &d_indext, &magt,
                    d_samples); // find max of |abs(·)|^2 -> index and magt
            magt = sqrt(magt) / (float)d_samples;
            d_test_statistics = magt / d_noise_power;

            LOG_AT_LEVEL(INFO) << "Channel: " << d_channel
                    << ", doing Tong PCSS acquisition of satellite: "
                    << d_satellite << ", sample stamp: " << d_sample_counter
                    << ", bin_freq " << d_doppler << ", doppler_max: "
                    << d_doppler_max << ", K " << d_K << ", sigma: "
                    << d_noise_power << ", mag: " << d_test_statistics
                    << ", vt: " << vt;

            if ((d_test_statistics > vt) && (indext = d_indext))
                {
                    d_K++;
                    if (d_K == d_A)
                        {
                            d_code_phase = d_indext;
                            positive_acquisition = true;
                            d_doppler_freq = d_doppler;
                            d_acq_sample_stamp = d_sample_counter;
                            LOG_AT_LEVEL(INFO) << "positive acquisition";
                            LOG_AT_LEVEL(INFO) << "satellite " << d_satellite;
                            LOG_AT_LEVEL(INFO) << "sample_stamp " << d_sample_counter;
                            LOG_AT_LEVEL(INFO) << "test statistics value "
                                    << d_test_statistics;
                            LOG_AT_LEVEL(INFO) << "test statistics threshold " << vt;
                            LOG_AT_LEVEL(INFO) << "code phase " << d_code_phase;
                            LOG_AT_LEVEL(INFO) << "doppler " << d_doppler_freq;
                            LOG_AT_LEVEL(INFO) << "magnitude " << magt;
                            LOG_AT_LEVEL(INFO) << "input signal power " << d_noise_power;
                            d_dwells = 0;
                            d_active = false;
                        }
                    else d_dwells++;
                }
            else
                {
                    d_K--;
                    if ((d_K == 0) || (d_dwells > d_max_dwells))
                        {
                            d_K = d_B;
                            d_dwells = 0;
                            d_fbins++;
                            if (d_fbins > max_freq_step)
                                {
                                    d_fbins = 0;
                                    LOG_AT_LEVEL(INFO) << "negative acquisition";
                                    LOG_AT_LEVEL(INFO) << "satellite " << d_satellite;
                                    LOG_AT_LEVEL(INFO) << "sample_stamp" << d_sample_counter;
                                    LOG_AT_LEVEL(INFO) << "test statistics value "
                                            << d_test_statistics;
                                    LOG_AT_LEVEL(INFO) << "test statistics threshold " << vt;
                                    LOG_AT_LEVEL(INFO) << "input signal power "
                                            << d_noise_power;
                                    d_active = false;
                                }
                            else
                                {
                                    d_doppler = d_doppler + pow(-1, d_fbins + 1) * d_fbins
                                            * d_doppler_step;
                                }
                        }
                    else d_dwells++;
                }

            // Record results to files
            //            if( d_dump )
            //                {
            //                    filename.str( "" );
            //                    filename << "./data/fft_" << doppler << "_.dat";
            //                    std::cout << filename.str().c_str();
            //                    std::cout << ".\n";
            //                    d_dump_file.open( filename.str().c_str(), std::ios::out
            //                            | std::ios::binary );
            //                    d_dump_file.write( (char*) d_ifft->get_outbuf(), n ); //write directly |abs(·)|^2 in this Doppler bin
            //                    d_dump_file.close();
            //                }


            if (d_active == false)
                {
                    if (positive_acquisition)
                        {
                            acquisition_message = 1;
                        }
                    else
                        {
                            acquisition_message = 2;
                        }

                    d_channel_internal_queue->push(acquisition_message);
                }

            consume_each(1);
        }
    return 0;
}

float gps_l1_ca_tong_pcps_acquisition_cc::calculate_envelope(
        const gr_complex* _input_signal, std::complex<float>* _local_code,
        std::complex<float>* _local_if_sin)
{
    float mag = 0.0;
    std::complex<float> tmp_cpx = 0.0;
    //std::cout << "tmp_cpx " << tmp_cpx << std::endl;

    for (unsigned int i = 0; i < d_samples; i++)
        {
            tmp_cpx = tmp_cpx + _input_signal[i] * _local_code[i]
                                                               * _local_if_sin[i];
        }
    //std::cout << "tmp_cpx " << tmp_cpx << std::endl;

    mag = abs(tmp_cpx);
    return mag;
}
