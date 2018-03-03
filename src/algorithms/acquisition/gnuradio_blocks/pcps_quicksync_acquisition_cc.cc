/*!
 * \file pcps_quicksync_acquisition_cc.cc
 * \brief This class implements a Parallel Code Phase Search Acquisition
 * \author Damian Miralles Sanchez, 2014. dmiralles2009(at)gmail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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

#include "pcps_quicksync_acquisition_cc.h"
#include "control_message_factory.h"
#include "GPS_L1_CA.h"
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <cmath>
#include <sstream>


using google::LogMessage;

pcps_quicksync_acquisition_cc_sptr pcps_quicksync_make_acquisition_cc(
    unsigned int folding_factor,
    unsigned int sampled_ms, unsigned int max_dwells,
    unsigned int doppler_max, long freq, long fs_in,
    int samples_per_ms, int samples_per_code,
    bool bit_transition_flag,
    bool dump,
    std::string dump_filename)
{
    return pcps_quicksync_acquisition_cc_sptr(
        new pcps_quicksync_acquisition_cc(
            folding_factor,
            sampled_ms, max_dwells, doppler_max,
            freq, fs_in, samples_per_ms,
            samples_per_code,
            bit_transition_flag,
            dump, dump_filename));
}


pcps_quicksync_acquisition_cc::pcps_quicksync_acquisition_cc(
    unsigned int folding_factor,
    unsigned int sampled_ms, unsigned int max_dwells,
    unsigned int doppler_max, long freq, long fs_in,
    int samples_per_ms, int samples_per_code,
    bool bit_transition_flag,
    bool dump, std::string dump_filename) : gr::block("pcps_quicksync_acquisition_cc",
                                                gr::io_signature::make(1, 1, (sizeof(gr_complex) * sampled_ms * samples_per_ms)),
                                                gr::io_signature::make(0, 0, (sizeof(gr_complex) * sampled_ms * samples_per_ms)))
{
    this->message_port_register_out(pmt::mp("events"));
    d_sample_counter = 0;  // SAMPLE COUNTER
    d_active = false;
    d_state = 0;
    d_freq = freq;
    d_fs_in = fs_in;
    d_samples_per_ms = samples_per_ms;
    d_samples_per_code = samples_per_code;
    d_sampled_ms = sampled_ms;
    d_max_dwells = max_dwells;
    d_well_count = 0;
    d_doppler_max = doppler_max;
    d_mag = 0;
    d_input_power = 0.0;
    d_num_doppler_bins = 0;
    d_bit_transition_flag = bit_transition_flag;
    d_folding_factor = folding_factor;

    //fft size is reduced.
    d_fft_size = (d_samples_per_code) / d_folding_factor;

    d_fft_codes = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_magnitude = static_cast<float*>(volk_gnsssdr_malloc(d_samples_per_code * d_folding_factor * sizeof(float), volk_gnsssdr_get_alignment()));
    d_magnitude_folded = static_cast<float*>(volk_gnsssdr_malloc(d_fft_size * sizeof(float), volk_gnsssdr_get_alignment()));

    d_possible_delay = new unsigned int[d_folding_factor];
    d_corr_output_f = new float[d_folding_factor];

    /*Create the d_code signal , which would store the values of the code in its
    original form to perform later correlation in time domain*/
    d_code = new gr_complex[d_samples_per_code]();

    // Direct FFT
    d_fft_if = new gr::fft::fft_complex(d_fft_size, true);
    // Inverse FFT
    d_ifft = new gr::fft::fft_complex(d_fft_size, false);

    // For dumping samples into a file
    d_dump = dump;
    d_dump_filename = dump_filename;

    d_corr_acumulator = 0;
    d_signal_folded = 0;
    d_code_folded = new gr_complex[d_fft_size]();
    d_noise_floor_power = 0;
    d_doppler_resolution = 0;
    d_threshold = 0;
    d_doppler_step = 0;
    d_grid_doppler_wipeoffs = 0;
    d_fft_if2 = 0;
    d_gnss_synchro = 0;
    d_code_phase = 0;
    d_doppler_freq = 0;
    d_test_statistics = 0;
    d_channel = 0;
    //d_code_folded = 0;

    // DLOG(INFO) << "END CONSTRUCTOR";
}


pcps_quicksync_acquisition_cc::~pcps_quicksync_acquisition_cc()
{
    //DLOG(INFO) << "START DESTROYER";
    if (d_num_doppler_bins > 0)
        {
            for (unsigned int i = 0; i < d_num_doppler_bins; i++)
                {
                    volk_gnsssdr_free(d_grid_doppler_wipeoffs[i]);
                }
            delete[] d_grid_doppler_wipeoffs;
        }

    volk_gnsssdr_free(d_fft_codes);
    volk_gnsssdr_free(d_magnitude);
    volk_gnsssdr_free(d_magnitude_folded);

    delete d_ifft;
    delete d_fft_if;
    delete d_code;
    delete d_possible_delay;
    delete d_corr_output_f;
    delete[] d_code_folded;

    if (d_dump)
        {
            d_dump_file.close();
        }
    // DLOG(INFO) << "END DESTROYER";
}


void pcps_quicksync_acquisition_cc::set_local_code(std::complex<float>* code)
{
    /*save a local copy of the code without the folding process to perform corre-
    lation in time in the final steps of the acquisition stage*/
    memcpy(d_code, code, sizeof(gr_complex) * d_samples_per_code);

    //d_code_folded = new gr_complex[d_fft_size]();
    memcpy(d_fft_if->get_inbuf(), d_code_folded, sizeof(gr_complex) * (d_fft_size));

    /*perform folding of the code by the factorial factor parameter. Notice that
    folding of the code in the time stage would result in a downsampled spectrum
    in the frequency domain after applying the fftw operation*/
    for (unsigned int i = 0; i < d_folding_factor; i++)
        {
            std::transform((code + i * d_fft_size), (code + ((i + 1) * d_fft_size)),
                d_fft_if->get_inbuf(), d_fft_if->get_inbuf(),
                std::plus<gr_complex>());
        }

    d_fft_if->execute();  // We need the FFT of local code

    //Conjugate the local code
    volk_32fc_conjugate_32fc(d_fft_codes, d_fft_if->get_outbuf(), d_fft_size);
}


void pcps_quicksync_acquisition_cc::init()
{
    d_gnss_synchro->Flag_valid_acquisition = false;
    d_gnss_synchro->Flag_valid_symbol_output = false;
    d_gnss_synchro->Flag_valid_pseudorange = false;
    d_gnss_synchro->Flag_valid_word = false;

    //DLOG(INFO) << "START init";
    d_gnss_synchro->Acq_delay_samples = 0.0;
    d_gnss_synchro->Acq_doppler_hz = 0.0;
    d_gnss_synchro->Acq_samplestamp_samples = 0;
    d_mag = 0.0;
    d_input_power = 0.0;

    if (d_doppler_step == 0) d_doppler_step = 250;

    // Count the number of bins
    d_num_doppler_bins = 0;
    for (int doppler = static_cast<int>(-d_doppler_max);
         doppler <= static_cast<int>(d_doppler_max);
         doppler += d_doppler_step)
        {
            d_num_doppler_bins++;
        }

    // Create the carrier Doppler wipeoff signals
    d_grid_doppler_wipeoffs = new gr_complex*[d_num_doppler_bins];
    for (unsigned int doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            d_grid_doppler_wipeoffs[doppler_index] = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_samples_per_code * d_folding_factor * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
            int doppler = -static_cast<int>(d_doppler_max) + d_doppler_step * doppler_index;
            float phase_step_rad = GPS_TWO_PI * (d_freq + doppler) / static_cast<float>(d_fs_in);
            float _phase[1];
            _phase[0] = 0;
            volk_gnsssdr_s32f_sincos_32fc(d_grid_doppler_wipeoffs[doppler_index], -phase_step_rad, _phase, d_samples_per_code * d_folding_factor);
        }
    // DLOG(INFO) << "end init";
}


void pcps_quicksync_acquisition_cc::set_state(int state)
{
    d_state = state;
    if (d_state == 1)
        {
            d_gnss_synchro->Acq_delay_samples = 0.0;
            d_gnss_synchro->Acq_doppler_hz = 0.0;
            d_gnss_synchro->Acq_samplestamp_samples = 0;
            d_well_count = 0;
            d_mag = 0.0;
            d_input_power = 0.0;
            d_test_statistics = 0.0;
            d_active = 1;
        }
    else if (d_state == 0)
        {
        }
    else
        {
            LOG(ERROR) << "State can only be set to 0 or 1";
        }
}

int pcps_quicksync_acquisition_cc::general_work(int noutput_items,
    gr_vector_int& ninput_items, gr_vector_const_void_star& input_items,
    gr_vector_void_star& output_items __attribute__((unused)))
{
    /*
     * By J.Arribas, L.Esteve and M.Molina
     * Acquisition strategy (Kay Borre book + CFAR threshold):
     * 1. Compute the input signal power estimation
     * 2. Doppler serial search loop
     * 3. Perform the FFT-based circular convolution (parallel time search)
     * 4. Record the maximum peak and the associated synchronization parameters
     * 5. Compute the test statistics and compare to the threshold
     * 6. Declare positive or negative acquisition using a message queue
     */
    //DLOG(INFO) << "START GENERAL WORK";
    int acquisition_message = -1;  //0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL
    //std::cout<<"general_work in quicksync gnuradio block"<<std::endl;
    switch (d_state)
        {
        case 0:
            {
                //DLOG(INFO) << "START CASE 0";
                if (d_active)
                    {
                        //restart acquisition variables
                        d_gnss_synchro->Acq_delay_samples = 0.0;
                        d_gnss_synchro->Acq_doppler_hz = 0.0;
                        d_gnss_synchro->Acq_samplestamp_samples = 0;
                        d_well_count = 0;
                        d_mag = 0.0;
                        d_input_power = 0.0;
                        d_test_statistics = 0.0;

                        d_state = 1;
                    }

                d_sample_counter += d_sampled_ms * d_samples_per_ms * ninput_items[0];  // sample counter
                consume_each(ninput_items[0]);
                //DLOG(INFO) << "END CASE 0";
                break;
            }

        case 1:
            {
                /* initialize acquisition  implementing the QuickSync algorithm*/
                //DLOG(INFO) << "START CASE 1";
                int doppler;
                uint32_t indext = 0;
                float magt = 0.0;
                const gr_complex* in = reinterpret_cast<const gr_complex*>(input_items[0]);  //Get the input samples pointer

                gr_complex* in_temp = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_samples_per_code * d_folding_factor * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
                gr_complex* in_temp_folded = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));

                /*Create a signal to store a signal of size 1ms, to perform correlation
            in time. No folding on this data is required*/
                gr_complex* in_1code = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_samples_per_code * sizeof(gr_complex), volk_gnsssdr_get_alignment()));

                /*Stores the values of the correlation output between the local code
            and the signal with doppler shift corrected */
                gr_complex* corr_output = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_samples_per_code * sizeof(gr_complex), volk_gnsssdr_get_alignment()));

                /*Stores a copy of the folded version of the signal.This is used for
            the FFT operations in future steps of excecution*/
                // gr_complex in_folded[d_fft_size];
                float fft_normalization_factor = static_cast<float>(d_fft_size) * static_cast<float>(d_fft_size);

                d_input_power = 0.0;
                d_mag = 0.0;
                d_test_statistics = 0.0;
                d_noise_floor_power = 0.0;

                d_sample_counter += d_sampled_ms * d_samples_per_ms;  // sample counter

                d_well_count++;

                DLOG(INFO) << "Channel: " << d_channel
                           << " , doing acquisition of satellite: "
                           << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
                           << " ,algorithm: pcps_quicksync_acquisition"
                           << " ,folding factor: " << d_folding_factor
                           << " ,sample stamp: " << d_sample_counter << ", threshold: "
                           << d_threshold << ", doppler_max: " << d_doppler_max
                           << ", doppler_step: " << d_doppler_step << ", Signal Size: "
                           << d_samples_per_code * d_folding_factor;


                /* 1- Compute the input signal power estimation. This operation is
               being performed in a signal of size nxp */
                volk_32fc_magnitude_squared_32f(d_magnitude, in, d_samples_per_code * d_folding_factor);
                volk_32f_accumulator_s32f(&d_input_power, d_magnitude, d_samples_per_code * d_folding_factor);
                d_input_power /= static_cast<float>(d_samples_per_code * d_folding_factor);

                for (unsigned int doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
                    {
                        /*Ensure that the signal is going to start with all samples
                    at zero. This is done to avoid over acumulation when performing
                    the folding process to be stored in d_fft_if->get_inbuf()*/
                        d_signal_folded = new gr_complex[d_fft_size]();
                        memcpy(d_fft_if->get_inbuf(), d_signal_folded, sizeof(gr_complex) * (d_fft_size));

                        /*Doppler search steps and then multiplication of the incoming
                    signal with the doppler wipeoffs to eliminate frequency offset
                     */
                        doppler = -static_cast<int>(d_doppler_max) + d_doppler_step * doppler_index;

                        /*Perform multiplication of the incoming signal with the
                   complex exponential vector. This removes the frequency doppler
                   shift offset*/
                        volk_32fc_x2_multiply_32fc(in_temp, in,
                            d_grid_doppler_wipeoffs[doppler_index],
                            d_samples_per_code * d_folding_factor);

                        /*Perform folding of the carrier wiped-off incoming signal. Since
                   superlinear method is being used the folding factor in the
                   incoming raw data signal is of d_folding_factor^2*/
                        for (int i = 0; i < static_cast<int>(d_folding_factor * d_folding_factor); i++)
                            {
                                std::transform((in_temp + i * d_fft_size),
                                    (in_temp + ((i + 1) * d_fft_size)),
                                    d_fft_if->get_inbuf(),
                                    d_fft_if->get_inbuf(),
                                    std::plus<gr_complex>());
                            }

                        /* 3- Perform the FFT-based convolution  (parallel time search)
                   Compute the FFT of the carrier wiped--off incoming signal*/
                        d_fft_if->execute();

                        /*Multiply carrier wiped--off, Fourier transformed incoming
                    signal with the local FFT'd code reference using SIMD
                    operations with VOLK library*/
                        volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(),
                            d_fft_if->get_outbuf(), d_fft_codes, d_fft_size);

                        /* compute the inverse FFT of the aliased signal*/
                        d_ifft->execute();

                        /* Compute the magnitude and get the maximum value with its
                   index position*/
                        volk_32fc_magnitude_squared_32f(d_magnitude_folded,
                            d_ifft->get_outbuf(), d_fft_size);

                        /* Normalize the maximum value to correct the scale factor
                   introduced by FFTW*/
                        //volk_32f_s32f_multiply_32f_a(d_magnitude_folded,d_magnitude_folded,
                        // (1 / (fft_normalization_factor * fft_normalization_factor)), d_fft_size);
                        volk_gnsssdr_32f_index_max_32u(&indext, d_magnitude_folded, d_fft_size);

                        magt = d_magnitude_folded[indext] / (fft_normalization_factor * fft_normalization_factor);

                        delete[] d_signal_folded;

                        // 4- record the maximum peak and the associated synchronization parameters
                        if (d_mag < magt)
                            {
                                d_mag = magt;

                                /* In case that d_bit_transition_flag = true, we compare the potentially
                            new maximum test statistics (d_mag/d_input_power) with the value in
                            d_test_statistics. When the second dwell is being processed, the value
                            of d_mag/d_input_power could be lower than d_test_statistics (i.e,
                            the maximum test statistics in the previous dwell is greater than
                            current d_mag/d_input_power). Note that d_test_statistics is not
                            restarted between consecutive dwells in multidwell operation.*/
                                if (d_test_statistics < (d_mag / d_input_power) || !d_bit_transition_flag)
                                    {
                                        unsigned int detected_delay_samples_folded = 0;
                                        detected_delay_samples_folded = (indext % d_samples_per_code);
                                        gr_complex complex_acumulator[100];
                                        //gr_complex complex_acumulator[d_folding_factor];

                                        for (int i = 0; i < static_cast<int>(d_folding_factor); i++)
                                            {
                                                d_possible_delay[i] = detected_delay_samples_folded + (i)*d_fft_size;
                                            }

                                        for (int i = 0; i < static_cast<int>(d_folding_factor); i++)
                                            {
                                                /*Copy a signal of 1 code length into suggested buffer.
                                                                      The copied signal must have doppler effect corrected*/
                                                memcpy(in_1code, &in_temp[d_possible_delay[i]],
                                                    sizeof(gr_complex) * (d_samples_per_code));

                                                /*Perform multiplication of the unmodified local
                                                              generated code with the incoming signal with doppler
                                                              effect corrected and accumulates its value. This
                                                              is indeed correlation in time for an specific value
                                                              of a shift*/
                                                volk_32fc_x2_multiply_32fc(corr_output, in_1code, d_code, d_samples_per_code);

                                                for (int j = 0; j < d_samples_per_code; j++)
                                                    {
                                                        complex_acumulator[i] += (corr_output[j]);
                                                    }
                                            }
                                        /*Obtain maximun value of correlation given the possible delay selected */
                                        volk_32fc_magnitude_squared_32f(d_corr_output_f, complex_acumulator, d_folding_factor);
                                        volk_gnsssdr_32f_index_max_32u(&indext, d_corr_output_f, d_folding_factor);

                                        /*Now save the real code phase in the gnss_syncro block for use in other stages*/
                                        d_gnss_synchro->Acq_delay_samples = static_cast<double>(d_possible_delay[indext]);
                                        d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
                                        d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter;

                                        /* 5- Compute the test statistics and compare to the threshold d_test_statistics = 2 * d_fft_size * d_mag / d_input_power;*/
                                        d_test_statistics = d_mag / d_input_power;
                                        //delete complex_acumulator;
                                    }
                            }

                        // Record results to file if required
                        if (d_dump)
                            {
                                /*Since QuickSYnc performs a folded correlation in frequency by means
                            of the FFT, it is esential to also keep the values obtained from the
                            possible delay to show how it is maximize*/
                                std::stringstream filename;
                                std::streamsize n = sizeof(float) * (d_fft_size);  // complex file write
                                filename.str("");
                                filename << "../data/test_statistics_" << d_gnss_synchro->System
                                         << "_" << d_gnss_synchro->Signal << "_sat_"
                                         << d_gnss_synchro->PRN << "_doppler_" << doppler << ".dat";
                                d_dump_file.open(filename.str().c_str(), std::ios::out | std::ios::binary);
                                d_dump_file.write(reinterpret_cast<char*>(d_magnitude_folded), n);  //write directly |abs(x)|^2 in this Doppler bin?
                                d_dump_file.close();
                            }
                    }

                if (!d_bit_transition_flag)
                    {
                        if (d_test_statistics > d_threshold)
                            {
                                d_state = 2;  // Positive acquisition
                            }
                        else if (d_well_count == d_max_dwells)
                            {
                                d_state = 3;  // Negative acquisition
                            }
                    }
                else
                    {
                        if (d_well_count == d_max_dwells)  // d_max_dwells = 2
                            {
                                if (d_test_statistics > d_threshold)
                                    {
                                        d_state = 2;  // Positive acquisition
                                    }
                                else
                                    {
                                        d_state = 3;  // Negative acquisition
                                    }
                            }
                    }

                volk_gnsssdr_free(in_temp);
                volk_gnsssdr_free(in_temp_folded);
                volk_gnsssdr_free(in_1code);
                volk_gnsssdr_free(corr_output);
                consume_each(1);

                break;
            }

        case 2:
            {
                //DLOG(INFO) << "START CASE 2";
                // 6.1- Declare positive acquisition using a message port
                DLOG(INFO) << "positive acquisition";
                DLOG(INFO) << "satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;
                DLOG(INFO) << "sample_stamp " << d_sample_counter;
                DLOG(INFO) << "test statistics value " << d_test_statistics;
                DLOG(INFO) << "test statistics threshold " << d_threshold;
                DLOG(INFO) << "folding factor " << d_folding_factor;
                DLOG(INFO) << "possible delay correlation output";
                for (int i = 0; i < static_cast<int>(d_folding_factor); i++) DLOG(INFO) << d_possible_delay[i] << "\t\t\t" << d_corr_output_f[i];
                DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
                DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
                DLOG(INFO) << "magnitude folded " << d_mag;
                DLOG(INFO) << "input signal power " << d_input_power;

                d_active = false;
                d_state = 0;

                d_sample_counter += d_sampled_ms * d_samples_per_ms * ninput_items[0];  // sample counter
                consume_each(ninput_items[0]);

                acquisition_message = 1;
                this->message_port_pub(pmt::mp("events"), pmt::from_long(acquisition_message));
                //DLOG(INFO) << "END CASE 2";
                break;
            }

        case 3:
            {
                //DLOG(INFO) << "START CASE 3";
                // 6.2- Declare negative acquisition using a message port
                DLOG(INFO) << "negative acquisition";
                DLOG(INFO) << "satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;
                DLOG(INFO) << "sample_stamp " << d_sample_counter;
                DLOG(INFO) << "test statistics value " << d_test_statistics;
                DLOG(INFO) << "test statistics threshold " << d_threshold;
                DLOG(INFO) << "folding factor " << d_folding_factor;
                DLOG(INFO) << "possible delay    corr output";
                for (int i = 0; i < static_cast<int>(d_folding_factor); i++) DLOG(INFO) << d_possible_delay[i] << "\t\t\t" << d_corr_output_f[i];
                DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
                DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
                DLOG(INFO) << "magnitude folded " << d_mag;
                DLOG(INFO) << "input signal power " << d_input_power;

                d_active = false;
                d_state = 0;

                d_sample_counter += d_sampled_ms * d_samples_per_ms * ninput_items[0];  // sample counter
                consume_each(ninput_items[0]);

                acquisition_message = 2;
                this->message_port_pub(pmt::mp("events"), pmt::from_long(acquisition_message));
                //DLOG(INFO) << "END CASE 3";
                break;
            }
        }
    return noutput_items;
}
