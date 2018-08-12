/*!
 * \file pcps_cccwsr_acquisition_cc.cc
 * \brief This class implements a Parallel Code Phase Search acquisition
 *  with Coherent Channel Combining With Sign Recovery scheme.
 * \author Marc Molina, 2013. marc.molina.pena(at)gmail.com
 *
 * D.Borio, C.O'Driscoll, G.Lachapelle, "Coherent, Noncoherent and
 * Differentially Coherent Combining Techniques for Acquisition of
 * New Composite GNSS Signals", IEEE Transactions On Aerospace and
 * Electronic Systems vol. 45 no. 3, July 2009, section IV
 *
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "pcps_cccwsr_acquisition_cc.h"
#include <sstream>
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include "control_message_factory.h"
#include "GPS_L1_CA.h"  //GPS_TWO_PI


using google::LogMessage;

pcps_cccwsr_acquisition_cc_sptr pcps_cccwsr_make_acquisition_cc(
    unsigned int sampled_ms, unsigned int max_dwells,
    unsigned int doppler_max, long fs_in,
    int samples_per_ms, int samples_per_code,
    bool dump, std::string dump_filename)
{
    return pcps_cccwsr_acquisition_cc_sptr(
        new pcps_cccwsr_acquisition_cc(sampled_ms, max_dwells, doppler_max, fs_in,
            samples_per_ms, samples_per_code, dump, dump_filename));
}

pcps_cccwsr_acquisition_cc::pcps_cccwsr_acquisition_cc(
    unsigned int sampled_ms, unsigned int max_dwells,
    unsigned int doppler_max, long fs_in,
    int samples_per_ms, int samples_per_code,
    bool dump,
    std::string dump_filename) : gr::block("pcps_cccwsr_acquisition_cc",
                                     gr::io_signature::make(1, 1, sizeof(gr_complex) * sampled_ms * samples_per_ms),
                                     gr::io_signature::make(0, 0, sizeof(gr_complex) * sampled_ms * samples_per_ms))
{
    this->message_port_register_out(pmt::mp("events"));
    d_sample_counter = 0ULL;  // SAMPLE COUNTER
    d_active = false;
    d_state = 0;
    d_fs_in = fs_in;
    d_samples_per_ms = samples_per_ms;
    d_samples_per_code = samples_per_code;
    d_sampled_ms = sampled_ms;
    d_max_dwells = max_dwells;
    d_well_count = 0;
    d_doppler_max = doppler_max;
    d_fft_size = d_sampled_ms * d_samples_per_ms;
    d_mag = 0;
    d_input_power = 0.0;
    d_num_doppler_bins = 0;

    d_fft_code_data = static_cast<gr_complex *>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_fft_code_pilot = static_cast<gr_complex *>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_data_correlation = static_cast<gr_complex *>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_pilot_correlation = static_cast<gr_complex *>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_correlation_plus = static_cast<gr_complex *>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_correlation_minus = static_cast<gr_complex *>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_magnitude = static_cast<float *>(volk_gnsssdr_malloc(d_fft_size * sizeof(float), volk_gnsssdr_get_alignment()));

    // Direct FFT
    d_fft_if = new gr::fft::fft_complex(d_fft_size, true);

    // Inverse FFT
    d_ifft = new gr::fft::fft_complex(d_fft_size, false);

    // For dumping samples into a file
    d_dump = dump;
    d_dump_filename = dump_filename;

    d_doppler_resolution = 0;
    d_threshold = 0;
    d_doppler_step = 0;
    d_grid_doppler_wipeoffs = 0;
    d_gnss_synchro = 0;
    d_code_phase = 0;
    d_doppler_freq = 0;
    d_test_statistics = 0;
    d_channel = 0;
}

pcps_cccwsr_acquisition_cc::~pcps_cccwsr_acquisition_cc()
{
    if (d_num_doppler_bins > 0)
        {
            for (unsigned int i = 0; i < d_num_doppler_bins; i++)
                {
                    volk_gnsssdr_free(d_grid_doppler_wipeoffs[i]);
                }
            delete[] d_grid_doppler_wipeoffs;
        }

    volk_gnsssdr_free(d_fft_code_data);
    volk_gnsssdr_free(d_fft_code_pilot);
    volk_gnsssdr_free(d_data_correlation);
    volk_gnsssdr_free(d_pilot_correlation);
    volk_gnsssdr_free(d_correlation_plus);
    volk_gnsssdr_free(d_correlation_minus);
    volk_gnsssdr_free(d_magnitude);

    delete d_ifft;
    delete d_fft_if;

    if (d_dump)
        {
            d_dump_file.close();
        }
}

void pcps_cccwsr_acquisition_cc::set_local_code(std::complex<float> *code_data,
    std::complex<float> *code_pilot)
{
    // Data code (E1B)
    memcpy(d_fft_if->get_inbuf(), code_data, sizeof(gr_complex) * d_fft_size);

    d_fft_if->execute();  // We need the FFT of local code

    //Conjugate the local code
    volk_32fc_conjugate_32fc(d_fft_code_data, d_fft_if->get_outbuf(), d_fft_size);

    // Pilot code (E1C)
    memcpy(d_fft_if->get_inbuf(), code_pilot, sizeof(gr_complex) * d_fft_size);

    d_fft_if->execute();  // We need the FFT of local code

    //Conjugate the local code,
    volk_32fc_conjugate_32fc(d_fft_code_pilot, d_fft_if->get_outbuf(), d_fft_size);
}

void pcps_cccwsr_acquisition_cc::init()
{
    d_gnss_synchro->Flag_valid_acquisition = false;
    d_gnss_synchro->Flag_valid_symbol_output = false;
    d_gnss_synchro->Flag_valid_pseudorange = false;
    d_gnss_synchro->Flag_valid_word = false;

    d_gnss_synchro->Acq_delay_samples = 0.0;
    d_gnss_synchro->Acq_doppler_hz = 0.0;
    d_gnss_synchro->Acq_samplestamp_samples = 0;
    d_mag = 0.0;
    d_input_power = 0.0;

    // Count the number of bins
    d_num_doppler_bins = 0;
    for (int doppler = static_cast<int>(-d_doppler_max);
         doppler <= static_cast<int>(d_doppler_max);
         doppler += d_doppler_step)
        {
            d_num_doppler_bins++;
        }

    // Create the carrier Doppler wipeoff signals
    d_grid_doppler_wipeoffs = new gr_complex *[d_num_doppler_bins];
    for (unsigned int doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            d_grid_doppler_wipeoffs[doppler_index] = static_cast<gr_complex *>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));

            int doppler = -static_cast<int>(d_doppler_max) + d_doppler_step * doppler_index;
            float phase_step_rad = GPS_TWO_PI * doppler / static_cast<float>(d_fs_in);
            float _phase[1];
            _phase[0] = 0;
            volk_gnsssdr_s32f_sincos_32fc(d_grid_doppler_wipeoffs[doppler_index], -phase_step_rad, _phase, d_fft_size);
        }
}


void pcps_cccwsr_acquisition_cc::set_state(int state)
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
        }
    else if (d_state == 0)
        {
        }
    else
        {
            LOG(ERROR) << "State can only be set to 0 or 1";
        }
}


int pcps_cccwsr_acquisition_cc::general_work(int noutput_items,
    gr_vector_int &ninput_items, gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items __attribute__((unused)))
{
    int acquisition_message = -1;  //0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL

    switch (d_state)
        {
        case 0:
            {
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

                d_sample_counter += static_cast<uint64_t>(d_fft_size * ninput_items[0]);  // sample counter
                consume_each(ninput_items[0]);

                break;
            }
        case 1:
            {
                // initialize acquisition algorithm
                int doppler;

                uint32_t indext = 0;
                uint32_t indext_plus = 0;
                uint32_t indext_minus = 0;
                float magt = 0.0;
                float magt_plus = 0.0;
                float magt_minus = 0.0;
                const gr_complex *in = reinterpret_cast<const gr_complex *>(input_items[0]);  //Get the input samples pointer
                float fft_normalization_factor = static_cast<float>(d_fft_size) * static_cast<float>(d_fft_size);

                d_sample_counter += static_cast<uint64_t>(d_fft_size);  // sample counter

                d_well_count++;

                DLOG(INFO) << "Channel: " << d_channel
                           << " , doing acquisition of satellite: " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
                           << " ,sample stamp: " << d_sample_counter << ", threshold: "
                           << d_threshold << ", doppler_max: " << d_doppler_max
                           << ", doppler_step: " << d_doppler_step;

                // 1- Compute the input signal power estimation
                volk_32fc_magnitude_squared_32f(d_magnitude, in, d_fft_size);
                volk_32f_accumulator_s32f(&d_input_power, d_magnitude, d_fft_size);
                d_input_power /= static_cast<float>(d_fft_size);

                // 2- Doppler frequency search loop
                for (unsigned int doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
                    {
                        // doppler search steps

                        doppler = -static_cast<int>(d_doppler_max) + d_doppler_step * doppler_index;

                        volk_32fc_x2_multiply_32fc(d_fft_if->get_inbuf(), in,
                            d_grid_doppler_wipeoffs[doppler_index], d_fft_size);

                        // 3- Perform the FFT-based convolution  (parallel time search)
                        // Compute the FFT of the carrier wiped--off incoming signal
                        d_fft_if->execute();

                        // Multiply carrier wiped--off, Fourier transformed incoming signal
                        // with the local FFT'd data code reference (E1B) using SIMD operations
                        // with VOLK library
                        volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(),
                            d_fft_if->get_outbuf(), d_fft_code_data, d_fft_size);

                        // compute the inverse FFT
                        d_ifft->execute();

                        // Copy the result of the correlation between wiped--off signal and data code in
                        // d_data_correlation.
                        memcpy(d_data_correlation, d_ifft->get_outbuf(), sizeof(gr_complex) * d_fft_size);

                        // Multiply carrier wiped--off, Fourier transformed incoming signal
                        // with the local FFT'd pilot code reference (E1C) using SIMD operations
                        // with VOLK library
                        volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(),
                            d_fft_if->get_outbuf(), d_fft_code_pilot, d_fft_size);

                        // Compute the inverse FFT
                        d_ifft->execute();

                        // Copy the result of the correlation between wiped--off signal and pilot code in
                        // d_data_correlation.
                        memcpy(d_pilot_correlation, d_ifft->get_outbuf(), sizeof(gr_complex) * d_fft_size);

                        for (unsigned int i = 0; i < d_fft_size; i++)
                            {
                                d_correlation_plus[i] = std::complex<float>(
                                    d_data_correlation[i].real() - d_pilot_correlation[i].imag(),
                                    d_data_correlation[i].imag() + d_pilot_correlation[i].real());

                                d_correlation_minus[i] = std::complex<float>(
                                    d_data_correlation[i].real() + d_pilot_correlation[i].imag(),
                                    d_data_correlation[i].imag() - d_pilot_correlation[i].real());
                            }

                        volk_32fc_magnitude_squared_32f(d_magnitude, d_correlation_plus, d_fft_size);
                        volk_gnsssdr_32f_index_max_32u(&indext_plus, d_magnitude, d_fft_size);
                        magt_plus = d_magnitude[indext_plus] / (fft_normalization_factor * fft_normalization_factor);

                        volk_32fc_magnitude_squared_32f(d_magnitude, d_correlation_minus, d_fft_size);
                        volk_gnsssdr_32f_index_max_32u(&indext_minus, d_magnitude, d_fft_size);
                        magt_minus = d_magnitude[indext_minus] / (fft_normalization_factor * fft_normalization_factor);

                        if (magt_plus >= magt_minus)
                            {
                                magt = magt_plus;
                                indext = indext_plus;
                            }
                        else
                            {
                                magt = magt_minus;
                                indext = indext_minus;
                            }

                        // 4- record the maximum peak and the associated synchronization parameters
                        if (d_mag < magt)
                            {
                                d_mag = magt;
                                d_gnss_synchro->Acq_delay_samples = static_cast<double>(indext % d_samples_per_code);
                                d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
                                d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter;
                            }

                        // Record results to file if required
                        if (d_dump)
                            {
                                std::stringstream filename;
                                std::streamsize n = 2 * sizeof(float) * (d_fft_size);  // complex file write
                                filename.str("");
                                filename << "../data/test_statistics_" << d_gnss_synchro->System
                                         << "_" << d_gnss_synchro->Signal << "_sat_"
                                         << d_gnss_synchro->PRN << "_doppler_" << doppler << ".dat";
                                d_dump_file.open(filename.str().c_str(), std::ios::out | std::ios::binary);
                                d_dump_file.write(reinterpret_cast<char *>(d_ifft->get_outbuf()), n);  //write directly |abs(x)|^2 in this Doppler bin?
                                d_dump_file.close();
                            }
                    }

                // 5- Compute the test statistics and compare to the threshold
                //d_test_statistics = 2 * d_fft_size * d_mag / d_input_power;
                d_test_statistics = d_mag / d_input_power;

                // 6- Declare positive or negative acquisition using a message port
                if (d_test_statistics > d_threshold)
                    {
                        d_state = 2;  // Positive acquisition
                    }
                else if (d_well_count == d_max_dwells)
                    {
                        d_state = 3;  // Negative acquisition
                    }

                consume_each(1);

                break;
            }

        case 2:
            {
                // 6.1- Declare positive acquisition using a message port
                DLOG(INFO) << "positive acquisition";
                DLOG(INFO) << "satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;
                DLOG(INFO) << "sample_stamp " << d_sample_counter;
                DLOG(INFO) << "test statistics value " << d_test_statistics;
                DLOG(INFO) << "test statistics threshold " << d_threshold;
                DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
                DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
                DLOG(INFO) << "magnitude " << d_mag;
                DLOG(INFO) << "input signal power " << d_input_power;

                d_active = false;
                d_state = 0;

                d_sample_counter += static_cast<uint64_t>(d_fft_size * ninput_items[0]);  // sample counter
                consume_each(ninput_items[0]);

                acquisition_message = 1;
                this->message_port_pub(pmt::mp("events"), pmt::from_long(acquisition_message));

                break;
            }

        case 3:
            {
                // 6.2- Declare negative acquisition using a message port
                DLOG(INFO) << "negative acquisition";
                DLOG(INFO) << "satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;
                DLOG(INFO) << "sample_stamp " << d_sample_counter;
                DLOG(INFO) << "test statistics value " << d_test_statistics;
                DLOG(INFO) << "test statistics threshold " << d_threshold;
                DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
                DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
                DLOG(INFO) << "magnitude " << d_mag;
                DLOG(INFO) << "input signal power " << d_input_power;

                d_active = false;
                d_state = 0;

                d_sample_counter += static_cast<uint64_t>(d_fft_size * ninput_items[0]);  // sample counter
                consume_each(ninput_items[0]);

                acquisition_message = 2;
                this->message_port_pub(pmt::mp("events"), pmt::from_long(acquisition_message));

                break;
            }
        }

    return noutput_items;
}
