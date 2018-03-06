/*!
 * \file galileo_e5a_noncoherent_iq_acquisition_caf_cc.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E5a data and pilot Signals
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \based on work from:
 *          <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          <li> Marc Molina, 2013. marc.molina.pena@gmail.com
 *          </ul>
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

#include "galileo_e5a_noncoherent_iq_acquisition_caf_cc.h"
#include <sstream>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include "control_message_factory.h"

using google::LogMessage;

galileo_e5a_noncoherentIQ_acquisition_caf_cc_sptr galileo_e5a_noncoherentIQ_make_acquisition_caf_cc(
    unsigned int sampled_ms,
    unsigned int max_dwells,
    unsigned int doppler_max, long freq, long fs_in,
    int samples_per_ms, int samples_per_code,
    bool bit_transition_flag,
    bool dump,
    std::string dump_filename,
    bool both_signal_components_,
    int CAF_window_hz_,
    int Zero_padding_)
{
    return galileo_e5a_noncoherentIQ_acquisition_caf_cc_sptr(
        new galileo_e5a_noncoherentIQ_acquisition_caf_cc(sampled_ms, max_dwells, doppler_max, freq, fs_in, samples_per_ms,
            samples_per_code, bit_transition_flag, dump, dump_filename, both_signal_components_, CAF_window_hz_, Zero_padding_));
}

galileo_e5a_noncoherentIQ_acquisition_caf_cc::galileo_e5a_noncoherentIQ_acquisition_caf_cc(
    unsigned int sampled_ms,
    unsigned int max_dwells,
    unsigned int doppler_max, long freq, long fs_in,
    int samples_per_ms, int samples_per_code,
    bool bit_transition_flag,
    bool dump,
    std::string dump_filename,
    bool both_signal_components_,
    int CAF_window_hz_,
    int Zero_padding_) : gr::block("galileo_e5a_noncoherentIQ_acquisition_caf_cc",
                             gr::io_signature::make(1, 1, sizeof(gr_complex)),
                             gr::io_signature::make(0, 0, sizeof(gr_complex)))
{
    this->message_port_register_out(pmt::mp("events"));
    d_sample_counter = 0;  // SAMPLE COUNTER
    d_active = false;
    d_state = 0;
    d_freq = freq;
    d_fs_in = fs_in;
    d_samples_per_ms = samples_per_ms;
    d_samples_per_code = samples_per_code;
    d_max_dwells = max_dwells;
    d_well_count = 0;
    d_doppler_max = doppler_max;
    if (Zero_padding_ > 0)
        {
            d_sampled_ms = 1;
        }
    else
        {
            d_sampled_ms = sampled_ms;
        }
    d_fft_size = sampled_ms * d_samples_per_ms;
    d_mag = 0;
    d_input_power = 0.0;
    d_num_doppler_bins = 0;
    d_bit_transition_flag = bit_transition_flag;
    d_buffer_count = 0;
    d_both_signal_components = both_signal_components_;
    d_CAF_window_hz = CAF_window_hz_;

    d_inbuffer = static_cast<gr_complex *>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_fft_code_I_A = static_cast<gr_complex *>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_magnitudeIA = static_cast<float *>(volk_gnsssdr_malloc(d_fft_size * sizeof(float), volk_gnsssdr_get_alignment()));

    if (d_both_signal_components == true)
        {
            d_fft_code_Q_A = static_cast<gr_complex *>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
            d_magnitudeQA = static_cast<float *>(volk_gnsssdr_malloc(d_fft_size * sizeof(float), volk_gnsssdr_get_alignment()));
        }
    else
        {
            d_fft_code_Q_A = 0;
            d_magnitudeQA = 0;
        }
    // IF COHERENT INTEGRATION TIME > 1
    if (d_sampled_ms > 1)
        {
            d_fft_code_I_B = static_cast<gr_complex *>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
            d_magnitudeIB = static_cast<float *>(volk_gnsssdr_malloc(d_fft_size * sizeof(float), volk_gnsssdr_get_alignment()));
            if (d_both_signal_components == true)
                {
                    d_fft_code_Q_B = static_cast<gr_complex *>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
                    d_magnitudeQB = static_cast<float *>(volk_gnsssdr_malloc(d_fft_size * sizeof(float), volk_gnsssdr_get_alignment()));
                }
            else
                {
                    d_fft_code_Q_B = 0;
                    d_magnitudeQB = 0;
                }
        }
    else
        {
            d_fft_code_I_B = 0;
            d_magnitudeIB = 0;
            d_fft_code_Q_B = 0;
            d_magnitudeQB = 0;
        }

    // Direct FFT
    d_fft_if = new gr::fft::fft_complex(d_fft_size, true);

    // Inverse FFT
    d_ifft = new gr::fft::fft_complex(d_fft_size, false);

    // For dumping samples into a file
    d_dump = dump;
    d_dump_filename = dump_filename;

    d_doppler_resolution = 0;
    d_threshold = 0;
    d_doppler_step = 250;
    d_grid_doppler_wipeoffs = 0;
    d_gnss_synchro = 0;
    d_code_phase = 0;
    d_doppler_freq = 0;
    d_test_statistics = 0;
    d_CAF_vector = 0;
    d_CAF_vector_I = 0;
    d_CAF_vector_Q = 0;
    d_channel = 0;
    d_gr_stream_buffer = 0;
}

galileo_e5a_noncoherentIQ_acquisition_caf_cc::~galileo_e5a_noncoherentIQ_acquisition_caf_cc()
{
    if (d_num_doppler_bins > 0)
        {
            for (unsigned int i = 0; i < d_num_doppler_bins; i++)
                {
                    volk_gnsssdr_free(d_grid_doppler_wipeoffs[i]);
                }
            delete[] d_grid_doppler_wipeoffs;
        }

    volk_gnsssdr_free(d_inbuffer);
    volk_gnsssdr_free(d_fft_code_I_A);
    volk_gnsssdr_free(d_magnitudeIA);
    if (d_both_signal_components == true)
        {
            volk_gnsssdr_free(d_fft_code_Q_A);
            volk_gnsssdr_free(d_magnitudeQA);
        }
    // IF INTEGRATION TIME > 1
    if (d_sampled_ms > 1)
        {
            volk_gnsssdr_free(d_fft_code_I_B);
            volk_gnsssdr_free(d_magnitudeIB);
            if (d_both_signal_components == true)
                {
                    volk_gnsssdr_free(d_fft_code_Q_B);
                    volk_gnsssdr_free(d_magnitudeQB);
                }
        }
    if (d_CAF_window_hz > 0)
        {
            volk_gnsssdr_free(d_CAF_vector);
            volk_gnsssdr_free(d_CAF_vector_I);
            if (d_both_signal_components == true)
                {
                    volk_gnsssdr_free(d_CAF_vector_Q);
                }
        }

    delete d_fft_if;
    delete d_ifft;

    if (d_dump)
        {
            d_dump_file.close();
        }
}


void galileo_e5a_noncoherentIQ_acquisition_caf_cc::set_local_code(std::complex<float> *codeI, std::complex<float> *codeQ)
{
    // DATA SIGNAL
    // Three replicas of data primary code. CODE A: (1,1,1)
    memcpy(d_fft_if->get_inbuf(), codeI, sizeof(gr_complex) * d_fft_size);

    d_fft_if->execute();  // We need the FFT of local code

    //Conjugate the local code
    volk_32fc_conjugate_32fc(d_fft_code_I_A, d_fft_if->get_outbuf(), d_fft_size);

    // SAME FOR PILOT SIGNAL
    if (d_both_signal_components == true)
        {
            // Three replicas of pilot primary code. CODE A: (1,1,1)
            memcpy(d_fft_if->get_inbuf(), codeQ, sizeof(gr_complex) * d_fft_size);

            d_fft_if->execute();  // We need the FFT of local code

            //Conjugate the local code
            volk_32fc_conjugate_32fc(d_fft_code_Q_A, d_fft_if->get_outbuf(), d_fft_size);
        }
    // IF INTEGRATION TIME > 1 code, we need to evaluate the other possible combination
    // Note: max integration time allowed = 3ms (dealt in adapter)
    if (d_sampled_ms > 1)
        {
            // DATA CODE B: First replica is inverted (0,1,1)
            volk_32fc_s32fc_multiply_32fc(&(d_fft_if->get_inbuf())[0],
                &codeI[0], gr_complex(-1, 0),
                d_samples_per_code);

            d_fft_if->execute();  // We need the FFT of local code

            //Conjugate the local code
            volk_32fc_conjugate_32fc(d_fft_code_I_B, d_fft_if->get_outbuf(), d_fft_size);

            if (d_both_signal_components == true)
                {
                    // PILOT CODE B: First replica is inverted (0,1,1)
                    volk_32fc_s32fc_multiply_32fc(&(d_fft_if->get_inbuf())[0],
                        &codeQ[0], gr_complex(-1, 0),
                        d_samples_per_code);
                    d_fft_if->execute();  // We need the FFT of local code

                    //Conjugate the local code
                    volk_32fc_conjugate_32fc(d_fft_code_Q_B, d_fft_if->get_outbuf(), d_fft_size);
                }
        }
}

void galileo_e5a_noncoherentIQ_acquisition_caf_cc::init()
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
    const double GALILEO_TWO_PI = 6.283185307179600;

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
            float phase_step_rad = GALILEO_TWO_PI * (d_freq + doppler) / static_cast<float>(d_fs_in);
            float _phase[1];
            _phase[0] = 0;
            volk_gnsssdr_s32f_sincos_32fc(d_grid_doppler_wipeoffs[doppler_index], -phase_step_rad, _phase, d_fft_size);
        }

    /* CAF Filtering to resolve doppler ambiguity. Phase and quadrature must be processed
     * separately before non-coherent integration */
    //    if (d_CAF_filter)
    if (d_CAF_window_hz > 0)
        {
            d_CAF_vector = static_cast<float *>(volk_gnsssdr_malloc(d_num_doppler_bins * sizeof(float), volk_gnsssdr_get_alignment()));
            d_CAF_vector_I = static_cast<float *>(volk_gnsssdr_malloc(d_num_doppler_bins * sizeof(float), volk_gnsssdr_get_alignment()));
            if (d_both_signal_components == true)
                {
                    d_CAF_vector_Q = static_cast<float *>(volk_gnsssdr_malloc(d_num_doppler_bins * sizeof(float), volk_gnsssdr_get_alignment()));
                }
        }
}


void galileo_e5a_noncoherentIQ_acquisition_caf_cc::set_state(int state)
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


int galileo_e5a_noncoherentIQ_acquisition_caf_cc::general_work(int noutput_items __attribute__((unused)),
    gr_vector_int &ninput_items, gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items __attribute__((unused)))
{
    /*
     * By J.Arribas, L.Esteve, M.Molina and M.Sales
     * Acquisition strategy (Kay Borre book + CFAR threshold):
     * 1. Compute the input signal power estimation
     * 2. Doppler serial search loop
     * 3. Perform the FFT-based circular convolution (parallel time search)
     * 4. OPTIONAL: CAF filter to avoid doppler ambiguity
     * 5. Record the maximum peak and the associated synchronization parameters
     * 6. Compute the test statistics and compare to the threshold
     * 7. Declare positive or negative acquisition using a message port
     */

    int acquisition_message = -1;  //0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL
    /* States:     0 Stop Channel
     *         1 Load the buffer until it reaches fft_size
     *         2 Acquisition algorithm
     *         3 Positive acquisition
     *         4 Negative acquisition
     */
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
                d_sample_counter += ninput_items[0];  // sample counter
                consume_each(ninput_items[0]);

                break;
            }
        case 1:
            {
                const gr_complex *in = reinterpret_cast<const gr_complex *>(input_items[0]);  //Get the input samples pointer
                unsigned int buff_increment;
                if ((ninput_items[0] + d_buffer_count) <= d_fft_size)
                    {
                        buff_increment = ninput_items[0];
                    }
                else
                    {
                        buff_increment = d_fft_size - d_buffer_count;
                    }
                memcpy(&d_inbuffer[d_buffer_count], in, sizeof(gr_complex) * buff_increment);
                // If buffer will be full in next iteration
                if (d_buffer_count >= (d_fft_size - d_gr_stream_buffer))
                    {
                        d_state = 2;
                    }
                d_buffer_count += buff_increment;
                d_sample_counter += buff_increment;  // sample counter
                consume_each(buff_increment);
                break;
            }
        case 2:
            {
                // Fill last part of the buffer and reset counter
                const gr_complex *in = reinterpret_cast<const gr_complex *>(input_items[0]);  //Get the input samples pointer
                if (d_buffer_count < d_fft_size)
                    {
                        memcpy(&d_inbuffer[d_buffer_count], in, sizeof(gr_complex) * (d_fft_size - d_buffer_count));
                    }
                d_sample_counter += (d_fft_size - d_buffer_count);  // sample counter

                // initialize acquisition algorithm
                int doppler;
                uint32_t indext = 0;
                uint32_t indext_IA = 0;
                uint32_t indext_IB = 0;
                uint32_t indext_QA = 0;
                uint32_t indext_QB = 0;
                float magt = 0.0;
                float magt_IA = 0.0;
                float magt_IB = 0.0;
                float magt_QA = 0.0;
                float magt_QB = 0.0;
                float fft_normalization_factor = static_cast<float>(d_fft_size) * static_cast<float>(d_fft_size);
                d_input_power = 0.0;
                d_mag = 0.0;
                d_well_count++;

                DLOG(INFO) << "Channel: " << d_channel
                           << " , doing acquisition of satellite: " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
                           << " ,sample stamp: " << d_sample_counter << ", threshold: "
                           << d_threshold << ", doppler_max: " << d_doppler_max
                           << ", doppler_step: " << d_doppler_step;

                // 1- Compute the input signal power estimation
                volk_32fc_magnitude_squared_32f(d_magnitudeIA, d_inbuffer, d_fft_size);
                volk_32f_accumulator_s32f(&d_input_power, d_magnitudeIA, d_fft_size);
                d_input_power /= static_cast<float>(d_fft_size);

                // 2- Doppler frequency search loop
                for (unsigned int doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
                    {
                        // doppler search steps

                        doppler = -static_cast<int>(d_doppler_max) + d_doppler_step * doppler_index;

                        volk_32fc_x2_multiply_32fc(d_fft_if->get_inbuf(), d_inbuffer,
                            d_grid_doppler_wipeoffs[doppler_index], d_fft_size);

                        // 3- Perform the FFT-based convolution  (parallel time search)
                        // Compute the FFT of the carrier wiped--off incoming signal
                        d_fft_if->execute();

                        // CODE IA
                        // Multiply carrier wiped--off, Fourier transformed incoming signal
                        // with the local FFT'd code reference using SIMD operations with VOLK library
                        volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(),
                            d_fft_if->get_outbuf(), d_fft_code_I_A, d_fft_size);

                        // compute the inverse FFT
                        d_ifft->execute();

                        // Search maximum
                        volk_32fc_magnitude_squared_32f(d_magnitudeIA, d_ifft->get_outbuf(), d_fft_size);
                        volk_gnsssdr_32f_index_max_32u(&indext_IA, d_magnitudeIA, d_fft_size);
                        // Normalize the maximum value to correct the scale factor introduced by FFTW
                        magt_IA = d_magnitudeIA[indext_IA] / (fft_normalization_factor * fft_normalization_factor);

                        if (d_both_signal_components == true)
                            {
                                // REPEAT FOR ALL CODES. CODE_QA
                                volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(),
                                    d_fft_if->get_outbuf(), d_fft_code_Q_A, d_fft_size);
                                d_ifft->execute();
                                volk_32fc_magnitude_squared_32f(d_magnitudeQA, d_ifft->get_outbuf(), d_fft_size);
                                volk_gnsssdr_32f_index_max_32u(&indext_QA, d_magnitudeQA, d_fft_size);
                                magt_QA = d_magnitudeQA[indext_QA] / (fft_normalization_factor * fft_normalization_factor);
                            }
                        if (d_sampled_ms > 1)  // If Integration time > 1 code
                            {
                                // REPEAT FOR ALL CODES. CODE_IB
                                volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(),
                                    d_fft_if->get_outbuf(), d_fft_code_I_B, d_fft_size);
                                d_ifft->execute();
                                volk_32fc_magnitude_squared_32f(d_magnitudeIB, d_ifft->get_outbuf(), d_fft_size);
                                volk_gnsssdr_32f_index_max_32u(&indext_IB, d_magnitudeIB, d_fft_size);
                                magt_IB = d_magnitudeIB[indext_IB] / (fft_normalization_factor * fft_normalization_factor);

                                if (d_both_signal_components == true)
                                    {
                                        // REPEAT FOR ALL CODES. CODE_QB
                                        volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(),
                                            d_fft_if->get_outbuf(), d_fft_code_Q_B, d_fft_size);
                                        d_ifft->execute();
                                        volk_32fc_magnitude_squared_32f(d_magnitudeQB, d_ifft->get_outbuf(), d_fft_size);
                                        volk_gnsssdr_32f_index_max_32u(&indext_QB, d_magnitudeQB, d_fft_size);
                                        magt_QB = d_magnitudeIB[indext_QB] / (fft_normalization_factor * fft_normalization_factor);
                                    }
                            }

                        // Integrate noncoherently the two best combinations (I² + Q²)
                        // and store the result in the I channel.
                        // If CAF filter to resolve doppler ambiguity is needed,
                        // peak is stored before non-coherent integration.
                        if (d_sampled_ms > 1)  // T_integration > 1 code
                            {
                                if (magt_IA >= magt_IB)
                                    {
                                        // if (d_CAF_filter) {d_CAF_vector_I[doppler_index] = magt_IA;}
                                        if (d_CAF_window_hz > 0)
                                            {
                                                d_CAF_vector_I[doppler_index] = d_magnitudeIA[indext_IA];
                                            }
                                        if (d_both_signal_components)
                                            {
                                                // Integrate non-coherently I+Q
                                                if (magt_QA >= magt_QB)
                                                    {
                                                        // if (d_CAF_filter) {d_CAF_vector_Q[doppler_index] = magt_QA;}
                                                        if (d_CAF_window_hz > 0)
                                                            {
                                                                d_CAF_vector_Q[doppler_index] = d_magnitudeQA[indext_QA];
                                                            }
                                                        for (unsigned int i = 0; i < d_fft_size; i++)
                                                            {
                                                                d_magnitudeIA[i] += d_magnitudeQA[i];
                                                            }
                                                    }
                                                else
                                                    {
                                                        // if (d_CAF_filter) {d_CAF_vector_Q[doppler_index] = magt_QB;}
                                                        if (d_CAF_window_hz > 0)
                                                            {
                                                                d_CAF_vector_Q[doppler_index] = d_magnitudeQB[indext_QB];
                                                            }
                                                        for (unsigned int i = 0; i < d_fft_size; i++)
                                                            {
                                                                d_magnitudeIA[i] += d_magnitudeQB[i];
                                                            }
                                                    }
                                            }
                                        volk_gnsssdr_32f_index_max_32u(&indext, d_magnitudeIA, d_fft_size);
                                        magt = d_magnitudeIA[indext] / (fft_normalization_factor * fft_normalization_factor);
                                    }
                                else
                                    {
                                        // if (d_CAF_filter) {d_CAF_vector_I[doppler_index] = magt_IB;}
                                        if (d_CAF_window_hz > 0)
                                            {
                                                d_CAF_vector_I[doppler_index] = d_magnitudeIB[indext_IB];
                                            }
                                        if (d_both_signal_components)
                                            {
                                                // Integrate non-coherently I+Q
                                                if (magt_QA >= magt_QB)
                                                    {
                                                        //if (d_CAF_filter) {d_CAF_vector_Q[doppler_index] = magt_QA;}
                                                        if (d_CAF_window_hz > 0)
                                                            {
                                                                d_CAF_vector_Q[doppler_index] = d_magnitudeQA[indext_QA];
                                                            }
                                                        for (unsigned int i = 0; i < d_fft_size; i++)
                                                            {
                                                                d_magnitudeIB[i] += d_magnitudeQA[i];
                                                            }
                                                    }
                                                else
                                                    {
                                                        // if (d_CAF_filter) {d_CAF_vector_Q[doppler_index] = magt_QB;}
                                                        if (d_CAF_window_hz > 0)
                                                            {
                                                                d_CAF_vector_Q[doppler_index] = d_magnitudeQB[indext_QB];
                                                            }
                                                        for (unsigned int i = 0; i < d_fft_size; i++)
                                                            {
                                                                d_magnitudeIB[i] += d_magnitudeQB[i];
                                                            }
                                                    }
                                            }
                                        volk_gnsssdr_32f_index_max_32u(&indext, d_magnitudeIB, d_fft_size);
                                        magt = d_magnitudeIB[indext] / (fft_normalization_factor * fft_normalization_factor);
                                    }
                            }
                        else
                            {
                                // if (d_CAF_filter) {d_CAF_vector_I[doppler_index] = magt_IA;}
                                if (d_CAF_window_hz > 0)
                                    {
                                        d_CAF_vector_I[doppler_index] = d_magnitudeIA[indext_IA];
                                    }
                                if (d_both_signal_components)
                                    {
                                        // if (d_CAF_filter) {d_CAF_vector_Q[doppler_index] = magt_QA;}
                                        if (d_CAF_window_hz > 0)
                                            {
                                                d_CAF_vector_Q[doppler_index] = d_magnitudeQA[indext_QA];
                                            }
                                        // NON-Coherent integration of only 1 code
                                        for (unsigned int i = 0; i < d_fft_size; i++)
                                            {
                                                d_magnitudeIA[i] += d_magnitudeQA[i];
                                            }
                                    }
                                volk_gnsssdr_32f_index_max_32u(&indext, d_magnitudeIA, d_fft_size);
                                magt = d_magnitudeIA[indext] / (fft_normalization_factor * fft_normalization_factor);
                            }

                        // 4- record the maximum peak and the associated synchronization parameters
                        if (d_mag < magt)
                            {
                                d_mag = magt;
                                // In case that d_bit_transition_flag = true, we compare the potentially
                                // new maximum test statistics (d_mag/d_input_power) with the value in
                                // d_test_statistics. When the second dwell is being processed, the value
                                // of d_mag/d_input_power could be lower than d_test_statistics (i.e,
                                // the maximum test statistics in the previous dwell is greater than
                                // current d_mag/d_input_power). Note that d_test_statistics is not
                                // restarted between consecutive dwells in multidwell operation.
                                if (d_test_statistics < (d_mag / d_input_power) || !d_bit_transition_flag)
                                    {
                                        d_gnss_synchro->Acq_delay_samples = static_cast<double>(indext % d_samples_per_code);
                                        d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
                                        d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter;

                                        // 5- Compute the test statistics and compare to the threshold
                                        d_test_statistics = d_mag / d_input_power;
                                    }
                            }

                        // Record results to file if required
                        if (d_dump)
                            {
                                std::stringstream filename;
                                std::streamsize n = sizeof(float) * (d_fft_size);  // noncomplex file write
                                filename.str("");
                                filename << "../data/test_statistics_E5a_sat_"
                                         << d_gnss_synchro->PRN << "_doppler_" << doppler << ".dat";
                                d_dump_file.open(filename.str().c_str(), std::ios::out | std::ios::binary);
                                if (d_sampled_ms > 1)  // If integration time > 1 code
                                    {
                                        if (magt_IA >= magt_IB)
                                            {
                                                d_dump_file.write(reinterpret_cast<char *>(d_magnitudeIA), n);
                                            }
                                        else
                                            {
                                                d_dump_file.write(reinterpret_cast<char *>(d_magnitudeIB), n);
                                            }
                                    }
                                else
                                    {
                                        d_dump_file.write(reinterpret_cast<char *>(d_magnitudeIA), n);
                                    }
                                d_dump_file.close();
                            }
                    }
                // std::cout << "d_mag " << d_mag << ".d_sample_counter " << d_sample_counter << ". acq delay " << d_gnss_synchro->Acq_delay_samples<< " indext "<< indext << std::endl;
                // 6 OPTIONAL: CAF filter to avoid Doppler ambiguity in bit transition.
                if (d_CAF_window_hz > 0)
                    {
                        int CAF_bins_half;
                        float *accum = static_cast<float *>(volk_gnsssdr_malloc(sizeof(float), volk_gnsssdr_get_alignment()));
                        CAF_bins_half = d_CAF_window_hz / (2 * d_doppler_step);
                        float weighting_factor;
                        weighting_factor = 0.5 / static_cast<float>(CAF_bins_half);
                        // weighting_factor = 0;
                        // std::cout << "weighting_factor " << weighting_factor << std::endl;
                        // Initialize first iterations
                        for (int doppler_index = 0; doppler_index < CAF_bins_half; doppler_index++)
                            {
                                d_CAF_vector[doppler_index] = 0;
                                // volk_32f_accumulator_s32f_a(&d_CAF_vector[doppler_index], d_CAF_vector_I, CAF_bins_half+doppler_index+1);
                                for (int i = 0; i < CAF_bins_half + doppler_index + 1; i++)
                                    {
                                        d_CAF_vector[doppler_index] += d_CAF_vector_I[i] * (1 - weighting_factor * static_cast<unsigned int>((doppler_index - i)));
                                    }
                                // d_CAF_vector[doppler_index] /= CAF_bins_half+doppler_index+1;
                                d_CAF_vector[doppler_index] /= 1 + CAF_bins_half + doppler_index - weighting_factor * CAF_bins_half * (CAF_bins_half + 1) / 2 - weighting_factor * doppler_index * (doppler_index + 1) / 2;  // triangles = [n*(n+1)/2]
                                if (d_both_signal_components)
                                    {
                                        accum[0] = 0;
                                        // volk_32f_accumulator_s32f_a(&accum[0], d_CAF_vector_Q, CAF_bins_half+doppler_index+1);
                                        for (int i = 0; i < CAF_bins_half + doppler_index + 1; i++)
                                            {
                                                accum[0] += d_CAF_vector_Q[i] * (1 - weighting_factor * static_cast<unsigned int>(abs(doppler_index - i)));
                                            }
                                        // accum[0] /= CAF_bins_half+doppler_index+1;
                                        accum[0] /= 1 + CAF_bins_half + doppler_index - weighting_factor * CAF_bins_half * (CAF_bins_half + 1) / 2 - weighting_factor * doppler_index * (doppler_index + 1) / 2;  // triangles = [n*(n+1)/2]
                                        d_CAF_vector[doppler_index] += accum[0];
                                    }
                            }
                        // Body loop
                        for (unsigned int doppler_index = CAF_bins_half; doppler_index < d_num_doppler_bins - CAF_bins_half; doppler_index++)
                            {
                                d_CAF_vector[doppler_index] = 0;
                                // volk_32f_accumulator_s32f_a(&d_CAF_vector[doppler_index], &d_CAF_vector_I[doppler_index-CAF_bins_half], 2*CAF_bins_half+1);
                                for (int i = doppler_index - CAF_bins_half; i < static_cast<int>(doppler_index + CAF_bins_half + 1); i++)
                                    {
                                        d_CAF_vector[doppler_index] += d_CAF_vector_I[i] * (1 - weighting_factor * static_cast<unsigned int>((doppler_index - i)));
                                    }
                                // d_CAF_vector[doppler_index] /= 2*CAF_bins_half+1;
                                d_CAF_vector[doppler_index] /= 1 + 2 * CAF_bins_half - 2 * weighting_factor * CAF_bins_half * (CAF_bins_half + 1) / 2;
                                if (d_both_signal_components)
                                    {
                                        accum[0] = 0;
                                        // volk_32f_accumulator_s32f_a(&accum[0], &d_CAF_vector_Q[doppler_index-CAF_bins_half], 2*CAF_bins_half);
                                        for (int i = doppler_index - CAF_bins_half; i < static_cast<int>(doppler_index + CAF_bins_half + 1); i++)
                                            {
                                                accum[0] += d_CAF_vector_Q[i] * (1 - weighting_factor * static_cast<unsigned int>((doppler_index - i)));
                                            }
                                        // accum[0] /= 2*CAF_bins_half+1;
                                        accum[0] /= 1 + 2 * CAF_bins_half - 2 * weighting_factor * CAF_bins_half * (CAF_bins_half + 1) / 2;
                                        d_CAF_vector[doppler_index] += accum[0];
                                    }
                            }
                        // Final iterations
                        for (int doppler_index = d_num_doppler_bins - CAF_bins_half; doppler_index < static_cast<int>(d_num_doppler_bins); doppler_index++)
                            {
                                d_CAF_vector[doppler_index] = 0;
                                // volk_32f_accumulator_s32f_a(&d_CAF_vector[doppler_index], &d_CAF_vector_I[doppler_index-CAF_bins_half], CAF_bins_half + (d_num_doppler_bins-doppler_index));
                                for (int i = doppler_index - CAF_bins_half; i < static_cast<int>(d_num_doppler_bins); i++)
                                    {
                                        d_CAF_vector[doppler_index] += d_CAF_vector_I[i] * (1 - weighting_factor * (abs(doppler_index - i)));
                                    }
                                // d_CAF_vector[doppler_index] /= CAF_bins_half+(d_num_doppler_bins-doppler_index);
                                d_CAF_vector[doppler_index] /= 1 + CAF_bins_half + (d_num_doppler_bins - doppler_index - 1) - weighting_factor * CAF_bins_half * (CAF_bins_half + 1) / 2 - weighting_factor * (d_num_doppler_bins - doppler_index - 1) * (d_num_doppler_bins - doppler_index) / 2;
                                if (d_both_signal_components)
                                    {
                                        accum[0] = 0;
                                        // volk_32f_accumulator_s32f_a(&accum[0], &d_CAF_vector_Q[doppler_index-CAF_bins_half], CAF_bins_half + (d_num_doppler_bins-doppler_index));
                                        for (int i = doppler_index - CAF_bins_half; i < static_cast<int>(d_num_doppler_bins); i++)
                                            {
                                                accum[0] += d_CAF_vector_Q[i] * (1 - weighting_factor * (abs(doppler_index - i)));
                                            }
                                        // accum[0] /= CAF_bins_half+(d_num_doppler_bins-doppler_index);
                                        accum[0] /= 1 + CAF_bins_half + (d_num_doppler_bins - doppler_index - 1) - weighting_factor * CAF_bins_half * (CAF_bins_half + 1) / 2 - weighting_factor * (d_num_doppler_bins - doppler_index - 1) * (d_num_doppler_bins - doppler_index) / 2;
                                        d_CAF_vector[doppler_index] += accum[0];
                                    }
                            }

                        // Recompute the maximum doppler peak
                        volk_gnsssdr_32f_index_max_32u(&indext, d_CAF_vector, d_num_doppler_bins);
                        doppler = -static_cast<int>(d_doppler_max) + d_doppler_step * indext;
                        d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
                        // Dump if required, appended at the end of the file
                        if (d_dump)
                            {
                                std::stringstream filename;
                                std::streamsize n = sizeof(float) * (d_num_doppler_bins);  // noncomplex file write
                                filename.str("");
                                filename << "../data/test_statistics_E5a_sat_" << d_gnss_synchro->PRN << "_CAF.dat";
                                d_dump_file.open(filename.str().c_str(), std::ios::out | std::ios::binary);
                                d_dump_file.write(reinterpret_cast<char *>(d_CAF_vector), n);
                                d_dump_file.close();
                            }
                        volk_gnsssdr_free(accum);
                    }

                if (d_well_count == d_max_dwells)
                    {
                        if (d_test_statistics > d_threshold)
                            {
                                d_state = 3;  // Positive acquisition
                            }
                        else
                            {
                                d_state = 4;  // Negative acquisition
                            }
                    }
                else
                    {
                        d_state = 1;
                    }

                consume_each(d_fft_size - d_buffer_count);
                d_buffer_count = 0;
                break;
            }
        case 3:
            {
                // 7.1- Declare positive acquisition using a message port
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

                acquisition_message = 1;
                this->message_port_pub(pmt::mp("events"), pmt::from_long(acquisition_message));
                d_sample_counter += ninput_items[0];  // sample counter
                consume_each(ninput_items[0]);
                break;
            }
        case 4:
            {
                // 7.2- Declare negative acquisition using a message port
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

                d_sample_counter += ninput_items[0];  // sample counter
                consume_each(ninput_items[0]);
                acquisition_message = 2;
                this->message_port_pub(pmt::mp("events"), pmt::from_long(acquisition_message));
                break;
            }
        }

    return 0;
}
