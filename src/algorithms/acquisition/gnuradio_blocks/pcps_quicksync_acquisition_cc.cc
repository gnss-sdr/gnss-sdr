/*!
 * \file pcps_quicksync_acquisition_cc.cc
 * \brief This class implements a Parallel Code Phase Search Acquisition
 * \author Damian Miralles Sanchez, 2014. dmiralles2009(at)gmail.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "pcps_quicksync_acquisition_cc.h"
#include "MATH_CONSTANTS.h"
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <array>
#include <cmath>
#include <exception>
#include <sstream>


pcps_quicksync_acquisition_cc_sptr pcps_quicksync_make_acquisition_cc(
    uint32_t folding_factor,
    uint32_t sampled_ms,
    uint32_t max_dwells,
    uint32_t doppler_max,
    int64_t fs_in,
    int32_t samples_per_ms,
    int32_t samples_per_code,
    bool bit_transition_flag,
    bool dump,
    const std::string& dump_filename,
    bool enable_monitor_output)
{
    return pcps_quicksync_acquisition_cc_sptr(
        new pcps_quicksync_acquisition_cc(
            folding_factor,
            sampled_ms, max_dwells, doppler_max,
            fs_in, samples_per_ms,
            samples_per_code,
            bit_transition_flag,
            dump, dump_filename,
            enable_monitor_output));
}


pcps_quicksync_acquisition_cc::pcps_quicksync_acquisition_cc(
    uint32_t folding_factor,
    uint32_t sampled_ms, uint32_t max_dwells,
    uint32_t doppler_max, int64_t fs_in,
    int32_t samples_per_ms, int32_t samples_per_code,
    bool bit_transition_flag,
    bool dump,
    const std::string& dump_filename,
    bool enable_monitor_output)
    : gr::block("pcps_quicksync_acquisition_cc",
          gr::io_signature::make(1, 1, static_cast<int>(sizeof(gr_complex) * sampled_ms * samples_per_ms)),
          gr::io_signature::make(0, 1, sizeof(Gnss_Synchro))),
      d_dump_filename(dump_filename),
      d_gnss_synchro(nullptr),
      d_fs_in(fs_in),
      d_sample_counter(0ULL),
      d_noise_floor_power(0),
      d_threshold(0),
      d_doppler_freq(0),
      d_mag(0),
      d_input_power(0.0),
      d_test_statistics(0),
      d_samples_per_ms(samples_per_ms),
      d_samples_per_code(samples_per_code),
      d_state(0),
      d_channel(0),
      d_folding_factor(folding_factor),
      d_doppler_resolution(0),
      d_doppler_max(doppler_max),
      d_doppler_step(0),
      d_sampled_ms(sampled_ms),
      d_max_dwells(max_dwells),
      d_well_count(0),
      d_fft_size((d_samples_per_code) / d_folding_factor),
      d_num_doppler_bins(0),
      d_code_phase(0),
      d_bit_transition_flag(bit_transition_flag),
      d_active(false),
      d_dump(dump),
      d_enable_monitor_output(enable_monitor_output)
{
    this->message_port_register_out(pmt::mp("events"));

    d_fft_codes = std::vector<gr_complex>(d_fft_size);
    d_magnitude = std::vector<float>(d_samples_per_code * d_folding_factor);
    d_magnitude_folded = std::vector<float>(d_fft_size);
    d_possible_delay = std::vector<uint32_t>(d_folding_factor);
    d_corr_output_f = std::vector<float>(d_folding_factor);

    // Create the d_code vector, which would store the values of the code in its
    // original form to perform later correlation in time domain
    d_code = std::vector<gr_complex>(d_samples_per_code, lv_cmake(0.0F, 0.0F));

    d_fft_if = gnss_fft_fwd_make_unique(d_fft_size);
    d_ifft = gnss_fft_rev_make_unique(d_fft_size);

    d_code_folded = std::vector<gr_complex>(d_fft_size, lv_cmake(0.0F, 0.0F));
    d_signal_folded = std::vector<gr_complex>(d_fft_size);
}


pcps_quicksync_acquisition_cc::~pcps_quicksync_acquisition_cc()
{
    try
        {
            if (d_dump)
                {
                    d_dump_file.close();
                }
        }
    catch (const std::ofstream::failure& e)
        {
            std::cerr << "Problem closing Acquisition dump file: " << d_dump_filename << '\n';
        }
    catch (const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
}


void pcps_quicksync_acquisition_cc::set_local_code(std::complex<float>* code)
{
    /* save a local copy of the code without the folding process to perform corre-
    lation in time in the final steps of the acquisition stage */
    std::copy(code, code + d_samples_per_code, d_code.data());

    std::copy(d_code_folded.data(), d_code_folded.data() + d_fft_size, d_fft_if->get_inbuf());

    /* perform folding of the code by the factorial factor parameter. Notice that
    folding of the code in the time stage would result in a downsampled spectrum
    in the frequency domain after applying the fftw operation */
    for (uint32_t i = 0; i < d_folding_factor; i++)
        {
            std::transform((code + i * d_fft_size), (code + ((i + 1) * d_fft_size)),
                d_fft_if->get_inbuf(), d_fft_if->get_inbuf(),
                std::plus<gr_complex>());
        }

    d_fft_if->execute();  // We need the FFT of local code

    // Conjugate the local code
    volk_32fc_conjugate_32fc(d_fft_codes.data(), d_fft_if->get_outbuf(), d_fft_size);
}


void pcps_quicksync_acquisition_cc::init()
{
    d_gnss_synchro->Flag_valid_acquisition = false;
    d_gnss_synchro->Flag_valid_symbol_output = false;
    d_gnss_synchro->Flag_valid_pseudorange = false;
    d_gnss_synchro->Flag_valid_word = false;
    d_gnss_synchro->Acq_delay_samples = 0.0;
    d_gnss_synchro->Acq_doppler_hz = 0.0;
    d_gnss_synchro->Acq_samplestamp_samples = 0ULL;
    d_gnss_synchro->Acq_doppler_step = 0U;
    d_mag = 0.0;
    d_input_power = 0.0;

    if (d_doppler_step == 0)
        {
            d_doppler_step = 250;
        }

    // Count the number of bins
    d_num_doppler_bins = 0;
    for (auto doppler = static_cast<int32_t>(-d_doppler_max);
         doppler <= static_cast<int32_t>(d_doppler_max);
         doppler += d_doppler_step)
        {
            d_num_doppler_bins++;
        }

    // Create the carrier Doppler wipeoff signals
    d_grid_doppler_wipeoffs = std::vector<std::vector<gr_complex>>(d_num_doppler_bins, std::vector<gr_complex>(d_samples_per_code * d_folding_factor));
    for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            int32_t doppler = -static_cast<int32_t>(d_doppler_max) + d_doppler_step * doppler_index;
            float phase_step_rad = static_cast<float>(TWO_PI) * doppler / static_cast<float>(d_fs_in);
            std::array<float, 1> _phase{};
            volk_gnsssdr_s32f_sincos_32fc(d_grid_doppler_wipeoffs[doppler_index].data(), -phase_step_rad, _phase.data(), d_samples_per_code * d_folding_factor);
        }
}


void pcps_quicksync_acquisition_cc::set_state(int32_t state)
{
    d_state = state;
    if (d_state == 1)
        {
            d_gnss_synchro->Acq_delay_samples = 0.0;
            d_gnss_synchro->Acq_doppler_hz = 0.0;
            d_gnss_synchro->Acq_samplestamp_samples = 0ULL;
            d_gnss_synchro->Acq_doppler_step = 0U;
            d_well_count = 0;
            d_mag = 0.0;
            d_input_power = 0.0;
            d_test_statistics = 0.0;
            d_active = true;
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
    gr_vector_void_star& output_items)
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
    // DLOG(INFO) << "START GENERAL WORK";
    int32_t acquisition_message = -1;  // 0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL
    switch (d_state)
        {
        case 0:
            {
                // DLOG(INFO) << "START CASE 0";
                if (d_active)
                    {
                        // restart acquisition variables
                        d_gnss_synchro->Acq_delay_samples = 0.0;
                        d_gnss_synchro->Acq_doppler_hz = 0.0;
                        d_gnss_synchro->Acq_samplestamp_samples = 0ULL;
                        d_gnss_synchro->Acq_doppler_step = 0U;
                        d_well_count = 0;
                        d_mag = 0.0;
                        d_input_power = 0.0;
                        d_test_statistics = 0.0;

                        d_state = 1;
                    }

                d_sample_counter += static_cast<uint64_t>(d_sampled_ms) * d_samples_per_ms * ninput_items[0];  // sample counter
                consume_each(ninput_items[0]);
                // DLOG(INFO) << "END CASE 0";
                break;
            }

        case 1:
            {
                // initialize acquisition  implementing the QuickSync algorithm
                // DLOG(INFO) << "START CASE 1";
                int32_t doppler;
                uint32_t indext = 0;
                float magt = 0.0;
                const auto* in = reinterpret_cast<const gr_complex*>(input_items[0]);  // Get the input samples pointer

                std::vector<gr_complex> in_temp(d_samples_per_code * d_folding_factor);
                // Create a signal to store a signal of size 1ms, to perform correlation
                // in time. No folding on this data is required
                std::vector<gr_complex> in_1code(d_samples_per_code);

                // Stores the values of the correlation output between the local code
                // and the signal with doppler shift corrected
                std::vector<gr_complex> corr_output(d_samples_per_code);

                // Stores a copy of the folded version of the signal.This is used for
                // the FFT operations in future steps of execution*/
                // gr_complex in_folded[d_fft_size];
                float fft_normalization_factor = static_cast<float>(d_fft_size) * static_cast<float>(d_fft_size);

                d_input_power = 0.0;
                d_mag = 0.0;
                d_test_statistics = 0.0;
                d_noise_floor_power = 0.0;

                d_sample_counter += static_cast<uint64_t>(d_sampled_ms) * d_samples_per_ms;  // sample counter

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


                // 1- Compute the input signal power estimation. This operation is
                // being performed in a signal of size nxp
                volk_32fc_magnitude_squared_32f(d_magnitude.data(), in, d_samples_per_code * d_folding_factor);
                volk_32f_accumulator_s32f(&d_input_power, d_magnitude.data(), d_samples_per_code * d_folding_factor);
                d_input_power /= static_cast<float>(d_samples_per_code * d_folding_factor);

                for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
                    {
                        // Ensure that the signal is going to start with all samples
                        // at zero. This is done to avoid over acumulation when performing
                        // the folding process to be stored in d_fft_if->get_inbuf()
                        d_signal_folded = std::vector<gr_complex>(d_fft_size, lv_cmake(0.0F, 0.0F));
                        std::copy(d_signal_folded.data(), d_signal_folded.data() + d_fft_size, d_fft_if->get_inbuf());

                        // Doppler search steps and then multiplication of the incoming
                        // signal with the doppler wipeoffs to eliminate frequency offset
                        doppler = -static_cast<int32_t>(d_doppler_max) + d_doppler_step * doppler_index;

                        // Perform multiplication of the incoming signal with the
                        // complex exponential vector. This removes the frequency doppler
                        // shift offset
                        volk_32fc_x2_multiply_32fc(in_temp.data(), in,
                            d_grid_doppler_wipeoffs[doppler_index].data(),
                            d_samples_per_code * d_folding_factor);

                        // Perform folding of the carrier wiped-off incoming signal. Since
                        // superlinear method is being used the folding factor in the
                        // incoming raw data signal is of d_folding_factor^2
                        for (int32_t i = 0; i < static_cast<int32_t>(d_folding_factor * d_folding_factor); i++)
                            {
                                std::transform((in_temp.data() + i * d_fft_size),
                                    (in_temp.data() + ((i + 1) * d_fft_size)),
                                    d_fft_if->get_inbuf(),
                                    d_fft_if->get_inbuf(),
                                    std::plus<gr_complex>());
                            }

                        // 3- Perform the FFT-based convolution  (parallel time search)
                        // Compute the FFT of the carrier wiped--off incoming signal
                        d_fft_if->execute();

                        // Multiply carrier wiped--off, Fourier transformed incoming
                        // signal with the local FFT'd code reference using SIMD
                        // operations with VOLK library
                        volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(),
                            d_fft_if->get_outbuf(), d_fft_codes.data(), d_fft_size);

                        // compute the inverse FFT of the aliased signal
                        d_ifft->execute();

                        // Compute the magnitude and get the maximum value with its
                        // index position
                        volk_32fc_magnitude_squared_32f(d_magnitude_folded.data(),
                            d_ifft->get_outbuf(), d_fft_size);

                        // Normalize the maximum value to correct the scale factor
                        // introduced by FFTW
                        volk_gnsssdr_32f_index_max_32u(&indext, d_magnitude_folded.data(), d_fft_size);

                        magt = d_magnitude_folded[indext] / (fft_normalization_factor * fft_normalization_factor);

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
                                        uint32_t detected_delay_samples_folded = 0;
                                        detected_delay_samples_folded = (indext % d_samples_per_code);
                                        std::array<gr_complex, 100> complex_acumulator{};

                                        for (int32_t i = 0; i < static_cast<int32_t>(d_folding_factor); i++)
                                            {
                                                d_possible_delay[i] = detected_delay_samples_folded + (i)*d_fft_size;
                                            }

                                        for (int32_t i = 0; i < static_cast<int32_t>(d_folding_factor); i++)
                                            {
                                                // Copy a signal of 1 code length into suggested buffer.
                                                // The copied signal must have doppler effect corrected
                                                std::copy_n(&in_temp[d_possible_delay[i]], d_samples_per_code, in_1code.data());

                                                // Perform multiplication of the unmodified local
                                                // generated code with the incoming signal with doppler
                                                // effect corrected and accumulates its value. This
                                                // is indeed correlation in time for an specific value
                                                // of a shift
                                                volk_32fc_x2_multiply_32fc(corr_output.data(), in_1code.data(), d_code.data(), d_samples_per_code);

                                                for (int32_t j = 0; j < d_samples_per_code; j++)
                                                    {
                                                        complex_acumulator[i] += (corr_output[j]);
                                                    }
                                            }
                                        // Obtain maximum value of correlation given the possible delay selected
                                        volk_32fc_magnitude_squared_32f(d_corr_output_f.data(), complex_acumulator.data(), d_folding_factor);
                                        volk_gnsssdr_32f_index_max_32u(&indext, d_corr_output_f.data(), d_folding_factor);

                                        // Now save the real code phase in the gnss_syncro block for use in other stages
                                        d_gnss_synchro->Acq_delay_samples = static_cast<double>(d_possible_delay[indext]);
                                        d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
                                        d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter;
                                        d_gnss_synchro->Acq_doppler_step = d_doppler_step;

                                        // 5- Compute the test statistics and compare to the threshold d_test_statistics = 2 * d_fft_size * d_mag / d_input_power;
                                        d_test_statistics = d_mag / d_input_power;
                                    }
                            }

                        // Record results to file if required
                        if (d_dump)
                            {
                                // Since QuickSYnc performs a folded correlation in frequency by means
                                // of the FFT, it is essential to also keep the values obtained from the
                                // possible delay to show how it is maximize
                                std::stringstream filename;
                                std::streamsize n = sizeof(float) * (d_fft_size);  // complex file write
                                filename.str("");
                                filename << "../data/test_statistics_" << d_gnss_synchro->System
                                         << "_" << d_gnss_synchro->Signal[0] << d_gnss_synchro->Signal[1] << "_sat_"
                                         << d_gnss_synchro->PRN << "_doppler_" << doppler << ".dat";
                                d_dump_file.open(filename.str().c_str(), std::ios::out | std::ios::binary);
                                d_dump_file.write(reinterpret_cast<char*>(d_magnitude_folded.data()), n);  // write directly |abs(x)|^2 in this Doppler bin?
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

                consume_each(1);

                break;
            }

        case 2:
            {
                // DLOG(INFO) << "START CASE 2";
                // 6.1- Declare positive acquisition using a message port
                DLOG(INFO) << "positive acquisition";
                DLOG(INFO) << "satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;
                DLOG(INFO) << "sample_stamp " << d_sample_counter;
                DLOG(INFO) << "test statistics value " << d_test_statistics;
                DLOG(INFO) << "test statistics threshold " << d_threshold;
                DLOG(INFO) << "folding factor " << d_folding_factor;
                DLOG(INFO) << "possible delay correlation output";
                for (int32_t i = 0; i < static_cast<int32_t>(d_folding_factor); i++)
                    {
                        DLOG(INFO) << d_possible_delay[i] << "\t\t\t" << d_corr_output_f[i];
                    }
                DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
                DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
                DLOG(INFO) << "magnitude folded " << d_mag;
                DLOG(INFO) << "input signal power " << d_input_power;

                d_active = false;
                d_state = 0;

                d_sample_counter += static_cast<uint64_t>(d_sampled_ms) * d_samples_per_ms * ninput_items[0];  // sample counter
                consume_each(ninput_items[0]);

                acquisition_message = 1;
                this->message_port_pub(pmt::mp("events"), pmt::from_long(acquisition_message));
                // DLOG(INFO) << "END CASE 2";

                // Copy and push current Gnss_Synchro to monitor queue
                if (d_enable_monitor_output)
                    {
                        auto** out = reinterpret_cast<Gnss_Synchro**>(&output_items[0]);
                        Gnss_Synchro current_synchro_data = Gnss_Synchro();
                        current_synchro_data = *d_gnss_synchro;
                        *out[0] = current_synchro_data;
                        noutput_items = 1;  // Number of Gnss_Synchro objects produced
                    }

                break;
            }

        case 3:
            {
                // DLOG(INFO) << "START CASE 3";
                // 6.2- Declare negative acquisition using a message port
                DLOG(INFO) << "negative acquisition";
                DLOG(INFO) << "satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;
                DLOG(INFO) << "sample_stamp " << d_sample_counter;
                DLOG(INFO) << "test statistics value " << d_test_statistics;
                DLOG(INFO) << "test statistics threshold " << d_threshold;
                DLOG(INFO) << "folding factor " << d_folding_factor;
                DLOG(INFO) << "possible delay    corr output";
                for (int32_t i = 0; i < static_cast<int32_t>(d_folding_factor); i++)
                    {
                        DLOG(INFO) << d_possible_delay[i] << "\t\t\t" << d_corr_output_f[i];
                    }
                DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
                DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
                DLOG(INFO) << "magnitude folded " << d_mag;
                DLOG(INFO) << "input signal power " << d_input_power;

                d_active = false;
                d_state = 0;

                d_sample_counter += static_cast<uint64_t>(d_sampled_ms) * d_samples_per_ms * ninput_items[0];  // sample counter
                consume_each(ninput_items[0]);

                acquisition_message = 2;
                this->message_port_pub(pmt::mp("events"), pmt::from_long(acquisition_message));
                // DLOG(INFO) << "END CASE 3";
                break;
            }
        }
    return noutput_items;
}
