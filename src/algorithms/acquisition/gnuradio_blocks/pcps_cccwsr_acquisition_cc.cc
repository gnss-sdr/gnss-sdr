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

#include "pcps_cccwsr_acquisition_cc.h"
#include "MATH_CONSTANTS.h"  // TWO_PI
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <algorithm>
#include <array>
#include <exception>
#include <sstream>
#include <utility>


pcps_cccwsr_acquisition_cc_sptr pcps_cccwsr_make_acquisition_cc(
    uint32_t sampled_ms,
    uint32_t max_dwells,
    uint32_t doppler_max,
    int64_t fs_in,
    int32_t samples_per_ms,
    int32_t samples_per_code,
    bool dump,
    const std::string &dump_filename,
    bool enable_monitor_output)
{
    return pcps_cccwsr_acquisition_cc_sptr(
        new pcps_cccwsr_acquisition_cc(sampled_ms, max_dwells, doppler_max, fs_in,
            samples_per_ms, samples_per_code, dump, dump_filename, enable_monitor_output));
}


pcps_cccwsr_acquisition_cc::pcps_cccwsr_acquisition_cc(
    uint32_t sampled_ms,
    uint32_t max_dwells,
    uint32_t doppler_max,
    int64_t fs_in,
    int32_t samples_per_ms,
    int32_t samples_per_code,
    bool dump,
    const std::string &dump_filename,
    bool enable_monitor_output)
    : gr::block("pcps_cccwsr_acquisition_cc",
          gr::io_signature::make(1, 1, static_cast<int>(sizeof(gr_complex) * sampled_ms * samples_per_ms)),
          gr::io_signature::make(0, 1, sizeof(Gnss_Synchro))),
      d_dump_filename(dump_filename),
      d_gnss_synchro(nullptr),
      d_fs_in(fs_in),
      d_sample_counter(0ULL),
      d_threshold(0),
      d_doppler_freq(0),
      d_mag(0),
      d_input_power(0.0),
      d_test_statistics(0),
      d_state(0),
      d_samples_per_ms(samples_per_ms),
      d_samples_per_code(samples_per_code),
      d_doppler_resolution(0),
      d_doppler_max(doppler_max),
      d_doppler_step(0),
      d_sampled_ms(sampled_ms),
      d_max_dwells(max_dwells),
      d_well_count(0),
      d_fft_size(d_sampled_ms * d_samples_per_ms),
      d_num_doppler_bins(0),
      d_code_phase(0),
      d_channel(0),
      d_active(false),
      d_dump(dump),
      d_enable_monitor_output(enable_monitor_output)
{
    this->message_port_register_out(pmt::mp("events"));

    d_fft_code_data = std::vector<gr_complex>(d_fft_size);
    d_fft_code_pilot = std::vector<gr_complex>(d_fft_size);
    d_data_correlation = std::vector<gr_complex>(d_fft_size);
    d_pilot_correlation = std::vector<gr_complex>(d_fft_size);
    d_correlation_plus = std::vector<gr_complex>(d_fft_size);
    d_correlation_minus = std::vector<gr_complex>(d_fft_size);
    d_magnitude = std::vector<float>(d_fft_size);

    d_fft_if = gnss_fft_fwd_make_unique(d_fft_size);
    d_ifft = gnss_fft_rev_make_unique(d_fft_size);
}


pcps_cccwsr_acquisition_cc::~pcps_cccwsr_acquisition_cc()
{
    try
        {
            if (d_dump)
                {
                    d_dump_file.close();
                }
        }
    catch (const std::ofstream::failure &e)
        {
            std::cerr << "Problem closing Acquisition dump file: " << d_dump_filename << '\n';
        }
    catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
}


void pcps_cccwsr_acquisition_cc::set_local_code(std::complex<float> *code_data,
    std::complex<float> *code_pilot)
{
    // Data code (E1B)
    std::copy(code_data, code_data + d_fft_size, d_fft_if->get_inbuf());

    d_fft_if->execute();  // We need the FFT of local code

    // Conjugate the local code
    volk_32fc_conjugate_32fc(d_fft_code_data.data(), d_fft_if->get_outbuf(), d_fft_size);

    // Pilot code (E1C)
    std::copy(code_pilot, code_pilot + d_fft_size, d_fft_if->get_inbuf());

    d_fft_if->execute();  // We need the FFT of local code

    // Conjugate the local code,
    volk_32fc_conjugate_32fc(d_fft_code_pilot.data(), d_fft_if->get_outbuf(), d_fft_size);
}


void pcps_cccwsr_acquisition_cc::init()
{
    d_gnss_synchro->Flag_valid_acquisition = false;
    d_gnss_synchro->Flag_valid_symbol_output = false;
    d_gnss_synchro->Flag_valid_pseudorange = false;
    d_gnss_synchro->Flag_valid_word = false;
    d_gnss_synchro->Acq_doppler_step = 0U;
    d_gnss_synchro->Acq_delay_samples = 0.0;
    d_gnss_synchro->Acq_doppler_hz = 0.0;
    d_gnss_synchro->Acq_samplestamp_samples = 0ULL;
    d_mag = 0.0;
    d_input_power = 0.0;

    // Count the number of bins
    d_num_doppler_bins = 0;
    for (auto doppler = static_cast<int32_t>(-d_doppler_max);
         doppler <= static_cast<int32_t>(d_doppler_max);
         doppler += d_doppler_step)
        {
            d_num_doppler_bins++;
        }

    // Create the carrier Doppler wipeoff signals
    d_grid_doppler_wipeoffs = std::vector<std::vector<gr_complex>>(d_num_doppler_bins, std::vector<gr_complex>(d_fft_size));
    for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            int32_t doppler = -static_cast<int32_t>(d_doppler_max) + d_doppler_step * doppler_index;
            float phase_step_rad = static_cast<float>(TWO_PI) * doppler / static_cast<float>(d_fs_in);
            std::array<float, 1> _phase{};
            volk_gnsssdr_s32f_sincos_32fc(d_grid_doppler_wipeoffs[doppler_index].data(), -phase_step_rad, _phase.data(), d_fft_size);
        }
}


void pcps_cccwsr_acquisition_cc::set_state(int32_t state)
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
    gr_vector_void_star &output_items)
{
    int32_t acquisition_message = -1;  // 0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL

    switch (d_state)
        {
        case 0:
            {
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

                d_sample_counter += static_cast<uint64_t>(d_fft_size) * ninput_items[0];  // sample counter
                consume_each(ninput_items[0]);

                break;
            }
        case 1:
            {
                // initialize acquisition algorithm
                int32_t doppler;

                uint32_t indext = 0;
                uint32_t indext_plus = 0;
                uint32_t indext_minus = 0;
                float magt = 0.0;
                float magt_plus = 0.0;
                float magt_minus = 0.0;
                const auto *in = reinterpret_cast<const gr_complex *>(input_items[0]);  // Get the input samples pointer
                float fft_normalization_factor = static_cast<float>(d_fft_size) * static_cast<float>(d_fft_size);

                d_sample_counter += static_cast<uint64_t>(d_fft_size);  // sample counter

                d_well_count++;

                DLOG(INFO) << "Channel: " << d_channel
                           << " , doing acquisition of satellite: " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
                           << " ,sample stamp: " << d_sample_counter << ", threshold: "
                           << d_threshold << ", doppler_max: " << d_doppler_max
                           << ", doppler_step: " << d_doppler_step;

                // 1- Compute the input signal power estimation
                volk_32fc_magnitude_squared_32f(d_magnitude.data(), in, d_fft_size);
                volk_32f_accumulator_s32f(&d_input_power, d_magnitude.data(), d_fft_size);
                d_input_power /= static_cast<float>(d_fft_size);

                // 2- Doppler frequency search loop
                for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
                    {
                        // doppler search steps
                        doppler = -static_cast<int32_t>(d_doppler_max) + d_doppler_step * doppler_index;

                        volk_32fc_x2_multiply_32fc(d_fft_if->get_inbuf(), in,
                            d_grid_doppler_wipeoffs[doppler_index].data(), d_fft_size);

                        // 3- Perform the FFT-based convolution  (parallel time search)
                        // Compute the FFT of the carrier wiped--off incoming signal
                        d_fft_if->execute();

                        // Multiply carrier wiped--off, Fourier transformed incoming signal
                        // with the local FFT'd data code reference (E1B) using SIMD operations
                        // with VOLK library
                        volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(),
                            d_fft_if->get_outbuf(), d_fft_code_data.data(), d_fft_size);

                        // compute the inverse FFT
                        d_ifft->execute();

                        // Copy the result of the correlation between wiped--off signal and data code in
                        // d_data_correlation.
                        std::copy(d_ifft->get_outbuf(), d_ifft->get_outbuf() + d_fft_size, d_data_correlation.data());

                        // Multiply carrier wiped--off, Fourier transformed incoming signal
                        // with the local FFT'd pilot code reference (E1C) using SIMD operations
                        // with VOLK library
                        volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(),
                            d_fft_if->get_outbuf(), d_fft_code_pilot.data(), d_fft_size);

                        // Compute the inverse FFT
                        d_ifft->execute();

                        // Copy the result of the correlation between wiped--off signal and pilot code in
                        // d_data_correlation.
                        std::copy(d_ifft->get_outbuf(), d_ifft->get_outbuf() + d_fft_size, d_pilot_correlation.data());

                        for (uint32_t i = 0; i < d_fft_size; i++)
                            {
                                d_correlation_plus[i] = std::complex<float>(
                                    d_data_correlation[i].real() - d_pilot_correlation[i].imag(),
                                    d_data_correlation[i].imag() + d_pilot_correlation[i].real());

                                d_correlation_minus[i] = std::complex<float>(
                                    d_data_correlation[i].real() + d_pilot_correlation[i].imag(),
                                    d_data_correlation[i].imag() - d_pilot_correlation[i].real());
                            }

                        volk_32fc_magnitude_squared_32f(d_magnitude.data(), d_correlation_plus.data(), d_fft_size);
                        volk_gnsssdr_32f_index_max_32u(&indext_plus, d_magnitude.data(), d_fft_size);
                        magt_plus = d_magnitude[indext_plus] / (fft_normalization_factor * fft_normalization_factor);

                        volk_32fc_magnitude_squared_32f(d_magnitude.data(), d_correlation_minus.data(), d_fft_size);
                        volk_gnsssdr_32f_index_max_32u(&indext_minus, d_magnitude.data(), d_fft_size);
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
                                d_gnss_synchro->Acq_doppler_step = d_doppler_step;
                            }

                        // Record results to file if required
                        if (d_dump)
                            {
                                std::stringstream filename;
                                std::streamsize n = 2 * sizeof(float) * (d_fft_size);  // complex file write
                                filename.str("");
                                filename << "../data/test_statistics_" << d_gnss_synchro->System
                                         << "_" << d_gnss_synchro->Signal[0] << d_gnss_synchro->Signal[1] << "_sat_"
                                         << d_gnss_synchro->PRN << "_doppler_" << doppler << ".dat";
                                d_dump_file.open(filename.str().c_str(), std::ios::out | std::ios::binary);
                                d_dump_file.write(reinterpret_cast<char *>(d_ifft->get_outbuf()), n);  // write directly |abs(x)|^2 in this Doppler bin?
                                d_dump_file.close();
                            }
                    }

                // 5- Compute the test statistics and compare to the threshold
                // d_test_statistics = 2 * d_fft_size * d_mag / d_input_power;
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

                d_sample_counter += static_cast<uint64_t>(d_fft_size) * ninput_items[0];  // sample counter
                consume_each(ninput_items[0]);

                acquisition_message = 1;
                this->message_port_pub(pmt::mp("events"), pmt::from_long(acquisition_message));

                // Copy and push current Gnss_Synchro to monitor queue
                if (d_enable_monitor_output)
                    {
                        auto **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);
                        Gnss_Synchro current_synchro_data = Gnss_Synchro();
                        current_synchro_data = *d_gnss_synchro;
                        *out[0] = current_synchro_data;
                        noutput_items = 1;  // Number of Gnss_Synchro objects produced
                    }

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

                d_sample_counter += static_cast<uint64_t>(d_fft_size) * ninput_items[0];  // sample counter
                consume_each(ninput_items[0]);

                acquisition_message = 2;
                this->message_port_pub(pmt::mp("events"), pmt::from_long(acquisition_message));

                break;
            }
        }

    return noutput_items;
}
