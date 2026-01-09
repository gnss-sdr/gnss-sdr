/*!
 * \file galileo_pcps_8ms_acquisition_cc.cc
 * \brief This class implements a Parallel Code Phase Search Acquisition for
 * Galileo E1 signals with coherent integration time = 8 ms (two codes)
 * \author Marc Molina, 2013. marc.molina.pena(at)gmail.com
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

#include "galileo_pcps_8ms_acquisition_cc.h"
#include "MATH_CONSTANTS.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <algorithm>
#include <array>
#include <exception>
#include <sstream>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

galileo_pcps_8ms_acquisition_cc_sptr galileo_pcps_8ms_make_acquisition_cc(const Acq_Conf &conf)
{
    return galileo_pcps_8ms_acquisition_cc_sptr(new galileo_pcps_8ms_acquisition_cc(conf));
}


galileo_pcps_8ms_acquisition_cc::galileo_pcps_8ms_acquisition_cc(const Acq_Conf &conf)
    : acquisition_impl_interface("galileo_pcps_8ms_acquisition_cc",
          gr::io_signature::make(1, 1, static_cast<int>(sizeof(gr_complex) * conf.sampled_ms * conf.samples_per_ms)),
          gr::io_signature::make(0, 1, sizeof(Gnss_Synchro))),
      d_acq_params(conf),
      d_gnss_synchro(nullptr),
      d_sample_counter(0ULL),
      d_mag(0),
      d_input_power(0.0),
      d_test_statistics(0),
      d_state(0),
      d_channel(0),
      d_well_count(0),
      d_fft_size(conf.sampled_ms * conf.samples_per_ms),
      d_num_doppler_bins(0),
      d_code_phase(0),
      d_active(false),
      d_fft_if(gnss_fft_fwd_make_unique(d_fft_size)),
      d_ifft(gnss_fft_rev_make_unique(d_fft_size)),
      d_fft_code_A(d_fft_size, lv_cmake(0.0F, 0.0F)),
      d_fft_code_B(d_fft_size, lv_cmake(0.0F, 0.0F)),
      d_magnitude(d_fft_size, 0.0F)
{
    this->message_port_register_out(pmt::mp("events"));

    // Count the number of bins
    for (auto doppler = -d_acq_params.doppler_max; doppler <= d_acq_params.doppler_max; doppler += d_acq_params.doppler_step)
        {
            d_num_doppler_bins++;
        }

    // Create the carrier Doppler wipeoff signals
    d_grid_doppler_wipeoffs = std::vector<std::vector<gr_complex>>(d_num_doppler_bins, std::vector<gr_complex>(d_fft_size));
    for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            int32_t doppler = -d_acq_params.doppler_max + d_acq_params.doppler_step * doppler_index;
            float phase_step_rad = static_cast<float>(TWO_PI) * doppler / static_cast<float>(d_acq_params.fs_in);
            std::array<float, 1> _phase{};
            volk_gnsssdr_s32f_sincos_32fc(d_grid_doppler_wipeoffs[doppler_index].data(), -phase_step_rad, _phase.data(), d_fft_size);
        }
}


galileo_pcps_8ms_acquisition_cc::~galileo_pcps_8ms_acquisition_cc()
{
    try
        {
            if (d_acq_params.dump)
                {
                    d_dump_file.close();
                }
        }
    catch (const std::ofstream::failure &e)
        {
            std::cerr << "Problem closing Acquisition dump file: " << d_acq_params.dump_filename << '\n';
        }
    catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
}


void galileo_pcps_8ms_acquisition_cc::set_local_code(std::complex<float> *code)
{
    // code A: two replicas of a primary code
    std::copy(code, code + d_fft_size, d_fft_if->get_inbuf());

    d_fft_if->execute();  // We need the FFT of local code

    // Conjugate the local code
    volk_32fc_conjugate_32fc(d_fft_code_A.data(), d_fft_if->get_outbuf(), d_fft_size);

    // code B: two replicas of a primary code; the second replica is inverted.
    const auto samples_per_code = static_cast<int32_t>(d_acq_params.samples_per_code);
#if VOLK_EQUAL_OR_GREATER_31
    auto minus_one = gr_complex(-1, 0);
    volk_32fc_s32fc_multiply2_32fc(&(d_fft_if->get_inbuf())[samples_per_code],
        &code[samples_per_code], &minus_one,
        samples_per_code);
#else
    volk_32fc_s32fc_multiply_32fc(&(d_fft_if->get_inbuf())[samples_per_code],
        &code[samples_per_code], gr_complex(-1, 0),
        samples_per_code);
#endif
    d_fft_if->execute();  // We need the FFT of local code

    // Conjugate the local code
    volk_32fc_conjugate_32fc(d_fft_code_B.data(), d_fft_if->get_outbuf(), d_fft_size);
}


int galileo_pcps_8ms_acquisition_cc::general_work(int noutput_items,
    gr_vector_int &ninput_items, gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    int32_t acquisition_message = -1;  // 0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL

    if (!d_active)
        {
            d_sample_counter += static_cast<uint64_t>(d_fft_size) * ninput_items[0];  // sample counter
            consume_each(ninput_items[0]);
            return 0;
        }

    switch (d_state)
        {
        case 0:
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
                break;
            }

        case 1:
            {
                // initialize acquisition algorithm
                int32_t doppler;
                uint32_t indext = 0;
                uint32_t indext_A = 0;
                uint32_t indext_B = 0;
                float magt = 0.0;
                float magt_A = 0.0;
                float magt_B = 0.0;
                const auto *in = reinterpret_cast<const gr_complex *>(input_items[0]);  // Get the input samples pointer
                float fft_normalization_factor = static_cast<float>(d_fft_size) * static_cast<float>(d_fft_size);
                d_input_power = 0.0;
                d_mag = 0.0;

                d_sample_counter += static_cast<uint64_t>(d_fft_size);  // sample counter

                d_well_count++;

                DLOG(INFO) << "Channel: " << d_channel
                           << " , doing acquisition of satellite: " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
                           << " , sample stamp: " << d_sample_counter
                           << ", threshold: " << d_acq_params.threshold
                           << ", doppler_max: " << d_acq_params.doppler_max
                           << ", doppler_step: " << d_acq_params.doppler_step;

                // 1- Compute the input signal power estimation
                volk_32fc_magnitude_squared_32f(d_magnitude.data(), in, d_fft_size);
                volk_32f_accumulator_s32f(&d_input_power, d_magnitude.data(), d_fft_size);
                d_input_power /= static_cast<float>(d_fft_size);

                // 2- Doppler frequency search loop
                for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
                    {
                        // doppler search steps
                        doppler = -d_acq_params.doppler_max + d_acq_params.doppler_step * doppler_index;

                        volk_32fc_x2_multiply_32fc(d_fft_if->get_inbuf(), in,
                            d_grid_doppler_wipeoffs[doppler_index].data(), d_fft_size);

                        // 3- Perform the FFT-based convolution  (parallel time search)
                        // Compute the FFT of the carrier wiped--off incoming signal
                        d_fft_if->execute();

                        // Multiply carrier wiped--off, Fourier transformed incoming signal
                        // with the local FFT'd code A reference using SIMD operations with
                        // VOLK library
                        volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(),
                            d_fft_if->get_outbuf(), d_fft_code_A.data(), d_fft_size);

                        // compute the inverse FFT
                        d_ifft->execute();

                        // Search maximum
                        volk_32fc_magnitude_squared_32f(d_magnitude.data(), d_ifft->get_outbuf(), d_fft_size);
                        volk_gnsssdr_32f_index_max_32u(&indext_A, d_magnitude.data(), d_fft_size);

                        // Normalize the maximum value to correct the scale factor introduced by FFTW
                        magt_A = d_magnitude[indext_A] / (fft_normalization_factor * fft_normalization_factor);

                        // Multiply carrier wiped--off, Fourier transformed incoming signal
                        // with the local FFT'd code B reference using SIMD operations with
                        // VOLK library
                        volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(),
                            d_fft_if->get_outbuf(), d_fft_code_B.data(), d_fft_size);

                        // compute the inverse FFT
                        d_ifft->execute();

                        // Search maximum
                        volk_32fc_magnitude_squared_32f(d_magnitude.data(), d_ifft->get_outbuf(), d_fft_size);
                        volk_gnsssdr_32f_index_max_32u(&indext_B, d_magnitude.data(), d_fft_size);

                        // Normalize the maximum value to correct the scale factor introduced by FFTW
                        magt_B = d_magnitude[indext_B] / (fft_normalization_factor * fft_normalization_factor);

                        // Take the greater magnitude
                        if (magt_A >= magt_B)
                            {
                                magt = magt_A;
                                indext = indext_A;
                            }
                        else
                            {
                                magt = magt_B;
                                indext = indext_B;
                            }

                        // 4- record the maximum peak and the associated synchronization parameters
                        if (d_mag < magt)
                            {
                                d_mag = magt;
                                d_gnss_synchro->Acq_delay_samples = static_cast<double>(indext % static_cast<int32_t>(d_acq_params.samples_per_code));
                                d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
                                d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter;
                                d_gnss_synchro->Acq_doppler_step = d_acq_params.doppler_step;
                            }

                        // Record results to file if required
                        if (d_acq_params.dump)
                            {
                                std::stringstream filename;
                                std::streamsize n = 2 * sizeof(float) * (d_fft_size);  // complex file write
                                filename.str("");
                                filename << "./test_statistics_" << d_gnss_synchro->System
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

                if (d_test_statistics > d_acq_params.threshold)
                    {
                        d_state = 2;  // Positive acquisition
                    }
                else if (d_well_count == d_acq_params.max_dwells)
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
                DLOG(INFO) << "test statistics threshold " << d_acq_params.threshold;
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
                LOG(INFO) << "Successful acquisition in channel " << d_channel
                          << " for satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;

                // Copy and push current Gnss_Synchro to monitor queue
                if (d_acq_params.enable_monitor_output)
                    {
                        auto **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);
                        Gnss_Synchro current_synchro_data = Gnss_Synchro();
                        current_synchro_data = *d_gnss_synchro;
                        *out[0] = std::move(current_synchro_data);
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
                DLOG(INFO) << "test statistics threshold " << d_acq_params.threshold;
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
