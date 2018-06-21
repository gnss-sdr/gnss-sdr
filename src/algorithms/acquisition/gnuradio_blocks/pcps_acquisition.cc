/*!
 * \file pcps_acquisition.cc
 * \brief This class implements a Parallel Code Phase Search Acquisition
 * \authors <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          <li> Marc Molina, 2013. marc.molina.pena@gmail.com
 *          <li> Cillian O'Driscoll, 2017. cillian(at)ieee.org
 *          </ul>
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

#include "pcps_acquisition.h"
#include "GPS_L1_CA.h"         // for GPS_TWO_PI
#include "GLONASS_L1_L2_CA.h"  // for GLONASS_TWO_PI"
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <matio.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <cstring>


using google::LogMessage;

pcps_acquisition_sptr pcps_make_acquisition(const Acq_Conf& conf_)
{
    return pcps_acquisition_sptr(new pcps_acquisition(conf_));
}


pcps_acquisition::pcps_acquisition(const Acq_Conf& conf_) : gr::block("pcps_acquisition",
                                                                gr::io_signature::make(1, 1, conf_.it_size * conf_.sampled_ms * conf_.samples_per_ms * (conf_.bit_transition_flag ? 2 : 1)),
                                                                gr::io_signature::make(0, 0, conf_.it_size * conf_.sampled_ms * conf_.samples_per_ms * (conf_.bit_transition_flag ? 2 : 1)))
{
    this->message_port_register_out(pmt::mp("events"));

    acq_parameters = conf_;
    d_sample_counter = 0;  // SAMPLE COUNTER
    d_active = false;
    d_state = 0;
    d_old_freq = 0;
    d_well_count = 0;
    d_fft_size = acq_parameters.sampled_ms * acq_parameters.samples_per_ms;
    d_mag = 0;
    d_input_power = 0.0;
    d_num_doppler_bins = 0;
    d_threshold = 0.0;
    d_doppler_step = 0;
    d_doppler_center_step_two = 0.0;
    d_test_statistics = 0.0;
    d_channel = 0;
    if (conf_.it_size == sizeof(gr_complex))
        {
            d_cshort = false;
        }
    else
        {
            d_cshort = true;
        }

    // COD:
    // Experimenting with the overlap/save technique for handling bit trannsitions
    // The problem: Circular correlation is asynchronous with the received code.
    // In effect the first code phase used in the correlation is the current
    // estimate of the code phase at the start of the input buffer. If this is 1/2
    // of the code period a bit transition would move all the signal energy into
    // adjacent frequency bands at +/- 1/T where T is the integration time.
    //
    // We can avoid this by doing linear correlation, effectively doubling the
    // size of the input buffer and padding the code with zeros.
    if (acq_parameters.bit_transition_flag)
        {
            d_fft_size *= 2;
            acq_parameters.max_dwells = 1;  //Activation of acq_parameters.bit_transition_flag invalidates the value of acq_parameters.max_dwells
        }

    d_fft_codes = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_magnitude = static_cast<float*>(volk_gnsssdr_malloc(d_fft_size * sizeof(float), volk_gnsssdr_get_alignment()));

    // Direct FFT
    d_fft_if = new gr::fft::fft_complex(d_fft_size, true);

    // Inverse FFT
    d_ifft = new gr::fft::fft_complex(d_fft_size, false);

    d_gnss_synchro = 0;
    d_grid_doppler_wipeoffs = nullptr;
    d_grid_doppler_wipeoffs_step_two = nullptr;
    d_worker_active = false;
    d_data_buffer = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    if (d_cshort)
        {
            d_data_buffer_sc = static_cast<lv_16sc_t*>(volk_gnsssdr_malloc(d_fft_size * sizeof(lv_16sc_t), volk_gnsssdr_get_alignment()));
        }
    else
        {
            d_data_buffer_sc = nullptr;
        }
    grid_ = arma::fmat();
    d_step_two = false;
    d_dump_number = 0;
}


pcps_acquisition::~pcps_acquisition()
{
    if (d_num_doppler_bins > 0)
        {
            for (unsigned int i = 0; i < d_num_doppler_bins; i++)
                {
                    volk_gnsssdr_free(d_grid_doppler_wipeoffs[i]);
                }
            delete[] d_grid_doppler_wipeoffs;
        }
    if (acq_parameters.make_2_steps)
        {
            for (unsigned int i = 0; i < acq_parameters.num_doppler_bins_step2; i++)
                {
                    volk_gnsssdr_free(d_grid_doppler_wipeoffs_step_two[i]);
                }
            delete[] d_grid_doppler_wipeoffs_step_two;
        }
    volk_gnsssdr_free(d_fft_codes);
    volk_gnsssdr_free(d_magnitude);
    delete d_ifft;
    delete d_fft_if;
    volk_gnsssdr_free(d_data_buffer);
    if (d_cshort)
        {
            volk_gnsssdr_free(d_data_buffer_sc);
        }
}


void pcps_acquisition::set_local_code(std::complex<float>* code)
{
    // reset the intermediate frequency
    d_old_freq = 0;
    // This will check if it's fdma, if yes will update the intermediate frequency and the doppler grid
    if (is_fdma())
        {
            update_grid_doppler_wipeoffs();
        }
    // COD
    // Here we want to create a buffer that looks like this:
    // [ 0 0 0 ... 0 c_0 c_1 ... c_L]
    // where c_i is the local code and there are L zeros and L chips
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with work function called by the scheduler
    if (acq_parameters.bit_transition_flag)
        {
            int offset = d_fft_size / 2;
            std::fill_n(d_fft_if->get_inbuf(), offset, gr_complex(0.0, 0.0));
            memcpy(d_fft_if->get_inbuf() + offset, code, sizeof(gr_complex) * offset);
        }
    else
        {
            memcpy(d_fft_if->get_inbuf(), code, sizeof(gr_complex) * d_fft_size);
        }

    d_fft_if->execute();  // We need the FFT of local code
    volk_32fc_conjugate_32fc(d_fft_codes, d_fft_if->get_outbuf(), d_fft_size);
}


bool pcps_acquisition::is_fdma()
{
    // Dealing with FDMA system
    if (strcmp(d_gnss_synchro->Signal, "1G") == 0)
        {
            d_old_freq += DFRQ1_GLO * GLONASS_PRN.at(d_gnss_synchro->PRN);
            LOG(INFO) << "Trying to acquire SV PRN " << d_gnss_synchro->PRN << " with freq " << d_old_freq << " in Glonass Channel " << GLONASS_PRN.at(d_gnss_synchro->PRN) << std::endl;
            return true;
        }
    else if (strcmp(d_gnss_synchro->Signal, "2G") == 0)
        {
            d_old_freq += DFRQ2_GLO * GLONASS_PRN.at(d_gnss_synchro->PRN);
            LOG(INFO) << "Trying to acquire SV PRN " << d_gnss_synchro->PRN << " with freq " << d_old_freq << " in Glonass Channel " << GLONASS_PRN.at(d_gnss_synchro->PRN) << std::endl;
            return true;
        }
    else
        {
            return false;
        }
}


void pcps_acquisition::update_local_carrier(gr_complex* carrier_vector, int correlator_length_samples, float freq)
{
    float phase_step_rad = GPS_TWO_PI * freq / static_cast<float>(acq_parameters.fs_in);
    float _phase[1];
    _phase[0] = 0;
    volk_gnsssdr_s32f_sincos_32fc(carrier_vector, -phase_step_rad, _phase, correlator_length_samples);
}


void pcps_acquisition::init()
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

    d_num_doppler_bins = static_cast<unsigned int>(std::ceil(static_cast<double>(static_cast<int>(acq_parameters.doppler_max) - static_cast<int>(-acq_parameters.doppler_max)) / static_cast<double>(d_doppler_step)));

    // Create the carrier Doppler wipeoff signals
    d_grid_doppler_wipeoffs = new gr_complex*[d_num_doppler_bins];
    if (acq_parameters.make_2_steps)
        {
            d_grid_doppler_wipeoffs_step_two = new gr_complex*[acq_parameters.num_doppler_bins_step2];
            for (unsigned int doppler_index = 0; doppler_index < acq_parameters.num_doppler_bins_step2; doppler_index++)
                {
                    d_grid_doppler_wipeoffs_step_two[doppler_index] = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
                }
        }
    for (unsigned int doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            d_grid_doppler_wipeoffs[doppler_index] = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
            int doppler = -static_cast<int>(acq_parameters.doppler_max) + d_doppler_step * doppler_index;
            update_local_carrier(d_grid_doppler_wipeoffs[doppler_index], d_fft_size, d_old_freq + doppler);
        }
    d_worker_active = false;

    if (acq_parameters.dump)
        {
            unsigned int effective_fft_size = (acq_parameters.bit_transition_flag ? (d_fft_size / 2) : d_fft_size);
            grid_ = arma::fmat(effective_fft_size, d_num_doppler_bins, arma::fill::zeros);
        }
}


void pcps_acquisition::update_grid_doppler_wipeoffs()
{
    for (unsigned int doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            int doppler = -static_cast<int>(acq_parameters.doppler_max) + d_doppler_step * doppler_index;
            update_local_carrier(d_grid_doppler_wipeoffs[doppler_index], d_fft_size, d_old_freq + doppler);
        }
}

void pcps_acquisition::update_grid_doppler_wipeoffs_step2()
{
    for (unsigned int doppler_index = 0; doppler_index < acq_parameters.num_doppler_bins_step2; doppler_index++)
        {
            float doppler = (static_cast<float>(doppler_index) - static_cast<float>(acq_parameters.num_doppler_bins_step2) / 2.0) * acq_parameters.doppler_step2;
            update_local_carrier(d_grid_doppler_wipeoffs_step_two[doppler_index], d_fft_size, d_doppler_center_step_two + doppler);
        }
}

void pcps_acquisition::set_state(int state)
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with work function called by the scheduler
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


void pcps_acquisition::send_positive_acquisition()
{
    // 6.1- Declare positive acquisition using a message port
    //0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL
    DLOG(INFO) << "positive acquisition"
               << ", satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
               << ", sample_stamp " << d_sample_counter
               << ", test statistics value " << d_test_statistics
               << ", test statistics threshold " << d_threshold
               << ", code phase " << d_gnss_synchro->Acq_delay_samples
               << ", doppler " << d_gnss_synchro->Acq_doppler_hz
               << ", magnitude " << d_mag
               << ", input signal power " << d_input_power;

    this->message_port_pub(pmt::mp("events"), pmt::from_long(1));
}


void pcps_acquisition::send_negative_acquisition()
{
    // 6.2- Declare negative acquisition using a message port
    //0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL
    DLOG(INFO) << "negative acquisition"
               << ", satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
               << ", sample_stamp " << d_sample_counter
               << ", test statistics value " << d_test_statistics
               << ", test statistics threshold " << d_threshold
               << ", code phase " << d_gnss_synchro->Acq_delay_samples
               << ", doppler " << d_gnss_synchro->Acq_doppler_hz
               << ", magnitude " << d_mag
               << ", input signal power " << d_input_power;

    this->message_port_pub(pmt::mp("events"), pmt::from_long(2));
}


void pcps_acquisition::dump_results(int effective_fft_size)
{
    d_dump_number++;
    std::string filename = acq_parameters.dump_filename;
    filename.append("_");
    filename.append(1, d_gnss_synchro->System);
    filename.append("_");
    filename.append(1, d_gnss_synchro->Signal[0]);
    filename.append(1, d_gnss_synchro->Signal[1]);
    filename.append("_ch_");
    filename.append(std::to_string(d_channel));
    filename.append("_");
    filename.append(std::to_string(d_dump_number));
    filename.append("_sat_");
    filename.append(std::to_string(d_gnss_synchro->PRN));
    filename.append(".mat");

    mat_t* matfp = Mat_CreateVer(filename.c_str(), NULL, MAT_FT_MAT73);
    if (matfp == NULL)
        {
            std::cout << "Unable to create or open Acquisition dump file" << std::endl;
            acq_parameters.dump = false;
        }
    else
        {
            size_t dims[2] = {static_cast<size_t>(effective_fft_size), static_cast<size_t>(d_num_doppler_bins)};
            matvar_t* matvar = Mat_VarCreate("grid", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, grid_.memptr(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            dims[0] = static_cast<size_t>(1);
            dims[1] = static_cast<size_t>(1);
            matvar = Mat_VarCreate("doppler_max", MAT_C_UINT32, MAT_T_UINT32, 1, dims, &acq_parameters.doppler_max, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("doppler_step", MAT_C_UINT32, MAT_T_UINT32, 1, dims, &d_doppler_step, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            Mat_Close(matfp);
        }
}


void pcps_acquisition::acquisition_core(unsigned long int samp_count)
{
    gr::thread::scoped_lock lk(d_setlock);

    // initialize acquisition algorithm
    uint32_t indext = 0;
    float magt = 0.0;
    const gr_complex* in = d_data_buffer;  // Get the input samples pointer
    int effective_fft_size = (acq_parameters.bit_transition_flag ? d_fft_size / 2 : d_fft_size);
    if (d_cshort)
        {
            volk_gnsssdr_16ic_convert_32fc(d_data_buffer, d_data_buffer_sc, d_fft_size);
        }
    float fft_normalization_factor = static_cast<float>(d_fft_size) * static_cast<float>(d_fft_size);

    d_input_power = 0.0;
    d_mag = 0.0;
    d_well_count++;

    DLOG(INFO) << "Channel: " << d_channel
               << " , doing acquisition of satellite: " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
               << " ,sample stamp: " << samp_count << ", threshold: "
               << d_threshold << ", doppler_max: " << acq_parameters.doppler_max
               << ", doppler_step: " << d_doppler_step
               << ", use_CFAR_algorithm_flag: " << (acq_parameters.use_CFAR_algorithm_flag ? "true" : "false");

    lk.unlock();
    if (acq_parameters.use_CFAR_algorithm_flag)
        {
            // 1- (optional) Compute the input signal power estimation
            volk_32fc_magnitude_squared_32f(d_magnitude, in, d_fft_size);
            volk_32f_accumulator_s32f(&d_input_power, d_magnitude, d_fft_size);
            d_input_power /= static_cast<float>(d_fft_size);
        }
    // 2- Doppler frequency search loop
    if (!d_step_two)
        {
            for (unsigned int doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
                {
                    // doppler search steps
                    int doppler = -static_cast<int>(acq_parameters.doppler_max) + d_doppler_step * doppler_index;

                    volk_32fc_x2_multiply_32fc(d_fft_if->get_inbuf(), in, d_grid_doppler_wipeoffs[doppler_index], d_fft_size);

                    // 3- Perform the FFT-based convolution  (parallel time search)
                    // Compute the FFT of the carrier wiped--off incoming signal
                    d_fft_if->execute();

                    // Multiply carrier wiped--off, Fourier transformed incoming signal
                    // with the local FFT'd code reference using SIMD operations with VOLK library
                    volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(), d_fft_if->get_outbuf(), d_fft_codes, d_fft_size);

                    // compute the inverse FFT
                    d_ifft->execute();

                    // Search maximum
                    size_t offset = (acq_parameters.bit_transition_flag ? effective_fft_size : 0);
                    volk_32fc_magnitude_squared_32f(d_magnitude, d_ifft->get_outbuf() + offset, effective_fft_size);
                    volk_gnsssdr_32f_index_max_32u(&indext, d_magnitude, effective_fft_size);
                    magt = d_magnitude[indext];

                    if (acq_parameters.use_CFAR_algorithm_flag)
                        {
                            // Normalize the maximum value to correct the scale factor introduced by FFTW
                            magt = d_magnitude[indext] / (fft_normalization_factor * fft_normalization_factor);
                        }
                    // 4- record the maximum peak and the associated synchronization parameters
                    if (d_mag < magt)
                        {
                            d_mag = magt;

                            if (!acq_parameters.use_CFAR_algorithm_flag)
                                {
                                    // Search grid noise floor approximation for this doppler line
                                    volk_32f_accumulator_s32f(&d_input_power, d_magnitude, effective_fft_size);
                                    d_input_power = (d_input_power - d_mag) / (effective_fft_size - 1);
                                }

                            // In case that acq_parameters.bit_transition_flag = true, we compare the potentially
                            // new maximum test statistics (d_mag/d_input_power) with the value in
                            // d_test_statistics. When the second dwell is being processed, the value
                            // of d_mag/d_input_power could be lower than d_test_statistics (i.e,
                            // the maximum test statistics in the previous dwell is greater than
                            // current d_mag/d_input_power). Note that d_test_statistics is not
                            // restarted between consecutive dwells in multidwell operation.

                            if (d_test_statistics < (d_mag / d_input_power) or !acq_parameters.bit_transition_flag)
                                {
                                    d_gnss_synchro->Acq_delay_samples = static_cast<double>(indext % acq_parameters.samples_per_code);
                                    d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
                                    d_gnss_synchro->Acq_samplestamp_samples = samp_count;

                                    // 5- Compute the test statistics and compare to the threshold
                                    //d_test_statistics = 2 * d_fft_size * d_mag / d_input_power;
                                    d_test_statistics = d_mag / d_input_power;
                                }
                        }
                    // Record results to file if required
                    if (acq_parameters.dump)
                        {
                            memcpy(grid_.colptr(doppler_index), d_magnitude, sizeof(float) * effective_fft_size);
                        }
                }
        }
    else
        {
            for (unsigned int doppler_index = 0; doppler_index < acq_parameters.num_doppler_bins_step2; doppler_index++)
                {
                    // doppler search steps
                    float doppler = d_doppler_center_step_two + (static_cast<float>(doppler_index) - static_cast<float>(acq_parameters.num_doppler_bins_step2) / 2.0) * acq_parameters.doppler_step2;

                    volk_32fc_x2_multiply_32fc(d_fft_if->get_inbuf(), in, d_grid_doppler_wipeoffs_step_two[doppler_index], d_fft_size);

                    // 3- Perform the FFT-based convolution  (parallel time search)
                    // Compute the FFT of the carrier wiped--off incoming signal
                    d_fft_if->execute();

                    // Multiply carrier wiped--off, Fourier transformed incoming signal
                    // with the local FFT'd code reference using SIMD operations with VOLK library
                    volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(), d_fft_if->get_outbuf(), d_fft_codes, d_fft_size);

                    // compute the inverse FFT
                    d_ifft->execute();

                    // Search maximum
                    size_t offset = (acq_parameters.bit_transition_flag ? effective_fft_size : 0);
                    volk_32fc_magnitude_squared_32f(d_magnitude, d_ifft->get_outbuf() + offset, effective_fft_size);
                    volk_gnsssdr_32f_index_max_32u(&indext, d_magnitude, effective_fft_size);
                    magt = d_magnitude[indext];

                    if (acq_parameters.use_CFAR_algorithm_flag)
                        {
                            // Normalize the maximum value to correct the scale factor introduced by FFTW
                            magt = d_magnitude[indext] / (fft_normalization_factor * fft_normalization_factor);
                        }
                    // 4- record the maximum peak and the associated synchronization parameters
                    if (d_mag < magt)
                        {
                            d_mag = magt;

                            if (!acq_parameters.use_CFAR_algorithm_flag)
                                {
                                    // Search grid noise floor approximation for this doppler line
                                    volk_32f_accumulator_s32f(&d_input_power, d_magnitude, effective_fft_size);
                                    d_input_power = (d_input_power - d_mag) / (effective_fft_size - 1);
                                }

                            // In case that acq_parameters.bit_transition_flag = true, we compare the potentially
                            // new maximum test statistics (d_mag/d_input_power) with the value in
                            // d_test_statistics. When the second dwell is being processed, the value
                            // of d_mag/d_input_power could be lower than d_test_statistics (i.e,
                            // the maximum test statistics in the previous dwell is greater than
                            // current d_mag/d_input_power). Note that d_test_statistics is not
                            // restarted between consecutive dwells in multidwell operation.

                            if (d_test_statistics < (d_mag / d_input_power) or !acq_parameters.bit_transition_flag)
                                {
                                    d_gnss_synchro->Acq_delay_samples = static_cast<double>(indext % acq_parameters.samples_per_code);
                                    d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
                                    d_gnss_synchro->Acq_samplestamp_samples = samp_count;

                                    // 5- Compute the test statistics and compare to the threshold
                                    //d_test_statistics = 2 * d_fft_size * d_mag / d_input_power;
                                    d_test_statistics = d_mag / d_input_power;
                                }
                        }
                    // Record results to file if required
                    if (acq_parameters.dump)
                        {
                            memcpy(grid_.colptr(doppler_index), d_magnitude, sizeof(float) * effective_fft_size);
                        }
                }
        }
    // Record results to file if required
    if (acq_parameters.dump)
        {
            pcps_acquisition::dump_results(effective_fft_size);
        }
    lk.lock();
    if (!acq_parameters.bit_transition_flag)
        {
            if (d_test_statistics > d_threshold)
                {
                    d_active = false;
                    if (acq_parameters.make_2_steps)
                        {
                            if (d_step_two)
                                {
                                    send_positive_acquisition();
                                    d_step_two = false;
                                    d_state = 0;  // Positive acquisition
                                }
                            else
                                {
                                    d_step_two = true;  // Clear input buffer and make small grid acquisition
                                    d_state = 0;
                                }
                        }
                    else
                        {
                            send_positive_acquisition();
                            d_state = 0;  // Positive acquisition
                        }
                }
            else if (d_well_count == acq_parameters.max_dwells)
                {
                    d_state = 0;
                    d_active = false;
                    d_step_two = false;
                    send_negative_acquisition();
                }
        }
    else
        {
            d_active = false;
            if (d_test_statistics > d_threshold)
                {
                    if (acq_parameters.make_2_steps)
                        {
                            if (d_step_two)
                                {
                                    send_positive_acquisition();
                                    d_step_two = false;
                                    d_state = 0;  // Positive acquisition
                                }
                            else
                                {
                                    d_step_two = true;  // Clear input buffer and make small grid acquisition
                                    d_state = 0;
                                }
                        }
                    else
                        {
                            send_positive_acquisition();
                            d_state = 0;  // Positive acquisition
                        }
                }
            else
                {
                    d_state = 0;  // Negative acquisition
                    d_step_two = false;
                    send_negative_acquisition();
                }
        }
    d_worker_active = false;
}


int pcps_acquisition::general_work(int noutput_items __attribute__((unused)),
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
     * 6. Declare positive or negative acquisition using a message port
     */

    gr::thread::scoped_lock lk(d_setlock);
    if (!d_active or d_worker_active)
        {
            d_sample_counter += d_fft_size * ninput_items[0];
            consume_each(ninput_items[0]);
            if (d_step_two)
                {
                    d_doppler_center_step_two = static_cast<float>(d_gnss_synchro->Acq_doppler_hz);
                    update_grid_doppler_wipeoffs_step2();
                    d_state = 0;
                    d_active = true;
                }
            return 0;
        }

    switch (d_state)
        {
        case 0:
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
                d_sample_counter += d_fft_size * ninput_items[0];  // sample counter
                consume_each(ninput_items[0]);
                break;
            }

        case 1:
            {
                // Copy the data to the core and let it know that new data is available
                if (d_cshort)
                    {
                        memcpy(d_data_buffer_sc, input_items[0], d_fft_size * sizeof(lv_16sc_t));
                    }
                else
                    {
                        memcpy(d_data_buffer, input_items[0], d_fft_size * sizeof(gr_complex));
                    }
                if (acq_parameters.blocking)
                    {
                        lk.unlock();
                        acquisition_core(d_sample_counter);
                    }
                else
                    {
                        gr::thread::thread d_worker(&pcps_acquisition::acquisition_core, this, d_sample_counter);
                        d_worker_active = true;
                    }
                d_sample_counter += d_fft_size;
                consume_each(1);
                break;
            }
        }
    return 0;
}
