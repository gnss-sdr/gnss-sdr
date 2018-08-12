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
                                                                gr::io_signature::make(1, 1, conf_.it_size),
                                                                gr::io_signature::make(0, 0, conf_.it_size))
{
    this->message_port_register_out(pmt::mp("events"));

    acq_parameters = conf_;
    d_sample_counter = 0ULL;  // SAMPLE COUNTER
    d_active = false;
    d_positive_acq = 0;
    d_state = 0;
    d_old_freq = 0LL;
    d_num_noncoherent_integrations_counter = 0U;
    d_consumed_samples = acq_parameters.sampled_ms * acq_parameters.samples_per_ms * (acq_parameters.bit_transition_flag ? 2 : 1);
    if (acq_parameters.sampled_ms == acq_parameters.ms_per_code)
        {
            d_fft_size = d_consumed_samples;
        }
    else
        {
            d_fft_size = d_consumed_samples * 2;
        }
    // d_fft_size = next power of two?  ////
    d_mag = 0;
    d_input_power = 0.0;
    d_num_doppler_bins = 0U;
    d_threshold = 0.0;
    d_doppler_step = 0U;
    d_doppler_center_step_two = 0.0;
    d_test_statistics = 0.0;
    d_channel = 0U;
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
            d_fft_size = d_consumed_samples * 2;
            acq_parameters.max_dwells = 1;  // Activation of acq_parameters.bit_transition_flag invalidates the value of acq_parameters.max_dwells
        }

    d_tmp_buffer = static_cast<float*>(volk_gnsssdr_malloc(d_fft_size * sizeof(float), volk_gnsssdr_get_alignment()));
    d_fft_codes = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_magnitude = static_cast<float*>(volk_gnsssdr_malloc(d_fft_size * sizeof(float), volk_gnsssdr_get_alignment()));
    d_input_signal = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));

    // Direct FFT
    d_fft_if = new gr::fft::fft_complex(d_fft_size, true);

    // Inverse FFT
    d_ifft = new gr::fft::fft_complex(d_fft_size, false);

    d_gnss_synchro = 0;
    d_grid_doppler_wipeoffs = nullptr;
    d_grid_doppler_wipeoffs_step_two = nullptr;
    d_magnitude_grid = nullptr;
    d_worker_active = false;
    d_data_buffer = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_consumed_samples * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    if (d_cshort)
        {
            d_data_buffer_sc = static_cast<lv_16sc_t*>(volk_gnsssdr_malloc(d_consumed_samples * sizeof(lv_16sc_t), volk_gnsssdr_get_alignment()));
        }
    else
        {
            d_data_buffer_sc = nullptr;
        }
    grid_ = arma::fmat();
    narrow_grid_ = arma::fmat();
    d_step_two = false;
    d_num_doppler_bins_step2 = acq_parameters.num_doppler_bins_step2;
    d_dump_number = 0LL;
    d_dump_channel = acq_parameters.dump_channel;
    d_samplesPerChip = acq_parameters.samples_per_chip;
    d_buffer_count = 0U;
    // todo: CFAR statistic not available for non-coherent integration
    if (acq_parameters.max_dwells == 1)
        {
            d_use_CFAR_algorithm_flag = acq_parameters.use_CFAR_algorithm_flag;
        }
    else
        {
            d_use_CFAR_algorithm_flag = false;
        }
}


pcps_acquisition::~pcps_acquisition()
{
    if (d_num_doppler_bins > 0)
        {
            for (uint32_t i = 0; i < d_num_doppler_bins; i++)
                {
                    volk_gnsssdr_free(d_grid_doppler_wipeoffs[i]);
                    volk_gnsssdr_free(d_magnitude_grid[i]);
                }
            delete[] d_grid_doppler_wipeoffs;
            delete[] d_magnitude_grid;
        }
    if (acq_parameters.make_2_steps)
        {
            for (uint32_t i = 0; i < d_num_doppler_bins_step2; i++)
                {
                    volk_gnsssdr_free(d_grid_doppler_wipeoffs_step_two[i]);
                }
            delete[] d_grid_doppler_wipeoffs_step_two;
        }
    volk_gnsssdr_free(d_fft_codes);
    volk_gnsssdr_free(d_magnitude);
    volk_gnsssdr_free(d_tmp_buffer);
    volk_gnsssdr_free(d_input_signal);
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
    d_old_freq = 0LL;
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
            int32_t offset = d_fft_size / 2;
            std::fill_n(d_fft_if->get_inbuf(), offset, gr_complex(0.0, 0.0));
            memcpy(d_fft_if->get_inbuf() + offset, code, sizeof(gr_complex) * offset);
        }
    else
        {
            if (acq_parameters.sampled_ms == acq_parameters.ms_per_code)
                {
                    memcpy(d_fft_if->get_inbuf(), code, sizeof(gr_complex) * d_consumed_samples);
                }
            else
                {
                    std::fill_n(d_fft_if->get_inbuf(), d_fft_size - d_consumed_samples, gr_complex(0.0, 0.0));
                    memcpy(d_fft_if->get_inbuf() + d_consumed_samples, code, sizeof(gr_complex) * d_consumed_samples);
                }
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


void pcps_acquisition::update_local_carrier(gr_complex* carrier_vector, int32_t correlator_length_samples, float freq)
{
    float phase_step_rad = GPS_TWO_PI * freq / static_cast<float>(acq_parameters.fs_in);
    float _phase[1];
    _phase[0] = 0.0;
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
    d_gnss_synchro->Acq_samplestamp_samples = 0ULL;
    d_mag = 0.0;
    d_input_power = 0.0;

    d_num_doppler_bins = static_cast<uint32_t>(std::ceil(static_cast<double>(static_cast<int32_t>(acq_parameters.doppler_max) - static_cast<int32_t>(-acq_parameters.doppler_max)) / static_cast<double>(d_doppler_step)));

    // Create the carrier Doppler wipeoff signals
    d_grid_doppler_wipeoffs = new gr_complex*[d_num_doppler_bins];
    if (acq_parameters.make_2_steps)
        {
            d_grid_doppler_wipeoffs_step_two = new gr_complex*[d_num_doppler_bins_step2];
            for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins_step2; doppler_index++)
                {
                    d_grid_doppler_wipeoffs_step_two[doppler_index] = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
                }
        }

    d_magnitude_grid = new float*[d_num_doppler_bins];
    for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            d_grid_doppler_wipeoffs[doppler_index] = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
            d_magnitude_grid[doppler_index] = static_cast<float*>(volk_gnsssdr_malloc(d_fft_size * sizeof(float), volk_gnsssdr_get_alignment()));
            for (uint32_t k = 0; k < d_fft_size; k++)
                {
                    d_magnitude_grid[doppler_index][k] = 0.0;
                }
            int32_t doppler = -static_cast<int32_t>(acq_parameters.doppler_max) + d_doppler_step * doppler_index;
            update_local_carrier(d_grid_doppler_wipeoffs[doppler_index], d_fft_size, d_old_freq + doppler);
        }

    d_worker_active = false;

    if (acq_parameters.dump)
        {
            uint32_t effective_fft_size = (acq_parameters.bit_transition_flag ? (d_fft_size / 2) : d_fft_size);
            grid_ = arma::fmat(effective_fft_size, d_num_doppler_bins, arma::fill::zeros);
            narrow_grid_ = arma::fmat(effective_fft_size, d_num_doppler_bins_step2, arma::fill::zeros);
        }
}


void pcps_acquisition::update_grid_doppler_wipeoffs()
{
    for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            int32_t doppler = -static_cast<int32_t>(acq_parameters.doppler_max) + d_doppler_step * doppler_index;
            update_local_carrier(d_grid_doppler_wipeoffs[doppler_index], d_fft_size, d_old_freq + doppler);
        }
}


void pcps_acquisition::update_grid_doppler_wipeoffs_step2()
{
    for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins_step2; doppler_index++)
        {
            float doppler = (static_cast<float>(doppler_index) - static_cast<float>(floor(d_num_doppler_bins_step2 / 2.0))) * acq_parameters.doppler_step2;
            update_local_carrier(d_grid_doppler_wipeoffs_step_two[doppler_index], d_fft_size, d_doppler_center_step_two + doppler);
        }
}


void pcps_acquisition::set_state(int32_t state)
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with work function called by the scheduler
    d_state = state;
    if (d_state == 1)
        {
            d_gnss_synchro->Acq_delay_samples = 0.0;
            d_gnss_synchro->Acq_doppler_hz = 0.0;
            d_gnss_synchro->Acq_samplestamp_samples = 0ULL;
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
    // Declare positive acquisition using a message port
    // 0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL
    DLOG(INFO) << "positive acquisition"
               << ", satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
               << ", sample_stamp " << d_sample_counter
               << ", test statistics value " << d_test_statistics
               << ", test statistics threshold " << d_threshold
               << ", code phase " << d_gnss_synchro->Acq_delay_samples
               << ", doppler " << d_gnss_synchro->Acq_doppler_hz
               << ", magnitude " << d_mag
               << ", input signal power " << d_input_power;
    d_positive_acq = 1;
    this->message_port_pub(pmt::mp("events"), pmt::from_long(1));
}


void pcps_acquisition::send_negative_acquisition()
{
    // Declare negative acquisition using a message port
    // 0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL
    DLOG(INFO) << "negative acquisition"
               << ", satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
               << ", sample_stamp " << d_sample_counter
               << ", test statistics value " << d_test_statistics
               << ", test statistics threshold " << d_threshold
               << ", code phase " << d_gnss_synchro->Acq_delay_samples
               << ", doppler " << d_gnss_synchro->Acq_doppler_hz
               << ", magnitude " << d_mag
               << ", input signal power " << d_input_power;
    d_positive_acq = 0;
    this->message_port_pub(pmt::mp("events"), pmt::from_long(2));
}


void pcps_acquisition::dump_results(int32_t effective_fft_size)
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
            matvar_t* matvar = Mat_VarCreate("acq_grid", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, grid_.memptr(), 0);
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

            matvar = Mat_VarCreate("d_positive_acq", MAT_C_INT32, MAT_T_INT32, 1, dims, &d_positive_acq, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            float aux = static_cast<float>(d_gnss_synchro->Acq_doppler_hz);
            matvar = Mat_VarCreate("acq_doppler_hz", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims, &aux, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            aux = static_cast<float>(d_gnss_synchro->Acq_delay_samples);
            matvar = Mat_VarCreate("acq_delay_samples", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims, &aux, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("test_statistic", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims, &d_test_statistics, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("threshold", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims, &d_threshold, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("input_power", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims, &d_input_power, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("sample_counter", MAT_C_UINT64, MAT_T_UINT64, 1, dims, &d_sample_counter, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("PRN", MAT_C_UINT32, MAT_T_UINT32, 1, dims, &d_gnss_synchro->PRN, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("num_dwells", MAT_C_UINT32, MAT_T_UINT32, 1, dims, &d_num_noncoherent_integrations_counter, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            if (acq_parameters.make_2_steps)
                {
                    dims[0] = static_cast<size_t>(effective_fft_size);
                    dims[1] = static_cast<size_t>(d_num_doppler_bins_step2);
                    matvar = Mat_VarCreate("acq_grid_narrow", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, narrow_grid_.memptr(), 0);
                    Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
                    Mat_VarFree(matvar);

                    dims[0] = static_cast<size_t>(1);
                    dims[1] = static_cast<size_t>(1);
                    matvar = Mat_VarCreate("doppler_step_narrow", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims, &acq_parameters.doppler_step2, 0);
                    Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
                    Mat_VarFree(matvar);

                    aux = d_doppler_center_step_two - static_cast<float>(floor(d_num_doppler_bins_step2 / 2.0)) * acq_parameters.doppler_step2;
                    matvar = Mat_VarCreate("doppler_grid_narrow_min", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims, &aux, 0);
                    Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
                    Mat_VarFree(matvar);
                }

            Mat_Close(matfp);
        }
}


float pcps_acquisition::max_to_input_power_statistic(uint32_t& indext, int32_t& doppler, float input_power, uint32_t num_doppler_bins, int32_t doppler_max, int32_t doppler_step)
{
    float grid_maximum = 0.0;
    uint32_t index_doppler = 0U;
    uint32_t tmp_intex_t = 0U;
    uint32_t index_time = 0U;
    float fft_normalization_factor = static_cast<float>(d_fft_size) * static_cast<float>(d_fft_size);

    // Find the correlation peak and the carrier frequency
    for (uint32_t i = 0; i < num_doppler_bins; i++)
        {
            volk_gnsssdr_32f_index_max_32u(&tmp_intex_t, d_magnitude_grid[i], d_fft_size);
            if (d_magnitude_grid[i][tmp_intex_t] > grid_maximum)
                {
                    grid_maximum = d_magnitude_grid[i][tmp_intex_t];
                    index_doppler = i;
                    index_time = tmp_intex_t;
                }
        }
    indext = index_time;
    if (!d_step_two)
        {
            doppler = -static_cast<int32_t>(doppler_max) + doppler_step * static_cast<int32_t>(index_doppler);
        }
    else
        {
            doppler = static_cast<int32_t>(d_doppler_center_step_two + (static_cast<float>(index_doppler) - static_cast<float>(floor(d_num_doppler_bins_step2 / 2.0))) * acq_parameters.doppler_step2);
        }

    float magt = grid_maximum / (fft_normalization_factor * fft_normalization_factor);
    return magt / input_power;
}


float pcps_acquisition::first_vs_second_peak_statistic(uint32_t& indext, int32_t& doppler, uint32_t num_doppler_bins, int32_t doppler_max, int32_t doppler_step)
{
    // Look for correlation peaks in the results
    // Find the highest peak and compare it to the second highest peak
    // The second peak is chosen not closer than 1 chip to the highest peak

    float firstPeak = 0.0;
    uint32_t index_doppler = 0U;
    uint32_t tmp_intex_t = 0U;
    uint32_t index_time = 0U;

    // Find the correlation peak and the carrier frequency
    for (uint32_t i = 0; i < num_doppler_bins; i++)
        {
            volk_gnsssdr_32f_index_max_32u(&tmp_intex_t, d_magnitude_grid[i], d_fft_size);
            if (d_magnitude_grid[i][tmp_intex_t] > firstPeak)
                {
                    firstPeak = d_magnitude_grid[i][tmp_intex_t];
                    index_doppler = i;
                    index_time = tmp_intex_t;
                }
        }
    indext = index_time;

    if (!d_step_two)
        {
            doppler = -static_cast<int32_t>(doppler_max) + doppler_step * static_cast<int32_t>(index_doppler);
        }
    else
        {
            doppler = static_cast<int32_t>(d_doppler_center_step_two + (static_cast<float>(index_doppler) - static_cast<float>(floor(d_num_doppler_bins_step2 / 2.0))) * acq_parameters.doppler_step2);
        }

    // Find 1 chip wide code phase exclude range around the peak
    int32_t excludeRangeIndex1 = index_time - d_samplesPerChip;
    int32_t excludeRangeIndex2 = index_time + d_samplesPerChip;

    // Correct code phase exclude range if the range includes array boundaries
    if (excludeRangeIndex1 < 0)
        {
            excludeRangeIndex1 = d_fft_size + excludeRangeIndex1;
        }
    else if (excludeRangeIndex2 >= static_cast<int32_t>(d_fft_size))
        {
            excludeRangeIndex2 = excludeRangeIndex2 - d_fft_size;
        }

    int32_t idx = excludeRangeIndex1;
    memcpy(d_tmp_buffer, d_magnitude_grid[index_doppler], d_fft_size);
    do
        {
            d_tmp_buffer[idx] = 0.0;
            idx++;
            if (idx == static_cast<int32_t>(d_fft_size)) idx = 0;
        }
    while (idx != excludeRangeIndex2);

    // Find the second highest correlation peak in the same freq. bin ---
    volk_gnsssdr_32f_index_max_32u(&tmp_intex_t, d_tmp_buffer, d_fft_size);
    float secondPeak = d_tmp_buffer[tmp_intex_t];

    // Compute the test statistics and compare to the threshold
    return firstPeak / secondPeak;
}


void pcps_acquisition::acquisition_core(uint64_t samp_count)
{
    gr::thread::scoped_lock lk(d_setlock);

    // Initialize acquisition algorithm
    int32_t doppler = 0;
    uint32_t indext = 0U;
    int32_t effective_fft_size = (acq_parameters.bit_transition_flag ? d_fft_size / 2 : d_fft_size);
    if (d_cshort)
        {
            volk_gnsssdr_16ic_convert_32fc(d_data_buffer, d_data_buffer_sc, d_consumed_samples);
        }
    memcpy(d_input_signal, d_data_buffer, d_consumed_samples * sizeof(gr_complex));
    if (d_fft_size > d_consumed_samples)
        {
            for (uint32_t i = d_consumed_samples; i < d_fft_size; i++)
                {
                    d_input_signal[i] = gr_complex(0.0, 0.0);
                }
        }
    const gr_complex* in = d_input_signal;  // Get the input samples pointer

    d_input_power = 0.0;
    d_mag = 0.0;
    d_num_noncoherent_integrations_counter++;

    DLOG(INFO) << "Channel: " << d_channel
               << " , doing acquisition of satellite: " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
               << " ,sample stamp: " << samp_count << ", threshold: "
               << d_threshold << ", doppler_max: " << acq_parameters.doppler_max
               << ", doppler_step: " << d_doppler_step
               << ", use_CFAR_algorithm_flag: " << (d_use_CFAR_algorithm_flag ? "true" : "false");

    lk.unlock();

    if (d_use_CFAR_algorithm_flag or acq_parameters.bit_transition_flag)
        {
            // Compute the input signal power estimation
            volk_32fc_magnitude_squared_32f(d_tmp_buffer, in, d_fft_size);
            volk_32f_accumulator_s32f(&d_input_power, d_tmp_buffer, d_fft_size);
            d_input_power /= static_cast<float>(d_fft_size);
        }

    // Doppler frequency grid loop
    if (!d_step_two)
        {
            for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
                {
                    // Remove Doppler
                    volk_32fc_x2_multiply_32fc(d_fft_if->get_inbuf(), in, d_grid_doppler_wipeoffs[doppler_index], d_fft_size);

                    // Perform the FFT-based convolution  (parallel time search)
                    // Compute the FFT of the carrier wiped--off incoming signal
                    d_fft_if->execute();

                    // Multiply carrier wiped--off, Fourier transformed incoming signal with the local FFT'd code reference
                    volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(), d_fft_if->get_outbuf(), d_fft_codes, d_fft_size);

                    // Compute the inverse FFT
                    d_ifft->execute();

                    // Compute squared magnitude (and accumulate in case of non-coherent integration)
                    size_t offset = (acq_parameters.bit_transition_flag ? effective_fft_size : 0);
                    if (d_num_noncoherent_integrations_counter == 1)
                        {
                            volk_32fc_magnitude_squared_32f(d_magnitude_grid[doppler_index], d_ifft->get_outbuf() + offset, effective_fft_size);
                        }
                    else
                        {
                            volk_32fc_magnitude_squared_32f(d_tmp_buffer, d_ifft->get_outbuf() + offset, effective_fft_size);
                            volk_32f_x2_add_32f(d_magnitude_grid[doppler_index], d_magnitude_grid[doppler_index], d_tmp_buffer, effective_fft_size);
                        }
                    // Record results to file if required
                    if (acq_parameters.dump and d_channel == d_dump_channel)
                        {
                            memcpy(grid_.colptr(doppler_index), d_magnitude_grid[doppler_index], sizeof(float) * effective_fft_size);
                        }
                }

            // Compute the test statistic
            if (d_use_CFAR_algorithm_flag)
                {
                    d_test_statistics = max_to_input_power_statistic(indext, doppler, d_input_power, d_num_doppler_bins, acq_parameters.doppler_max, d_doppler_step);
                }
            else
                {
                    d_test_statistics = first_vs_second_peak_statistic(indext, doppler, d_num_doppler_bins, acq_parameters.doppler_max, d_doppler_step);
                }
            d_gnss_synchro->Acq_delay_samples = static_cast<double>(std::fmod(static_cast<float>(indext), acq_parameters.samples_per_code));
            d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
            d_gnss_synchro->Acq_samplestamp_samples = samp_count;
        }
    else
        {
            for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins_step2; doppler_index++)
                {
                    volk_32fc_x2_multiply_32fc(d_fft_if->get_inbuf(), in, d_grid_doppler_wipeoffs_step_two[doppler_index], d_fft_size);

                    // Perform the FFT-based convolution  (parallel time search)
                    // Compute the FFT of the carrier wiped--off incoming signal
                    d_fft_if->execute();

                    // Multiply carrier wiped--off, Fourier transformed incoming signal
                    // with the local FFT'd code reference using SIMD operations with VOLK library
                    volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(), d_fft_if->get_outbuf(), d_fft_codes, d_fft_size);

                    // compute the inverse FFT
                    d_ifft->execute();

                    size_t offset = (acq_parameters.bit_transition_flag ? effective_fft_size : 0);
                    if (d_num_noncoherent_integrations_counter == 1)
                        {
                            volk_32fc_magnitude_squared_32f(d_magnitude_grid[doppler_index], d_ifft->get_outbuf() + offset, effective_fft_size);
                        }
                    else
                        {
                            volk_32fc_magnitude_squared_32f(d_tmp_buffer, d_ifft->get_outbuf() + offset, effective_fft_size);
                            volk_32f_x2_add_32f(d_magnitude_grid[doppler_index], d_magnitude_grid[doppler_index], d_tmp_buffer, effective_fft_size);
                        }
                    // Record results to file if required
                    if (acq_parameters.dump and d_channel == d_dump_channel)
                        {
                            memcpy(narrow_grid_.colptr(doppler_index), d_magnitude_grid[doppler_index], sizeof(float) * effective_fft_size);
                        }
                }
            // Compute the test statistic
            if (d_use_CFAR_algorithm_flag)
                {
                    d_test_statistics = max_to_input_power_statistic(indext, doppler, d_input_power, d_num_doppler_bins_step2, static_cast<int32_t>(d_doppler_center_step_two - (static_cast<float>(d_num_doppler_bins_step2) / 2.0) * acq_parameters.doppler_step2), acq_parameters.doppler_step2);
                }
            else
                {
                    d_test_statistics = first_vs_second_peak_statistic(indext, doppler, d_num_doppler_bins_step2, static_cast<int32_t>(d_doppler_center_step_two - (static_cast<float>(d_num_doppler_bins_step2) / 2.0) * acq_parameters.doppler_step2), acq_parameters.doppler_step2);
                }
            d_gnss_synchro->Acq_delay_samples = static_cast<double>(std::fmod(static_cast<float>(indext), acq_parameters.samples_per_code));
            d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
            d_gnss_synchro->Acq_samplestamp_samples = samp_count;
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
                                    d_num_noncoherent_integrations_counter = 0;
                                    d_positive_acq = 0;
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
                    d_buffer_count = 0;
                    d_state = 1;
                }

            if (d_num_noncoherent_integrations_counter == acq_parameters.max_dwells)
                {
                    if (d_state != 0) send_negative_acquisition();
                    d_state = 0;
                    d_active = false;
                    d_step_two = false;
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
                                    d_num_noncoherent_integrations_counter = 0U;
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

    if ((d_num_noncoherent_integrations_counter == acq_parameters.max_dwells) or (d_positive_acq == 1))
        {
            // Record results to file if required
            if (acq_parameters.dump and d_channel == d_dump_channel)
                {
                    pcps_acquisition::dump_results(effective_fft_size);
                }
            d_num_noncoherent_integrations_counter = 0U;
            d_positive_acq = 0;
            // Reset grid
            for (uint32_t i = 0; i < d_num_doppler_bins; i++)
                {
                    for (uint32_t k = 0; k < d_fft_size; k++)
                        {
                            d_magnitude_grid[i][k] = 0.0;
                        }
                }
        }
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
            if (!acq_parameters.blocking_on_standby)
                {
                    d_sample_counter += static_cast<uint64_t>(ninput_items[0]);
                    consume_each(ninput_items[0]);
                }
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
                // Restart acquisition variables
                d_gnss_synchro->Acq_delay_samples = 0.0;
                d_gnss_synchro->Acq_doppler_hz = 0.0;
                d_gnss_synchro->Acq_samplestamp_samples = 0ULL;
                d_mag = 0.0;
                d_input_power = 0.0;
                d_test_statistics = 0.0;
                d_state = 1;
                d_buffer_count = 0U;
                if (!acq_parameters.blocking_on_standby)
                    {
                        d_sample_counter += static_cast<uint64_t>(ninput_items[0]);  // sample counter
                        consume_each(ninput_items[0]);
                    }
                break;
            }
        case 1:
            {
                uint32_t buff_increment;
                if (d_cshort)
                    {
                        const lv_16sc_t* in = reinterpret_cast<const lv_16sc_t*>(input_items[0]);  // Get the input samples pointer
                        if ((ninput_items[0] + d_buffer_count) <= d_consumed_samples)
                            {
                                buff_increment = ninput_items[0];
                            }
                        else
                            {
                                buff_increment = d_consumed_samples - d_buffer_count;
                            }
                        memcpy(&d_data_buffer_sc[d_buffer_count], in, sizeof(lv_16sc_t) * buff_increment);
                    }
                else
                    {
                        const gr_complex* in = reinterpret_cast<const gr_complex*>(input_items[0]);  // Get the input samples pointer
                        if ((ninput_items[0] + d_buffer_count) <= d_consumed_samples)
                            {
                                buff_increment = ninput_items[0];
                            }
                        else
                            {
                                buff_increment = d_consumed_samples - d_buffer_count;
                            }
                        memcpy(&d_data_buffer[d_buffer_count], in, sizeof(gr_complex) * buff_increment);
                    }

                // If buffer will be full in next iteration
                if (d_buffer_count >= d_consumed_samples)
                    {
                        d_state = 2;
                    }
                d_buffer_count += buff_increment;
                d_sample_counter += static_cast<uint64_t>(buff_increment);
                consume_each(buff_increment);
                break;
            }
        case 2:
            {
                // Copy the data to the core and let it know that new data is available
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
                consume_each(0);
                d_buffer_count = 0U;
                break;
            }
        }
    return 0;
}
