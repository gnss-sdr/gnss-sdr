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

#include "pcps_acquisition.h"
#include "GLONASS_L1_L2_CA.h"  // for GLONASS_PRN
#include "MATH_CONSTANTS.h"    // for TWO_PI
#include "gnss_frequencies.h"
#include "gnss_sdr_create_directory.h"
#include "gnss_sdr_filesystem.h"
#include "gnss_synchro.h"
#include <boost/math/special_functions/gamma.hpp>
#include <gnuradio/io_signature.h>
#include <matio.h>
#include <pmt/pmt.h>        // for from_long
#include <pmt/pmt_sugar.h>  // for mp
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <algorithm>  // for std::fill_n, std::min, std::copy
#include <array>
#include <cmath>  // for floor, fmod, rint, ceil
#include <iostream>
#include <map>


pcps_acquisition_sptr pcps_make_acquisition(const Acq_Conf& conf_)
{
    return pcps_acquisition_sptr(new pcps_acquisition(conf_));
}


pcps_acquisition::pcps_acquisition(const Acq_Conf& conf_)
    : gr::block("pcps_acquisition",
          gr::io_signature::make(1, 1, conf_.it_size),
          gr::io_signature::make(0, 1, sizeof(Gnss_Synchro))),
      d_acq_parameters(conf_),
      d_gnss_synchro(nullptr),
      d_dump_filename(conf_.dump_filename),
      d_dump_number(0LL),
      d_sample_counter(0ULL),
      d_threshold(0.0),
      d_mag(0),
      d_input_power(0.0),
      d_test_statistics(0.0),
      d_doppler_center_step_two(0.0),
      d_state(0),
      d_positive_acq(0),
      d_doppler_center(0U),
      d_doppler_bias(0),
      d_channel(0U),
      d_samplesPerChip(conf_.samples_per_chip),
      d_doppler_step(conf_.doppler_step),
      d_num_noncoherent_integrations_counter(0U),
      d_consumed_samples(conf_.sampled_ms * conf_.samples_per_ms * (conf_.bit_transition_flag ? 2.0 : 1.0)),
      d_num_doppler_bins(0U),
      d_num_doppler_bins_step2(conf_.num_doppler_bins_step2),
      d_dump_channel(conf_.dump_channel),
      d_buffer_count(0U),
      d_active(false),
      d_worker_active(false),
      d_step_two(false),
      d_use_CFAR_algorithm_flag(conf_.use_CFAR_algorithm_flag),
      d_dump(conf_.dump)
{
    this->message_port_register_out(pmt::mp("events"));

    if (d_acq_parameters.sampled_ms == d_acq_parameters.ms_per_code)
        {
            d_fft_size = d_consumed_samples;
        }
    else
        {
            d_fft_size = d_consumed_samples * 2;
        }
    // d_fft_size = next power of two?  ////

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
    // if (d_acq_parameters.bit_transition_flag)
    // {
    //  d_fft_size = d_consumed_samples * 2;
    //  d_acq_parameters.max_dwells = 1;  // Activation of d_acq_parameters.bit_transition_flag invalidates the value of d_acq_parameters.max_dwells
    // }

    d_tmp_buffer = volk_gnsssdr::vector<float>(d_fft_size);
    d_fft_codes = volk_gnsssdr::vector<std::complex<float>>(d_fft_size);
    d_input_signal = volk_gnsssdr::vector<std::complex<float>>(d_fft_size);
    d_fft_if = gnss_fft_fwd_make_unique(d_fft_size);
    d_ifft = gnss_fft_rev_make_unique(d_fft_size);

    d_grid = arma::fmat();
    d_narrow_grid = arma::fmat();

    if (conf_.it_size == sizeof(gr_complex))
        {
            d_cshort = false;
        }
    else
        {
            d_cshort = true;
        }

    d_data_buffer = volk_gnsssdr::vector<std::complex<float>>(d_consumed_samples);
    if (d_cshort)
        {
            d_data_buffer_sc = volk_gnsssdr::vector<lv_16sc_t>(d_consumed_samples);
        }

    if (d_dump)
        {
            std::string dump_path;
            // Get path
            if (d_dump_filename.find_last_of('/') != std::string::npos)
                {
                    const std::string dump_filename_ = d_dump_filename.substr(d_dump_filename.find_last_of('/') + 1);
                    dump_path = d_dump_filename.substr(0, d_dump_filename.find_last_of('/'));
                    d_dump_filename = dump_filename_;
                }
            else
                {
                    dump_path = std::string(".");
                }
            if (d_dump_filename.empty())
                {
                    d_dump_filename = "acquisition";
                }
            // remove extension if any
            if (d_dump_filename.substr(1).find_last_of('.') != std::string::npos)
                {
                    d_dump_filename = d_dump_filename.substr(0, d_dump_filename.find_last_of('.'));
                }
            d_dump_filename = dump_path + fs::path::preferred_separator + d_dump_filename;
            // create directory
            if (!gnss_sdr_create_directory(dump_path))
                {
                    std::cerr << "GNSS-SDR cannot create dump file for the Acquisition block. Wrong permissions?\n";
                    d_dump = false;
                }
        }
}


void pcps_acquisition::set_resampler_latency(uint32_t latency_samples)
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with work function called by the scheduler
    d_acq_parameters.resampler_latency_samples = latency_samples;
}


void pcps_acquisition::set_local_code(std::complex<float>* code)
{
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
    if (d_acq_parameters.bit_transition_flag)
        {
            const int32_t offset = d_fft_size / 2;
            std::fill_n(d_fft_if->get_inbuf(), offset, gr_complex(0.0, 0.0));
            std::copy(code, code + offset, d_fft_if->get_inbuf() + offset);
        }
    else
        {
            if (d_acq_parameters.sampled_ms == d_acq_parameters.ms_per_code)
                {
                    std::copy(code, code + d_consumed_samples, d_fft_if->get_inbuf());
                }
            else
                {
                    std::fill_n(d_fft_if->get_inbuf(), d_fft_size - d_consumed_samples, gr_complex(0.0, 0.0));
                    std::copy(code, code + d_consumed_samples, d_fft_if->get_inbuf() + d_consumed_samples);
                }
        }

    d_fft_if->execute();  // We need the FFT of local code
    volk_32fc_conjugate_32fc(d_fft_codes.data(), d_fft_if->get_outbuf(), d_fft_size);
}


bool pcps_acquisition::is_fdma()
{
    // reset the intermediate frequency
    d_doppler_bias = 0;
    // Dealing with FDMA system
    if (strcmp(d_gnss_synchro->Signal, "1G") == 0)
        {
            d_doppler_bias = static_cast<int32_t>(DFRQ1_GLO * GLONASS_PRN.at(d_gnss_synchro->PRN));
            DLOG(INFO) << "Trying to acquire SV PRN " << d_gnss_synchro->PRN << " with freq " << d_doppler_bias << " in Glonass Channel " << GLONASS_PRN.at(d_gnss_synchro->PRN) << '\n';
            return true;
        }
    if (strcmp(d_gnss_synchro->Signal, "2G") == 0)
        {
            d_doppler_bias += static_cast<int32_t>(DFRQ2_GLO * GLONASS_PRN.at(d_gnss_synchro->PRN));
            DLOG(INFO) << "Trying to acquire SV PRN " << d_gnss_synchro->PRN << " with freq " << d_doppler_bias << " in Glonass Channel " << GLONASS_PRN.at(d_gnss_synchro->PRN) << '\n';
            return true;
        }
    return false;
}


void pcps_acquisition::update_local_carrier(own::span<gr_complex> carrier_vector, float freq) const
{
    float phase_step_rad;
    if (d_acq_parameters.use_automatic_resampler)
        {
            phase_step_rad = static_cast<float>(TWO_PI) * freq / static_cast<float>(d_acq_parameters.resampled_fs);
        }
    else
        {
            phase_step_rad = static_cast<float>(TWO_PI) * freq / static_cast<float>(d_acq_parameters.fs_in);
        }
    std::array<float, 1> _phase{};
    volk_gnsssdr_s32f_sincos_32fc(carrier_vector.data(), -phase_step_rad, _phase.data(), carrier_vector.size());
}


void pcps_acquisition::init()
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

    d_num_doppler_bins = static_cast<uint32_t>(std::ceil(static_cast<double>(static_cast<int32_t>(d_acq_parameters.doppler_max) - static_cast<int32_t>(-d_acq_parameters.doppler_max)) / static_cast<double>(d_doppler_step)));

    // Create the carrier Doppler wipeoff signals
    if (d_grid_doppler_wipeoffs.empty())
        {
            d_grid_doppler_wipeoffs = volk_gnsssdr::vector<volk_gnsssdr::vector<std::complex<float>>>(d_num_doppler_bins, volk_gnsssdr::vector<std::complex<float>>(d_fft_size));
        }
    if (d_acq_parameters.make_2_steps && (d_grid_doppler_wipeoffs_step_two.empty()))
        {
            d_grid_doppler_wipeoffs_step_two = volk_gnsssdr::vector<volk_gnsssdr::vector<std::complex<float>>>(d_num_doppler_bins_step2, volk_gnsssdr::vector<std::complex<float>>(d_fft_size));
        }

    if (d_magnitude_grid.empty())
        {
            d_magnitude_grid = volk_gnsssdr::vector<volk_gnsssdr::vector<float>>(d_num_doppler_bins, volk_gnsssdr::vector<float>(d_fft_size));
        }

    for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            std::fill(d_magnitude_grid[doppler_index].begin(), d_magnitude_grid[doppler_index].end(), 0.0);
        }

    update_grid_doppler_wipeoffs();
    d_worker_active = false;

    if (d_dump)
        {
            const uint32_t effective_fft_size = (d_acq_parameters.bit_transition_flag ? (d_fft_size / 2) : d_fft_size);
            d_grid = arma::fmat(effective_fft_size, d_num_doppler_bins, arma::fill::zeros);
            d_narrow_grid = arma::fmat(effective_fft_size, d_num_doppler_bins_step2, arma::fill::zeros);
        }
}


void pcps_acquisition::update_grid_doppler_wipeoffs()
{
    for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            const int32_t doppler = -static_cast<int32_t>(d_acq_parameters.doppler_max) + d_doppler_center + d_doppler_step * doppler_index;
            update_local_carrier(d_grid_doppler_wipeoffs[doppler_index], static_cast<float>(d_doppler_bias + doppler));
        }
}


void pcps_acquisition::update_grid_doppler_wipeoffs_step2()
{
    for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins_step2; doppler_index++)
        {
            const float doppler = (static_cast<float>(doppler_index) - static_cast<float>(floor(d_num_doppler_bins_step2 / 2.0))) * d_acq_parameters.doppler_step2;
            update_local_carrier(d_grid_doppler_wipeoffs_step_two[doppler_index], d_doppler_center_step_two + doppler);
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
            d_gnss_synchro->Acq_doppler_step = 0U;
            d_mag = 0.0;
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
               << ", input signal power " << d_input_power
               << ", Assist doppler_center " << d_doppler_center;
    d_positive_acq = 1;

    if (!d_channel_fsm.expired())
        {
            // the channel FSM is set, so, notify it directly the positive acquisition to minimize delays
            d_channel_fsm.lock()->Event_valid_acquisition();
        }
    else
        {
            this->message_port_pub(pmt::mp("events"), pmt::from_long(1));
        }

    // Copy and push current Gnss_Synchro to monitor queue
    if (d_acq_parameters.enable_monitor_output)
        {
            Gnss_Synchro current_synchro_data = Gnss_Synchro();
            current_synchro_data = *d_gnss_synchro;
            d_monitor_queue.push(current_synchro_data);
        }
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
    std::string filename = d_dump_filename;
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

    mat_t* matfp = Mat_CreateVer(filename.c_str(), nullptr, MAT_FT_MAT73);
    if (matfp == nullptr)
        {
            std::cout << "Unable to create or open Acquisition dump file\n";
            // d_acq_parameters.dump = false;
        }
    else
        {
            std::array<size_t, 2> dims{static_cast<size_t>(effective_fft_size), static_cast<size_t>(d_num_doppler_bins)};
            matvar_t* matvar = Mat_VarCreate("acq_grid", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), d_grid.memptr(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            dims[0] = static_cast<size_t>(1);
            dims[1] = static_cast<size_t>(1);
            matvar = Mat_VarCreate("doppler_max", MAT_C_INT32, MAT_T_INT32, 1, dims.data(), &d_acq_parameters.doppler_max, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("doppler_step", MAT_C_INT32, MAT_T_INT32, 1, dims.data(), &d_doppler_step, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("d_positive_acq", MAT_C_INT32, MAT_T_INT32, 1, dims.data(), &d_positive_acq, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            auto aux = static_cast<float>(d_gnss_synchro->Acq_doppler_hz);
            matvar = Mat_VarCreate("acq_doppler_hz", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims.data(), &aux, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            aux = static_cast<float>(d_gnss_synchro->Acq_delay_samples);
            matvar = Mat_VarCreate("acq_delay_samples", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims.data(), &aux, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("test_statistic", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims.data(), &d_test_statistics, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("threshold", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims.data(), &d_threshold, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("input_power", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims.data(), &d_input_power, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("sample_counter", MAT_C_UINT64, MAT_T_UINT64, 1, dims.data(), &d_sample_counter, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("PRN", MAT_C_UINT32, MAT_T_UINT32, 1, dims.data(), &d_gnss_synchro->PRN, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("num_dwells", MAT_C_INT32, MAT_T_INT32, 1, dims.data(), &d_num_noncoherent_integrations_counter, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            if (d_acq_parameters.make_2_steps)
                {
                    dims[0] = static_cast<size_t>(effective_fft_size);
                    dims[1] = static_cast<size_t>(d_num_doppler_bins_step2);
                    matvar = Mat_VarCreate("acq_grid_narrow", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), d_narrow_grid.memptr(), 0);
                    Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
                    Mat_VarFree(matvar);

                    dims[0] = static_cast<size_t>(1);
                    dims[1] = static_cast<size_t>(1);
                    matvar = Mat_VarCreate("doppler_step_narrow", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims.data(), &d_acq_parameters.doppler_step2, 0);
                    Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
                    Mat_VarFree(matvar);

                    aux = d_doppler_center_step_two - static_cast<float>(floor(d_num_doppler_bins_step2 / 2.0)) * d_acq_parameters.doppler_step2;
                    matvar = Mat_VarCreate("doppler_grid_narrow_min", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims.data(), &aux, 0);
                    Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
                    Mat_VarFree(matvar);
                }

            Mat_Close(matfp);
        }
}


float pcps_acquisition::max_to_input_power_statistic(uint32_t& indext, int32_t& doppler, uint32_t num_doppler_bins, int32_t doppler_max, int32_t doppler_step)
{
    float grid_maximum = 0.0;
    uint32_t index_doppler = 0U;
    uint32_t tmp_intex_t = 0U;
    uint32_t index_time = 0U;
    const int32_t effective_fft_size = (d_acq_parameters.bit_transition_flag ? d_fft_size / 2 : d_fft_size);

    // Find the correlation peak and the carrier frequency
    for (uint32_t i = 0; i < num_doppler_bins; i++)
        {
            volk_gnsssdr_32f_index_max_32u(&tmp_intex_t, d_magnitude_grid[i].data(), effective_fft_size);
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
            const auto index_opp = (index_doppler + d_num_doppler_bins / 2) % d_num_doppler_bins;
            d_input_power = static_cast<float>(std::accumulate(d_magnitude_grid[index_opp].data(), d_magnitude_grid[index_opp].data() + effective_fft_size, static_cast<float>(0.0)) / effective_fft_size / 2.0 / d_num_noncoherent_integrations_counter);
            doppler = -static_cast<int32_t>(doppler_max) + d_doppler_center + doppler_step * static_cast<int32_t>(index_doppler);
        }
    else
        {
            doppler = static_cast<int32_t>(d_doppler_center_step_two + (static_cast<float>(index_doppler) - static_cast<float>(floor(d_num_doppler_bins_step2 / 2.0))) * d_acq_parameters.doppler_step2);
        }

    return grid_maximum / d_input_power;
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
            volk_gnsssdr_32f_index_max_32u(&tmp_intex_t, d_magnitude_grid[i].data(), d_fft_size);
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
            doppler = -static_cast<int32_t>(doppler_max) + d_doppler_center + doppler_step * static_cast<int32_t>(index_doppler);
        }
    else
        {
            doppler = static_cast<int32_t>(d_doppler_center_step_two + (static_cast<float>(index_doppler) - static_cast<float>(floor(d_num_doppler_bins_step2 / 2.0))) * d_acq_parameters.doppler_step2);
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
    std::copy(d_magnitude_grid[index_doppler].data(), d_magnitude_grid[index_doppler].data() + d_fft_size, d_tmp_buffer.data());
    do
        {
            d_tmp_buffer[idx] = 0.0;
            idx++;
            if (idx == static_cast<int32_t>(d_fft_size))
                {
                    idx = 0;
                }
        }
    while (idx != excludeRangeIndex2);

    // Find the second highest correlation peak in the same freq. bin ---
    volk_gnsssdr_32f_index_max_32u(&tmp_intex_t, d_tmp_buffer.data(), d_fft_size);
    const float secondPeak = d_tmp_buffer[tmp_intex_t];

    // Compute the test statistics and compare to the threshold
    return firstPeak / secondPeak;
}


void pcps_acquisition::acquisition_core(uint64_t samp_count)
{
    gr::thread::scoped_lock lk(d_setlock);

    // Initialize acquisition algorithm
    int32_t doppler = 0;
    uint32_t indext = 0U;
    const int32_t effective_fft_size = (d_acq_parameters.bit_transition_flag ? d_fft_size / 2 : d_fft_size);
    if (d_cshort)
        {
            volk_gnsssdr_16ic_convert_32fc(d_data_buffer.data(), d_data_buffer_sc.data(), d_consumed_samples);
        }
    std::copy(d_data_buffer.data(), d_data_buffer.data() + d_consumed_samples, d_input_signal.data());
    if (d_fft_size > d_consumed_samples)
        {
            for (uint32_t i = d_consumed_samples; i < d_fft_size; i++)
                {
                    d_input_signal[i] = gr_complex(0.0, 0.0);
                }
        }
    const gr_complex* in = d_input_signal.data();  // Get the input samples pointer

    d_mag = 0.0;
    d_num_noncoherent_integrations_counter++;

    DLOG(INFO) << "Channel: " << d_channel
               << " , doing acquisition of satellite: " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
               << " ,sample stamp: " << samp_count << ", threshold: "
               << d_threshold << ", doppler_max: " << d_acq_parameters.doppler_max
               << ", doppler_step: " << d_doppler_step
               << ", use_CFAR_algorithm_flag: " << (d_use_CFAR_algorithm_flag ? "true" : "false");

    if (d_acq_parameters.blocking)
        {
            lk.unlock();
        }

    // Doppler frequency grid loop
    if (!d_step_two)
        {
            for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
                {
                    // Remove Doppler
                    volk_32fc_x2_multiply_32fc(d_fft_if->get_inbuf(), in, d_grid_doppler_wipeoffs[doppler_index].data(), d_fft_size);

                    // Perform the FFT-based convolution  (parallel time search)
                    // Compute the FFT of the carrier wiped--off incoming signal
                    d_fft_if->execute();

                    // Multiply carrier wiped--off, Fourier transformed incoming signal with the local FFT'd code reference
                    volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(), d_fft_if->get_outbuf(), d_fft_codes.data(), d_fft_size);

                    // Compute the inverse FFT
                    d_ifft->execute();

                    // Compute squared magnitude (and accumulate in case of non-coherent integration)
                    const size_t offset = (d_acq_parameters.bit_transition_flag ? effective_fft_size : 0);
                    if (d_num_noncoherent_integrations_counter == 1)
                        {
                            volk_32fc_magnitude_squared_32f(d_magnitude_grid[doppler_index].data(), d_ifft->get_outbuf() + offset, effective_fft_size);
                        }
                    else
                        {
                            volk_32fc_magnitude_squared_32f(d_tmp_buffer.data(), d_ifft->get_outbuf() + offset, effective_fft_size);
                            volk_32f_x2_add_32f(d_magnitude_grid[doppler_index].data(), d_magnitude_grid[doppler_index].data(), d_tmp_buffer.data(), effective_fft_size);
                        }
                    // Record results to file if required
                    if (d_dump and d_channel == d_dump_channel)
                        {
                            std::copy(d_magnitude_grid[doppler_index].data(), d_magnitude_grid[doppler_index].data() + effective_fft_size, d_grid.colptr(doppler_index));
                        }
                }

            // Compute the test statistic
            if (d_use_CFAR_algorithm_flag)
                {
                    d_test_statistics = max_to_input_power_statistic(indext, doppler, d_num_doppler_bins, d_acq_parameters.doppler_max, d_doppler_step);
                }
            else
                {
                    d_test_statistics = first_vs_second_peak_statistic(indext, doppler, d_num_doppler_bins, d_acq_parameters.doppler_max, d_doppler_step);
                }
            if (d_acq_parameters.use_automatic_resampler)
                {
                    // take into account the acquisition resampler ratio
                    d_gnss_synchro->Acq_delay_samples = static_cast<double>(std::fmod(static_cast<float>(indext), d_acq_parameters.samples_per_code)) * d_acq_parameters.resampler_ratio;
                    d_gnss_synchro->Acq_delay_samples -= static_cast<double>(d_acq_parameters.resampler_latency_samples);  // account the resampler filter latency
                    d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
                    d_gnss_synchro->Acq_samplestamp_samples = rint(static_cast<double>(samp_count) * d_acq_parameters.resampler_ratio);
                }
            else
                {
                    d_gnss_synchro->Acq_delay_samples = static_cast<double>(std::fmod(static_cast<float>(indext), d_acq_parameters.samples_per_code));
                    d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
                    d_gnss_synchro->Acq_samplestamp_samples = samp_count;
                }
        }
    else
        {
            for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins_step2; doppler_index++)
                {
                    volk_32fc_x2_multiply_32fc(d_fft_if->get_inbuf(), in, d_grid_doppler_wipeoffs_step_two[doppler_index].data(), d_fft_size);

                    // Perform the FFT-based convolution  (parallel time search)
                    // Compute the FFT of the carrier wiped--off incoming signal
                    d_fft_if->execute();

                    // Multiply carrier wiped--off, Fourier transformed incoming signal
                    // with the local FFT'd code reference using SIMD operations with VOLK library
                    volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(), d_fft_if->get_outbuf(), d_fft_codes.data(), d_fft_size);

                    // compute the inverse FFT
                    d_ifft->execute();

                    const size_t offset = (d_acq_parameters.bit_transition_flag ? effective_fft_size : 0);
                    if (d_num_noncoherent_integrations_counter == 1)
                        {
                            volk_32fc_magnitude_squared_32f(d_magnitude_grid[doppler_index].data(), d_ifft->get_outbuf() + offset, effective_fft_size);
                        }
                    else
                        {
                            volk_32fc_magnitude_squared_32f(d_tmp_buffer.data(), d_ifft->get_outbuf() + offset, effective_fft_size);
                            volk_32f_x2_add_32f(d_magnitude_grid[doppler_index].data(), d_magnitude_grid[doppler_index].data(), d_tmp_buffer.data(), effective_fft_size);
                        }
                    // Record results to file if required
                    if (d_dump and d_channel == d_dump_channel)
                        {
                            std::copy(d_magnitude_grid[doppler_index].data(), d_magnitude_grid[doppler_index].data() + effective_fft_size, d_narrow_grid.colptr(doppler_index));
                        }
                }
            // Compute the test statistic
            if (d_use_CFAR_algorithm_flag)
                {
                    d_test_statistics = max_to_input_power_statistic(indext, doppler, d_num_doppler_bins_step2, static_cast<int32_t>(d_doppler_center_step_two - (static_cast<float>(d_num_doppler_bins_step2) / 2.0) * d_acq_parameters.doppler_step2), d_acq_parameters.doppler_step2);
                }
            else
                {
                    d_test_statistics = first_vs_second_peak_statistic(indext, doppler, d_num_doppler_bins_step2, static_cast<int32_t>(d_doppler_center_step_two - (static_cast<float>(d_num_doppler_bins_step2) / 2.0) * d_acq_parameters.doppler_step2), d_acq_parameters.doppler_step2);
                }

            if (d_acq_parameters.use_automatic_resampler)
                {
                    // take into account the acquisition resampler ratio
                    d_gnss_synchro->Acq_delay_samples = static_cast<double>(std::fmod(static_cast<float>(indext), d_acq_parameters.samples_per_code)) * d_acq_parameters.resampler_ratio;
                    d_gnss_synchro->Acq_delay_samples -= static_cast<double>(d_acq_parameters.resampler_latency_samples);  // account the resampler filter latency
                    d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
                    d_gnss_synchro->Acq_samplestamp_samples = rint(static_cast<double>(samp_count) * d_acq_parameters.resampler_ratio);
                    d_gnss_synchro->Acq_doppler_step = d_acq_parameters.doppler_step2;
                }
            else
                {
                    d_gnss_synchro->Acq_delay_samples = static_cast<double>(std::fmod(static_cast<float>(indext), d_acq_parameters.samples_per_code));
                    d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
                    d_gnss_synchro->Acq_samplestamp_samples = samp_count;
                    d_gnss_synchro->Acq_doppler_step = d_acq_parameters.doppler_step2;
                }
        }

    if (d_acq_parameters.blocking)
        {
            lk.lock();
        }

    if (!d_acq_parameters.bit_transition_flag)
        {
            if (d_test_statistics > d_threshold)
                {
                    d_active = false;
                    if (d_acq_parameters.make_2_steps)
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
                                    d_doppler_center_step_two = static_cast<float>(d_gnss_synchro->Acq_doppler_hz);
                                    update_grid_doppler_wipeoffs_step2();
                                    d_num_noncoherent_integrations_counter = 0;
                                    d_positive_acq = 0;
                                    d_state = 0;
                                }
                            calculate_threshold();
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

            if (d_num_noncoherent_integrations_counter == d_acq_parameters.max_dwells)
                {
                    if (d_state != 0)
                        {
                            send_negative_acquisition();
                        }
                    d_state = 0;
                    d_active = false;
                    const bool was_step_two = d_step_two;
                    d_step_two = false;
                    if (was_step_two)
                        {
                            calculate_threshold();
                        }
                }
        }
    else
        {
            d_active = false;
            if (d_test_statistics > d_threshold)
                {
                    if (d_acq_parameters.make_2_steps)
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
                                    d_doppler_center_step_two = static_cast<float>(d_gnss_synchro->Acq_doppler_hz);
                                    update_grid_doppler_wipeoffs_step2();
                                    d_num_noncoherent_integrations_counter = 0U;
                                    d_state = 0;
                                }
                            calculate_threshold();
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
                    const bool was_step_two = d_step_two;
                    d_step_two = false;
                    if (was_step_two)
                        {
                            calculate_threshold();
                        }
                    send_negative_acquisition();
                }
        }
    d_worker_active = false;

    if ((d_num_noncoherent_integrations_counter == d_acq_parameters.max_dwells) or (d_positive_acq == 1) or (d_acq_parameters.bit_transition_flag))
        {
            // Record results to file if required
            if (d_dump and d_channel == d_dump_channel)
                {
                    pcps_acquisition::dump_results(effective_fft_size);
                }
            d_num_noncoherent_integrations_counter = 0U;
            d_positive_acq = 0;
        }
}


// Called by gnuradio to enable drivers, etc for i/o devices.
bool pcps_acquisition::start()
{
    d_sample_counter = 0ULL;
    calculate_threshold();
    return true;
}


void pcps_acquisition::calculate_threshold()
{
    const float pfa = (d_step_two ? d_acq_parameters.pfa2 : d_acq_parameters.pfa);

    if (pfa <= 0.0)
        {
            return;
        }

    const auto effective_fft_size = static_cast<int>(d_acq_parameters.bit_transition_flag ? (d_fft_size / 2) : d_fft_size);
    const int num_doppler_bins = (d_step_two ? d_num_doppler_bins_step2 : d_num_doppler_bins);

    const int num_bins = effective_fft_size * num_doppler_bins;

    d_threshold = static_cast<float>(2.0 * boost::math::gamma_p_inv(2.0 * (d_acq_parameters.bit_transition_flag ? 1 : d_acq_parameters.max_dwells), std::pow(1.0 - pfa, 1.0 / static_cast<float>(num_bins))));
}


int pcps_acquisition::general_work(int noutput_items __attribute__((unused)),
    gr_vector_int& ninput_items,
    gr_vector_const_void_star& input_items,
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
     * 6. Declare positive or negative acquisition using a message port
     */
    gr::thread::scoped_lock lk(d_setlock);
    if (!d_active or d_worker_active)
        {
            // do not consume samples while performing a non-coherent integration
            bool consume_samples = ((!d_active) || (d_worker_active && (d_num_noncoherent_integrations_counter == d_acq_parameters.max_dwells)));
            if ((!d_acq_parameters.blocking_on_standby) && consume_samples)
                {
                    d_sample_counter += static_cast<uint64_t>(ninput_items[0]);
                    consume_each(ninput_items[0]);
                }
            if (d_step_two)
                {
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
                d_gnss_synchro->Acq_doppler_step = 0U;
                d_mag = 0.0;
                d_state = 1;
                d_buffer_count = 0U;
                if (!d_acq_parameters.blocking_on_standby)
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
                        const auto* in = reinterpret_cast<const lv_16sc_t*>(input_items[0]);  // Get the input samples pointer
                        if ((ninput_items[0] + d_buffer_count) <= d_consumed_samples)
                            {
                                buff_increment = ninput_items[0];
                            }
                        else
                            {
                                buff_increment = d_consumed_samples - d_buffer_count;
                            }
                        std::copy(in, in + buff_increment, d_data_buffer_sc.begin() + d_buffer_count);
                    }
                else
                    {
                        const auto* in = reinterpret_cast<const gr_complex*>(input_items[0]);  // Get the input samples pointer
                        if ((ninput_items[0] + d_buffer_count) <= d_consumed_samples)
                            {
                                buff_increment = ninput_items[0];
                            }
                        else
                            {
                                buff_increment = d_consumed_samples - d_buffer_count;
                            }
                        std::copy(in, in + buff_increment, d_data_buffer.begin() + d_buffer_count);
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
                if (d_acq_parameters.blocking)
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

    // Send outputs to the monitor
    if (d_acq_parameters.enable_monitor_output)
        {
            auto** out = reinterpret_cast<Gnss_Synchro**>(&output_items[0]);
            if (!d_monitor_queue.empty())
                {
                    int num_gnss_synchro_objects = d_monitor_queue.size();
                    for (int i = 0; i < num_gnss_synchro_objects; ++i)
                        {
                            Gnss_Synchro current_synchro_data = d_monitor_queue.front();
                            d_monitor_queue.pop();
                            *out[i] = current_synchro_data;
                        }
                    return num_gnss_synchro_objects;
                }
        }

    return 0;
}
