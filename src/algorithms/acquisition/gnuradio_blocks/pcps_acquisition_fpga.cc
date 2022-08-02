/*!
 * \file pcps_acquisition_fpga.cc
 * \brief This class implements a Parallel Code Phase Search Acquisition for the FPGA
 * \authors <ul>
 *          <li> Marc Majoral, 2019. mmajoral(at)cttc.es
 *          <li> Javier Arribas, 2019. jarribas(at)cttc.es
 *          </ul>
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "pcps_acquisition_fpga.h"
#include "gnss_sdr_make_unique.h"  // for std::make_unique in C++11
#include "gnss_synchro.h"
#include <glog/logging.h>
#include <cmath>     // for ceil
#include <iostream>  // for operator<<
#include <utility>   // for move


pcps_acquisition_fpga_sptr pcps_make_acquisition_fpga(Acq_Conf_Fpga& conf_)
{
    return pcps_acquisition_fpga_sptr(new pcps_acquisition_fpga(conf_));
}


pcps_acquisition_fpga::pcps_acquisition_fpga(Acq_Conf_Fpga& conf_)
    : d_acq_parameters(conf_),
      d_gnss_synchro(nullptr),
      d_sample_counter(0ULL),
      d_threshold(0.0),
      d_mag(0),
      d_input_power(0.0),
      d_test_statistics(0.0),
      d_doppler_step2(d_acq_parameters.doppler_step2),
      d_doppler_center_step_two(0.0),
      d_doppler_center(0U),
      d_state(0),
      d_doppler_index(0U),
      d_channel(0U),
      d_doppler_step(0U),
      d_doppler_max(d_acq_parameters.doppler_max),
      d_fft_size(d_acq_parameters.samples_per_code),
      d_num_doppler_bins(0U),
      d_downsampling_factor(d_acq_parameters.downsampling_factor),
      d_select_queue_Fpga(d_acq_parameters.select_queue_Fpga),
      d_total_block_exp(d_acq_parameters.total_block_exp),
      d_num_doppler_bins_step2(d_acq_parameters.num_doppler_bins_step2),
      d_max_num_acqs(d_acq_parameters.max_num_acqs),
      d_active(false),
      d_make_2_steps(d_acq_parameters.make_2_steps)
{
    d_acquisition_fpga = std::make_unique<Fpga_Acquisition>(d_acq_parameters.device_name, d_acq_parameters.code_length, d_acq_parameters.doppler_max, d_fft_size,
        d_acq_parameters.fs_in, d_acq_parameters.select_queue_Fpga, d_acq_parameters.all_fft_codes, d_acq_parameters.excludelimit);
}


void pcps_acquisition_fpga::set_local_code()
{
    d_acquisition_fpga->set_local_code(d_gnss_synchro->PRN);
}


void pcps_acquisition_fpga::init()
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

    d_num_doppler_bins = static_cast<uint32_t>(std::ceil(static_cast<double>(static_cast<int32_t>(d_doppler_max) - static_cast<int32_t>(-d_doppler_max)) / static_cast<double>(d_doppler_step))) + 1;
}


void pcps_acquisition_fpga::set_state(int32_t state)
{
    d_state = state;
    if (d_state == 1)
        {
            d_gnss_synchro->Acq_delay_samples = 0.0;
            d_gnss_synchro->Acq_doppler_hz = 0.0;
            d_gnss_synchro->Acq_samplestamp_samples = 0;
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


void pcps_acquisition_fpga::send_positive_acquisition()
{
    // Declare positive acquisition using a message port
    // 0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL
    DLOG(INFO) << "positive acquisition"
               << ", satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
               << ", sample_stamp " << d_gnss_synchro->Acq_samplestamp_samples
               << ", test statistics value " << d_test_statistics
               << ", test statistics threshold " << d_threshold
               << ", code phase " << d_gnss_synchro->Acq_delay_samples
               << ", doppler " << d_gnss_synchro->Acq_doppler_hz
               << ", magnitude " << d_mag
               << ", input signal power " << d_input_power
               << ", Assist doppler_center " << d_doppler_center;

    // the channel FSM is set, so, notify it directly the positive acquisition to minimize delays
    d_channel_fsm.lock()->Event_valid_acquisition();
}


void pcps_acquisition_fpga::send_negative_acquisition()
{
    // Declare negative acquisition using a message port
    DLOG(INFO) << "negative acquisition"
               << ", satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
               << ", sample_stamp " << d_gnss_synchro->Acq_samplestamp_samples
               << ", test statistics value " << d_test_statistics
               << ", test statistics threshold " << d_threshold
               << ", code phase " << d_gnss_synchro->Acq_delay_samples
               << ", doppler " << d_gnss_synchro->Acq_doppler_hz
               << ", magnitude " << d_mag
               << ", input signal power " << d_input_power;

    if (d_acq_parameters.repeat_satellite == true)
        {
            d_channel_fsm.lock()->Event_failed_acquisition_repeat();
        }
    else
        {
            d_channel_fsm.lock()->Event_failed_acquisition_no_repeat();
        }
}


void pcps_acquisition_fpga::acquisition_core(uint32_t num_doppler_bins, uint32_t doppler_step, int32_t doppler_min)
{
    uint32_t indext = 0U;
    float firstpeak = 0.0;
    float secondpeak = 0.0;
    uint32_t total_block_exp;
    uint64_t initial_sample;

    d_acquisition_fpga->set_doppler_sweep(num_doppler_bins, doppler_step, doppler_min);
    d_acquisition_fpga->run_acquisition();
    d_acquisition_fpga->read_acquisition_results(&indext,
        &firstpeak,
        &secondpeak,
        &initial_sample,
        &d_input_power,
        &d_doppler_index,
        &total_block_exp);

    const int32_t doppler = static_cast<int32_t>(doppler_min) + doppler_step * (d_doppler_index - 1);

    if (total_block_exp > d_total_block_exp)
        {
            // if the attenuation factor of the FPGA FFT-IFFT is smaller than the reference attenuation factor then we need to update the reference attenuation factor
            d_total_block_exp = total_block_exp;
            d_test_statistics = 0;
        }
    else
        {
            if (secondpeak > 0)
                {
                    d_test_statistics = firstpeak / secondpeak;
                }
            else
                {
                    d_test_statistics = 0.0;
                }
        }

    d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
    d_sample_counter = initial_sample;

    if (d_select_queue_Fpga == 0)
        {
            if (d_downsampling_factor > 1)
                {
                    d_gnss_synchro->Acq_delay_samples = static_cast<double>(d_downsampling_factor * (indext));
                    d_gnss_synchro->Acq_samplestamp_samples = d_downsampling_factor * static_cast<uint64_t>(d_sample_counter) - static_cast<uint64_t>(44);  // 33;  // 41;  // + 81*0.5;  // delay due to the downsampling filter in the acquisition
                }
            else
                {
                    d_gnss_synchro->Acq_delay_samples = static_cast<double>(indext);
                    d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter;  // delay due to the downsampling filter in the acquisition
                }
        }
    else
        {
            d_gnss_synchro->Acq_delay_samples = static_cast<double>(indext);
            d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter;  // delay due to the downsampling filter in the acquisition
        }
}


void pcps_acquisition_fpga::set_active(bool active)
{
    d_active = active;
    d_input_power = 0.0;
    d_mag = 0.0;

    DLOG(INFO) << "Channel: " << d_channel
               << " , doing acquisition of satellite: " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
               << " ,sample stamp: " << d_sample_counter << ", threshold: "
               << d_threshold << ", doppler_max: " << d_doppler_max
               << ", doppler_step: " << d_doppler_step;

    d_acquisition_fpga->open_device();
    d_acquisition_fpga->configure_acquisition();
    d_acquisition_fpga->write_local_code();
    d_acquisition_fpga->set_block_exp(d_total_block_exp);

    acquisition_core(d_num_doppler_bins, d_doppler_step, -d_doppler_max + d_doppler_center);
    if (!d_make_2_steps)
        {
            d_acquisition_fpga->close_device();
            if (d_test_statistics > d_threshold)
                {
                    d_active = false;
                    send_positive_acquisition();
                    d_state = 0;  // Positive acquisition
                }
            else
                {
                    d_state = 0;
                    d_active = false;
                    send_negative_acquisition();
                }
        }
    else
        {
            if (d_test_statistics > d_threshold)
                {
                    d_doppler_center_step_two = static_cast<float>(d_gnss_synchro->Acq_doppler_hz);

                    uint32_t num_second_acq = 1;

                    while (num_second_acq < d_max_num_acqs)
                        {
                            acquisition_core(d_num_doppler_bins_step2, d_doppler_step2, d_doppler_center_step_two - static_cast<float>(floor(d_num_doppler_bins_step2 / 2.0)) * d_doppler_step2);
                            if (d_test_statistics > d_threshold)
                                {
                                    d_active = false;
                                    send_positive_acquisition();
                                    d_state = 0;  // Positive acquisition
                                    break;
                                }
                            num_second_acq += 1;
                        }
                    d_acquisition_fpga->close_device();
                    if (d_test_statistics <= d_threshold)
                        {
                            d_state = 0;
                            d_active = false;
                            send_negative_acquisition();
                        }
                }
            else
                {
                    d_acquisition_fpga->close_device();
                    d_state = 0;
                    d_active = false;
                    send_negative_acquisition();
                }
        }
}


void pcps_acquisition_fpga::reset_acquisition()
{
    // this function triggers a HW reset of the FPGA PL.
    d_acquisition_fpga->open_device();
    d_acquisition_fpga->reset_acquisition();
    d_acquisition_fpga->close_device();
}


void pcps_acquisition_fpga::stop_acquisition()
{
    // this function stops the acquisition and the other FPGA Modules.
    d_acquisition_fpga->open_device();
    d_acquisition_fpga->stop_acquisition();
    d_acquisition_fpga->close_device();
}
