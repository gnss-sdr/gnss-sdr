/*!
 * \file gps_pcps_acquisition_fpga_sc.cc
 * \brief This class implements a Parallel Code Phase Search Acquisition in the FPGA.
 * This file is based on the file gps_pcps_acquisition_sc.cc
 * \authors <ul>
 *          <li> Marc Majoral, 2017. mmajoral(at)cttc.cat
 *          </ul>
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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

#include "gps_pcps_acquisition_fpga_sc.h"
#include <sstream>
#include <boost/filesystem.hpp>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include "control_message_factory.h"
#include "GPS_L1_CA.h"  //GPS_TWO_PI
using google::LogMessage;

void wait3(int seconds)
{
    boost::this_thread::sleep_for(boost::chrono::seconds{seconds});
}


gps_pcps_acquisition_fpga_sc_sptr gps_pcps_make_acquisition_fpga_sc(
    unsigned int sampled_ms, unsigned int max_dwells,
    unsigned int doppler_max, long freq, long fs_in, int samples_per_ms,
    int samples_per_code, int vector_length, unsigned int nsamples_total,
    bool bit_transition_flag, bool use_CFAR_algorithm_flag,
    unsigned int select_queue_Fpga, std::string device_name, bool dump,
    std::string dump_filename)
{
    return gps_pcps_acquisition_fpga_sc_sptr(
        new gps_pcps_acquisition_fpga_sc(sampled_ms, max_dwells,
            doppler_max, freq, fs_in, samples_per_ms, samples_per_code,
            vector_length, nsamples_total, bit_transition_flag,
            use_CFAR_algorithm_flag, select_queue_Fpga, device_name,
            dump, dump_filename));
}


gps_pcps_acquisition_fpga_sc::gps_pcps_acquisition_fpga_sc(
    unsigned int sampled_ms, unsigned int max_dwells,
    unsigned int doppler_max, long freq, long fs_in, int samples_per_ms,
    int samples_per_code, int vector_length, unsigned int nsamples_total,
    bool bit_transition_flag, bool use_CFAR_algorithm_flag,
    unsigned int select_queue_Fpga, std::string device_name, bool dump,
    std::string dump_filename) :

                                 gr::block("pcps_acquisition_fpga_sc",
                                     gr::io_signature::make(0, 0, sizeof(lv_16sc_t)),
                                     gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_out(pmt::mp("events"));
    d_sample_counter = 0;  // SAMPLE COUNTER
    d_active = false;
    d_state = 0;
    d_samples_per_code = samples_per_code;
    d_max_dwells = max_dwells;  // Note : d_max_dwells is not used in the FPGA implementation
    d_well_count = 0;
    d_doppler_max = doppler_max;
    d_fft_size = sampled_ms * samples_per_ms;
    d_mag = 0;
    d_num_doppler_bins = 0;
    d_bit_transition_flag = bit_transition_flag;          // Note : bit transition flag is ignored and assumed 0 in the FPGA implementation
    d_use_CFAR_algorithm_flag = use_CFAR_algorithm_flag;  // Note : user CFAR algorithm flag is ignored and assumed 0 in the FPGA implementation
    d_threshold = 0.0;
    d_doppler_step = 250;
    d_channel = 0;

    // For dumping samples into a file
    d_dump = dump;
    d_dump_filename = dump_filename;

    d_gnss_synchro = 0;

    // instantiate HW accelerator class
    acquisition_fpga_8sc = std::make_shared<gps_fpga_acquisition_8sc>(device_name, vector_length, d_fft_size, nsamples_total, fs_in, freq, sampled_ms, select_queue_Fpga);
}


gps_pcps_acquisition_fpga_sc::~gps_pcps_acquisition_fpga_sc()
{
    if (d_dump)
        {
            d_dump_file.close();
        }

    acquisition_fpga_8sc->free();
}


void gps_pcps_acquisition_fpga_sc::set_local_code()
{
    acquisition_fpga_8sc->set_local_code(d_gnss_synchro->PRN);
}


void gps_pcps_acquisition_fpga_sc::init()
{
    d_gnss_synchro->Flag_valid_acquisition = false;
    d_gnss_synchro->Flag_valid_symbol_output = false;
    d_gnss_synchro->Flag_valid_pseudorange = false;
    d_gnss_synchro->Flag_valid_word = false;
    d_gnss_synchro->Acq_delay_samples = 0.0;
    d_gnss_synchro->Acq_doppler_hz = 0.0;
    d_gnss_synchro->Acq_samplestamp_samples = 0;
    d_mag = 0.0;

    d_num_doppler_bins = ceil(
        static_cast<double>(static_cast<int>(d_doppler_max) - static_cast<int>(-d_doppler_max)) / static_cast<double>(d_doppler_step));

    acquisition_fpga_8sc->open_device();

    acquisition_fpga_8sc->init();
}


void gps_pcps_acquisition_fpga_sc::set_state(int state)
{
    d_state = state;
    if (d_state == 1)
        {
            d_gnss_synchro->Acq_delay_samples = 0.0;
            d_gnss_synchro->Acq_doppler_hz = 0.0;
            d_gnss_synchro->Acq_samplestamp_samples = 0;
            d_well_count = 0;
            d_mag = 0.0;
        }
    else if (d_state == 0)
        {
        }
    else
        {
            LOG(ERROR) << "State can only be set to 0 or 1";
        }
}


void gps_pcps_acquisition_fpga_sc::set_active(bool active)
{
    float temp_peak_to_noise_level = 0.0;
    float peak_to_noise_level = 0.0;
    float input_power;
    float test_statistics = 0.0;
    acquisition_fpga_8sc->block_samples();  // block the samples to run the acquisition this is only necessary for the tests

    d_active = active;

    int acquisition_message = -1;  //0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL

    d_state = 1;

    // initialize acquisition algorithm
    int doppler;
    uint32_t indext = 0;
    float magt = 0.0;
    //int effective_fft_size = ( d_bit_transition_flag ? d_fft_size/2 : d_fft_size );
    int effective_fft_size = d_fft_size;
    //float fft_normalization_factor = static_cast<float>(d_fft_size) * static_cast<float>(d_fft_size);

    d_mag = 0.0;

    unsigned int initial_sample;

    d_well_count++;

    DLOG(INFO) << "Channel: " << d_channel
               << " , doing acquisition of satellite: " << d_gnss_synchro->System
               << " " << d_gnss_synchro->PRN << " ,sample stamp: "
               << d_sample_counter << ", threshold: "
               << ", threshold: "
               << d_threshold << ", doppler_max: " << d_doppler_max
               << ", doppler_step: " << d_doppler_step;

    // Doppler frequency search loop
    for (unsigned int doppler_index = 0; doppler_index < d_num_doppler_bins;
         doppler_index++)
        {
            doppler = -static_cast<int>(d_doppler_max) + d_doppler_step * doppler_index;

            acquisition_fpga_8sc->set_phase_step(doppler_index);
            acquisition_fpga_8sc->run_acquisition();  // runs acquisition and waits until it is finished

            acquisition_fpga_8sc->read_acquisition_results(&indext, &magt,
                &initial_sample, &input_power);

            d_sample_counter = initial_sample;

            temp_peak_to_noise_level = static_cast<float>(magt) / static_cast<float>(input_power);
            if (peak_to_noise_level < temp_peak_to_noise_level)
                {
                    peak_to_noise_level = temp_peak_to_noise_level;
                    d_mag = magt;

                    input_power = (input_power - d_mag) / (effective_fft_size - 1);

                    d_gnss_synchro->Acq_delay_samples =
                        static_cast<double>(indext % d_samples_per_code);
                    d_gnss_synchro->Acq_doppler_hz =
                        static_cast<double>(doppler);
                    d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter;
                    test_statistics = d_mag / input_power;
                }

            // Record results to file if required
            if (d_dump)
                {
                    std::stringstream filename;
                    //std::streamsize n = 2 * sizeof(float) * (d_fft_size); // complex file write
                    filename.str("");

                    boost::filesystem::path p = d_dump_filename;
                    filename << p.parent_path().string()
                             << boost::filesystem::path::preferred_separator
                             << p.stem().string() << "_"
                             << d_gnss_synchro->System << "_"
                             << d_gnss_synchro->Signal << "_sat_"
                             << d_gnss_synchro->PRN << "_doppler_" << doppler
                             << p.extension().string();

                    DLOG(INFO) << "Writing ACQ out to " << filename.str();

                    d_dump_file.open(filename.str().c_str(),
                        std::ios::out | std::ios::binary);
                    d_dump_file.close();
                }
        }

    if (test_statistics > d_threshold)
        {
            d_state = 2;  // Positive acquisition

            // 6.1- Declare positive acquisition using a message port
            DLOG(INFO) << "positive acquisition";
            DLOG(INFO) << "satellite " << d_gnss_synchro->System << " "
                       << d_gnss_synchro->PRN;
            DLOG(INFO) << "sample_stamp " << d_sample_counter;
            DLOG(INFO) << "test statistics value " << test_statistics;
            DLOG(INFO) << "test statistics threshold " << d_threshold;
            DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
            DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
            DLOG(INFO) << "magnitude " << d_mag;
            DLOG(INFO) << "input signal power " << input_power;

            d_active = false;
            d_state = 0;

            acquisition_message = 1;
            this->message_port_pub(pmt::mp("events"),
                pmt::from_long(acquisition_message));
        }
    else
        {
            d_state = 3;  // Negative acquisition

            // 6.2- Declare negative acquisition using a message port
            DLOG(INFO) << "negative acquisition";
            DLOG(INFO) << "satellite " << d_gnss_synchro->System << " "
                       << d_gnss_synchro->PRN;
            DLOG(INFO) << "sample_stamp " << d_sample_counter;
            DLOG(INFO) << "test statistics value " << test_statistics;
            DLOG(INFO) << "test statistics threshold " << d_threshold;
            DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
            DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
            DLOG(INFO) << "magnitude " << d_mag;
            DLOG(INFO) << "input signal power " << input_power;

            d_active = false;
            d_state = 0;

            acquisition_message = 2;
            this->message_port_pub(pmt::mp("events"),
                pmt::from_long(acquisition_message));
        }

    acquisition_fpga_8sc->unblock_samples();

    acquisition_fpga_8sc->close_device();

    DLOG(INFO) << "Done. Consumed 1 item.";
}


int gps_pcps_acquisition_fpga_sc::general_work(int noutput_items,
    gr_vector_int &ninput_items, gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items __attribute__((unused)))
{
    // general work not used with the acquisition
    return noutput_items;
}
