/*!
 * \file pcps_acquisition_fpga.cc
 * \brief This class implements a Parallel Code Phase Search Acquisition in the FPGA
 *
 * Note: The CFAR algorithm is not implemented in the FPGA.
 * Note 2: The bit transition flag is not implemented in the FPGA
 *
 * \authors <ul>
 *          <li> Marc Majoral, 2017. mmajoral(at)cttc.cat
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          <li> Marc Molina, 2013. marc.molina.pena@gmail.com
 *          <li> Cillian O'Driscoll, 2017. cillian(at)ieee.org
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


#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include "pcps_acquisition_fpga.h"


#define AQ_DOWNSAMPLING_DELAY 40 // delay due to the downsampling filter in the acquisition

using google::LogMessage;

pcps_acquisition_fpga_sptr pcps_make_acquisition_fpga(pcpsconf_fpga_t conf_)
{
    return pcps_acquisition_fpga_sptr(new pcps_acquisition_fpga(conf_));
}


pcps_acquisition_fpga::pcps_acquisition_fpga(pcpsconf_fpga_t conf_) : gr::block("pcps_acquisition_fpga",
                                                           gr::io_signature::make(0, 0, 0),
                                                           gr::io_signature::make(0, 0, 0))
{
 //   printf("acq constructor start\n");
    this->message_port_register_out(pmt::mp("events"));

    acq_parameters = conf_;
    d_sample_counter = 0;  // SAMPLE COUNTER
    d_active = false;
    d_state = 0;
    //d_fft_size = acq_parameters.sampled_ms * acq_parameters.samples_per_ms;
    d_fft_size = acq_parameters.samples_per_code;
    d_mag = 0;
    d_input_power = 0.0;
    d_num_doppler_bins = 0;
    d_threshold = 0.0;
    d_doppler_step = 0;
    d_test_statistics = 0.0;
    d_channel = 0;
    d_gnss_synchro = 0;

    //printf("zzzz acq_parameters.code_length = %d\n", acq_parameters.code_length);
    //printf("zzzz acq_parameters.samples_per_ms = %d\n", acq_parameters.samples_per_ms);
    //printf("zzzz d_fft_size = %d\n", d_fft_size);

    // this one works we don't know why
//    acquisition_fpga = std::make_shared <fpga_acquisition>
//          (acq_parameters.device_name, acq_parameters.code_length, acq_parameters.doppler_max, acq_parameters.samples_per_ms,
//                  acq_parameters.fs_in, acq_parameters.freq, acq_parameters.sampled_ms, acq_parameters.select_queue_Fpga, acq_parameters.all_fft_codes);

    // this one is the one it should be but it doesn't work
    acquisition_fpga = std::make_shared <fpga_acquisition>
          (acq_parameters.device_name, acq_parameters.code_length, acq_parameters.doppler_max, d_fft_size,
                  acq_parameters.fs_in, acq_parameters.freq, acq_parameters.sampled_ms, acq_parameters.select_queue_Fpga, acq_parameters.all_fft_codes);

//    acquisition_fpga = std::make_shared <fpga_acquisition>
//          (acq_parameters.device_name, acq_parameters.samples_per_code, acq_parameters.doppler_max, acq_parameters.samples_per_code,
//                  acq_parameters.fs_in, acq_parameters.freq, acq_parameters.sampled_ms, acq_parameters.select_queue_Fpga, acq_parameters.all_fft_codes);

    // debug
    //debug_d_max_absolute = 0.0;
    //debug_d_input_power_absolute = 0.0;
  //  printf("acq constructor end\n");
}


pcps_acquisition_fpga::~pcps_acquisition_fpga()
{
  //  printf("acq destructor start\n");
    acquisition_fpga->free();
  //  printf("acq destructor end\n");
}


void pcps_acquisition_fpga::set_local_code()
{
 //   printf("acq set local code start\n");
    acquisition_fpga->set_local_code(d_gnss_synchro->PRN);
 //   printf("acq set local code end\n");
}


void pcps_acquisition_fpga::init()
{
  //  printf("acq init start\n");
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

    acquisition_fpga->init();
  //  printf("acq init end\n");
}


void pcps_acquisition_fpga::set_state(int state)
{
 //   printf("acq set state start\n");
    d_state = state;
    if (d_state == 1)
        {
            d_gnss_synchro->Acq_delay_samples = 0.0;
            d_gnss_synchro->Acq_doppler_hz = 0.0;
            d_gnss_synchro->Acq_samplestamp_samples = 0;
            //d_well_count = 0;
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
 //   printf("acq set state end\n");
}


void pcps_acquisition_fpga::send_positive_acquisition()
{
//    printf("acq send positive acquisition start\n");
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
//    printf("acq send positive acquisition end\n");
}


void pcps_acquisition_fpga::send_negative_acquisition()
{
 //   printf("acq send negative acquisition start\n");
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
//    printf("acq send negative acquisition end\n");
}


void pcps_acquisition_fpga::set_active(bool active)
{
 //   printf("acq set active start\n");
    d_active = active;

    // initialize acquisition algorithm
    uint32_t indext = 0;
    float magt = 0.0;
    float fft_normalization_factor = static_cast<float>(d_fft_size) * static_cast<float>(d_fft_size);

    d_input_power = 0.0;
    d_mag = 0.0;

    DLOG(INFO) << "Channel: " << d_channel
               << " , doing acquisition of satellite: " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
               << " ,sample stamp: " << d_sample_counter << ", threshold: "
               << d_threshold << ", doppler_max: " << acq_parameters.doppler_max
               << ", doppler_step: " << d_doppler_step
               // no CFAR algorithm in the FPGA
               << ", use_CFAR_algorithm_flag: false";

    unsigned int initial_sample;
    float input_power_all = 0.0;
    float input_power_computed = 0.0;

    float temp_d_input_power;

    // loop through acquisition
/*
    for (unsigned int doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            // doppler search steps
            int doppler = -static_cast<int>(acq_parameters.doppler_max) + d_doppler_step * doppler_index;

            //acquisition_fpga->set_phase_step(doppler_index);
            acquisition_fpga->set_doppler_sweep_debug(1, doppler_index);
            acquisition_fpga->run_acquisition(); // runs acquisition and waits until it is finished
            acquisition_fpga->read_acquisition_results(&indext, &magt,
                    &initial_sample, &d_input_power, &d_doppler_index);
            d_sample_counter = initial_sample;

            if (d_mag < magt)
                {
                    d_mag = magt;

                    temp_d_input_power = d_input_power;

                    input_power_all = d_input_power / (d_fft_size - 1);
                    input_power_computed = (d_input_power - d_mag) / (d_fft_size - 1);
                    d_input_power = (d_input_power - d_mag) / (d_fft_size - 1);

                    d_gnss_synchro->Acq_delay_samples = static_cast<double>(indext % acq_parameters.samples_per_code);
                    d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
                    d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter;

                    d_test_statistics = (d_mag / d_input_power); //* correction_factor;
                }

            // In the case of the FPGA the option of dumping the results of the acquisition to a file is not available
            // because the IFFT vector is not available
        }
*/

    // debug
    //acquisition_fpga->block_samples();

    // run loop in hw
    acquisition_fpga->set_doppler_sweep(d_num_doppler_bins);
    acquisition_fpga->run_acquisition();
    acquisition_fpga->read_acquisition_results(&indext, &magt,
            &initial_sample, &d_input_power, &d_doppler_index);


    // debug
    //acquisition_fpga->unblock_samples();

    d_mag = magt;


    // debug
    debug_d_max_absolute = magt;
    debug_d_input_power_absolute = d_input_power;
    debug_indext = indext;
    debug_doppler_index = d_doppler_index;

  //  temp_d_input_power = d_input_power;

    d_input_power = (d_input_power - d_mag) / (d_fft_size - 1);
    int doppler = -static_cast<int>(acq_parameters.doppler_max) + d_doppler_step * d_doppler_index;
    //d_gnss_synchro->Acq_delay_samples = static_cast<double>(2*(indext % (2*acq_parameters.samples_per_code)));
    d_gnss_synchro->Acq_delay_samples = static_cast<double>(indext % acq_parameters.samples_per_code);
    d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
    d_sample_counter = initial_sample;
    //d_gnss_synchro->Acq_samplestamp_samples = 2*d_sample_counter - 81; // delay due to the downsampling filter in the acquisition
    //d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter - 40; // delay due to the downsampling filter in the acquisition
    d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter; // delay due to the downsampling filter in the acquisition
    d_test_statistics = (d_mag / d_input_power); //* correction_factor;

    // debug
//    if (d_gnss_synchro->Acq_delay_samples > acq_parameters.code_length)
//        {
//            printf("d_gnss_synchro->Acq_samplestamp_samples = %d\n", d_gnss_synchro->Acq_samplestamp_samples);
//            printf("d_gnss_synchro->Acq_delay_samples = %f\n", d_gnss_synchro->Acq_delay_samples);
//        }

   // if (temp_d_input_power > debug_d_input_power_absolute)
   //     {
   //         debug_d_max_absolute = d_mag;
   //         debug_d_input_power_absolute = temp_d_input_power;
   //     }
   // printf ("max debug_d_max_absolute = %f\n", debug_d_max_absolute);
   // printf ("debug_d_input_power_absolute = %f\n", debug_d_input_power_absolute);

//    printf("&&&&& d_test_statistics = %f\n", d_test_statistics);
//    printf("&&&&& debug_d_max_absolute =%f\n",debug_d_max_absolute);
//    printf("&&&&& debug_d_input_power_absolute =%f\n",debug_d_input_power_absolute);
//    printf("&&&&& debug_indext = %d\n",debug_indext);
//    printf("&&&&& debug_doppler_index = %d\n",debug_doppler_index);

    if (d_test_statistics > d_threshold)
        {
            d_active = false;
            printf("##### d_test_statistics = %f\n", d_test_statistics);
            printf("##### debug_d_max_absolute =%f\n",debug_d_max_absolute);
            printf("##### debug_d_input_power_absolute =%f\n",debug_d_input_power_absolute);
            printf("##### initial_sample = %d\n",initial_sample);
            printf("##### debug_doppler_index = %d\n",debug_doppler_index);
            send_positive_acquisition();
            d_state = 0;  // Positive acquisition
        }
    else
        {
            d_state = 0;
            d_active = false;
            send_negative_acquisition();
        }

 //   printf("acq set active end\n");

}


int pcps_acquisition_fpga::general_work(int noutput_items __attribute__((unused)),
    gr_vector_int& ninput_items, gr_vector_const_void_star& input_items,
    gr_vector_void_star& output_items __attribute__((unused)))
{
    // the general work is not used with the acquisition that uses the FPGA
    return noutput_items;
}
