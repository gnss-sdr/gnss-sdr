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

#include <unistd.h> // for the usleep function only (debug)

#define AQ_DOWNSAMPLING_DELAY 40  // delay due to the downsampling filter in the acquisition

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
    d_sample_counter = 0ULL;  // SAMPLE COUNTER
    d_active = false;
    d_state = 0;
    //d_fft_size = acq_parameters.sampled_ms * acq_parameters.samples_per_ms;
    d_fft_size = acq_parameters.samples_per_code;
    d_mag = 0;
    d_input_power = 0.0;
    d_num_doppler_bins = 0U;
    d_threshold = 0.0;
    d_doppler_step = 0U;
    d_test_statistics = 0.0;
    d_channel = 0U;
    d_gnss_synchro = 0;
    d_single_doppler_flag = false;

    d_downsampling_factor = acq_parameters.downsampling_factor;
    //printf("AAAAAAAAAA downsampling_factor = %f\n", d_downsampling_factor);
    d_select_queue_Fpga = acq_parameters.select_queue_Fpga;
    //printf("zzzz acq_parameters.code_length = %d\n", acq_parameters.code_length);
    //printf("zzzz acq_parameters.samples_per_ms = %d\n", acq_parameters.samples_per_ms);
    //printf("zzzz d_fft_size = %d\n", d_fft_size);

    // this one works we don't know why
    //    acquisition_fpga = std::make_shared <fpga_acquisition>
    //          (acq_parameters.device_name, acq_parameters.code_length, acq_parameters.doppler_max, acq_parameters.samples_per_ms,
    //                  acq_parameters.fs_in, acq_parameters.freq, acq_parameters.sampled_ms, acq_parameters.select_queue_Fpga, acq_parameters.all_fft_codes);


    d_total_block_exp = acq_parameters.total_block_exp;

    // this one is the one it should be but it doesn't work
    acquisition_fpga = std::make_shared<fpga_acquisition>(acq_parameters.device_name, acq_parameters.code_length, acq_parameters.doppler_max, d_fft_size,
        acq_parameters.fs_in, acq_parameters.sampled_ms, acq_parameters.select_queue_Fpga, acq_parameters.all_fft_codes, acq_parameters.excludelimit);

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

    if (d_single_doppler_flag == 1)
    {
    	d_num_doppler_bins = 1;
    }
    else
    {
    	d_num_doppler_bins = static_cast<uint32_t>(std::ceil(static_cast<double>(static_cast<int32_t>(acq_parameters.doppler_max) - static_cast<int32_t>(-acq_parameters.doppler_max)) / static_cast<double>(d_doppler_step))) + 1;
    }
    	//printf("acq gnuradioblock doppler_max = %lu\n", (unsigned long) static_cast<int32_t>(acq_parameters.doppler_max));
    //printf("acq gnuradioblock doppler_step = %lu\n", (unsigned long) d_doppler_step);
    //printf("acq gnuradioblock d_num_doppler_bins = %lu\n", (unsigned long) d_num_doppler_bins);
    acquisition_fpga->init();
    //  printf("acq init end\n");
}


void pcps_acquisition_fpga::set_state(int32_t state)
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

    //printf("acq sending positive acquisition\n");
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

    //printf("acq sending negative acquisition\n");
    this->message_port_pub(pmt::mp("events"), pmt::from_long(2));
    //    printf("acq send negative acquisition end\n");
}


void pcps_acquisition_fpga::set_active(bool active)
{

    //   printf("acq set active start\n");
    d_active = active;

    // initialize acquisition algorithm
    uint32_t indext = 0U;
    float firstpeak = 0.0;
    float secondpeak = 0.0;
    uint32_t total_block_exp;
    //float fft_normalization_factor = static_cast<float>(d_fft_size) * static_cast<float>(d_fft_size);

    d_input_power = 0.0;
    d_mag = 0.0;

    DLOG(INFO) << "Channel: " << d_channel
               << " , doing acquisition of satellite: " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
               << " ,sample stamp: " << d_sample_counter << ", threshold: "
               << d_threshold << ", doppler_max: " << acq_parameters.doppler_max
               << ", doppler_step: " << d_doppler_step
               // no CFAR algorithm in the FPGA
               << ", use_CFAR_algorithm_flag: false";

    uint64_t initial_sample;
    float input_power_all = 0.0;
    float input_power_computed = 0.0;

    float temp_d_input_power;

    // debug
    //acquisition_fpga->block_samples();


   // while(1)
   //{

    //printf("######### acq ENTERING SET ACTIVE\n");

    // run loop in hw
    //printf("LAUNCH ACQ\n");
    //printf("acq lib d_num_doppler_bins = %d\n", d_num_doppler_bins);
    //printf("writing config for channel %d -----------------------------------------\n", (int) d_channel);
    acquisition_fpga->configure_acquisition();
    acquisition_fpga->set_doppler_sweep(d_num_doppler_bins);

    //printf("d_num_doppler_bins = %d\n", (int) d_num_doppler_bins);
    acquisition_fpga->write_local_code();

    //acquisition_fpga->set_doppler_sweep(2);
    //printf("acq lib going to launch acquisition\n");
    acquisition_fpga->set_block_exp(d_total_block_exp);

    //printf("running acq for channel %d\n", (int) d_channel);

    acquisition_fpga->run_acquisition();
    //printf("acq lib going to read the acquisition results\n");
    //read_acquisition_results(&indext, &firstpeak, &secondpeak, &initial_sample, &d_input_power, &d_doppler_index);

    //printf("reading results for channel %d\n", (int) d_channel);
    acquisition_fpga->read_acquisition_results(&indext, &firstpeak, &secondpeak, &initial_sample, &d_input_power, &d_doppler_index, &total_block_exp);

    //printf("returned d_doppler_index = %d\n", d_doppler_index);

    //printf("gnuradio block : d_total_block_exp = %d total_block_exp = %d\n", (int) d_total_block_exp, (int) total_block_exp);

    if (total_block_exp > d_total_block_exp)
    {
    	printf("changing blk exp..... d_total_block_exp = %d total_block_exp = %d chan = %d\n", d_total_block_exp, total_block_exp, d_channel);
    	d_total_block_exp = total_block_exp;

    }

    //printf("end channel %d -----------------------------------------------------\n", (int) d_channel);
    //printf("READ ACQ RESULTS\n");

    // debug
    //acquisition_fpga->unblock_samples();


    //usleep(5000000);
    //} // end while test

    int32_t doppler;

    // NEW SATELLITE DETECTION ALGORITHM STARTS HERE ----------------------------------------------------

	if (d_single_doppler_flag == false)
	{
		doppler = -static_cast<int32_t>(acq_parameters.doppler_max) + d_doppler_step * (d_doppler_index - 1);
		//doppler = -static_cast<int32_t>(acq_parameters.doppler_max) + d_doppler_step * (d_doppler_index); // this is the wrong one
	}
	else
	{
		doppler = static_cast<int32_t>(acq_parameters.doppler_max);
	}

	if (secondpeak > 0)
	{
		d_test_statistics = firstpeak/secondpeak;
	}
	else
	{
		d_test_statistics = 0.0;
	}

//    // OLD SATELLITE DETECTION ALGORITHM STARTS HERE ----------------------------------------------------
//
//    d_mag = magt;
//
//
//    // debug
//    //debug_d_max_absolute = magt;
//    //debug_d_input_power_absolute = d_input_power;
//    //debug_indext = indext;
//    //debug_doppler_index = d_doppler_index;
//
//    //  temp_d_input_power = d_input_power;
//
//    d_input_power = (d_input_power - d_mag) / (d_fft_size - 1);
//    //int32_t doppler;
//	if (d_single_doppler_flag == false)
//	{
//		doppler = -static_cast<int32_t>(acq_parameters.doppler_max) + d_doppler_step * (d_doppler_index - 1);
//		//doppler = -static_cast<int32_t>(acq_parameters.doppler_max) + d_doppler_step * (d_doppler_index); // this is the wrong one
//	}
//	else
//	{
//		doppler = static_cast<int32_t>(acq_parameters.doppler_max);
//	}
//    //d_gnss_synchro->Acq_delay_samples = static_cast<double>(2*(indext % (2*acq_parameters.samples_per_code)));
//
//
//    //printf("acq gnuradioblock doppler = %d\n", doppler);
//
//	// END OF OLD SATELLITE ALGORITHM --------------------------------------------------------------------

    d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
    d_sample_counter = initial_sample;

    if (d_select_queue_Fpga == 0)
    {
    	if (d_downsampling_factor > 1)
    	{
    		//printf("yes here\n");
    		d_gnss_synchro->Acq_delay_samples = static_cast<double>(d_downsampling_factor*(indext % acq_parameters.samples_per_code));
    		//d_gnss_synchro->Acq_samplestamp_samples = d_downsampling_factor*d_sample_counter - 81*0.25*d_downsampling_factor; // delay due to the downsampling filter in the acquisition
    		d_gnss_synchro->Acq_samplestamp_samples = d_downsampling_factor*d_sample_counter - 44; //33; //41; //+ 81*0.5; // delay due to the downsampling filter in the acquisition
    		//d_gnss_synchro->Acq_samplestamp_samples = d_downsampling_factor*d_sample_counter - 81/d_downsampling_factor; // delay due to the downsampling filter in the acquisition
    		//d_gnss_synchro->Acq_delay_samples = static_cast<double>(2*(indext % acq_parameters.samples_per_code));
    		//d_gnss_synchro->Acq_delay_samples = static_cast<double>(2*(indext));
    		//d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter*2 - 81;
        	//d_gnss_synchro->Acq_delay_samples = static_cast<double>(indext % acq_parameters.samples_per_code);
        	//d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter;

    	}
    	else
    	{
    		//printf("xxxxxxxxxxxxxxxx no here\n");
        	d_gnss_synchro->Acq_delay_samples = static_cast<double>(indext % acq_parameters.samples_per_code);
        	d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter;  // delay due to the downsampling filter in the acquisition
        	//d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter - 40;  // delay due to the downsampling filter in the acquisition
        	//d_gnss_synchro->Acq_samplestamp_samples = d_downsampling_factor*d_sample_counter - 81*0.5*d_downsampling_factor;
        }
    }
    else
    {
    	d_gnss_synchro->Acq_delay_samples = static_cast<double>(indext % acq_parameters.samples_per_code);
    	d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter;  // delay due to the downsampling filter in the acquisition
    }



    //d_gnss_synchro->Acq_samplestamp_samples = 2*d_sample_counter - 81; // delay due to the downsampling filter in the acquisition
    //d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter - 40; // delay due to the downsampling filter in the acquisition

//    // OLD SATELLITE DETECTION ALGORITHM STARTS HERE AGAIN -----------------------------------------------
//
//    d_test_statistics = (d_mag / d_input_power);                 //* correction_factor;
//
//    // END OF OLD SATELLITE ALGORITHM AGAIN --------------------------------------------------------------------



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
            //            printf("##### d_test_statistics = %f\n", d_test_statistics);
            //            printf("##### debug_d_max_absolute =%f\n",debug_d_max_absolute);
            //            printf("##### debug_d_input_power_absolute =%f\n",debug_d_input_power_absolute);
            //            printf("##### initial_sample = %llu\n",initial_sample);
            //            printf("##### debug_doppler_index = %d\n",debug_doppler_index);
            send_positive_acquisition();
            d_state = 0;  // Positive acquisition

            // printf("acq d_gnss_synchro->Acq_delay_samples = %f\n: ",d_gnss_synchro->Acq_delay_samples);
            // printf("acq d_gnss_synchro->Acq_samplestamp_samples = %d\n", (unsigned int) d_gnss_synchro->Acq_samplestamp_samples);
            // printf("acq d_gnss_synchro->Acq_doppler_hz = %f\n", d_gnss_synchro->Acq_doppler_hz);
            // printf("acq d_gnss_synchro->PRN = %d\n", (int) d_gnss_synchro->PRN);
        }
    else
        {
            d_state = 0;
            d_active = false;
            send_negative_acquisition();
        }

    //printf("######### acq LEAVING SET ACTIVE\n");
    //printf("acq set active end\n");
}


int pcps_acquisition_fpga::general_work(int noutput_items __attribute__((unused)),
    gr_vector_int& ninput_items, gr_vector_const_void_star& input_items,
    gr_vector_void_star& output_items __attribute__((unused)))
{
	//printf("ACQ GENERAL WORK CALLED\n");
    // the general work is not used with the acquisition that uses the FPGA
    return noutput_items;
}


// this function is only used for the unit tests
void pcps_acquisition_fpga::set_single_doppler_flag(unsigned int single_doppler_flag)
{
	acquisition_fpga->set_single_doppler_flag(single_doppler_flag);
	d_single_doppler_flag = true;
}

// this function is only used for the unit tests
void pcps_acquisition_fpga::read_acquisition_results(uint32_t *max_index,
    float *max_magnitude, float *second_magnitude, uint64_t *initial_sample, uint32_t *doppler_index, uint32_t *total_fft_scaling_factor)
{
	float input_power; // not used
    acquisition_fpga->read_acquisition_results(max_index, max_magnitude, second_magnitude, initial_sample, &input_power, doppler_index, total_fft_scaling_factor);


    if (d_select_queue_Fpga == 0)
    {
    	if (d_downsampling_factor > 1)
    	{
    		//printf("yes here\n");
    		*max_index = static_cast<double>(d_downsampling_factor*(*max_index));
    		//d_gnss_synchro->Acq_samplestamp_samples = d_downsampling_factor*d_sample_counter - 81*0.25*d_downsampling_factor; // delay due to the downsampling filter in the acquisition
    		*initial_sample = d_downsampling_factor*(*initial_sample) - 44; //33; //41; //+ 81*0.5; // delay due to the downsampling filter in the acquisition
    		//d_gnss_synchro->Acq_samplestamp_samples = d_downsampling_factor*d_sample_counter - 81/d_downsampling_factor; // delay due to the downsampling filter in the acquisition
    		//d_gnss_synchro->Acq_delay_samples = static_cast<double>(2*(indext % acq_parameters.samples_per_code));
    		//d_gnss_synchro->Acq_delay_samples = static_cast<double>(2*(indext));
    		//d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter*2 - 81;
        	//d_gnss_synchro->Acq_delay_samples = static_cast<double>(indext % acq_parameters.samples_per_code);
        	//d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter;

    	}
    	else
    	{
    		//printf("xxxxxxxxxxxxxxxx no here\n");
    		//max_index = static_cast<double>(indext % acq_parameters.samples_per_code);
    		//initial_sample = d_sample_counter;  // delay due to the downsampling filter in the acquisition
        	//d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter - 40;  // delay due to the downsampling filter in the acquisition
        	//d_gnss_synchro->Acq_samplestamp_samples = d_downsampling_factor*d_sample_counter - 81*0.5*d_downsampling_factor;
        }
    }
//    else
//    {
//    	d_gnss_synchro->Acq_delay_samples = static_cast<double>(indext % acq_parameters.samples_per_code);
//    	d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter;  // delay due to the downsampling filter in the acquisition
//    }




//	acquisition_fpga->read_acquisition_results(max_index, max_magnitude,
//	        initial_sample, power_sum, doppler_index);
}

// this function is only used for the unit tests
void pcps_acquisition_fpga::reset_acquisition(void)
{
	acquisition_fpga->reset_acquisition();
}

void pcps_acquisition_fpga::read_fpga_total_scale_factor(uint32_t *total_scale_factor, uint32_t *fw_scale_factor)
{
	acquisition_fpga->read_fpga_total_scale_factor(total_scale_factor, fw_scale_factor);
}



