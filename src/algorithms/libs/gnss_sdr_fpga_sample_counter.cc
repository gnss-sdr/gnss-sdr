/*!
 * \file gnss_sdr_fpga_sample_counter.cc
 * \brief Simple block to report the current receiver time based on the output of the tracking or telemetry blocks
 * \author Javier Arribas 2018. jarribas(at)cttc.es
 *
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

#include "gnss_sdr_fpga_sample_counter.h"
#include "gnss_synchro.h"
#include <gnuradio/io_signature.h>
#include <cmath>
#include <iostream>
#include <string>
#include <glog/logging.h>
#include <fcntl.h>     // libraries used by the GIPO
#include <sys/mman.h>  // libraries used by the GIPO

#include <inttypes.h>

#define PAGE_SIZE 0x10000                     // default page size for the multicorrelator memory map
#define TEST_REG_SANITY_CHECK 0x55AA          // value to check the presence of the test register (to detect the hw)

gnss_sdr_fpga_sample_counter::gnss_sdr_fpga_sample_counter(double _fs, int32_t _interval_ms) : gr::block("fpga_fpga_sample_counter",
                                                                                                   gr::io_signature::make(0, 0, 0),
                                                                                                   gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    message_port_register_out(pmt::mp("fpga_sample_counter"));
    set_max_noutput_items(1);
    interval_ms = _interval_ms;
    fs = _fs;
    //printf("CREATOR fs =  %f\n", fs);
    //printf("CREATOR interval_ms = %" PRIu32 "\n", interval_ms);
    samples_per_output = std::round(fs * static_cast<double>(interval_ms) / 1e3);
    //printf("CREATOR samples_per_output =  %" PRIu32 "\n", samples_per_output);
    //todo: Load here the hardware counter register with this amount of samples. It should produce an
    //interrupt every samples_per_output count.
    //The hardware timer must keep always interrupting the PS. It must not wait for the interrupt to
    //be served.
    open_device();

    sample_counter = 0ULL;
    current_T_rx_ms = 0;
    current_s = 0;
    current_m = 0;
    current_h = 0;
    current_days = 0;
    report_interval_ms = 1000;     // default reporting 1 second
    flag_enable_send_msg = false;  // enable it for reporting time with asynchronous message
    flag_m = false;
    flag_h = false;
    flag_days = false;
}


gnss_sdr_fpga_sample_counter_sptr gnss_sdr_make_fpga_sample_counter(double _fs, int32_t _interval_ms)
{
    gnss_sdr_fpga_sample_counter_sptr fpga_sample_counter_(new gnss_sdr_fpga_sample_counter(_fs, _interval_ms));
    return fpga_sample_counter_;
}


// Called by gnuradio to enable drivers, etc for i/o devices.
bool gnss_sdr_fpga_sample_counter::start()
{
    //todo: place here the RE-INITIALIZATION routines. This function will be called by GNURadio at every start of the flowgraph.

	// configure the number of samples per output in the FPGA and enable the interrupts
    configure_samples_per_output(samples_per_output);

	// return true if everything is ok.
    return true;
}


// Called by GNURadio to disable drivers, etc for i/o devices.
bool gnss_sdr_fpga_sample_counter::stop()
{
    //todo: place here the routines to stop the associated hardware (if needed).This function will be called by GNURadio at every stop of the flowgraph.
    // return true if everything is ok.
	close_device();

    return true;
}


int gnss_sdr_fpga_sample_counter::general_work(int noutput_items __attribute__((unused)),
    __attribute__((unused)) gr_vector_int &ninput_items,
    __attribute__((unused)) gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    //todo: Call here a function that waits for an interrupt. Do not open a thread,
    //it must be a simple call to a BLOCKING function.
    // The function will return the actual absolute sample count of the internal counter of the timmer.
    // store the sample count in class member sample_counter
    // Possible problem: what happen if the PS is overloaded and gnuradio does not call this function
    // with the sufficient rate to catch all the interrupts in the counter. To be evaluated later.

	uint32_t counter = wait_for_interrupt_and_read_counter();
	uint64_t samples_passed = 2*static_cast<uint64_t>(samples_per_output) - static_cast<uint64_t>(counter); // ellapsed samples
	//printf("============================================ interrupter : samples_passed = %" PRIu64 "\n", samples_passed);
	// Note: at this moment the sample counter is implemented as a sample counter that decreases to zero and then it is automatically
	// reloaded again and keeps counter. It is done in this way to minimize the logic in the FPGA and maximize the FPGA clock performance
	// (it takes less resources and latency in the FPGA to compare a number against a fixed value like zero than to compare it to a programmable
	// variable number).

	sample_counter = sample_counter + samples_passed; //samples_per_output;
    Gnss_Synchro *out = reinterpret_cast<Gnss_Synchro *>(output_items[0]);
    out[0] = Gnss_Synchro();
    out[0].Flag_valid_symbol_output = false;
    out[0].Flag_valid_word = false;
    out[0].Channel_ID = -1;
    out[0].fs = fs;
    if ((current_T_rx_ms % report_interval_ms) == 0)
        {
    		//printf("time to print sample_counter = %" PRIu64 "\n", sample_counter);
			//printf("time to print current Tx ms : %" PRIu64 "\n", current_T_rx_ms);
			//printf("time to print report_interval_ms : %" PRIu32 "\n", report_interval_ms);
			//printf("time to print %f\n", (current_T_rx_ms % report_interval_ms));
			current_s++;
            if ((current_s % 60) == 0)
                {
                    current_s = 0;
                    current_m++;
                    flag_m = true;
                    if ((current_m % 60) == 0)
                        {
                            current_m = 0;
                            current_h++;
                            flag_h = true;
                            if ((current_h % 24) == 0)
                                {
                                    current_h = 0;
                                    current_days++;
                                    flag_days = true;
                                }
                        }
                }

            if (flag_days)
                {
                    std::string day;
                    if (current_days == 1)
                        {
                            day = " day ";
                        }
                    else
                        {
                            day = " days ";
                        }
                    std::cout << "Current receiver time: " << current_days << day << current_h << " h " << current_m << " min " << current_s << " s" << std::endl;
                }
            else
                {
                    if (flag_h)
                        {
                            std::cout << "Current receiver time: " << current_h << " h " << current_m << " min " << current_s << " s" << std::endl;
                        }
                    else
                        {
                            if (flag_m)
                                {
                                    std::cout << "Current receiver time: " << current_m << " min " << current_s << " s" << std::endl;
                                }
                            else
                                {
                                    std::cout << "Current receiver time: " << current_s << " s" << std::endl;
                                }
                        }
                }
            if (flag_enable_send_msg)
                {
                    message_port_pub(pmt::mp("receiver_time"), pmt::from_double(static_cast<double>(current_T_rx_ms) / 1000.0));
                }
        }
    out[0].Tracking_sample_counter = sample_counter;
    //current_T_rx_ms = (sample_counter * 1000) / samples_per_output;
    current_T_rx_ms = interval_ms*(sample_counter) / samples_per_output;
    return 1;
}

uint32_t gnss_sdr_fpga_sample_counter::test_register(uint32_t writeval)
{
    uint32_t readval;
    // write value to test register
    map_base[3] = writeval;
    // read value from test register
    readval = map_base[3];
    // return read value
    return readval;
}

void gnss_sdr_fpga_sample_counter::configure_samples_per_output(uint32_t interval)
{
	// note : the counter is a 48-bit value in the HW.
	//printf("============================================ total counter - interrupted interval : %" PRIu32 "\n", interval);
	//uint64_t temp_interval;
	//temp_interval = (interval & static_cast<uint32_t>(0xFFFFFFFF));
	//printf("LSW counter - interrupted interval : %" PRIu32 "\n", static_cast<uint32_t>(temp_interval));
	//map_base[0] = static_cast<uint32_t>(temp_interval);
	map_base[0] = interval - 1;
	//temp_interval = (interval >> 32) & static_cast<uint32_t>(0xFFFFFFFF);
	//printf("MSbits counter - interrupted interval : %" PRIu32 "\n", static_cast<uint32_t>(temp_interval));
	//map_base[1] = static_cast<uint32_t>(temp_interval); // writing the most significant bits also enables the interrupts
}

void gnss_sdr_fpga_sample_counter::open_device()
{
    // open communication with HW accelerator
    if ((fd = open(device_name.c_str(), O_RDWR | O_SYNC)) == -1)
        {
            LOG(WARNING) << "Cannot open deviceio" << device_name;
            std::cout << "Counter-Intr: cannot open deviceio" << device_name << std::endl;
        }
    map_base = reinterpret_cast<volatile uint32_t *>(mmap(NULL, PAGE_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));

    if (map_base == reinterpret_cast<void *>(-1))
        {
            LOG(WARNING) << "Cannot map the FPGA acquisition module into user memory";
            std::cout << "Counter-Intr: cannot map deviceio" << device_name << std::endl;
        }

    // sanity check : check test register
    uint32_t writeval = TEST_REG_SANITY_CHECK;
    uint32_t readval;
    readval = gnss_sdr_fpga_sample_counter::test_register(writeval);
    if (writeval != readval)
        {
            LOG(WARNING) << "Acquisition test register sanity check failed";
        }
    else
        {
            LOG(INFO) << "Acquisition test register sanity check success!";
            //std::cout << "Acquisition test register sanity check success!" << std::endl;
        }
}

void gnss_sdr_fpga_sample_counter::close_device()
{
	//printf("=========================================== NOW closing device ...\n");
	map_base[2] = 0; // disable the generation of the interrupt in the device

    uint32_t *aux = const_cast<uint32_t *>(map_base);
    if (munmap(static_cast<void *>(aux), PAGE_SIZE) == -1)
        {
            printf("Failed to unmap memory uio\n");
        }
    close(fd);
}

uint32_t gnss_sdr_fpga_sample_counter::wait_for_interrupt_and_read_counter()
{
    int32_t irq_count;
    ssize_t nb;
    int32_t counter;

    // enable interrupts
    int32_t reenable = 1;
    write(fd, reinterpret_cast<void *>(&reenable), sizeof(int32_t));

    // wait for interrupt
    //printf("============================================ interrupter : going to wait for interupt\n");
    nb = read(fd, &irq_count, sizeof(irq_count));
    //printf("============================================ interrupter : interrupt received\n");
    //printf("interrupt received\n");
    if (nb != sizeof(irq_count))
        {
            printf("acquisition module Read failed to retrieve 4 bytes!\n");
            printf("acquisition module Interrupt number %d\n", irq_count);
        }

    // it is a rising edge interrupt, the interrupt does not need to be acknowledged
    //map_base[1] = 0; // writing anything to reg 1 acknowledges the interrupt

    // add number of passed samples or read the current counter value for more accuracy
    counter = samples_per_output; //map_base[0];
    return counter;

}


