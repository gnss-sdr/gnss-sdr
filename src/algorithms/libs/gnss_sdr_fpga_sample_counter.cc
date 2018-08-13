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

gnss_sdr_fpga_sample_counter::gnss_sdr_fpga_sample_counter(double _fs, int32_t _interval_ms) : gr::block("fpga_fpga_sample_counter",
                                                                                                   gr::io_signature::make(0, 0, 0),
                                                                                                   gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    message_port_register_out(pmt::mp("fpga_sample_counter"));
    set_max_noutput_items(1);
    interval_ms = _interval_ms;
    fs = _fs;
    samples_per_output = std::round(fs * static_cast<double>(interval_ms) / 1e3);
    //todo: Load here the hardware counter register with this amount of samples. It should produce an
    //interrupt every samples_per_output count.
    //The hardware timer must keep always interrupting the PS. It must not wait for the interrupt to
    //be served.

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
    // return true if everything is ok.
    return true;
}


// Called by GNURadio to disable drivers, etc for i/o devices.
bool gnss_sdr_fpga_sample_counter::stop()
{
    //todo: place here the routines to stop the associated hardware (if needed).This function will be called by GNURadio at every stop of the flowgraph.
    // return true if everything is ok.
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

    Gnss_Synchro *out = reinterpret_cast<Gnss_Synchro *>(output_items[0]);
    out[0] = Gnss_Synchro();
    out[0].Flag_valid_symbol_output = false;
    out[0].Flag_valid_word = false;
    out[0].Channel_ID = -1;
    out[0].fs = fs;
    if ((current_T_rx_ms % report_interval_ms) == 0)
        {
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
    current_T_rx_ms = (sample_counter * 1000) / samples_per_output;
    return 1;
}
