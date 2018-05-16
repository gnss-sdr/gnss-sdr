/*!
 * \file gnss_sdr_sample_counter.cc
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

#include "gnss_sdr_sample_counter.h"
#include "gnss_synchro.h"
#include <gnuradio/io_signature.h>
#include <cmath>
#include <iostream>
#include <string>

gnss_sdr_sample_counter::gnss_sdr_sample_counter(double _fs, size_t _size) : gr::sync_decimator("sample_counter",
                                                                                 gr::io_signature::make(1, 1, _size),
                                                                                 gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
                                                                                 static_cast<unsigned int>(std::floor(_fs * 0.001)))
{
    message_port_register_out(pmt::mp("sample_counter"));
    set_max_noutput_items(1);
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


gnss_sdr_sample_counter_sptr gnss_sdr_make_sample_counter(double _fs, size_t _size)
{
    gnss_sdr_sample_counter_sptr sample_counter_(new gnss_sdr_sample_counter(_fs, _size));
    return sample_counter_;
}


int gnss_sdr_sample_counter::work(int noutput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items __attribute__((unused)),
    gr_vector_void_star &output_items)
{
    Gnss_Synchro *out = reinterpret_cast<Gnss_Synchro *>(output_items[0]);
    out[0] = Gnss_Synchro();
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
    current_T_rx_ms++;
    return 1;
}
