/*!
 * \file gnss_sdr_sample_counter.cc
 * \brief Simple block to report the current receiver time based on the output of the tracking or telemetry blocks
 * \author Javier Arribas 2017. jarribas(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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

#include "gnss_sdr_sample_counter.h"
#include "gnss_synchro.h"
#include <gnuradio/io_signature.h>

gnss_sdr_sample_counter::gnss_sdr_sample_counter() : gr::sync_block("sample_counter",
                                                         gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
                                                         gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_out(pmt::mp("sample_counter"));
    last_T_rx_s = 0;
    report_interval_s = 1;         //default reporting 1 second
    flag_enable_send_msg = false;  //enable it for reporting time with asynchronous message
}


gnss_sdr_sample_counter_sptr gnss_sdr_make_sample_counter()
{
    gnss_sdr_sample_counter_sptr sample_counter_(new gnss_sdr_sample_counter());
    return sample_counter_;
}


int gnss_sdr_sample_counter::work(int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items __attribute__((unused)))
{
    const Gnss_Synchro *in = reinterpret_cast<const Gnss_Synchro *>(input_items[0]);  // input

    double current_T_rx_s = in[noutput_items - 1].Tracking_sample_counter / static_cast<double>(in[noutput_items - 1].fs);
    if ((current_T_rx_s - last_T_rx_s) > report_interval_s)
        {
            std::cout << "Current receiver time: " << floor(current_T_rx_s) << " [s]" << std::endl;
            if (flag_enable_send_msg == true)
                {
                    this->message_port_pub(pmt::mp("receiver_time"), pmt::from_double(current_T_rx_s));
                }
            last_T_rx_s = current_T_rx_s;
        }
    return noutput_items;
}
