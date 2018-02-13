/*!
 * \file gnss_sdr_sample_counter.h
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
#ifndef GNSS_SDR_sample_counter_H_
#define GNSS_SDR_sample_counter_H_

#include <gnuradio/sync_decimator.h>
#include <boost/shared_ptr.hpp>


class gnss_sdr_sample_counter;

typedef boost::shared_ptr<gnss_sdr_sample_counter> gnss_sdr_sample_counter_sptr;

gnss_sdr_sample_counter_sptr gnss_sdr_make_sample_counter (double _fs);

class gnss_sdr_sample_counter : public gr::sync_decimator
{
    friend gnss_sdr_sample_counter_sptr gnss_sdr_make_sample_counter(double _fs);
    gnss_sdr_sample_counter (double _fs);
    long long int current_T_rx_ms;
    int report_interval_ms;
    bool flag_enable_send_msg;

public:
    int work(int noutput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items);
};

#endif /*GNSS_SDR_sample_counter_H_*/
