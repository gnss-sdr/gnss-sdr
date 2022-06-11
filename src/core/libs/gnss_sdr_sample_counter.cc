/*!
 * \file gnss_sdr_sample_counter.cc
 * \brief Simple block to report the current receiver time based on the output of the tracking or telemetry blocks
 * \author Javier Arribas 2018. jarribas(at)cttc.es
 *
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

#include "gnss_sdr_sample_counter.h"
#include "gnss_synchro.h"
#include "gnss_time.h"
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>        // for from_double
#include <pmt/pmt_sugar.h>  // for mp
#include <cmath>            // for round
#include <iostream>         // for operator<<
#include <memory>
#include <string>  // for string
#include <vector>

#if PMT_USES_BOOST_ANY
#include <boost/any.hpp>
namespace wht = boost;
#else
#include <any>
namespace wht = std;
#endif

gnss_sdr_sample_counter::gnss_sdr_sample_counter(
    double _fs,
    int32_t _interval_ms,
    size_t _size)
    : gr::sync_decimator("sample_counter",
          gr::io_signature::make(1, 1, _size),
          gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
          static_cast<uint32_t>(std::round(_fs * static_cast<double>(_interval_ms) / 1e3))),
      fs(_fs),
      current_T_rx_ms(0),
      sample_counter(0),
      interval_ms(_interval_ms),
      report_interval_ms(1000),  // default reporting 1 second
      samples_per_output(std::round(fs * static_cast<double>(interval_ms) / 1e3)),
      current_s(0),
      current_m(0),
      current_h(0),
      current_days(0),
      flag_m(false),
      flag_h(false),
      flag_days(false),
      flag_enable_send_msg(false)  // enable it for reporting time with asynchronous message
{
    message_port_register_out(pmt::mp("sample_counter"));
    set_max_noutput_items(1);
    set_tag_propagation_policy(TPP_DONT);  // no tag propagation, the time tag will be adjusted and regenerated in work()
}


gnss_sdr_sample_counter_sptr gnss_sdr_make_sample_counter(double _fs, int32_t _interval_ms, size_t _size)
{
    gnss_sdr_sample_counter_sptr sample_counter_(new gnss_sdr_sample_counter(_fs, _interval_ms, _size));
    return sample_counter_;
}


int64_t gnss_sdr_sample_counter::uint64diff(uint64_t first, uint64_t second)
{
    uint64_t abs_diff = (first > second) ? (first - second) : (second - first);
    assert(abs_diff <= INT64_MAX);
    return (first > second) ? (int64_t)abs_diff : -(int64_t)abs_diff;
}


int gnss_sdr_sample_counter::work(int noutput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items __attribute__((unused)),
    gr_vector_void_star &output_items)
{
    auto *out = reinterpret_cast<Gnss_Synchro *>(output_items[0]);
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
                    std::cout << "Current receiver time: " << current_days << day << current_h << " h " << current_m << " min " << current_s << " s\n";
                }
            else
                {
                    if (flag_h)
                        {
                            std::cout << "Current receiver time: " << current_h << " h " << current_m << " min " << current_s << " s\n";
                        }
                    else
                        {
                            if (flag_m)
                                {
                                    std::cout << "Current receiver time: " << current_m << " min " << current_s << " s\n";
                                }
                            else
                                {
                                    std::cout << "Current receiver time: " << current_s << " s\n";
                                }
                        }
                }
            if (flag_enable_send_msg)
                {
                    message_port_pub(pmt::mp("receiver_time"), pmt::from_double(static_cast<double>(current_T_rx_ms) / 1000.0));
                }
        }
    sample_counter += samples_per_output;
    out[0].Tracking_sample_counter = sample_counter;
    current_T_rx_ms += interval_ms;

    // *************** time tags ****************
    std::vector<gr::tag_t> tags_vec;
    // notice that nitems_read is updated in decimation blocks after leaving work() with return 1, equivalent to call consume_each
    this->get_tags_in_range(tags_vec, 0, this->nitems_read(0), this->nitems_read(0) + samples_per_output);
    for (const auto &it : tags_vec)
        {
            try
                {
                    if (pmt::any_ref(it.value).type().hash_code() == typeid(const std::shared_ptr<GnssTime>).hash_code())
                        {
                            // recompute timestamp to match the last sample in the consumed samples in this batch
                            int64_t diff_samplecount = uint64diff(out[0].Tracking_sample_counter, it.offset);
                            const auto last_timetag = wht::any_cast<const std::shared_ptr<GnssTime>>(pmt::any_ref(it.value));
                            double intpart;
                            last_timetag->tow_ms_fraction += modf(1000.0 * static_cast<double>(diff_samplecount) / fs, &intpart);

                            last_timetag->tow_ms = last_timetag->tow_ms + static_cast<int>(intpart);
                            last_timetag->rx_time = static_cast<double>(out[0].Tracking_sample_counter) / fs;
                            add_item_tag(0, this->nitems_written(0) + 1, pmt::mp("timetag"), pmt::make_any(last_timetag));
                            // std::cout << "COUNTER TAG: this->nitems_read(0):" << this->nitems_read(0) << " sample_counter:" << sample_counter
                            //          << " it->offset:" << it->offset << " diff:" << diff_samplecount << "\n";
                            // getchar();
                        }
                    else
                        {
                            std::cout << "hash code not match\n";
                        }
                }
            catch (const wht::bad_any_cast &e)
                {
                    std::cout << "msg Bad any_cast: " << e.what();
                }
            catch (const std::exception &ee)
                {
                    return 1;
                }
        }

    // ************ end time tags **************

    return 1;
}
