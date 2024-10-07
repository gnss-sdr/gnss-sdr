/*!
 * \file gnss_sdr_timestamp.h
 * \brief  GNURadio block that adds to sample stream timestamp metadata information stored on a sepparated file
 * \author Javier Arribas, 2021. jarribas(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "gnss_sdr_timestamp.h"
#include "command_event.h"
#include <gnuradio/io_signature.h>  // for io_signature
#include <pmt/pmt.h>                // for make_any
#include <pmt/pmt_sugar.h>          // for mp
#include <algorithm>                // for min
#include <cmath>
#include <cstring>  // for memcpy
#include <memory>
#include <utility>


Gnss_Sdr_Timestamp::Gnss_Sdr_Timestamp(size_t sizeof_stream_item,
    std::string timestamp_file, double clock_offset_ms, int items_to_samples)
    : gr::sync_block("Timestamp",
          gr::io_signature::make(1, 20, sizeof_stream_item),
          gr::io_signature::make(1, 20, sizeof_stream_item)),
      d_timefile(std::move(timestamp_file)),
      d_clock_offset_ms(clock_offset_ms),
      d_fraction_ms_offset(modf(d_clock_offset_ms, &d_integer_ms_offset)),  // optional clockoffset parameter to convert UTC timestamps to GPS time in some receiver's configuration
      d_items_to_samples(items_to_samples),
      d_next_timetag_samplecount(0),
      d_get_next_timetag(true)
{
}


gnss_shared_ptr<Gnss_Sdr_Timestamp> gnss_sdr_make_Timestamp(size_t sizeof_stream_item, std::string timestamp_file, double clock_offset_ms, int items_to_samples)
{
    gnss_shared_ptr<Gnss_Sdr_Timestamp> Timestamp_(new Gnss_Sdr_Timestamp(sizeof_stream_item, std::move(timestamp_file), clock_offset_ms, items_to_samples));
    return Timestamp_;
}


bool Gnss_Sdr_Timestamp::read_next_timetag()
{
    d_timefilestream.read(reinterpret_cast<char*>(&d_next_timetag_samplecount), sizeof(uint64_t));
    if (!d_timefilestream)
        {
            return false;
        }
    d_timefilestream.read(reinterpret_cast<char*>(&next_timetag.week), sizeof(int32_t));
    if (!d_timefilestream)
        {
            return false;
        }
    d_timefilestream.read(reinterpret_cast<char*>(&next_timetag.tow_ms), sizeof(int32_t));
    if (!d_timefilestream)
        {
            return false;
        }
    return true;
}


bool Gnss_Sdr_Timestamp::start()
{
    d_timefilestream.open(d_timefile, std::ios::in | std::ios::binary);

    if (d_timefilestream.is_open() == false)
        {
            std::cout << "ERROR: Could not open timestamp file: " << d_timefile << "\n";
            return false;
        }
    else
        {
            return true;
        }
}


int64_t Gnss_Sdr_Timestamp::uint64diff(uint64_t first, uint64_t second)
{
    uint64_t abs_diff = (first > second) ? (first - second) : (second - first);
    assert(abs_diff <= INT64_MAX);
    return (first > second) ? static_cast<int64_t>(abs_diff) : -static_cast<int64_t>(abs_diff);
}


int Gnss_Sdr_Timestamp::work(int noutput_items,
    gr_vector_const_void_star& input_items,
    gr_vector_void_star& output_items)
{
    // multichannel support
    if (d_get_next_timetag == true)
        {
            if (read_next_timetag() == false)
                {
                    // std::cout << "End of TimeTag file reached!\n";
                    // return 0;  // todo: find why return -1 does not stop gnss-sdr!
                }
            d_get_next_timetag = false;
        }
    for (size_t ch = 0; ch < output_items.size(); ch++)
        {
            std::memcpy(output_items[ch], input_items[ch], noutput_items * input_signature()->sizeof_stream_item(ch));
            int64_t diff_samplecount = uint64diff(this->nitems_written(ch), d_next_timetag_samplecount * d_items_to_samples);
            // std::cout << "diff_samplecount: " << diff_samplecount << ", noutput_items: " << noutput_items << "\n";
            if (diff_samplecount <= noutput_items and std::labs(diff_samplecount) <= noutput_items)
                {
                    const std::shared_ptr<GnssTime> tmp_obj = std::make_shared<GnssTime>(GnssTime());
                    tmp_obj->tow_ms = next_timetag.tow_ms + static_cast<int>(d_integer_ms_offset);
                    tmp_obj->week = next_timetag.week;
                    tmp_obj->tow_ms_fraction = d_fraction_ms_offset;
                    tmp_obj->rx_time = 0;
                    add_item_tag(ch, this->nitems_written(ch) - diff_samplecount, pmt::mp("timetag"), pmt::make_any(tmp_obj));
                    // std::cout << "[" << this->nitems_written(ch) - diff_samplecount << "] Sent TimeTag SC: " << d_next_timetag_samplecount * bytes_to_samples << ", Week: " << next_timetag.week << ", TOW: " << next_timetag.tow_ms << " [ms] \n";
                    d_get_next_timetag = true;
                }
        }

    return noutput_items;
}
