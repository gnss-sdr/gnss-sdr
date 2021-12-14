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


#ifndef GNSS_SDR_GNSS_SDR_TIMESTAMP_H
#define GNSS_SDR_GNSS_SDR_TIMESTAMP_H

#include "gnss_block_interface.h"
#include "gnss_time.h"
#include <gnuradio/sync_block.h>  // for sync_block
#include <gnuradio/types.h>       // for gr_vector_const_void_star
#include <pmt/pmt.h>
#include <cstddef>  // for size_t
#include <cstdint>
#include <fstream>
#include <string>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_libs
 * \{ */


class Gnss_Sdr_Timestamp;

gnss_shared_ptr<Gnss_Sdr_Timestamp> gnss_sdr_make_Timestamp(
    size_t sizeof_stream_item,
    std::string timestamp_file,
    double clock_offset_ms);


class Gnss_Sdr_Timestamp : public gr::sync_block
{
public:
    int work(int noutput_items,
        gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items);
    bool start();

private:
    friend gnss_shared_ptr<Gnss_Sdr_Timestamp> gnss_sdr_make_Timestamp(
        size_t sizeof_stream_item,
        std::string timestamp_file,
        double clock_offset_ms);

    Gnss_Sdr_Timestamp(size_t sizeof_stream_item,
        std::string timestamp_file,
        double clock_offset_ms);

    int64_t uint64diff(uint64_t first, uint64_t second);
    bool read_next_timetag();
    std::string d_timefile;
    std::fstream d_timefilestream;
    GnssTime next_timetag{};
    double d_clock_offset_ms;
    double d_fraction_ms_offset;
    double d_integer_ms_offset;
    uint64_t d_next_timetag_samplecount;
    bool d_get_next_timetag;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_SDR_TIMESTAMP_H
