/*!
 * \file gnss_sdr_sample_counter.h
 * \brief Simple block to report the current receiver time based on the output of the tracking or telemetry blocks
 * \author Javier Arribas 2018. jarribas(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GNSS_SDR_SAMPLE_COUNTER_H
#define GNSS_SDR_GNSS_SDR_SAMPLE_COUNTER_H

#include <gnuradio/sync_decimator.h>
#include <gnuradio/types.h>  // for gr_vector_const_void_star
#include <cstddef>           // for size_t
#include <cstdint>
#if GNURADIO_USES_STD_POINTERS
#include <memory>
#else
#include <boost/shared_ptr.hpp>
#endif

class gnss_sdr_sample_counter;

#if GNURADIO_USES_STD_POINTERS
using gnss_sdr_sample_counter_sptr = std::shared_ptr<gnss_sdr_sample_counter>;
#else
using gnss_sdr_sample_counter_sptr = boost::shared_ptr<gnss_sdr_sample_counter>;
#endif

gnss_sdr_sample_counter_sptr gnss_sdr_make_sample_counter(
    double _fs,
    int32_t _interval_ms,
    size_t _size);

class gnss_sdr_sample_counter : public gr::sync_decimator
{
public:
    ~gnss_sdr_sample_counter() = default;
    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend gnss_sdr_sample_counter_sptr gnss_sdr_make_sample_counter(
        double _fs,
        int32_t _interval_ms,
        size_t _size);

    gnss_sdr_sample_counter(double _fs,
        int32_t _interval_ms,
        size_t _size);

    double fs;
    int64_t current_T_rx_ms;  // Receiver time in ms since the beginning of the run
    uint64_t sample_counter;
    int32_t interval_ms;
    int32_t report_interval_ms;
    uint32_t samples_per_output;
    uint32_t current_s;     // Receiver time in seconds, modulo 60
    uint32_t current_m;     // Receiver time in minutes, modulo 60
    uint32_t current_h;     // Receiver time in hours, modulo 24
    uint32_t current_days;  // Receiver time in days since the beginning of the run
    bool flag_m;            // True if the receiver has been running for at least 1 minute
    bool flag_h;            // True if the receiver has been running for at least 1 hour
    bool flag_days;         // True if the receiver has been running for at least 1 day
    bool flag_enable_send_msg;
};

#endif  // GNSS_SDR_GNSS_SDR_SAMPLE_COUNTER_H
