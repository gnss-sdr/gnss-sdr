/*!
 * \file gnss_sdr_time_counter.h
 * \brief Simple block to report the current receiver time based on the output of the tracking or telemetry blocks
 * \author Antonio Ramos 2018. antonio.ramosdet(at)gmail.com
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

#ifndef GNSS_SDR_GNSS_SDR_TIME_COUNTER_H
#define GNSS_SDR_GNSS_SDR_TIME_COUNTER_H

#include <gnuradio/block.h>
#include <gnuradio/types.h>  // for gr_vector_const_void_star
#include <cstdint>
#include <memory>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * \{ */


class gnss_sdr_time_counter;

using gnss_sdr_time_counter_sptr = std::shared_ptr<gnss_sdr_time_counter>;

gnss_sdr_time_counter_sptr gnss_sdr_make_time_counter();

class gnss_sdr_time_counter : public gr::block
{
public:
    ~gnss_sdr_time_counter() = default;
    int general_work(int noutput_items __attribute__((unused)),
        gr_vector_int &ninput_items __attribute__((unused)),
        gr_vector_const_void_star &input_items __attribute__((unused)),
        gr_vector_void_star &output_items);

private:
    gnss_sdr_time_counter();
    friend gnss_sdr_time_counter_sptr gnss_sdr_make_time_counter();

    int64_t current_T_rx_ms;  // Receiver time in ms since the beginning of the run
    int32_t report_interval_ms;
    uint32_t current_s;     // Receiver time in seconds, modulo 60
    uint32_t current_m;     // Receiver time in minutes, modulo 60
    uint32_t current_h;     // Receiver time in hours, modulo 24
    uint32_t current_days;  // Receiver time in days since the beginning of the run
    bool flag_m;            // True if the receiver has been running for at least 1 minute
    bool flag_h;            // True if the receiver has been running for at least 1 hour
    bool flag_days;         // True if the receiver has been running for at least 1 day
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_SDR_SAMPLE_COUNTER_H
