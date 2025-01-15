/*!
 * \file gnss_sdr_fpga_sample_counter.h
 * \brief Simple block to report the current receiver time based on the output
 * of the tracking or telemetry blocks
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

#ifndef GNSS_SDR_GNSS_SDR_FPGA_SAMPLE_COUNTER_H
#define GNSS_SDR_GNSS_SDR_FPGA_SAMPLE_COUNTER_H

#include "gnss_block_interface.h"
#include <gnuradio/block.h>
#include <gnuradio/types.h>  // for gr_vector_const_void_star
#include <cstdint>
#include <string>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * \{ */


class gnss_sdr_fpga_sample_counter;

using gnss_sdr_fpga_sample_counter_sptr = gnss_shared_ptr<gnss_sdr_fpga_sample_counter>;

gnss_sdr_fpga_sample_counter_sptr gnss_sdr_make_fpga_sample_counter(double _fs, int32_t _interval_ms);

class gnss_sdr_fpga_sample_counter : public gr::block
{
public:
    ~gnss_sdr_fpga_sample_counter();
    int general_work(int noutput_items,
        gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    const std::string device_name = "counter";  // UIO device name

    static const uint32_t FPGA_PAGE_SIZE = 0x1000;         // default page size for the multicorrelator memory map
    static const uint32_t test_reg_sanity_check = 0x55AA;  // value to check the presence of the test register (to detect the hw)

    friend gnss_sdr_fpga_sample_counter_sptr gnss_sdr_make_fpga_sample_counter(double _fs, int32_t _interval_ms);
    gnss_sdr_fpga_sample_counter(double _fs, int32_t _interval_ms);
    uint32_t test_register(uint32_t writeval);
    void configure_samples_per_output(uint32_t interval);
    void close_device(void);
    void open_device(void);
    bool start();
    bool stop();
    void wait_for_interrupt(void) const;

    volatile uint32_t *map_base;  // driver memory map

    double fs;
    uint64_t sample_counter;
    uint64_t last_sample_counter;
    uint64_t current_T_rx_ms;  // Receiver time in ms since the beginning of the run

    uint32_t samples_per_output;
    uint32_t samples_per_report;
    uint32_t interval_ms;
    uint32_t current_s;     // Receiver time in seconds, modulo 60
    uint32_t current_m;     // Receiver time in minutes, modulo 60
    uint32_t current_h;     // Receiver time in hours, modulo 24
    uint32_t current_days;  // Receiver time in days since the beginning of the run
    int32_t report_interval_ms;
    int32_t fd;  // driver descriptor

    bool flag_enable_send_msg;
    bool flag_m;     // True if the receiver has been running for at least 1 minute
    bool flag_h;     // True if the receiver has been running for at least 1 hour
    bool flag_days;  // True if the receiver has been running for at least 1 day
    bool is_open;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_SDR_FPGA_SAMPLE_COUNTER_H
