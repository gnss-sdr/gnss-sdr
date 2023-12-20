/*!
 * \file gnss_sdr_fpga_sample_counter.cc
 * \brief Simple block to report the current receiver time based on the output
 * of the tracking or telemetry blocks
 * \author Marc Majoral 2019. mmajoral(at)cttc.es
 * \author Javier Arribas 2018. jarribas(at)cttc.es
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

#include "gnss_sdr_fpga_sample_counter.h"
#include "gnss_synchro.h"
#include "uio_fpga.h"
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>        // for from_double
#include <pmt/pmt_sugar.h>  // for mp
#include <cmath>            // for round
#include <fcntl.h>          // for O_RDWR, libraries used by the GIPO
#include <iostream>         // for operator<<
#include <sys/mman.h>       // libraries used by the GIPO
#include <unistd.h>         // for write, close, read, ssize_t


#ifndef TEMP_FAILURE_RETRY
#define TEMP_FAILURE_RETRY(exp)              \
    ({                                       \
        decltype(exp) _rc;                   \
        do                                   \
            {                                \
                _rc = (exp);                 \
            }                                \
        while (_rc == -1 && errno == EINTR); \
        _rc;                                 \
    })
#endif


gnss_sdr_fpga_sample_counter::gnss_sdr_fpga_sample_counter(
    double _fs,
    int32_t _interval_ms)
    : gr::block("fpga_fpga_sample_counter",
          gr::io_signature::make(0, 0, 0),
          gr::io_signature::make(1, 1, sizeof(Gnss_Synchro))),
      fs(_fs),
      sample_counter(0ULL),
      last_sample_counter(0ULL),
      current_T_rx_ms(0),
      interval_ms(_interval_ms),
      current_s(0),
      current_m(0),
      current_h(0),
      current_days(0),
      report_interval_ms(1000),     // default reporting 1 second
      flag_enable_send_msg(false),  // enable it for reporting time with asynchronous message
      flag_m(false),
      flag_h(false),
      flag_days(false),
      is_open(true)
{
    message_port_register_out(pmt::mp("fpga_sample_counter"));
    set_max_noutput_items(1);
    samples_per_output = std::round(fs * static_cast<double>(interval_ms) / 1e3);
    samples_per_report = std::round(fs * static_cast<double>(report_interval_ms) / 1e3);
    open_device();
}


gnss_sdr_fpga_sample_counter_sptr gnss_sdr_make_fpga_sample_counter(double _fs, int32_t _interval_ms)
{
    gnss_sdr_fpga_sample_counter_sptr fpga_sample_counter_(new gnss_sdr_fpga_sample_counter(_fs, _interval_ms));
    return fpga_sample_counter_;
}


gnss_sdr_fpga_sample_counter::~gnss_sdr_fpga_sample_counter()
{
    if (is_open)
        {
            close_device();
        }
}


// Called by GNU Radio to enable drivers, etc for i/o devices.
bool gnss_sdr_fpga_sample_counter::start()
{
    // configure the number of samples per output in the FPGA and enable the interrupts
    configure_samples_per_output(samples_per_output);

    // return true if everything is ok.
    return true;
}


// Called by GNU Radio to disable drivers, etc for i/o devices.
bool gnss_sdr_fpga_sample_counter::stop()
{
    close_device();
    is_open = false;
    return true;
}


uint32_t gnss_sdr_fpga_sample_counter::test_register(uint32_t writeval)
{
    uint32_t readval;
    // write value to test register
    map_base[3] = writeval;
    // read value from test register
    readval = map_base[3];
    // return read value
    return readval;
}


void gnss_sdr_fpga_sample_counter::configure_samples_per_output(uint32_t interval)
{
    // note : the counter is a 48-bit value in the HW.
    map_base[0] = interval - 1;
}


void gnss_sdr_fpga_sample_counter::open_device()
{
    // UIO device file
    std::string device_io_name;
    // find the uio device file corresponding to the sample counter module
    if (find_uio_dev_file_name(device_io_name, device_name, 0) < 0)
        {
            std::cout << "Cannot find the FPGA uio device file corresponding to device name " << device_name << std::endl;
            throw std::exception();
        }

    // open communication with HW accelerator
    if ((fd = open(device_io_name.c_str(), O_RDWR | O_SYNC)) == -1)
        {
            LOG(WARNING) << "Cannot open deviceio" << device_io_name;
            std::cout << "Counter-Intr: cannot open deviceio" << device_io_name << '\n';
        }
    map_base = reinterpret_cast<volatile uint32_t *>(mmap(nullptr, FPGA_PAGE_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));

    if (map_base == reinterpret_cast<void *>(-1))
        {
            LOG(WARNING) << "Cannot map the FPGA acquisition module into user memory";
            std::cout << "Counter-Intr: cannot map deviceio" << device_io_name << '\n';
        }

    // sanity check : check test register
    uint32_t writeval = test_reg_sanity_check;
    uint32_t readval;
    readval = gnss_sdr_fpga_sample_counter::test_register(writeval);
    if (writeval != readval)
        {
            LOG(WARNING) << "Acquisition test register sanity check failed";
        }
    else
        {
            LOG(INFO) << "Acquisition test register sanity check success!";
            // std::cout << "Acquisition test register sanity check success!\n";
        }
}


void gnss_sdr_fpga_sample_counter::close_device()
{
    map_base[2] = 0;  // disable the generation of the interrupt in the device

    auto *aux = const_cast<uint32_t *>(map_base);
    if (munmap(static_cast<void *>(aux), FPGA_PAGE_SIZE) == -1)
        {
            std::cout << "Failed to unmap memory uio\n";
        }
    close(fd);
}


int gnss_sdr_fpga_sample_counter::general_work(int noutput_items __attribute__((unused)),
    __attribute__((unused)) gr_vector_int &ninput_items,
    __attribute__((unused)) gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    wait_for_interrupt();

    uint64_t sample_counter_tmp;
    uint64_t sample_counter_msw_tmp;
    sample_counter_tmp = map_base[0];
    sample_counter_msw_tmp = map_base[1];
    sample_counter_msw_tmp = sample_counter_msw_tmp << 32;
    sample_counter_tmp = sample_counter_tmp + sample_counter_msw_tmp;  // 2^32
    sample_counter = sample_counter_tmp;

    auto *out = reinterpret_cast<Gnss_Synchro *>(output_items[0]);
    out[0] = Gnss_Synchro();
    out[0].Flag_valid_symbol_output = false;
    out[0].Flag_valid_word = false;
    out[0].Channel_ID = -1;
    out[0].fs = fs;

    if ((sample_counter - last_sample_counter) >= samples_per_report)
        {
            last_sample_counter = sample_counter;

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
    out[0].Tracking_sample_counter = sample_counter;
    current_T_rx_ms = interval_ms * (sample_counter) / samples_per_output;
    return 1;
}


void gnss_sdr_fpga_sample_counter::wait_for_interrupt() const
{
    int32_t irq_count;
    ssize_t nb;

    // enable interrupts
    int32_t reenable = 1;
    const ssize_t nbytes = TEMP_FAILURE_RETRY(write(fd, reinterpret_cast<void *>(&reenable), sizeof(int32_t)));
    if (nbytes != sizeof(int32_t))
        {
            std::cerr << "Error re-enabling FPGA sample counter interrupt.\n";
        }

    // wait for interrupt
    nb = read(fd, &irq_count, sizeof(irq_count));
    if (nb != sizeof(irq_count))
        {
            std::cout << "fpga sample counter module read failed to retrieve 4 bytes!\n";
            std::cout << "fpga sample counter module interrupt number " << irq_count << '\n';
        }
}
