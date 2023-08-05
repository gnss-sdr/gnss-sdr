/*!
 * \file ad936x_iio_custom.h
 * \brief A direct IIO custom front-end driver for the AD936x AD front-end family with special FPGA custom functionalities.
 * \author Javier Arribas, jarribas(at)cttc.es
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef SRC_LIBS_ad936x_iio_custom_H_
#define SRC_LIBS_ad936x_iio_custom_H_

#include "concurrent_queue.h"
#include "gnss_time.h"
#include "pps_samplestamp.h"
#include <boost/atomic.hpp>
#include <memory>
#include <string>

#ifdef __APPLE__
#include <iio/iio.h>
#else
#include <iio.h>
#endif

#include "ad936x_iio_samples.h"
#include <ad9361.h>  // multichip sync and high level functions
#include <thread>
#include <vector>

class ad936x_iio_custom
{
public:
    ad936x_iio_custom(int debug_level_, int log_level_);
    virtual ~ad936x_iio_custom();
    bool initialize_device(std::string pluto_device_uri, std::string board_type);

    bool init_config_ad9361_rx(long long bandwidth_,
        long long sample_rate_,
        long long freq_,
        std::string rf_port_select_,
        std::string rf_filter,
        std::string gain_mode_rx0_,
        std::string gain_mode_rx1_,
        double rf_gain_rx0_,
        double rf_gain_rx1_,
        bool enable_ch0,
        bool enable_ch1,
        long long freq_2ch,
        double lo_attenuation_db_,
        bool high_side_lo_,
        int tx_lo_channel_);

    bool calibrate(int ch, double bw_hz);

    double get_rx_gain(int ch_num);
    bool setRXGain(int ch_num, std::string gain_mode, double gain_dB);

    bool set_antenna_port(int ch, int antenna_idx);
    double get_frequency(int ch);
    bool set_frequency(int ch, double freq_hz);

    bool start_sample_rx(bool ppsmode);
    void stop_record();

    void set_gnsstime_queue(std::shared_ptr<Concurrent_Queue<GnssTime>> queue);
    void set_pps_samplestamp_queue(std::shared_ptr<Concurrent_Queue<PpsSamplestamp>> queue);

    bool get_rx_frequency(long long &freq_hz);
    bool set_rx_frequency(long long freq_hz);
    bool read_die_temp(double &temp_c);

    void pop_sample_buffer(std::shared_ptr<ad936x_iio_samples> &current_buffer);

    void push_sample_buffer(std::shared_ptr<ad936x_iio_samples> &current_buffer);
    int n_channels;

private:
    std::shared_ptr<Concurrent_Queue<GnssTime>> GnssTime_queue;
    std::shared_ptr<Concurrent_Queue<PpsSamplestamp>> Pps_queue;
    bool check_device();
    bool get_iio_param(iio_device *dev, const std::string &param, std::string &value);
    void configure_params(struct iio_device *phy,
        const std::vector<std::string> &params);
    void set_params_rx(struct iio_device *phy_device,
        unsigned long long frequency,
        unsigned long samplerate, unsigned long bandwidth,
        bool quadrature, bool rfdc, bool bbdc,
        std::string gain1, double gain1_value,
        std::string gain2, double gain2_value,
        std::string port_select);

    bool config_ad9361_dds(uint64_t freq_rf_tx_hz_,
        double tx_attenuation_db_,
        int64_t freq_dds_tx_hz_,
        double scale_dds_,
        double phase_dds_deg_,
        int channel);

    void get_PPS_timestamp();
    void capture(const std::vector<std::string> &channels);

    bool select_rf_filter(std::string rf_filter);

    void monitor_thread_fn();

    void PlutoTxEnable(bool txon);
    void setPlutoGpo(int p);

    // Device structure
    struct iio_context *ctx;
    struct iio_device *phy;
    struct iio_device *stream_dev;
    struct iio_device *dds_dev;

    // stream

    uint64_t sample_rate_sps;


    int debug_level;
    int log_level;
    bool PPS_mode;

    std::mutex mtx;
    std::condition_variable cv;

    boost::atomic<bool> receive_samples;

    boost::atomic<bool> fpga_overflow;
    // using queues of smart pointers to preallocated buffers
    Concurrent_Queue<std::shared_ptr<ad936x_iio_samples>> free_buffers;
    Concurrent_Queue<std::shared_ptr<ad936x_iio_samples>> used_buffers;

    std::thread capture_samples_thread;
    std::thread overflow_monitor_thread;
    std::thread capture_time_thread;
};

#endif /* SRC_LIBS_ad936x_iio_custom_H_ */
