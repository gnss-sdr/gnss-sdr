/*!
 * \file ad936x_iio_source.h
 * \brief A direct IIO custom front-end gnss-sdr signal gnuradio block for the AD936x AD front-end family with special FPGA custom functionalities.
 * \author Javier Arribas, jarribas(at)cttc.es
 *
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


#ifndef GNSS_SDR_AD936X_IIO_SOURCE_H
#define GNSS_SDR_AD936X_IIO_SOURCE_H

#include "ad936x_iio_custom.h"
#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include "ppstcprx.h"
#include <gnuradio/block.h>
#include <pmt/pmt.h>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_gnuradio_blocks
 * \{ */


class ad936x_iio_source;

using ad936x_iio_source_sptr = gnss_shared_ptr<ad936x_iio_source>;

ad936x_iio_source_sptr ad936x_iio_make_source_sptr(
    std::string pluto_uri_,
    std::string board_type_,
    long long bandwidth_,
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
    bool ppsmode_,
    bool customsamplesize_,
    std::string fe_ip_,
    int fe_ctlport_,
    int ssize_,
    int bshift_,
    bool spattern_,
    double lo_attenuation_db_,
    bool high_side_lo_,
    int tx_lo_channel_);

/*!
 * \brief This class implements conversion between Labsat 2, 3 and 3 Wideband
 * formats to gr_complex
 */
class ad936x_iio_source : public gr::block
{
public:
    ~ad936x_iio_source();

    //! start the sample transmission
    bool start();
    //! stop the sample transmission
    bool stop();

    int general_work(int noutput_items,
        gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend ad936x_iio_source_sptr ad936x_iio_make_source_sptr(
        std::string pluto_uri_,
        std::string board_type_,
        long long bandwidth_,
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
        bool ppsmode_,
        bool customsamplesize_,
        std::string fe_ip_,
        int fe_ctlport_,
        int ssize_,
        int bshift_,
        bool spattern_,
        double lo_attenuation_db_,
        bool high_side_lo_,
        int tx_lo_channel_);

    ad936x_iio_source(
        std::string pluto_uri_,
        std::string board_type_,
        long long bandwidth_,
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
        bool ppsmode_,
        bool customsamplesize_,
        std::string fe_ip_,
        int fe_ctlport_,
        int ssize_,
        int bshift_,
        bool spattern_,
        double lo_attenuation_db_,
        bool high_side_lo_,
        int tx_lo_channel_);


    void ad9361_channel_demux_to_buffer(ad936x_iio_samples *samples_in, int nchannels, gr_vector_void_star &output_items);
    void ad9361_channel_demux_and_record(ad936x_iio_samples *samples_in, int nchannels, std::vector<std::fstream> *files_out);

    std::thread pps_rx_thread;
    std::unique_ptr<ad936x_iio_custom> ad936x_custom;
    std::shared_ptr<pps_tcp_rx> pps_rx;
    std::shared_ptr<Concurrent_Queue<PpsSamplestamp>> ppsqueue;

    std::vector<std::fstream> samplesfile;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_AD936X_IIO_SOURCE_H
