/*!
 * \file ad936x_custom_signal_source.h
 * \brief A direct IIO custom front-end gnss-sdr signal source for the AD936x AD front-end family with special FPGA custom functionalities.
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

#ifndef GNSS_SDR_Ad936xCustom_SIGNAL_SOURCE_H
#define GNSS_SDR_Ad936xCustom_SIGNAL_SOURCE_H

#include "ad936x_iio_source.h"
#include "concurrent_queue.h"
#include "conjugate_cc.h"
#include "signal_source_base.h"
#include "unpack_byte_2bit_cpx_samples.h"
#include "unpack_byte_4bit_samples.h"
#include "unpack_short_byte_samples.h"
#include <gnuradio/blocks/char_to_short.h>
#include <gnuradio/blocks/file_sink.h>
// #include <gnuradio/blocks/interleaved_char_to_complex.h>
#include <gnuradio/blocks/delay.h>
#include <gnuradio/blocks/interleaved_short_to_complex.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class instantiates the Ad936xCustom gnuradio signal source.
 * It has support also for a customized Ad936xCustom firmware and signal source to support PPS samplestamp reading.
 */
class Ad936xCustomSignalSource : public SignalSourceBase
{
public:
    Ad936xCustomSignalSource(const ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_stream,
        unsigned int out_stream, Concurrent_Queue<pmt::pmt_t>* queue);

    ~Ad936xCustomSignalSource() = default;

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;
    gr::basic_block_sptr get_right_block(int RF_channel) override;

private:
    unsigned int in_stream_;
    unsigned int out_stream_;
    gr::block_sptr ad936x_iio_source;
    std::vector<gr::blocks::file_sink::sptr> sink_;
    std::vector<std::string> filename_vec_;

    gr::blocks::delay::sptr gr_delay;
    std::vector<gr::blocks::char_to_short::sptr> gr_char_to_short_;
    std::vector<gr::blocks::interleaved_short_to_complex::sptr> gr_interleaved_short_to_complex_;
    //    std::vector<gr::blocks::interleaved_char_to_complex::sptr> gr_interleaved_char_to_complex_;
    std::vector<unpack_short_byte_samples_sptr> unpack_short_byte;
    std::vector<unpack_byte_4bit_samples_sptr> unpack_byte_fourbits;
    std::vector<unpack_byte_2bit_cpx_samples_sptr> unpack_byte_twobits;

    std::string item_type_;
    size_t item_size_;
    int64_t samples_;
    bool dump_;
    std::string dump_filename_;

    // Front-end settings
    std::string pluto_uri_;
    std::string board_type_;
    long long sample_rate_;
    long long bandwidth_;
    long long freq_;
    long long freq_2ch;
    std::string rf_port_select_;
    std::string rf_filter;
    std::string gain_mode_rx0_;
    std::string gain_mode_rx1_;
    double rf_gain_rx0_;
    double rf_gain_rx1_;
    bool enable_ch0;
    bool enable_ch1;
    bool PPS_mode_;
    std::string fe_ip_;
    int fe_ctlport_;
    int ssize_;
    int bshift_;
    bool spattern_;
    bool inverted_spectrum_ch0_;
    bool inverted_spectrum_ch1_;
    double lo_attenuation_db_;
    bool high_side_lo_;
    int tx_lo_channel_;
    double rx0_to_rx1_delay_ns_;
    bool delay_enabled;
    bool apply_delay_on_rx0;

    std::vector<bool> inverted_spectrum_vec;
    int n_channels;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_Ad936xCustom_SIGNAL_SOURCE_H
