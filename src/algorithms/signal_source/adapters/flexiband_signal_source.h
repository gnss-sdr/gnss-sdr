/*!
 * \file flexiband_signal_source.h
 * \brief ignal Source adapter for the Teleorbit Flexiband front-end device.
 * This adapter requires a Flexiband GNU Radio driver
 * installed (not included with GNSS-SDR)
 * \author Javier Arribas, jarribas(at)cttc.es
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


#ifndef GNSS_SDR_FLEXIBAND_SIGNAL_SOURCE_H
#define GNSS_SDR_FLEXIBAND_SIGNAL_SOURCE_H

#include "concurrent_queue.h"
#include "signal_source_base.h"
#include <gnuradio/blocks/char_to_float.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/float_to_complex.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/hier_block2.h>
#include <pmt/pmt.h>
#include <memory>
#include <string>
#include <vector>


/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class configures and reads samples from Teleorbit Flexiband front-end.
 * This software requires a Flexiband GNU Radio driver installed (not included with GNSS-SDR).
 */
class FlexibandSignalSource : public SignalSourceBase
{
public:
    FlexibandSignalSource(const ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_stream,
        unsigned int out_stream, Concurrent_Queue<pmt::pmt_t>* queue);

    ~FlexibandSignalSource() = default;

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
    boost::shared_ptr<gr::block> flexiband_source_;

    std::vector<boost::shared_ptr<gr::block>> char_to_float;
    std::vector<boost::shared_ptr<gr::block>> float_to_complex_;
    std::vector<gr::blocks::null_sink::sptr> null_sinks_;

    std::string item_type_;
    std::string firmware_filename_;
    std::string signal_file;

    size_t item_size_;
    unsigned int in_stream_;
    unsigned int out_stream_;

    int gain1_;
    int gain2_;
    int gain3_;
    int usb_packet_buffer_size_;
    int n_channels_;
    int sel_ch_;

    bool AGC_;
    bool flag_read_file;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_FLEXIBAND_SIGNAL_SOURCE_H
