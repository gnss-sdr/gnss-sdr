/*!
 * \file labsat_signal_source.h
 * \brief LabSat version 2, 3, and 3 Wideband format reader
 * \author Javier Arribas, jarribas(at)cttc.es
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


#ifndef GNSS_SDR_LABSAT_SIGNAL_SOURCE_H
#define GNSS_SDR_LABSAT_SIGNAL_SOURCE_H

#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include "signal_source_base.h"
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/throttle.h>
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
 * \brief This class reads samples stored in LabSat version 2, 3, and 3 Wideband
 * format.
 */
class LabsatSignalSource : public SignalSourceBase
{
public:
    LabsatSignalSource(const ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_stream,
        unsigned int out_stream, Concurrent_Queue<pmt::pmt_t>* queue);

    ~LabsatSignalSource() = default;

    inline size_t item_size() override
    {
        return item_size_;
    }

    size_t getRfChannels() const override;
    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;
    gr::basic_block_sptr get_right_block(int i) override;

private:
    gr::block_sptr labsat23_source_;
    std::vector<gr::blocks::file_sink::sptr> file_sink_;
    std::vector<gr::blocks::throttle::sptr> throttle_;
    std::vector<int> channels_selector_vec_;

    std::string item_type_;
    std::string filename_;
    std::string dump_filename_;

    size_t item_size_;

    unsigned int in_stream_;
    unsigned int out_stream_;

    bool enable_throttle_control_;
    bool dump_;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_LABSAT_SIGNAL_SOURCE_H
