/*!
 * \file labsat_signal_source.h
 * \brief Labsat 2 and 3 front-end signal sampler driver
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


#ifndef GNSS_SDR_LABSAT_SIGNAL_SOURCE_H
#define GNSS_SDR_LABSAT_SIGNAL_SOURCE_H

#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/throttle.h>
#include <gnuradio/hier_block2.h>
#include <pmt/pmt.h>
#include <memory>
#include <string>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class reads samples stored by a LabSat 2 or LabSat 3 device
 */
class LabsatSignalSource : public GNSSBlockInterface
{
public:
    LabsatSignalSource(const ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_stream,
        unsigned int out_stream, Concurrent_Queue<pmt::pmt_t>* queue);

    ~LabsatSignalSource() = default;

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "Labsat_Signal_Source".
     */
    inline std::string implementation() override
    {
        return "Labsat_Signal_Source";
    }

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    gr::block_sptr labsat23_source_;
    gr::blocks::file_sink::sptr file_sink_;
    gr::blocks::throttle::sptr throttle_;

    std::string role_;
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
