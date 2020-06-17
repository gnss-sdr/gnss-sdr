/*!
 * \file signal_generator.h
 * \brief Adapter of a class that generates synthesized GNSS signal.
 * \author Marc Molina, 2013. marc.molina.pena@gmail.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
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


#ifndef GNSS_SDR_SIGNAL_GENERATOR_H
#define GNSS_SDR_SIGNAL_GENERATOR_H

#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include "signal_generator_c.h"
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/vector_to_stream.h>
#include <gnuradio/hier_block2.h>
#include <pmt/pmt.h>
#include <memory>
#include <string>
#include <vector>

class ConfigurationInterface;

/*!
* \brief This class generates synthesized GNSS signal.
*
*/
class SignalGenerator : public GNSSBlockInterface
{
public:
    SignalGenerator(ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_stream,
        unsigned int out_stream, std::shared_ptr<Concurrent_Queue<pmt::pmt_t> > queue);

    ~SignalGenerator() = default;

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "GNSSSignalGenerator".
     */
    inline std::string implementation() override
    {
        return "GNSSSignalGenerator";
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
    std::string role_;
    unsigned int in_stream_;
    unsigned int out_stream_;
    std::string item_type_;
    size_t item_size_;
    bool dump_;
    std::string dump_filename_;
#if GNURADIO_USES_STD_POINTERS
    std::shared_ptr<gr::block> gen_source_;
#else
    boost::shared_ptr<gr::block> gen_source_;
#endif
    gr::blocks::vector_to_stream::sptr vector_to_stream_;
    gr::blocks::file_sink::sptr file_sink_;
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t> > queue_;
};

#endif  // GNSS_SDR_SIGNAL_GENERATOR_H
