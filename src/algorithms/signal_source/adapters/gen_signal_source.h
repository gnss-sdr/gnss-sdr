/*!
 * \file gen_signal_source.h
 * \brief It wraps blocks that generates synthesized GNSS signal and filters
 *  it.
 * \author Marc Molina, 2013. marc.molina.pena@gmail.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_GEN_SIGNAL_SOURCE_H
#define GNSS_SDR_GEN_SIGNAL_SOURCE_H


#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include <pmt/pmt.h>
#include <memory>
#include <string>

/*!
 * \brief This class wraps blocks that generates synthesized GNSS signal and
 * filters the signal.
 */
class GenSignalSource : public GNSSBlockInterface
{
public:
    //! Constructor
    GenSignalSource(GNSSBlockInterface *signal_generator, GNSSBlockInterface *filter,
        std::string role, std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue);

    //! Virtual destructor
    virtual ~GenSignalSource();

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    inline std::string role() override { return role_; }
    //! Returns "Signal Source"
    inline std::string implementation() override { return "Signal Source"; }
    inline size_t item_size() override { return 0; }
    inline GNSSBlockInterface *signal_generator() const { return signal_generator_; }

private:
    GNSSBlockInterface *signal_generator_;
    GNSSBlockInterface *filter_;
    std::string role_;
    std::string implementation_;
    bool connected_;
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue_;
};

#endif  // GNSS_SDR_GEN_SIGNAL_SOURCE_H
