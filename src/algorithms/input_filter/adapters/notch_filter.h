/*!
 * \file notch_filter.h
 * \brief Adapter of a multistate Notch filter
 * \author Antonio Ramos, 2017. antonio.ramosdet(at)gmail.com
 *
 * Detailed description of the file here if needed.
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

#ifndef GNSS_SDR_NOTCH_FILTER_H
#define GNSS_SDR_NOTCH_FILTER_H

#include "gnss_block_interface.h"
#include "notch_cc.h"
#include <gnuradio/blocks/file_sink.h>
#include <string>
#include <vector>

class ConfigurationInterface;

class NotchFilter : public GNSSBlockInterface
{
public:
    NotchFilter(ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_streams,
        unsigned int out_streams);

    ~NotchFilter() = default;

    std::string role()
    {
        return role_;
    }

    //! Returns "Notch_Filter"
    std::string implementation()
    {
        return "Notch_Filter";
    }

    size_t item_size()
    {
        return 0;
    }

    void connect(gr::top_block_sptr top_block);
    void disconnect(gr::top_block_sptr top_block);
    gr::basic_block_sptr get_left_block();
    gr::basic_block_sptr get_right_block();

private:
    notch_sptr notch_filter_;
    gr::blocks::file_sink::sptr file_sink_;
    std::string dump_filename_;
    std::string role_;
    std::string item_type_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    bool dump_;
};

#endif  // GNSS_SDR_NOTCH_FILTER_H
