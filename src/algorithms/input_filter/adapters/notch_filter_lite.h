/*!
 * \file notch_filter_lite.h
 * \brief Adapts a light version of a multistate notch filter
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

#ifndef GNSS_SDR_NOTCH_FILTER_LITE_H
#define GNSS_SDR_NOTCH_FILTER_LITE_H

#include "gnss_block_interface.h"
#include "notch_lite_cc.h"
#include <gnuradio/blocks/file_sink.h>
#include <string>
#include <vector>

class ConfigurationInterface;

class NotchFilterLite : public GNSSBlockInterface
{
public:
    NotchFilterLite(ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_streams,
        unsigned int out_streams);

    ~NotchFilterLite() = default;
    std::string role()
    {
        return role_;
    }

    //! Returns "Notch_Filter_Lite"
    std::string implementation()
    {
        return "Notch_Filter_Lite";
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
    bool dump_;
    std::string dump_filename_;
    std::string role_;
    std::string item_type_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    gr::blocks::file_sink::sptr file_sink_;
    notch_lite_sptr notch_filter_lite_;
};

#endif  // GNSS_SDR_NOTCH_FILTER_LITE_H
