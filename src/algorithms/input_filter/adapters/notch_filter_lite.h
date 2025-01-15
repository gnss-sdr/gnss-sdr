/*!
 * \file notch_filter_lite.h
 * \brief Adapts a light version of a multistate notch filter
 * \author Antonio Ramos, 2017. antonio.ramosdet(at)gmail.com
 *
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

#ifndef GNSS_SDR_NOTCH_FILTER_LITE_H
#define GNSS_SDR_NOTCH_FILTER_LITE_H

#include "gnss_block_interface.h"
#include "notch_lite_cc.h"
#include <gnuradio/blocks/file_sink.h>
#include <string>
#include <vector>

/** \addtogroup Input_Filter
 * \{ */
/** \addtogroup Input_filter_adapters
 * \{ */


class ConfigurationInterface;

class NotchFilterLite : public GNSSBlockInterface
{
public:
    NotchFilterLite(const ConfigurationInterface* configuration,
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
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block);
    void disconnect(gr::top_block_sptr top_block);
    gr::basic_block_sptr get_left_block();
    gr::basic_block_sptr get_right_block();

private:
    notch_lite_sptr notch_filter_lite_;
    gr::blocks::file_sink::sptr file_sink_;
    std::string dump_filename_;
    std::string role_;
    std::string item_type_;
    size_t item_size_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    bool dump_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_NOTCH_FILTER_LITE_H
