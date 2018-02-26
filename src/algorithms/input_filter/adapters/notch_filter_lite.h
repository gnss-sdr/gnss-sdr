/*!
 * \file notch_filter_lite.h
 * \brief Adapts a ligth version of a multistate notch filter
 * \author Antonio Ramos, 2017. antonio.ramosdet(at)gmail.com
 *
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_NOTCH_FILTER_LITE_H_
#define GNSS_SDR_NOTCH_FILTER_LITE_H_

#include "gnss_block_interface.h"
#include "notch_lite_cc.h"
#include <gnuradio/blocks/file_sink.h>
#include <string>
#include <vector>

class ConfigurationInterface;

class NotchFilterLite: public GNSSBlockInterface
{
public:
    NotchFilterLite(ConfigurationInterface* configuration,
            std::string role, unsigned int in_streams,
            unsigned int out_streams);

    virtual ~NotchFilterLite();
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

#endif //GNSS_SDR_NOTCH_FILTER_LITE_H_
