/*!
 * \file null_sink_output_filter.h
 * \brief Interface of an adapter of a null sink output filter block to an
 * OutputFilterInterface
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 * This class represents an implementation of an output filter that
 * sends its input to a null sink.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_NULL_SINK_OUTPUT_FILTER_H_
#define GNSS_SDR_NULL_SINK_OUTPUT_FILTER_H_

#include <string>
#include <gnuradio/blocks/null_sink.h>
#include "gnss_block_interface.h"


class ConfigurationInterface;

/*!
 * \brief This class implements a null sink output filter
 */
class NullSinkOutputFilter : public GNSSBlockInterface
{
public:
    NullSinkOutputFilter(ConfigurationInterface* configuration,
            std::string role,
            unsigned int in_streams,
            unsigned int out_streams);

    virtual ~NullSinkOutputFilter();
    std::string item_type()
    {
        return item_type_;
    }
    std::string role()
    {
        return role_;
    }
    //!  Returns "Null_Sink_Output_Filter"
    std::string implementation()
    {
        return "Null_Sink_Output_Filter";
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
    gr::blocks::null_sink::sptr sink_;
    size_t item_size_;
    std::string item_type_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif /*GNSS_SDR_NULL_SINK_OUTPUT_FILTER_H_*/
