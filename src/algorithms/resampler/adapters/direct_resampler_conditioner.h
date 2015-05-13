/*!
 * \file direct_resampler_conditioner.h
 * \brief Interface of an adapter of a direct resampler conditioner block
 * to a SignalConditionerInterface
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
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


#ifndef GNSS_SDR_DIRECT_RESAMPLER_CONDITIONER_H_
#define GNSS_SDR_DIRECT_RESAMPLER_CONDITIONER_H_

#include <string>
#include <gnuradio/hier_block2.h>
#include "gnss_block_interface.h"

class ConfigurationInterface;

/*!
 * \brief Interface of an adapter of a direct resampler conditioner block
 * to a SignalConditionerInterface
 */
class DirectResamplerConditioner: public GNSSBlockInterface
{
public:
    DirectResamplerConditioner(ConfigurationInterface* configuration,
            std::string role, unsigned int in_stream,
            unsigned int out_stream);

    virtual ~DirectResamplerConditioner();
    std::string role()
    {
        return role_;
    }
    //! returns "Direct_Resampler"
    std::string implementation()
    {
        return "Direct_Resampler";
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
    std::string role_;
    unsigned int in_stream_;
    unsigned int out_stream_;
    std::string item_type_;
    size_t item_size_;
    bool dump_;
    std::string dump_filename_;
    double sample_freq_in_;
    double sample_freq_out_;
    gr::block_sptr resampler_;
    gr::block_sptr file_sink_;
};

#endif /*GNSS_SDR_DIRECT_RESAMPLER_CONDITIONER_H_*/
