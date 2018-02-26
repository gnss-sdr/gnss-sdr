/*!
 * \file beamformer_filter.h
 * \brief Interface of an adapter of a digital beamformer
 * \author Javier Arribas jarribas (at) cttc.es
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


#ifndef GNSS_SDR_BEAMFORMER_FILTER_H_
#define GNSS_SDR_BEAMFORMER_FILTER_H_

#include "gnss_block_interface.h"
#include <gnuradio/hier_block2.h>
#include <string>

class ConfigurationInterface;

/*!
 * \brief Interface of an adapter of a direct resampler conditioner block
 * to a SignalConditionerInterface
 */
class BeamformerFilter: public GNSSBlockInterface
{
public:
    BeamformerFilter(ConfigurationInterface* configuration,
            std::string role, unsigned int in_stream,
            unsigned int out_stream);

    virtual ~BeamformerFilter();

    inline std::string role() override
    {
        return role_;
    }

    //! returns "Direct_Resampler"
    inline std::string implementation() override
    {
        return "Beamformer_Filter";
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
    unsigned long long samples_;
    bool dump_;
    std::string dump_filename_;
    gr::block_sptr beamformer_;
    gr::block_sptr file_sink_;
};

#endif /*GNSS_SDR_BEAMFORMER_FILTER_H_*/
