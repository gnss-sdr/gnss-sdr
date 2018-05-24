/*!
 * \file mmse_resampler_conditioner.h
 * \brief Interface of an adapter of a mmse resampler conditioner block
 * to a SignalConditionerInterface
 * \author Antonio Ramos, 2018. antonio.ramos(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_MMSE_RESAMPLER_CONDITIONER_H_
#define GNSS_SDR_MMSE_RESAMPLER_CONDITIONER_H_

#include "gnss_block_interface.h"
#ifdef GR_GREATER_38
#include <gnuradio/filter/mmse_resampler_cc.h>
#else
#include <gnuradio/filter/fractional_resampler_cc.h>
#endif
#include <string>

class ConfigurationInterface;

/*!
 * \brief Interface of a MMSE resampler block adapter
 * to a SignalConditionerInterface
 */
class MmseResamplerConditioner : public GNSSBlockInterface
{
public:
    MmseResamplerConditioner(ConfigurationInterface* configuration,
        std::string role, unsigned int in_stream,
        unsigned int out_stream);

    virtual ~MmseResamplerConditioner();

    inline std::string role() override
    {
        return role_;
    }

    inline std::string implementation() override
    {
        return "Mmse_Resampler";
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
    double sample_freq_in_;
    double sample_freq_out_;
#ifdef GR_GREATER_38
    gr::filter::mmse_resampler_cc::sptr resampler_;
#else
    gr::filter::fractional_resampler_cc::sptr resampler_;
#endif
    gr::block_sptr file_sink_;
};

#endif /*GNSS_SDR_FRACTIONAL_RESAMPLER_CONDITIONER_H_*/
