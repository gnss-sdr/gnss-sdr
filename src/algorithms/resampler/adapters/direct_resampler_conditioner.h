/*!
 * \file direct_resampler_conditioner.h
 * \brief Interface of an adapter of a direct resampler conditioner block
 * to a SignalConditionerInterface
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
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


#ifndef GNSS_SDR_DIRECT_RESAMPLER_CONDITIONER_H
#define GNSS_SDR_DIRECT_RESAMPLER_CONDITIONER_H

#include "gnss_block_interface.h"
#include <gnuradio/hier_block2.h>
#include <string>

/** \addtogroup Resampler
 * \{ */
/** \addtogroup Resampler_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief Interface of an adapter of a direct resampler conditioner block
 * to a SignalConditionerInterface
 */
class DirectResamplerConditioner : public GNSSBlockInterface
{
public:
    DirectResamplerConditioner(const ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_stream,
        unsigned int out_stream);

    ~DirectResamplerConditioner() = default;

    inline std::string role() override
    {
        return role_;
    }

    //! Returns "Direct_Resampler"
    inline std::string implementation() override
    {
        return "Direct_Resampler";
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
    gr::block_sptr resampler_;
    gr::block_sptr file_sink_;
    std::string role_;
    std::string item_type_;
    std::string dump_filename_;
    double sample_freq_in_;
    double sample_freq_out_;
    size_t item_size_;
    unsigned int in_stream_;
    unsigned int out_stream_;
    bool dump_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_DIRECT_RESAMPLER_CONDITIONER_H
