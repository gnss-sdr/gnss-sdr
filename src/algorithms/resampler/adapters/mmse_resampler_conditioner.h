/*!
 * \file mmse_resampler_conditioner.h
 * \brief Interface of an adapter of a mmse resampler conditioner block
 * to a SignalConditionerInterface
 * \author Antonio Ramos, 2018. antonio.ramos(at)cttc.es
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


#ifndef GNSS_SDR_MMSE_RESAMPLER_CONDITIONER_H
#define GNSS_SDR_MMSE_RESAMPLER_CONDITIONER_H

#include "gnss_block_interface.h"
#ifdef GR_GREATER_38
#include <gnuradio/filter/fir_filter_blk.h>
#include <gnuradio/filter/mmse_resampler_cc.h>
#else
#include <gnuradio/filter/fir_filter_ccf.h>
#include <gnuradio/filter/fractional_resampler_cc.h>
#endif

#include <gnuradio/filter/firdes.h>
#include <string>

/** \addtogroup Resampler
 * Classes for input signal resampling
 * \{ */
/** \addtogroup Resampler_adapters resampler_adapters
 * Classes that wrap GNU Radio resampler blocks with a GNSSBlockInterface
 * \{ */


class ConfigurationInterface;

/*!
 * \brief Interface of a MMSE resampler block adapter
 * to a SignalConditionerInterface
 */
class MmseResamplerConditioner : public GNSSBlockInterface
{
public:
    MmseResamplerConditioner(const ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_stream,
        unsigned int out_stream);

    ~MmseResamplerConditioner() = default;

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
#ifdef GR_GREATER_38
    gr::filter::mmse_resampler_cc::sptr resampler_;
#else
    gr::filter::fractional_resampler_cc::sptr resampler_;
#endif
    gr::filter::fir_filter_ccf::sptr fir_filter_ccf_;
    gr::block_sptr file_sink_;
    std::string role_;
    std::string item_type_;
    std::string dump_filename_;
    size_t item_size_;
    double sample_freq_in_;
    double sample_freq_out_;
    unsigned int in_stream_;
    unsigned int out_stream_;
    bool dump_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_FRACTIONAL_RESAMPLER_CONDITIONER_H
