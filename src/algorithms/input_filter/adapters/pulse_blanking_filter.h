/*!
 * \file pulse_blanking_filter.h
 * \brief Instantiates the GNSS-SDR pulse blanking filter
 * \author Javier Arribas 2017
 *         Antonio Ramos  2017
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

#ifndef GNSS_SDR_PULSE_BLANKING_FILTER_H
#define GNSS_SDR_PULSE_BLANKING_FILTER_H

#include "gnss_block_interface.h"
#include "pulse_blanking_cc.h"
#include <gnuradio/blocks/file_sink.h>
#ifdef GR_GREATER_38
#include <gnuradio/filter/freq_xlating_fir_filter.h>
#else
#include <gnuradio/filter/freq_xlating_fir_filter_ccf.h>
#endif
#include <string>

class ConfigurationInterface;

class PulseBlankingFilter : public GNSSBlockInterface
{
public:
    PulseBlankingFilter(ConfigurationInterface* configuration,
        std::string role, unsigned int in_streams,
        unsigned int out_streams);

    ~PulseBlankingFilter() = default;

    inline std::string role() override
    {
        return role_;
    }

    //! Returns "Pulse_Blanking_Filter"
    inline std::string implementation() override
    {
        return "Pulse_Blanking_Filter";
    }

    inline size_t item_size() override
    {
        return 0;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    ConfigurationInterface* config_;
    bool dump_;
    bool xlat_;
    std::string dump_filename_;
    std::string input_item_type_;
    size_t input_size_;
    std::string output_item_type_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    gr::blocks::file_sink::sptr file_sink_;
    pulse_blanking_cc_sptr pulse_blanking_cc_;
    gr::filter::freq_xlating_fir_filter_ccf::sptr freq_xlating_;
};

#endif  // GNSS_SDR_PULSE_BLANKING_FILTER_H
