/*!
 * \file pulse_blanking_filter.h
 * \brief Instantiates the GNSS-SDR pulse blanking filter
 * \author Javier Arribas 2017
 *         Antonio Ramos  2017
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

/** \addtogroup Input_Filter
 * \{ */
/** \addtogroup Input_filter_adapters
 * \{ */


class ConfigurationInterface;

class PulseBlankingFilter : public GNSSBlockInterface
{
public:
    PulseBlankingFilter(const ConfigurationInterface* configuration,
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
        return input_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    pulse_blanking_cc_sptr pulse_blanking_cc_;
    gr::filter::freq_xlating_fir_filter_ccf::sptr freq_xlating_;
    gr::blocks::file_sink::sptr file_sink_;
    std::string dump_filename_;
    std::string input_item_type_;
    std::string output_item_type_;
    std::string role_;
    size_t input_size_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    bool dump_;
    bool xlat_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_PULSE_BLANKING_FILTER_H
