/*!
 * \file pulse_blanking_filter.h
 * \brief Instantiates the GNSS-SDR pulse blanking filter
 * \author Javier Arribas 2017
 *         Antonio Ramos  2017
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

#ifndef GNSS_SDR_PULSE_BLANKING_FILTER_H_
#define GNSS_SDR_PULSE_BLANKING_FILTER_H_

#include "gnss_block_interface.h"
#include "pulse_blanking_cc.h"
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/filter/freq_xlating_fir_filter_ccf.h>
#include <string>

class ConfigurationInterface;

class PulseBlankingFilter: public GNSSBlockInterface
{
public:
    PulseBlankingFilter(ConfigurationInterface* configuration,
            std::string role, unsigned int in_streams,
            unsigned int out_streams);

    virtual ~PulseBlankingFilter();

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

#endif // GNSS_SDR_PULSE_BLANKING_FILTER_H_
