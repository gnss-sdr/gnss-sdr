/*!
 * \file ibyte_to_cshort.h
 * \brief Adapts a short interleaved sample stream into a std::complex<short> stream
 * \author Carles Fernandez-Prades, cfernandez(at)cttc.es
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

#ifndef GNSS_SDR_IBYTE_TO_CSHORT_H_
#define GNSS_SDR_IBYTE_TO_CSHORT_H_

#include "conjugate_sc.h"
#include "interleaved_byte_to_complex_short.h"
#include "gnss_block_interface.h"
#include <gnuradio/blocks/file_sink.h>
#include <string>


class ConfigurationInterface;

/*!
 * \brief Adapts a short integer (16 bits) interleaved sample stream into a std::complex<short> stream
 *
 */
class IbyteToCshort: public GNSSBlockInterface
{
public:
    IbyteToCshort(ConfigurationInterface* configuration,
            std::string role, unsigned int in_streams,
            unsigned int out_streams);

    virtual ~IbyteToCshort();

    inline std::string role() override
    {
        return role_;
    }

    //! Returns "Ibyte_To_Cshort"
    inline std::string implementation() override
    {
        return "Ibyte_To_Cshort";
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
    interleaved_byte_to_complex_short_sptr interleaved_byte_to_complex_short_;
    ConfigurationInterface* config_;
    bool dump_;
    std::string dump_filename_;
    std::string input_item_type_;
    std::string output_item_type_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    gr::blocks::file_sink::sptr file_sink_;
    conjugate_sc_sptr conjugate_sc_;
    bool inverted_spectrum;
};

#endif
