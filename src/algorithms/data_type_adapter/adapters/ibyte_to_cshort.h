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

#include <string>
#include <gnuradio/blocks/file_sink.h>
#include "gnss_block_interface.h"
#include "interleaved_byte_to_complex_short.h"



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

    std::string role()
    {
        return role_;
    }
    //! Returns "Ibyte_To_Cshort"
    std::string implementation()
    {
        return "Ibyte_To_Cshort";
    }
    size_t item_size()
    {
        return 0;
    }

    void connect(gr::top_block_sptr top_block);
    void disconnect(gr::top_block_sptr top_block);
    gr::basic_block_sptr get_left_block();
    gr::basic_block_sptr get_right_block();

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
};

#endif

