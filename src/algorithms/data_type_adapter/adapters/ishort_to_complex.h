/*!
 * \file ishort_to_complex.h
 * \brief Adapts an I/Q interleaved short integer sample stream to a gr_complex (float) stream
 * \author Javier Arribas, jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

#ifndef GNSS_SDR_ISHORT_TO_COMPLEX_H_
#define GNSS_SDR_ISHORT_TO_COMPLEX_H_

#include "gnss_synchro.h"
#include "gnss_block_interface.h"
#include <gnuradio/blocks/interleaved_short_to_complex.h>
#include <gnuradio/gr_msg_queue.h>

class ConfigurationInterface;

/*!
 * \brief Adapts an I/Q interleaved short integer sample stream to a gr_complex (float) stream
 *
 */
class IshortToComplex: public GNSSBlockInterface
{
public:
    IshortToComplex(ConfigurationInterface* configuration,
            std::string role, unsigned int in_streams,
            unsigned int out_streams, gr_msg_queue_sptr queue);

    virtual ~IshortToComplex();

    std::string role()
    {
        return role_;
    }
    //! Returns "IshortToComplex"
    std::string implementation()
    {
        return "IshortToComplex";
    }
    size_t item_size()
    {
        return 0;
    }

    void connect(gr_top_block_sptr top_block);
    void disconnect(gr_top_block_sptr top_block);
    gr_basic_block_sptr get_left_block();
    gr_basic_block_sptr get_right_block();

private:
    gr::blocks::interleaved_short_to_complex::sptr gr_interleaved_short_to_complex_;
    ConfigurationInterface* config_;
    bool dump_;
    std::string dump_filename_;
    std::string input_item_type_;
    std::string output_item_type_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    gr_msg_queue_sptr queue_;
    gr_block_sptr file_sink_;
};

#endif
