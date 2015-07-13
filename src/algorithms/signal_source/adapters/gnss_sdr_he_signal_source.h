/*!
 * \file gnss_sdr_he_signal_source.h
 * \brief GNSS-SDR Hacker's Edition Signal Sampler Board Driver.
 * \author Ajith Peter, Google Summer of Code 2014-15, ajith.peter(at)gmail.com
 *         Javier Arribas, 2014-15 jarribas(at)cttc.es
 *         Carles Fernandez Prades, 2014 carles.fernandez (at) cttc.cat
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
#ifndef GNSS_SDR_HE_SIGNAL_SOURCE_H_
#define GNSS_SDR_HE_SIGNAL_SOURCE_H_

#include <string>
#include <gnuradio/hier_block2.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/interleaved_short_to_complex.h>
#include "gnss_block_interface.h"
#include "unpack_byte_2bit_cpx_samples.h"

class ConfigurationInterface;

/*!
 * \brief This class reads samples from the GNSS-SDR Hacker's Edition Frontend
 * sampler board.
 */
class GNSS_SDR_HE_SignalSource: public GNSSBlockInterface
{
public:
  GNSS_SDR_HE_SignalSource(ConfigurationInterface* configuration,
            std::string role, unsigned int in_stream,
            unsigned int out_stream, gr::msg_queue::sptr queue);

    virtual ~GNSS_SDR_HE_SignalSource();
    std::string role()
    {
        return role_;
    }

    /*!
     * \brief Returns "GNSS_SDR_HE_SignalSource".
     */
    std::string implementation()
    {
        return "GNSS_SDR_HE_SignalSource";
    }
    size_t item_size()
    {
        return item_size_;
    }
    void connect(gr::top_block_sptr top_block);
    void disconnect(gr::top_block_sptr top_block);
    gr::basic_block_sptr get_left_block();
    gr::basic_block_sptr get_right_block();

private:
    std::string role_;
    unsigned int in_stream_;
    unsigned int out_stream_;
    std::string item_type_;
    size_t item_size_;
    long samples_;
    bool dump_;
    std::string dump_filename_;
    gr::block_sptr gn3s_source_;
    gr::blocks::file_sink::sptr sink_;
    boost::shared_ptr<gr::msg_queue> queue_;
    gr::block_sptr gnss_sdr_source_b_;
    unpack_byte_2bit_cpx_samples_sptr unpack_byte_;
    gr::blocks::interleaved_short_to_complex::sptr inter_shorts_to_cpx_;

};

#endif
