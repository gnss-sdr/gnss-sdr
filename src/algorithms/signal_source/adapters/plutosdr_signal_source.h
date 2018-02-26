/*!
 * \file plutosdr_signal_source.h
 * \brief Signal source for PlutoSDR
 * \author Rodrigo Muñoz, 2017, rmunozl(at)inacap.cl
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


#ifndef GNSS_SDR_PLUTOSDR_SIGNAL_SOURCE_H_
#define GNSS_SDR_PLUTOSDR_SIGNAL_SOURCE_H_

#include "gnss_block_interface.h"
#include <boost/shared_ptr.hpp>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/iio/pluto_source.h>
#include <string>


class ConfigurationInterface;

/*!
 */
class PlutosdrSignalSource: public GNSSBlockInterface
{
public:
    PlutosdrSignalSource(ConfigurationInterface* configuration,
            std::string role, unsigned int in_stream,
            unsigned int out_stream, boost::shared_ptr<gr::msg_queue> queue);

    virtual ~PlutosdrSignalSource();

    std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "Plutosdr_Signal_Source"
     */
    std::string implementation() override
    {
        return "Plutosdr_Signal_Source";
    }
    size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    std::string role_;

    // Front-end settings
    std::string uri_; // device direction
    unsigned long freq_; // frequency of local oscilator
    unsigned long sample_rate_;
    unsigned long bandwidth_;
    unsigned long buffer_size_; // reception buffer
    bool quadrature_;
    bool rf_dc_;
    bool bb_dc_;
    std::string gain_mode_;
    double rf_gain_;
    std::string filter_file_;
    bool filter_auto_;

    unsigned int in_stream_;
    unsigned int out_stream_;

    std::string item_type_;
    size_t item_size_;
    long samples_;
    bool dump_;
    std::string dump_filename_;

    gr::iio::pluto_source::sptr plutosdr_source_;

    boost::shared_ptr<gr::block> valve_;
    gr::blocks::file_sink::sptr file_sink_;
    boost::shared_ptr<gr::msg_queue> queue_;
};

#endif /*GNSS_SDR_PLUTOSDR_SIGNAL_SOURCE_H_*/
