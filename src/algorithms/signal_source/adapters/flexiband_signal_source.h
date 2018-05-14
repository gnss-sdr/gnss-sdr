/*!
 * \file raw_array_signal_source.h
 * \brief Signal Source adapter for the Teleorbit Flexiband front-end device.
 * This adapter requires a Flexiband GNURadio driver installed (not included with GNSS-SDR)
 * \author Javier Arribas, jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef FLEXIBAND_SIGNAL_SOURCE_H_
#define FLEXIBAND_SIGNAL_SOURCE_H_

#include "gnss_block_interface.h"
#include <gnuradio/hier_block2.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/char_to_float.h>
#include <gnuradio/blocks/float_to_complex.h>
#include <string>
#include <vector>


class ConfigurationInterface;

/*!
 * \brief This class configures and reads samples from Teleorbit Flexiband front-end.
 * This software requires a Flexiband GNU Radio driver installed (not included with GNSS-SDR).
 */
class FlexibandSignalSource : public GNSSBlockInterface
{
public:
    FlexibandSignalSource(ConfigurationInterface* configuration,
        std::string role, unsigned int in_stream,
        unsigned int out_stream, gr::msg_queue::sptr queue);

    virtual ~FlexibandSignalSource();

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "Flexiband_Signal_Source".
     */
    inline std::string implementation() override
    {
        return "Flexiband_Signal_Source";
    }

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;
    gr::basic_block_sptr get_right_block(int RF_channel) override;

private:
    std::string role_;
    unsigned int in_stream_;
    unsigned int out_stream_;
    std::string item_type_;
    size_t item_size_;

    std::string firmware_filename_;
    int gain1_;
    int gain2_;
    int gain3_;
    int usb_packet_buffer_size_;
    bool AGC_;
    std::string signal_file;
    bool flag_read_file;

    int RF_channels_;

    gr::block_sptr flexiband_source_;

    std::vector<boost::shared_ptr<gr::block>> char_to_float;
    std::vector<boost::shared_ptr<gr::block>> float_to_complex_;

    boost::shared_ptr<gr::msg_queue> queue_;
};

#endif /*FLEXIBAND_SIGNAL_SOURCE_H_*/
