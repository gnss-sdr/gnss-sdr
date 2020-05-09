/*!
 * \file byte_to_short.h
 * \brief Adapts an 8-bits sample stream (IF) to a short int stream (IF)
 * \author Carles Fernandez Prades, cfernandez(at)cttc.es
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

#ifndef GNSS_SDR_BYTE_TO_SHORT_H
#define GNSS_SDR_BYTE_TO_SHORT_H

#include "gnss_block_interface.h"
#include <gnuradio/blocks/char_to_short.h>
#include <gnuradio/blocks/file_sink.h>
#include <string>

class ConfigurationInterface;

/*!
 * \brief Adapts an 8-bits sample stream (IF) to a short int stream (IF)
 *
 */
class ByteToShort : public GNSSBlockInterface
{
public:
    ByteToShort(ConfigurationInterface* configuration,
        std::string role, unsigned int in_streams,
        unsigned int out_streams);

    ~ByteToShort() = default;

    inline std::string role() override
    {
        return role_;
    }

    //! Returns "Byte_To_Short"
    inline std::string implementation() override
    {
        return "Byte_To_Short";
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
    gr::blocks::char_to_short::sptr gr_char_to_short_;
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
