/*!
 * \file pass_through.h
 * \brief This class represent a block that just puts its input in its
 *        output.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
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




#ifndef PASS_THROUGH_H_
#define PASS_THROUGH_H_

#include <gr_hier_block2.h>
#include <gr_kludge_copy.h>
#include <gr_file_sink.h>

#include "gnss_block_interface.h"

class ConfigurationInterface;

class PassThrough : public GNSSBlockInterface
{

public:
    PassThrough(ConfigurationInterface* configuration,
            std::string role,
            unsigned int in_stream,
            unsigned int out_stream);

    virtual ~PassThrough();

    std::string role()
    {
        return role_;
    }
    std::string implementation()
    {
        return "PassThrough";
    }

    std::string item_type()
    {
        return item_type_;
    }
    size_t vector_size()
    {
        return vector_size_;
    }
    size_t item_size()
    {
        return item_size_;
    }

    void connect(gr_top_block_sptr top_block);
    void disconnect(gr_top_block_sptr top_block);
    gr_basic_block_sptr get_left_block();
    gr_basic_block_sptr get_right_block();

private:

    std::string item_type_;
    size_t vector_size_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;

    gr_kludge_copy_sptr kludge_copy_;
    size_t item_size_;
};

#endif /*PASS_THROUGH_H_*/
