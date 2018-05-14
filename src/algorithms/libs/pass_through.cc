/*!
 * \file pass_through.cc
 * \brief Implementation of a block that just puts its input in its
 *        output.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
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

#include "pass_through.h"
#include "configuration_interface.h"
#include <glog/logging.h>
#include <volk/volk.h>
#include <complex>

using google::LogMessage;

Pass_Through::Pass_Through(ConfigurationInterface* configuration, std::string role,
    unsigned int in_streams,
    unsigned int out_streams) : role_(role),
                                in_streams_(in_streams),
                                out_streams_(out_streams)
{
    std::string default_item_type = "gr_complex";
    std::string input_type = configuration->property(role + ".input_item_type", default_item_type);
    std::string output_type = configuration->property(role + ".output_item_type", default_item_type);
    if (input_type.compare(output_type) != 0)
        {
            LOG(WARNING) << "input_item_type and output_item_type are different in a Pass_Through implementation! Taking "
                         << input_type
                         << ", but item_size will supersede it.";
        }

    item_type_ = configuration->property(role + ".item_type", input_type);
    inverted_spectrum = configuration->property(role + ".inverted_spectrum", false);

    if (item_type_.compare("float") == 0)
        {
            item_size_ = sizeof(float);
        }
    else if (item_type_.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
            if (inverted_spectrum)
                {
                    conjugate_cc_ = make_conjugate_cc();
                }
        }
    else if (item_type_.compare("short") == 0)
        {
            item_size_ = sizeof(int16_t);
        }
    else if (item_type_.compare("ishort") == 0)
        {
            item_size_ = sizeof(int16_t);
        }
    else if (item_type_.compare("cshort") == 0)
        {
            item_size_ = sizeof(lv_16sc_t);
            if (inverted_spectrum)
                {
                    conjugate_sc_ = make_conjugate_sc();
                }
        }
    else if (item_type_.compare("byte") == 0)
        {
            item_size_ = sizeof(int8_t);
        }
    else if (item_type_.compare("ibyte") == 0)
        {
            item_size_ = sizeof(int8_t);
        }
    else if (item_type_.compare("cbyte") == 0)
        {
            item_size_ = sizeof(lv_8sc_t);
            if (inverted_spectrum)
                {
                    conjugate_ic_ = make_conjugate_ic();
                }
        }
    else
        {
            LOG(WARNING) << item_type_ << " unrecognized item type. Using float";
            item_size_ = sizeof(float);
        }

    kludge_copy_ = gr::blocks::copy::make(item_size_);
    DLOG(INFO) << "kludge_copy(" << kludge_copy_->unique_id() << ")";
}


Pass_Through::~Pass_Through()
{
}


void Pass_Through::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    DLOG(INFO) << "nothing to connect internally";
}


void Pass_Through::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to disconnect
}


gr::basic_block_sptr Pass_Through::get_left_block()
{
    if (inverted_spectrum)
        {
            if (item_type_.compare("gr_complex") == 0)
                {
                    return conjugate_cc_;
                }
            else if (item_type_.compare("cshort") == 0)
                {
                    return conjugate_sc_;
                }
            else if (item_type_.compare("cbyte") == 0)
                {
                    return conjugate_ic_;
                }
            else
                {
                    LOG(WARNING) << "Setting inverted_spectrum to true with item_type "
                                 << item_type_ << " is not defined and has no effect.";
                }
        }

    return kludge_copy_;
}


gr::basic_block_sptr Pass_Through::get_right_block()
{
    if (inverted_spectrum)
        {
            if (item_type_.compare("gr_complex") == 0)
                {
                    return conjugate_cc_;
                }
            else if (item_type_.compare("cshort") == 0)
                {
                    return conjugate_sc_;
                }
            else if (item_type_.compare("cbyte") == 0)
                {
                    return conjugate_ic_;
                }
            else
                {
                    DLOG(WARNING) << "Setting inverted_spectrum to true with item_type "
                                  << item_type_ << " is not defined and has no effect.";
                }
        }

    return kludge_copy_;
}
