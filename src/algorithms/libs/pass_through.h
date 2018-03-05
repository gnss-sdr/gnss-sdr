/*!
 * \file pass_through.h
 * \brief Interface of a block that just puts its input in its
 *        output.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
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

#ifndef GNSS_SDR_PASS_THROUGH_H_
#define GNSS_SDR_PASS_THROUGH_H_

#include "conjugate_cc.h"
#include "conjugate_sc.h"
#include "conjugate_ic.h"
#include "gnss_block_interface.h"
#include <gnuradio/hier_block2.h>
#include <gnuradio/blocks/copy.h>
#include <string>


class ConfigurationInterface;

/*!
 * \brief This class implements a block that connects input and output (does nothing)
 */
class Pass_Through : public GNSSBlockInterface
{
public:
    Pass_Through(ConfigurationInterface* configuration,
        std::string role,
        unsigned int in_stream,
        unsigned int out_stream);

    virtual ~Pass_Through();

    inline std::string role() override
    {
        return role_;
    }

    //! returns "Pass_Through"
    inline std::string implementation() override
    {
        return "Pass_Through";
    }

    inline std::string item_type() const
    {
        return item_type_;
    }

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    std::string item_type_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    //gr_kludge_copy_sptr kludge_copy_;
    gr::blocks::copy::sptr kludge_copy_;
    size_t item_size_;
    conjugate_cc_sptr conjugate_cc_;
    conjugate_sc_sptr conjugate_sc_;
    conjugate_ic_sptr conjugate_ic_;
    bool inverted_spectrum;
};

#endif /*GNSS_SDR_PASS_THROUGH_H_*/
