/*!
 * \file gen_signal_source.h
 * \brief It wraps blocks that generates synthesized GNSS signal and filters
 *  it.
 * \author Marc Molina, 2013. marc.molina.pena@gmail.com
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

#ifndef GNSS_SDR_GEN_SIGNAL_SOURCE_H_
#define GNSS_SDR_GEN_SIGNAL_SOURCE_H_


#include "gnss_block_interface.h"
#include <gnuradio/msg_queue.h>
#include <string>

/*!
 * \brief This class wraps blocks that generates synthesized GNSS signal and
 * filters the signal.
 */
class GenSignalSource : public GNSSBlockInterface
{
public:
    //! Constructor
    GenSignalSource(GNSSBlockInterface *signal_generator, GNSSBlockInterface *filter,
        std::string role, boost::shared_ptr<gr::msg_queue> queue);

    //! Virtual destructor
    virtual ~GenSignalSource();

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    inline std::string role() override { return role_; }

    //! Returns "Signal Source"
    inline std::string implementation() override { return "Signal Source"; }
    inline size_t item_size() override { return 0; }

    inline GNSSBlockInterface *signal_generator() const { return signal_generator_; }

private:
    GNSSBlockInterface *signal_generator_;
    GNSSBlockInterface *filter_;
    std::string role_;
    std::string implementation_;
    bool connected_;
    boost::shared_ptr<gr::msg_queue> queue_;
};

#endif /*GNSS_SDR_GEN_SIGNAL_SOURCE_H*/
