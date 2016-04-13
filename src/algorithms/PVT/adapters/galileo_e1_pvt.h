/*!
 * \file galileo_e1_pvt.h
 * \brief Interface of an adapter of a GALILEO E1 PVT solver block to a
 * PvtInterface.
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
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



#ifndef GNSS_SDR_GALILEO_E1_PVT_H_
#define GNSS_SDR_GALILEO_E1_PVT_H_

#include <string>
#include "pvt_interface.h"
#include "galileo_e1_pvt_cc.h"


class ConfigurationInterface;

/*!
 * \brief This class implements a PvtInterface for Galileo E1
 */
class GalileoE1Pvt : public PvtInterface
{
public:
    GalileoE1Pvt(ConfigurationInterface* configuration,
            std::string role,
            unsigned int in_streams,
            unsigned int out_streams);

    virtual ~GalileoE1Pvt();

    std::string role()
    {
        return role_;
    }

    //!  Returns "GALILEO_E1_PVT"
    std::string implementation()
    {
        return "GALILEO_E1_PVT";
    }

    void connect(gr::top_block_sptr top_block);
    void disconnect(gr::top_block_sptr top_block);
    gr::basic_block_sptr get_left_block();
    gr::basic_block_sptr get_right_block();

    void reset()
    {
        return;
    }

    //! All blocks must have an item_size() function implementation. Returns sizeof(gr_complex)
    size_t item_size()
    {
        return sizeof(gr_complex);
    }

private:
    galileo_e1_pvt_cc_sptr pvt_;
    bool dump_;
    //unsigned int fs_in_;
    std::string dump_filename_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif
