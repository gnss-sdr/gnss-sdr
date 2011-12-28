/*!
 * \file gps_l1_ca_pvt.h
 * \brief Interface of an adapter of a GPS L1 C/A PVT solver block to a
 * PVTInterface
 * Position Velocity and Time
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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



#ifndef GNSS_SDR_GPS_L1_CA_PVT_H_
#define GNSS_SDR_GPS_L1_CA_PVT_H_

#include "pvt_interface.h"
#include "gps_l1_ca_pvt_cc.h"
#include <gnuradio/gr_msg_queue.h>

class ConfigurationInterface;

/*!
 * \brief This class implements a PVT interface for GPS L1 C/A
 */
class GpsL1CaPvt : public PvtInterface
{

public:

    GpsL1CaPvt(ConfigurationInterface* configuration,
            std::string role,
            unsigned int in_streams,
            unsigned int out_streams,
            gr_msg_queue_sptr queue);

    virtual ~GpsL1CaPvt();

    std::string role()
    {
        return role_;
    }
    std::string implementation()
    {
        return "pvt";
    }

    void connect(gr_top_block_sptr top_block);
    void disconnect(gr_top_block_sptr top_block);
    gr_basic_block_sptr get_left_block();
    gr_basic_block_sptr get_right_block();

    void reset()
    {
        return;
    };
    // all blocks must have an intem_size() function implementation
    size_t item_size()
    {
        return sizeof(gr_complex);
    }

private:

    gps_l1_ca_pvt_cc_sptr pvt_;
    bool dump_;
    unsigned int fs_in_;
    std::string dump_filename_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    gr_msg_queue_sptr queue_;
};

#endif
