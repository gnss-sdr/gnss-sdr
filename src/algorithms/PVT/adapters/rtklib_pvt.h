/*!
 * \file rtklib_pvt.h
 * \brief Interface of a Position Velocity and Time computation block
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
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



#ifndef GNSS_SDR_RTKLIB_PVT_H_
#define GNSS_SDR_RTKLIB_PVT_H_

#include <string>
#include "pvt_interface.h"
#include "rtklib_pvt_cc.h"


class ConfigurationInterface;

/*!
 * \brief This class implements a PvtInterface for Galileo E1
 */
class RtklibPvt : public PvtInterface
{
public:
    RtklibPvt(ConfigurationInterface* configuration,
            std::string role,
            unsigned int in_streams,
            unsigned int out_streams);

    virtual ~RtklibPvt();

    std::string role()
    {
        return role_;
    }

    //!  Returns "RTKLIB_Pvt"
    std::string implementation()
    {
        return "RTKLIB_PVT";
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
    rtklib_pvt_cc_sptr pvt_;

    sol_t  sol_  = {{0,0}, {0,0,0,0,0,0}, {0,0,0,0,0,0}, {0,0,0,0,0,0}, '0', '0', '0', 0, 0, 0 };
    ambc_t ambc_ = { {{0,0}, {0,0}, {0,0}, {0,0}}, {0, 0, 0, 0}, {}, {}, 0, {'0'}};
    ssat_t ssat_ =  { '0', /* navigation system */
            '0', /* valid satellite flag single */
            {0.0}, /* azel[2] azimuth/elevation angles {az,el} (rad) */
            {0.0}, /* residuals of pseudorange (m) */
            {0.0}, /* residuals of carrier-phase (m) */
            {'0'}, /* valid satellite flag */
            {'0'}, /* signal strength (0.25 dBHz) */
            {'0'}, /* ambiguity fix flag (1:fix,2:float,3:hold) */
            {'0'}, /* cycle-slip flag */
            {'0'}, /* half-cycle valid flag */
            {},    /* lock counter of phase */
            {},    /* obs outage counter of phase */
            {},    /* cycle-slip counter */
            {},    /* reject counter */
            0.0,   /* geometry-free phase L1-L2 (m) */
            0.0,   /* geometry-free phase L1-L5 (m) */
            0.0,   /* MW-LC (m) */
            0.0,   /* phase windup (cycle) */
            {{{0,0}},{{0,0}}},  /* previous carrier-phase time */
            {{},{}} /* previous carrier-phase observable (cycle) */
    };

    rtk_t rtk;

    bool dump_;
    std::string dump_filename_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;

    std::string eph_xml_filename_;
    bool save_assistance_to_XML();
};

#endif
