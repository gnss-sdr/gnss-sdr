/*!
 * \file rtklib_pvt.h
 * \brief Interface of a Position Velocity and Time computation block
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_RTKLIB_PVT_H_
#define GNSS_SDR_RTKLIB_PVT_H_

#include "pvt_interface.h"
#include "rtklib_pvt_cc.h"
#include <string>


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

    inline std::string role() override
    {
        return role_;
    }

    //!  Returns "RTKLIB_Pvt"
    inline std::string implementation() override
    {
        return "RTKLIB_PVT";
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    inline void reset() override
    {
        return;
    }

    //! All blocks must have an item_size() function implementation. Returns sizeof(gr_complex)
    inline size_t item_size() override
    {
        return sizeof(gr_complex);
    }

private:
    rtklib_pvt_cc_sptr pvt_;
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
