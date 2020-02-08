/*!
 * \file rtklib_pvt.h
 * \brief Interface of a Position Velocity and Time computation block
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_RTKLIB_PVT_H_
#define GNSS_SDR_RTKLIB_PVT_H_

#include "pvt_interface.h"           // for PvtInterface
#include "rtklib.h"                  // for rtk_t
#include "rtklib_pvt_gs.h"           // for rtklib_pvt_gs_sptr
#include <gnuradio/gr_complex.h>     // for gr_complex
#include <gnuradio/runtime_types.h>  // for basic_block_sptr, top_block_sptr
#include <cstddef>                   // for size_t
#include <ctime>                     // for time_t
#include <map>                       // for map
#include <string>                    // for string

class ConfigurationInterface;
class Galileo_Almanac;
class Galileo_Ephemeris;
class Gps_Almanac;
class Gps_Ephemeris;

/*!
 * \brief This class implements a PvtInterface for the RTKLIB PVT block
 */
class Rtklib_Pvt : public PvtInterface
{
public:
    Rtklib_Pvt(ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    virtual ~Rtklib_Pvt();

    inline std::string role() override
    {
        return role_;
    }

    //!  Returns "RTKLIB_PVT"
    inline std::string implementation() override
    {
        return "RTKLIB_PVT";
    }

    void clear_ephemeris() override;
    std::map<int, Gps_Ephemeris> get_gps_ephemeris() const override;
    std::map<int, Galileo_Ephemeris> get_galileo_ephemeris() const override;
    std::map<int, Gps_Almanac> get_gps_almanac() const override;
    std::map<int, Galileo_Almanac> get_galileo_almanac() const override;

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

    bool get_latest_PVT(double* longitude_deg,
        double* latitude_deg,
        double* height_m,
        double* ground_speed_kmh,
        double* course_over_ground_deg,
        time_t* UTC_time) override;

private:
    rtklib_pvt_gs_sptr pvt_;
    rtk_t rtk{};
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif
