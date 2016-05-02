/*!
 * \file sbas_time.h
 * \brief Interface and implementation of classes to handle and relate sample stamp and GPS time based time scales
 * \author Daniel Fehr 2013. daniel.co(at)bluewin.ch
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

#ifndef GNSS_SDR_SBAS_TIME_H_
#define GNSS_SDR_SBAS_TIME_H_

#include <cmath>
#include <iostream>
#include <glog/logging.h>

#define EVENT 2 // logs important events which don't occur every update() call
#define FLOW 3  // logs the function calls of block processing functions

class Sbas_Time_Relation
{
public:
    Sbas_Time_Relation()
    {
        i_gps_week = 0;
        d_delta_sec = 0;
        b_valid = false;
        VLOG(FLOW) << "<<R>> new invalid time relation: i_gps_week=" << i_gps_week << " d_delta_sec=" << d_delta_sec;
    }

    Sbas_Time_Relation(double time_stamp_sec, int gps_week, double gps_sec)
    {
        i_gps_week = gps_week;
        d_delta_sec = gps_sec - time_stamp_sec;
        b_valid = true;
        VLOG(FLOW) << "<<R>> new time relation: i_gps_week=" << i_gps_week << " d_delta_sec=" << d_delta_sec;
    }

    bool to_gps_time(double time_stamp_sec, int &gps_week, double &gps_sec)
    {
        int delta_weeks = int(trunc(time_stamp_sec + d_delta_sec))/604800;
        gps_sec = time_stamp_sec + d_delta_sec - delta_weeks*604800;
        gps_week = i_gps_week + delta_weeks;
        VLOG(FLOW) << "<<R>> to gps time: time_stamp_sec=" << time_stamp_sec << " gps_week=" << gps_week << " gps_sec=" << gps_sec;
        return b_valid;
    }

    bool to_sample_stamp(int gps_week, double gps_sec, double &time_stamp_sec)
    {
        time_stamp_sec = (gps_sec - d_delta_sec) + (gps_week - i_gps_week)*604800;
        VLOG(FLOW) << "<<R>> to gps time: gps_week=" << gps_week << " gps_sec=" << gps_sec << " time_stamp_sec=" << time_stamp_sec;
        return b_valid;
    }

    bool is_valid()
    {
        return b_valid;
    }

private:
    int i_gps_week;
    double d_delta_sec;
    bool b_valid;
};


/*!
 * \brief Sbas_Time relates the relative sample stamp time scale with the absolute GPS time scale.
 * There are three different states for a Sbas_Time object:
 *     - only relative time (sample stamp) is known
 *     - only absolute time (gps time) is known
 *     - absolute and relative time and their relation is known
 */
class Sbas_Time
{
public:
    enum Sbas_Time_State {RELATIVE, /*ABSOLUTE,*/ RELATED, UNDEFINED};

    Sbas_Time()
    {
        e_state = UNDEFINED;
        i_gps_week = 0;
        d_gps_sec = 0;
        d_time_stamp_sec = 0;
    }

    Sbas_Time(double time_stamp_sec)
    {
        d_time_stamp_sec = time_stamp_sec;
        i_gps_week = 0;
        d_gps_sec = 0;
        e_state = RELATIVE;
    }
    /*
    Sbas_Time(int gps_week, double gps_sec)
    {
        i_gps_week = gps_week;
        d_gps_sec = gps_sec;
        d_time_stamp_sec = 0;
        e_state = ABSOLUTE;
    }*/
    Sbas_Time(double time_stamp_sec, Sbas_Time_Relation relation)
    {
        if(relation.is_valid())
            {    // construct a RELATED object
                d_time_stamp_sec = time_stamp_sec;
                relation.to_gps_time(d_time_stamp_sec, i_gps_week, d_gps_sec);
                e_state = RELATED;
            }
        else
            {    // construct a RELATIVE object
                *this = Sbas_Time(time_stamp_sec);
                VLOG(FLOW) << "<<R>> create RELATIVE time (invalid relation): time_stamp_sec=" << time_stamp_sec;
            }
    }
    /*Sbas_Time(int gps_week, double gps_sec, Sbas_Time_Relation relation)
    {
        i_gps_week = gps_week;
        d_gps_sec = gps_sec;
        relation.to_sample_stamp(gps_week, gps_sec, d_time_stamp_sec);
        e_state = RELATED;
    }*/

    void relate(Sbas_Time_Relation sbas_time_realtion)
    {
        switch (e_state)
        {
        case RELATIVE: *this = Sbas_Time(d_time_stamp_sec, sbas_time_realtion);
        break;

        //case ABSOLUTE: return Sbas_Time(i_gps_week, d_gps_sec, sbas_time_realtion);
        break;

        case RELATED:  LOG(INFO) << "Relating an already related Sbas_Time object is not possible";
        break;

        case UNDEFINED: std::cerr << "Sbas_Time object state undefined" << std::endl;
        break;

        default: std::cerr << "Sbas_Time object state not known" << std::endl;
        break;
        }
        return;
    }

    double get_time_stamp()
    {
        return d_time_stamp_sec;
        //return (e_state == RELATIVE || e_state == RELATED);
    }

    bool get_gps_time(int &gps_week, double &gps_sec)
    {
        gps_week = i_gps_week;
        gps_sec = d_gps_sec;
        return (/*e_state == ABSOLUTE ||*/ e_state == RELATED);
    }

    bool is_only_relativ() { return e_state == RELATIVE; }
    //bool is_only_absolute() {return e_state == ABSOLUTE;}
    bool is_related() { return e_state == RELATED; }
    Sbas_Time_State get_state() { return e_state; }

private:
    Sbas_Time_State e_state;
    double d_time_stamp_sec;
    int i_gps_week;
    double d_gps_sec;
};


#endif /* GNSS_SDR_SBAS_TIME_H_ */
