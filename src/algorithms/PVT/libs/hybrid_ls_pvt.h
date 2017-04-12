/*!
 * \file galileo_e1_ls_pvt.h
 * \brief Interface of a Least Squares Position, Velocity, and Time (PVT)
 * solver, based on K.Borre's Matlab receiver.
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_HYBRID_LS_PVT_H_
#define GNSS_SDR_HYBRID_LS_PVT_H_

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include "ls_pvt.h"
#include "galileo_navigation_message.h"
#include "gps_navigation_message.h"
#include "gps_cnav_navigation_message.h"
#include "gnss_synchro.h"


typedef struct {        /* time struct */
    time_t time;        /* time (s) expressed by standard time_t */
    double sec;         /* fraction of second under 1 s */
} gtime_t;

/*!
 * \brief This class implements a simple PVT Least Squares solution
 */
class hybrid_ls_pvt : public Ls_Pvt
{
private:
	/* convert calendar day/time to time -------------------------------------------
	* convert calendar day/time to gtime_t struct
	* args   : double *ep       I   day/time {year,month,day,hour,min,sec}
	* return : gtime_t struct
	* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
	*-----------------------------------------------------------------------------*/
    gtime_t epoch2time(const double *ep);

    /* gps time to time ------------------------------------------------------------
    * convert week and tow in gps time to gtime_t struct
    * args   : int    week      I   week number in gps time
    *          double sec       I   time of week in gps time (s)
    * return : gtime_t struct
    *-----------------------------------------------------------------------------*/
    gtime_t gpst2time(int week, double sec);

    /* get time string -------------------------------------------------------------
    * get time string
    * args   : gtime_t t        I   gtime_t struct
    *          int    n         I   number of decimals
    * return : time string
    * notes  : not reentrant, do not use multiple in a function
    *-----------------------------------------------------------------------------*/
    char *time_str(gtime_t t, int n);

    /* time to string --------------------------------------------------------------
    * convert gtime_t struct to string
    * args   : gtime_t t        I   gtime_t struct
    *          char   *s        O   string ("yyyy/mm/dd hh:mm:ss.ssss")
    *          int    n         I   number of decimals
    * return : none
    *-----------------------------------------------------------------------------*/
    void time2str(gtime_t t, char *s, int n);

    /* time to calendar day/time ---------------------------------------------------
    * convert gtime_t struct to calendar day/time
    * args   : gtime_t t        I   gtime_t struct
    *          double *ep       O   day/time {year,month,day,hour,min,sec}
    * return : none
    * notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
    *-----------------------------------------------------------------------------*/
    void time2epoch(gtime_t t, double *ep);

    /* adjust gps week number ------------------------------------------------------
    * adjust gps week number using cpu time
    * args   : int   week       I   not-adjusted gps week number
    * return : adjusted gps week number
    *-----------------------------------------------------------------------------*/
    int adjgpsweek(int week);

    /* time to gps time ------------------------------------------------------------
    * convert gtime_t struct to week and tow in gps time
    * args   : gtime_t t        I   gtime_t struct
    *          int    *week     IO  week number in gps time (NULL: no output)
    * return : time of week in gps time (s)
    *-----------------------------------------------------------------------------*/
    double time2gpst(gtime_t t, int *week);

    /* utc to gpstime --------------------------------------------------------------
    * convert utc to gpstime considering leap seconds
    * args   : gtime_t t        I   time expressed in utc
    * return : time expressed in gpstime
    * notes  : ignore slight time offset under 100 ns
    *-----------------------------------------------------------------------------*/
    gtime_t utc2gpst(gtime_t t);

    /* time difference -------------------------------------------------------------
    * difference between gtime_t structs
    * args   : gtime_t t1,t2    I   gtime_t structs
    * return : time difference (t1-t2) (s)
    *-----------------------------------------------------------------------------*/
    double timediff(gtime_t t1, gtime_t t2);

    /* add time --------------------------------------------------------------------
    * add time to gtime_t struct
    * args   : gtime_t t        I   gtime_t struct
    *          double sec       I   time to add (s)
    * return : gtime_t struct (t+sec)
    *-----------------------------------------------------------------------------*/
    gtime_t timeadd(gtime_t t, double sec);

    /* get current time in utc -----------------------------------------------------
    * get current time in utc
    * args   : none
    * return : current time in utc
    *-----------------------------------------------------------------------------*/
    gtime_t timeget();


public:
    hybrid_ls_pvt(int nchannels,std::string dump_filename, bool flag_dump_to_file);
    ~hybrid_ls_pvt();

    bool get_PVT(std::map<int,Gnss_Synchro> gnss_observables_map, double Rx_time, bool flag_averaging);
    int d_nchannels;                                        //!< Number of available channels for positioning

    std::map<int,Galileo_Ephemeris> galileo_ephemeris_map; //!< Map storing new Galileo_Ephemeris
    std::map<int,Gps_Ephemeris> gps_ephemeris_map; //!< Map storing new GPS_Ephemeris
    std::map<int,Gps_CNAV_Ephemeris> gps_cnav_ephemeris_map;
    
    Galileo_Utc_Model galileo_utc_model;
    Galileo_Iono galileo_iono;
    Galileo_Almanac galileo_almanac;

    Gps_Utc_Model gps_utc_model;
    Gps_Iono gps_iono;

    Gps_CNAV_Iono gps_cnav_iono;
    Gps_CNAV_Utc_Model gps_cnav_utc_model;

    double d_galileo_current_time;

    int count_valid_position;

    bool d_flag_dump_enabled;
    bool d_flag_averaging;

    std::string d_dump_filename;
    std::ofstream d_dump_file;
};

#endif
