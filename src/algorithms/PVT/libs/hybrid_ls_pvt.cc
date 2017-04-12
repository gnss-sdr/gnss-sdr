/*!
 * \file galileo_e1_ls_pvt.cc
 * \brief Implementation of a Least Squares Position, Velocity, and Time
 * (PVT) solver, based on K.Borre's Matlab receiver.
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

#include "hybrid_ls_pvt.h"
#include <glog/logging.h>
#include "GPS_L1_CA.h"
#include "Galileo_E1.h"


using google::LogMessage;

hybrid_ls_pvt::hybrid_ls_pvt(int nchannels, std::string dump_filename, bool flag_dump_to_file) : Ls_Pvt()
{
    // init empty ephemeris for all the available GNSS channels
    d_nchannels = nchannels;
    d_dump_filename = dump_filename;
    d_flag_dump_enabled = flag_dump_to_file;
    d_galileo_current_time = 0;
    count_valid_position = 0;
    d_flag_averaging = false;
    // ############# ENABLE DATA FILE LOG #################
    if (d_flag_dump_enabled == true)
    {
        if (d_dump_file.is_open() == false)
        {
            try
            {
                d_dump_file.exceptions (std::ifstream::failbit | std::ifstream::badbit);
                d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                LOG(INFO) << "PVT lib dump enabled Log file: " << d_dump_filename.c_str();
            }
            catch (const std::ifstream::failure &e)
            {
                LOG(WARNING) << "Exception opening PVT lib dump file " << e.what();
            }
        }
    }
}


hybrid_ls_pvt::~hybrid_ls_pvt()
{
    d_dump_file.close();
}


gtime_t hybrid_ls_pvt::epoch2time(const double *ep)
{
    const int doy[]={1,32,60,91,121,152,182,213,244,274,305,335};
    gtime_t time={0};
    int days,sec,year=(int)ep[0],mon=(int)ep[1],day=(int)ep[2];

    if (year<1970||2099<year||mon<1||12<mon) return time;

    /* leap year if year%4==0 in 1901-2099 */
    days=(year-1970)*365+(year-1969)/4+doy[mon-1]+day-2+(year%4==0&&mon>=3?1:0);
    sec=(int)floor(ep[5]);
    time.time=(time_t)days*86400+(int)ep[3]*3600+(int)ep[4]*60+sec;
    time.sec=ep[5]-sec;
    return time;
}

gtime_t hybrid_ls_pvt::gpst2time(int week, double sec)
{
	const static double gpst0[]={1980,1, 6,0,0,0}; /* gps time reference */
    gtime_t t=epoch2time(gpst0);

    if (sec<-1E9||1E9<sec) sec=0.0;
    t.time+=86400*7*week+(int)sec;
    t.sec=sec-(int)sec;
    return t;
}

void hybrid_ls_pvt::time2str(gtime_t t, char *s, int n)
{
    double ep[6];

    if (n<0) n=0; else if (n>12) n=12;
    if (1.0-t.sec<0.5/pow(10.0,n)) {t.time++; t.sec=0.0;};
    time2epoch(t,ep);
    sprintf(s,"%04.0f/%02.0f/%02.0f %02.0f:%02.0f:%0*.*f",ep[0],ep[1],ep[2],
            ep[3],ep[4],n<=0?2:n+3,n<=0?0:n,ep[5]);
}

double hybrid_ls_pvt::time2gpst(gtime_t t, int *week)
{
	const static double gpst0[]={1980,1, 6,0,0,0}; /* gps time reference */
    gtime_t t0=epoch2time(gpst0);
    time_t sec=t.time-t0.time;
    int w=(int)(sec/(86400*7));

    if (week) *week=w;
    return (double)(sec-w*86400*7)+t.sec;
}

void hybrid_ls_pvt::time2epoch(gtime_t t, double *ep)
{
    const int mday[]={ /* # of days in a month */
        31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
        31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
    };
    int days,sec,mon,day;

    /* leap year if year%4==0 in 1901-2099 */
    days=(int)(t.time/86400);
    sec=(int)(t.time-(time_t)days*86400);
    for (day=days%1461,mon=0;mon<48;mon++) {
        if (day>=mday[mon]) day-=mday[mon]; else break;
    }
    ep[0]=1970+days/1461*4+mon/12; ep[1]=mon%12+1; ep[2]=day+1;
    ep[3]=sec/3600; ep[4]=sec%3600/60; ep[5]=sec%60+t.sec;
}

char* hybrid_ls_pvt::time_str(gtime_t t, int n)
{
    static char buff[64];
    time2str(t,buff,n);
    return buff;
}


/* time difference -------------------------------------------------------------
* difference between gtime_t structs
* args   : gtime_t t1,t2    I   gtime_t structs
* return : time difference (t1-t2) (s)
*-----------------------------------------------------------------------------*/
double hybrid_ls_pvt::timediff(gtime_t t1, gtime_t t2)
{
    return difftime(t1.time,t2.time)+t1.sec-t2.sec;
}

/* add time --------------------------------------------------------------------
* add time to gtime_t struct
* args   : gtime_t t        I   gtime_t struct
*          double sec       I   time to add (s)
* return : gtime_t struct (t+sec)
*-----------------------------------------------------------------------------*/
gtime_t hybrid_ls_pvt::timeadd(gtime_t t, double sec)
{
    double tt;

    t.sec+=sec; tt=floor(t.sec); t.time+=(int)tt; t.sec-=tt;
    return t;
}


gtime_t hybrid_ls_pvt::utc2gpst(gtime_t t)
{
	const int MAXLEAPS=64;
	static double leaps[MAXLEAPS+1][7]={ /* leap seconds (y,m,d,h,m,s,utc-gpst) */
	    {2017,1,1,0,0,0,-18},
	    {2015,7,1,0,0,0,-17},
	    {2012,7,1,0,0,0,-16},
	    {2009,1,1,0,0,0,-15},
	    {2006,1,1,0,0,0,-14},
	    {1999,1,1,0,0,0,-13},
	    {1997,7,1,0,0,0,-12},
	    {1996,1,1,0,0,0,-11},
	    {1994,7,1,0,0,0,-10},
	    {1993,7,1,0,0,0, -9},
	    {1992,7,1,0,0,0, -8},
	    {1991,1,1,0,0,0, -7},
	    {1990,1,1,0,0,0, -6},
	    {1988,1,1,0,0,0, -5},
	    {1985,7,1,0,0,0, -4},
	    {1983,7,1,0,0,0, -3},
	    {1982,7,1,0,0,0, -2},
	    {1981,7,1,0,0,0, -1},
	    {0}
	};
    int i;
    for (i=0;leaps[i][0]>0;i++) {
        if (timediff(t,epoch2time(leaps[i]))>=0.0) return timeadd(t,-leaps[i][6]);
    }
    return t;
}

gtime_t hybrid_ls_pvt::timeget(void)
{
	static double timeoffset_=0.0;        /* time offset (s) */
    double ep[6]={0};

    struct timeval tv;
    struct tm *tt;

    if (!gettimeofday(&tv,NULL)&&(tt=gmtime(&tv.tv_sec))) {
        ep[0]=tt->tm_year+1900; ep[1]=tt->tm_mon+1; ep[2]=tt->tm_mday;
        ep[3]=tt->tm_hour; ep[4]=tt->tm_min; ep[5]=tt->tm_sec+tv.tv_usec*1E-6;
    }
    return timeadd(epoch2time(ep),timeoffset_);
}

int hybrid_ls_pvt::adjgpsweek(int week)
{
    int w;
    (void)time2gpst(utc2gpst(timeget()),&w);
    if (w<1560) w=1560; /* use 2009/12/1 if time is earlier than 2009/12/1 */
    return week+(w-week+512)/1024*1024;
}

bool hybrid_ls_pvt::get_PVT(std::map<int,Gnss_Synchro> gnss_observables_map, double Rx_time, bool flag_averaging)
{
    std::map<int,Gnss_Synchro>::iterator gnss_observables_iter;
    std::map<int,Galileo_Ephemeris>::iterator galileo_ephemeris_iter;
    std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter;
    std::map<int,Gps_CNAV_Ephemeris>::iterator gps_cnav_ephemeris_iter;

    arma::vec W;      // channels weight vector
    arma::vec obs;    // pseudoranges observation vector
    arma::mat satpos; // satellite positions matrix

    int Galileo_week_number = 0;
    int GPS_week = 0;
    double utc = 0.0;
    double GST = 0.0;
    double secondsperweek = 604800.0;

    double TX_time_corrected_s = 0.0;
    double SV_clock_bias_s = 0.0;

    d_flag_averaging = flag_averaging;

    // ********************************************************************************
    // ****** PREPARE THE LEAST SQUARES DATA (SV POSITIONS MATRIX AND OBS VECTORS) ****
    // ********************************************************************************
    int valid_obs = 0; //valid observations counter


    for(gnss_observables_iter = gnss_observables_map.begin();
            gnss_observables_iter != gnss_observables_map.end();
            gnss_observables_iter++)
        {
            switch(gnss_observables_iter->second.System)
            {
            case 'E':
                {
                    // 1 Gal - find the ephemeris for the current GALILEO SV observation. The SV PRN ID is the map key
                    galileo_ephemeris_iter = galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                    if (galileo_ephemeris_iter != galileo_ephemeris_map.end())
                        {
                            /*!
                             * \todo Place here the satellite CN0 (power level, or weight factor)
                             */
                            W.resize(valid_obs + 1, 1);
                            W(valid_obs) = 1;

                            // COMMON RX TIME PVT ALGORITHM
                            double Tx_time = Rx_time - gnss_observables_iter->second.Pseudorange_m / GALILEO_C_m_s;

                            // 2- compute the clock drift using the clock model (broadcast) for this SV
                            SV_clock_bias_s = galileo_ephemeris_iter->second.sv_clock_drift(Tx_time);

                            // 3- compute the current ECEF position for this SV using corrected TX time
                            TX_time_corrected_s = Tx_time - SV_clock_bias_s;
                            galileo_ephemeris_iter->second.satellitePosition(TX_time_corrected_s);

                            //store satellite positions in a matrix
                            satpos.resize(3, valid_obs + 1);
                            satpos(0, valid_obs) = galileo_ephemeris_iter->second.d_satpos_X;
                            satpos(1, valid_obs) = galileo_ephemeris_iter->second.d_satpos_Y;
                            satpos(2, valid_obs) = galileo_ephemeris_iter->second.d_satpos_Z;

                            // 4- fill the observations vector with the corrected observables
                            obs.resize(valid_obs + 1, 1);
                            obs(valid_obs) = gnss_observables_iter->second.Pseudorange_m + SV_clock_bias_s * GALILEO_C_m_s - d_rx_dt_s * GALILEO_C_m_s;
                            d_visible_satellites_IDs[valid_obs] = galileo_ephemeris_iter->second.i_satellite_PRN;
                            d_visible_satellites_CN0_dB[valid_obs] = gnss_observables_iter->second.CN0_dB_hz;

                            Galileo_week_number = galileo_ephemeris_iter->second.WN_5; //for GST
                            GST = galileo_ephemeris_iter->second.Galileo_System_Time(Galileo_week_number, Rx_time);

                            // SV ECEF DEBUG OUTPUT
                            DLOG(INFO) << "ECEF satellite SV ID=" << galileo_ephemeris_iter->second.i_satellite_PRN
                                       << " X=" << galileo_ephemeris_iter->second.d_satpos_X
                                       << " [m] Y=" << galileo_ephemeris_iter->second.d_satpos_Y
                                       << " [m] Z=" << galileo_ephemeris_iter->second.d_satpos_Z
                                       << " [m] PR_obs=" << obs(valid_obs) << " [m]";

                            valid_obs++;
                        }
                    else // the ephemeris are not available for this SV
                        {
                            DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->second.PRN;
                        }
                    break;
                }
            case 'G':
                {
                    // 1 GPS - find the ephemeris for the current GPS SV observation. The SV PRN ID is the map key
                    std::string sig_(gnss_observables_iter->second.Signal);
                    if(sig_.compare("1C") == 0)
                        {
                            gps_ephemeris_iter = gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                            if (gps_ephemeris_iter != gps_ephemeris_map.end())
                                {
                                    /*!
                                     * \todo Place here the satellite CN0 (power level, or weight factor)
                                     */
                                    W.resize(valid_obs + 1, 1);
                                    W(valid_obs) = 1;

                                    // COMMON RX TIME PVT ALGORITHM MODIFICATION (Like RINEX files)
                                    // first estimate of transmit time
                                    double Tx_time = Rx_time - gnss_observables_iter->second.Pseudorange_m / GPS_C_m_s;

                                    // 2- compute the clock drift using the clock model (broadcast) for this SV, not including relativistic effect
                                    SV_clock_bias_s = gps_ephemeris_iter->second.sv_clock_drift(Tx_time);

                                    // 3- compute the current ECEF position for this SV using corrected TX time and obtain clock bias including relativistic effect
                                    TX_time_corrected_s = Tx_time - SV_clock_bias_s;
                                    //compute satellite position, clock bias + relativistic correction
                                    double dts = gps_ephemeris_iter->second.satellitePosition(TX_time_corrected_s);

                                    //store satellite positions in a matrix
                                    satpos.resize(3, valid_obs + 1);
                                    satpos(0, valid_obs) = gps_ephemeris_iter->second.d_satpos_X;
                                    satpos(1, valid_obs) = gps_ephemeris_iter->second.d_satpos_Y;
                                    satpos(2, valid_obs) = gps_ephemeris_iter->second.d_satpos_Z;

                                    // 4- fill the observations vector with the corrected pseudoranges
                                    // compute code bias: TGD for single frequency
                                    // See IS-GPS-200E section 20.3.3.3.3.2
                                    double sqrt_Gamma=GPS_L1_FREQ_HZ/GPS_L2_FREQ_HZ;
                                    double Gamma=sqrt_Gamma*sqrt_Gamma;
                                    double P1_P2=(1.0-Gamma)*(gps_ephemeris_iter->second.d_TGD* GPS_C_m_s);
                                    double Code_bias_m= P1_P2/(1.0-Gamma);
                                    obs.resize(valid_obs + 1, 1);
                                    obs(valid_obs) = gnss_observables_iter->second.Pseudorange_m + dts * GPS_C_m_s-Code_bias_m-d_rx_dt_s * GPS_C_m_s;
                                    d_visible_satellites_IDs[valid_obs] = gps_ephemeris_iter->second.i_satellite_PRN;
                                    d_visible_satellites_CN0_dB[valid_obs] = gnss_observables_iter->second.CN0_dB_hz;

                                    // SV ECEF DEBUG OUTPUT
                                    LOG(INFO) << "(new)ECEF GPS L1 CA satellite SV ID=" << gps_ephemeris_iter->second.i_satellite_PRN
                                               << " TX Time corrected="<<TX_time_corrected_s
                                               << " [m] X=" << gps_ephemeris_iter->second.d_satpos_X
                                               << " [m] Y=" << gps_ephemeris_iter->second.d_satpos_Y
                                               << " [m] Z=" << gps_ephemeris_iter->second.d_satpos_Z
                                               << " [m] PR_obs=" << obs(valid_obs) << " [m]";

                                    //*** debug
                                    if (valid_obs==0)
                                    {
                                    	gtime_t rx_time=gpst2time(adjgpsweek(gps_ephemeris_iter->second.i_GPS_week),Rx_time);
                                    	gtime_t tx_time=gpst2time(adjgpsweek(gps_ephemeris_iter->second.i_GPS_week),Tx_time);
                                    	printf("RINEX RX TIME: %s,%f, TX TIME: %s,%f\n\r",time_str(rx_time,3),rx_time.sec,time_str(tx_time,3),tx_time.sec);
                                    }
                                    std::flush(std::cout);
                                    gtime_t tx_time_corr=gpst2time(adjgpsweek(gps_ephemeris_iter->second.i_GPS_week),TX_time_corrected_s);
                                    printf("SAT TX TIME [%i]: %s,%f PR:%f dt:%f\n\r",valid_obs,time_str(tx_time_corr,3),tx_time_corr.sec, obs(valid_obs),dts);
                                    std::flush(std::cout);
                                    //*** end debug

                                    valid_obs++;
                                    // compute the UTC time for this SV (just to print the associated UTC timestamp)
                                    GPS_week = gps_ephemeris_iter->second.i_GPS_week;
                                }
                            else // the ephemeris are not available for this SV
                                {
                                    DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->first;
                                }
                        }
                    if(sig_.compare("2S") == 0)
                        {
                            gps_cnav_ephemeris_iter = gps_cnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                            if (gps_cnav_ephemeris_iter != gps_cnav_ephemeris_map.end())
                                {
                                    /*!
                                     * \todo Place here the satellite CN0 (power level, or weight factor)
                                     */
                                    W.resize(valid_obs + 1, 1);
                                    W(valid_obs) = 1;

                                    // COMMON RX TIME PVT ALGORITHM MODIFICATION (Like RINEX files)
                                    // first estimate of transmit time
                                    double Tx_time = Rx_time - gnss_observables_iter->second.Pseudorange_m / GPS_C_m_s;

                                    // 2- compute the clock drift using the clock model (broadcast) for this SV
                                    SV_clock_bias_s = gps_cnav_ephemeris_iter->second.sv_clock_drift(Tx_time);
                                    // 3- compute the current ECEF position for this SV using corrected TX time
                                    TX_time_corrected_s = Tx_time - SV_clock_bias_s;
                                    //std::cout<<"TX time["<<gps_cnav_ephemeris_iter->second.i_satellite_PRN<<"]="<<TX_time_corrected_s<<std::endl;
                                    double dtr = gps_cnav_ephemeris_iter->second.satellitePosition(TX_time_corrected_s);
                                    //std::cout<<"L2 Tx_time: "<<Tx_time<<" SV_clock_bias_s: "<<SV_clock_bias_s<<" dtr: "<<dtr<<std::endl;
                                    //store satellite positions in a matrix
                                    satpos.resize(3, valid_obs + 1);
                                    satpos(0, valid_obs) = gps_cnav_ephemeris_iter->second.d_satpos_X;
                                    satpos(1, valid_obs) = gps_cnav_ephemeris_iter->second.d_satpos_Y;
                                    satpos(2, valid_obs) = gps_cnav_ephemeris_iter->second.d_satpos_Z;

                                    // 4- fill the observations vector with the corrected observables
                                    obs.resize(valid_obs + 1, 1);
                                    obs(valid_obs) = gnss_observables_iter->second.Pseudorange_m + dtr * GPS_C_m_s - d_rx_dt_s * GPS_C_m_s;
                                    d_visible_satellites_IDs[valid_obs] = gps_cnav_ephemeris_iter->second.i_satellite_PRN;
                                    d_visible_satellites_CN0_dB[valid_obs] = gnss_observables_iter->second.CN0_dB_hz;

                                    GPS_week = gps_cnav_ephemeris_iter->second.i_GPS_week;
                                    GPS_week=GPS_week%1024; //Necessary due to the increase of WN bits in CNAV message (10 in GPS NAV and 13 in CNAV)

                                    // SV ECEF DEBUG OUTPUT
                                    LOG(INFO) << "(new)ECEF GPS L2M satellite SV ID=" << gps_cnav_ephemeris_iter->second.i_satellite_PRN
                                                << " TX Time corrected="<<TX_time_corrected_s
                                                << " X=" << gps_cnav_ephemeris_iter->second.d_satpos_X
                                               << " [m] Y=" << gps_cnav_ephemeris_iter->second.d_satpos_Y
                                               << " [m] Z=" << gps_cnav_ephemeris_iter->second.d_satpos_Z
                                               << " [m] PR_obs=" << obs(valid_obs) << " [m]";

                                    valid_obs++;
                                }
                            else // the ephemeris are not available for this SV
                                {
                                    DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->second.PRN;
                                }
                        }
                    break;
                }
            default :
                DLOG(INFO) << "Hybrid observables: Unknown GNSS";
                break;
            }
        }

    // ********************************************************************************
    // ****** SOLVE LEAST SQUARES******************************************************
    // ********************************************************************************
    d_valid_observations = valid_obs;
    LOG(INFO) << "HYBRID PVT: valid observations=" << valid_obs;

    if(valid_obs >= 4)
        {
            arma::vec rx_position_and_time;
            DLOG(INFO) << "satpos=" << satpos;
            DLOG(INFO) << "obs=" << obs;
            DLOG(INFO) << "W=" << W;

            try
            {
                    // check if this is the initial position computation
                    if (d_rx_dt_s == 0)
                        {
                            // execute Bancroft's algorithm to estimate initial receiver position and time
                            DLOG(INFO) << " Executing Bancroft algorithm...";
                            rx_position_and_time = bancroftPos(satpos.t(), obs);
                            d_rx_pos = rx_position_and_time.rows(0, 2); // save ECEF position for the next iteration
                            d_rx_dt_s = rx_position_and_time(3) / GPS_C_m_s; // save time for the next iteration [meters]->[seconds]
                        }

                    // Execute WLS using previous position as the initialization point
                    rx_position_and_time = leastSquarePos(satpos, obs, W);

                    d_rx_pos = rx_position_and_time.rows(0, 2); // save ECEF position for the next iteration
                    d_rx_dt_s += rx_position_and_time(3) / GPS_C_m_s; // accumulate the rx time error for the next iteration [meters]->[seconds]

                    DLOG(INFO) << "Hybrid Position at TOW=" << Rx_time << " in ECEF (X,Y,Z,t[meters]) = " << rx_position_and_time;
                    DLOG(INFO) << "Accumulated rx clock error=" << d_rx_dt_s << " clock error for this iteration=" << rx_position_and_time(3) / GPS_C_m_s << " [s]";

                    // Compute GST and Gregorian time
                    if( GST != 0.0)
                        {
                            utc = galileo_utc_model.GST_to_UTC_time(GST, Galileo_week_number);
                        }
                    else
                        {
                            utc = gps_utc_model.utc_time(TX_time_corrected_s, GPS_week) + secondsperweek * static_cast<double>(GPS_week);
                        }

                    // get time string Gregorian calendar
                    boost::posix_time::time_duration t = boost::posix_time::seconds(utc);
                    // 22 August 1999 00:00 last Galileo start GST epoch (ICD sec 5.1.2)
                    boost::posix_time::ptime p_time(boost::gregorian::date(1999, 8, 22), t);
                    d_position_UTC_time = p_time;

                    cart2geo(static_cast<double>(rx_position_and_time(0)), static_cast<double>(rx_position_and_time(1)), static_cast<double>(rx_position_and_time(2)), 4);

                    DLOG(INFO) << "Hybrid Position at " << boost::posix_time::to_simple_string(p_time)
                               << " is Lat = " << d_latitude_d << " [deg], Long = " << d_longitude_d
                               << " [deg], Height= " << d_height_m << " [m]" << " RX time offset= " << d_rx_dt_s << " [s]";

                    // ###### Compute DOPs ########
                    compute_DOP();

                    // ######## LOG FILE #########
                    if(d_flag_dump_enabled == true)
                        {
                            // MULTIPLEXED FILE RECORDING - Record results to file
                            try
                            {
                                    double tmp_double;
                                    //  PVT GPS time
                                    tmp_double = Rx_time;
                                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                                    // ECEF User Position East [m]
                                    tmp_double = rx_position_and_time(0);
                                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                                    // ECEF User Position North [m]
                                    tmp_double = rx_position_and_time(1);
                                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                                    // ECEF User Position Up [m]
                                    tmp_double = rx_position_and_time(2);
                                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                                    // User clock offset [s]
                                    tmp_double = rx_position_and_time(3);
                                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                                    // GEO user position Latitude [deg]
                                    tmp_double = d_latitude_d;
                                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                                    // GEO user position Longitude [deg]
                                    tmp_double = d_longitude_d;
                                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                                    // GEO user position Height [m]
                                    tmp_double = d_height_m;
                                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                            }
                            catch (const std::ifstream::failure& e)
                            {
                                    LOG(WARNING) << "Exception writing PVT LS dump file " << e.what();
                            }
                        }

                    // MOVING AVERAGE PVT
                    pos_averaging(flag_averaging);
            }
            catch(const std::exception & e)
            {
                    d_rx_dt_s = 0; //reset rx time estimation
                    LOG(WARNING) << "Problem with the solver, invalid solution!" << e.what();
                    LOG(WARNING) << "satpos=" << satpos;
                    LOG(WARNING) << "obs=" << obs;
                    LOG(WARNING) << "W=" << W;
                    b_valid_position = false;
            }
        }
    else
        {
            b_valid_position = false;
        }
    return b_valid_position;
}
