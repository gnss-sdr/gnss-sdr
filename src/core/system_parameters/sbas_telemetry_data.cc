/*!
 * \file sbas_telemetry_data.cc
 * \brief Implementation of the SBAS telemetry parser based on SBAS RTKLIB functions
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

#include <cmath>
#include <cstring>
#include <iostream>
#include <stdarg.h>
#include <stdio.h>
#include <glog/logging.h>
#include "sbas_telemetry_data.h"
#include "sbas_ionospheric_correction.h"
#include "sbas_satellite_correction.h"
#include "sbas_ephemeris.h"


// logging levels
#define EVENT 2 // logs important events which don't occur every update() call
#define FLOW 3  // logs the function calls of block processing functions
#define DETAIL 4



Sbas_Telemetry_Data::Sbas_Telemetry_Data()
{
    fp_trace = nullptr; // file pointer of trace
    level_trace = 0;    // level of trace
    tick_trace = 0;     // tick time at traceopen (ms)

    d_nav.sbssat.iodp = -1; // make sure that in any case iodp is not equal to the received one
    prn_mask_changed();     // invalidate all satellite corrections

    for(size_t band = 0; band < sizeof(d_nav.sbsion)/sizeof(sbsion_t); band++)
        {
            d_nav.sbsion[band].iodi = -1; // make sure that in any case iodi is not equal to the received one
            igp_mask_changed(band);
        }
}

Sbas_Telemetry_Data::~Sbas_Telemetry_Data()
{

}


int Sbas_Telemetry_Data::update(Sbas_Raw_Msg sbas_raw_msg)
{
    VLOG(FLOW) << "<<T>> Sbas_Telemetry_Data.update():";
    int parsing_result;

    // if GPS time from MT12 is known (automatically handled by relate()):
    // express the rx time in terms of GPS time
    sbas_raw_msg.relate(mt12_time_ref);

    int mt = sbas_raw_msg.get_msg_type();
    // update internal state
    if(mt == 12) parsing_result = decode_mt12(sbas_raw_msg);
    //else if(mt == 9) parsing_result = parse_mt9(sbas_raw_msg);
    else
        {
            // use RTKLIB to parse the message -> updates d_nav structure
            sbsmsg_t sbas_raw_msg_rtklib;
            std::vector<unsigned char> msg_bytes = sbas_raw_msg.get_msg();
            // cast raw message to RTKLIB raw message struct
            sbas_raw_msg_rtklib.prn = sbas_raw_msg.get_prn();
            //sbas_raw_msg_rtklib.tow = sbas_raw_msg.get_tow();
            //sbas_raw_msg_rtklib.week = sbas_raw_msg.get_week();
            sbas_raw_msg_rtklib.sample_stamp = sbas_raw_msg.get_sample_stamp();
            for (std::vector<unsigned char>::const_iterator it = msg_bytes.begin(); it != msg_bytes.end() - 3; ++it)
                {
                    int i = it - msg_bytes.begin();
                    sbas_raw_msg_rtklib.msg[i] = *it;
                }
            parsing_result = sbsupdatecorr(&sbas_raw_msg_rtklib, &d_nav);
            VLOG(FLOW) << "<<T>> RTKLIB parsing result: " << parsing_result;
        }

    // update gnss-sdr correction data sets from RTKLIB d_nav structure, emit SBAS data into queues
    switch (parsing_result)
    {
    case  -1:  VLOG(FLOW) << "message parsing problem for MT" << sbas_raw_msg.get_msg_type(); break;
    case  2:
    case  3:
    case  4:
    case  5:
    case  6:
    case  7:
    case 24:
    case 25: updated_satellite_corrections(); break;
    case 18: break; // new iono band mask received -> dont update iono corrections because delays are not
    case 26: received_iono_correction(); break;
    case  9: /*updated_sbas_ephemeris(sbas_raw_msg);*/ break;

    default: break;
    }

    // send it to raw message queue
    //TODO: update to new GNURadio msg queue
    //if(raw_msg_queue != nullptr) raw_msg_queue->push(sbas_raw_msg);
    return parsing_result;
}


unsigned int getbitu(const unsigned char *buff, int pos, int len);



int getbits(const unsigned char *buff, int pos, int len);



int Sbas_Telemetry_Data::decode_mt12(Sbas_Raw_Msg sbas_raw_msg)
{
    const double rx_delay = 38000.0/300000.0; // estimated sbas signal geosat to ground signal travel time
    unsigned char * msg = sbas_raw_msg.get_msg().data();
    uint32_t gps_tow = getbitu(msg, 121, 20);
    uint32_t gps_week = getbitu(msg, 141, 10) + 1024; // consider last gps time week overflow
    double gps_tow_rx = double(gps_tow) + rx_delay;
    mt12_time_ref = Sbas_Time_Relation(sbas_raw_msg.get_sample_stamp(), gps_week, gps_tow_rx);
    VLOG(FLOW) << "<<T>> extracted GPS time from MT12: gps_tow=" << gps_tow << " gps_week=" << gps_week;
    return 12;
}




void Sbas_Telemetry_Data::updated_sbas_ephemeris(Sbas_Raw_Msg msg)
{
    VLOG(FLOW) << "<<T>> updated_sbas_ephemeris():" << std::endl;
    Sbas_Ephemeris seph;
    int satidx = msg.get_prn() - MINPRNSBS;
    seph_t seph_rtklib = d_nav.seph[satidx];
    // copy data
    seph.i_prn = msg.get_prn();
    seph.i_t0 = seph_rtklib.t0;
    seph.d_tof = seph_rtklib.tof;
    seph.i_sv_ura = seph_rtklib.sva;
    seph.b_sv_do_not_use = seph_rtklib.svh;
    memcpy(seph.d_pos, seph_rtklib.pos, sizeof(seph.d_pos));
    memcpy(seph.d_vel, seph_rtklib.vel, sizeof(seph.d_vel));
    memcpy(seph.d_acc, seph_rtklib.acc, sizeof(seph.d_acc));
    seph.d_af0 = seph_rtklib.af0;
    seph.d_af1 = seph_rtklib.af1;
    // print ephemeris for debugging purposes
    std::stringstream ss;
    seph.print(ss);
    VLOG(FLOW) << ss.str();

    //todo_Update to new GNURadio msg queue
    //if(ephemeris_queue != nullptr) ephemeris_queue->push(seph);
}



void Sbas_Telemetry_Data::received_iono_correction()
{
    VLOG(FLOW) << "<<T>> received_iono_correction():";
    std::stringstream ss;

    Sbas_Ionosphere_Correction iono_corr;
    for (size_t i_band = 0; i_band < sizeof(d_nav.sbsion)/sizeof(sbsion_t); i_band++)
        {
            ss << "<<T>> band=" << i_band
                    << " nigp=" << d_nav.sbsion[i_band].nigp << std::endl;
            ss << "<<T>> -> valid igps:";
            Igp_Band igp_band;
            for (int i_igp = 0; i_igp < d_nav.sbsion[i_band].nigp; i_igp++)
                {
                    if(d_nav.sbsion[i_band].igp[i_igp].valid)
                        {
                            // consider only valid IGPs, i.e, such ones which got updated at least once since instantiating sbas_telemtry_data
                            ss << " " << i_igp;
                            Igp igp;
                            igp.t0 = d_nav.sbsion[i_band].igp[i_igp].t0;
                            igp.d_latitude = d_nav.sbsion[i_band].igp[i_igp].lat;
                            igp.d_longitude = d_nav.sbsion[i_band].igp[i_igp].lon;
                            igp.d_give = d_nav.sbsion[i_band].igp[i_igp].give;
                            igp.d_delay = d_nav.sbsion[i_band].igp[i_igp].delay;
                            igp_band.d_igps.push_back(igp);
                        }
                }
            ss << std::endl;
            iono_corr.d_bands.push_back(igp_band);
        }
    VLOG(DETAIL) << ss.str();
    ss.str("");
    iono_corr.print(ss);
    VLOG(EVENT) << ss.str();

    // send to SBAS ionospheric correction queue
    //todo_Update to new GNURadio msg queue
    //if(iono_queue != nullptr) iono_queue->push(iono_corr);
}


// helper for comparing two POD structures with undefined padding between members
// not guaranteed to work always properly -> don't use it for critical tasks
template <class Struct>
inline bool are_equal(const Struct &s1, const Struct &s2)
{
    const size_t struct_size = sizeof(Struct);
    bool is_equal = true;
    bool is_equal_manual = true;
    std::stringstream ss;

    Struct *s1_;
    Struct *s2_;

    // reserve zero initialised memory
    s1_ = (Struct*) calloc (1, struct_size);
    s2_ = (Struct*) calloc (1, struct_size);

    // use assignment constructor which doesn't copy paddings
    *s1_ = s1;
    *s2_ = s2;

    // compare struct memory byte-wise
    is_equal_manual = true;
    ss.str();
    ss << "\n<<cmp>> compare objects of size=" << sizeof(Struct) << " (memcmp says is_equal=" << is_equal << ") :" << std::endl;
    for (size_t i = 0; i < sizeof(Struct); ++i)
        {
            char byte1 = ((char *)s1_)[i];
            char byte2 = ((char *)s2_)[i];
            if(byte1 != byte2) is_equal_manual = false;
            ss << "<<cmp>> s1=" << std::hex << std::setw(4) << std::setfill('0');
            ss << (short)byte1;
            ss << " s2=" << std::hex << std::setw(4) << std::setfill('0');
            ss << (short)byte2;
            ss << " equal=" << is_equal_manual;
            ss << std::endl;
        }

    free(s1_);
    free(s2_);

    return is_equal_manual;
}



void Sbas_Telemetry_Data::updated_satellite_corrections()
{
    VLOG(FLOW) << "<<T>> updated_satellite_corrections():";
    // for each satellite in the RTKLIB structure
    for (int i_sat = 0; i_sat < d_nav.sbssat.nsat; i_sat++)
        {
            std::stringstream ss;
            ss << "<<T>> sat=" << d_nav.sbssat.sat[i_sat].sat
               << " fastcorr.valid=" << d_nav.sbssat.sat[i_sat].fcorr.valid
               << " lcorr.valid=" << d_nav.sbssat.sat[i_sat].lcorr.valid;

            if(is_rtklib_sat_correction_valid(i_sat)) // check if ever updated by a received message
                {
                    int prn = d_nav.sbssat.sat[i_sat].sat;

                    // get fast correction from RTKLIB structure
                    sbsfcorr_t fcorr_rtklib = d_nav.sbssat.sat[i_sat].fcorr;
                    Fast_Correction fcorr;
                    fcorr.d_tof = Sbas_Time(fcorr_rtklib.t0, mt12_time_ref);
                    fcorr.d_prc = fcorr_rtklib.prc;
                    fcorr.d_rrc = fcorr_rtklib.rrc;
                    fcorr.d_dt = fcorr_rtklib.dt;
                    fcorr.d_udre = fcorr_rtklib.udre;    // UDRE
                    fcorr.d_ai = fcorr_rtklib.ai;
                    fcorr.d_tlat = d_nav.sbssat.tlat;

                    // get long term correction from RTKLIB structure
                    sbslcorr_t lcorr_rtklib = d_nav.sbssat.sat[i_sat].lcorr;
                    Long_Term_Correction lcorr;
                    lcorr.d_trx = lcorr_rtklib.trx;
                    lcorr.i_tapp = lcorr_rtklib.tapp;
                    lcorr.i_vel = lcorr_rtklib.vel;
                    lcorr.d_iode = lcorr_rtklib.iode;
                    memcpy(lcorr.d_dpos, lcorr_rtklib.dpos, sizeof(lcorr.d_dpos));
                    memcpy(lcorr.d_dvel, lcorr_rtklib.dvel, sizeof(lcorr.d_dvel));
                    lcorr.d_daf0 = lcorr_rtklib.daf0;
                    lcorr.d_daf1= lcorr_rtklib.daf1;

                    bool fast_correction_updated = false;
                    bool long_term_correction_updated = false;

                    // check if fast corrections got updated
                    std::map<int, Fast_Correction>::iterator it_old_fcorr = emitted_fast_corrections.find(prn);
                    if(it_old_fcorr == emitted_fast_corrections.end() || !are_equal < Fast_Correction>(fcorr, it_old_fcorr->second ))
                        {
                            // got updated
                            ss << " fast_correction_updated=" << true;
                            //ss << " not_found=" << (it_old_fcorr == emitted_fast_corrections.end());
                            //ss << " not_equal=" << (!are_equal<Fast_Correction>(fcorr, it_old_fcorr->second ));
                            fast_correction_updated = true;
                            emitted_fast_corrections[prn] = fcorr;
                        }

                    // check if long term corrections got updated
                    std::map<int, Long_Term_Correction>::iterator it_old_lcorr = emitted_long_term_corrections.find(prn);
                    if(it_old_lcorr == emitted_long_term_corrections.end() || !are_equal < Long_Term_Correction>(lcorr, it_old_lcorr->second ))
                        {
                            // got updated
                            ss << " long_term_correction_updated=" << true;
                            //ss << " not_found=" << (it_old_lcorr == emitted_long_term_corrections.end());
                            //ss << " not_equal=" << (!are_equal<Long_Term_Correction>(lcorr, it_old_lcorr->second ));
                            long_term_correction_updated = true;
                            emitted_long_term_corrections[prn] = lcorr;
                        }

                    Sbas_Satellite_Correction sbas_satelite_correction;
                    sbas_satelite_correction.d_prn = prn;
                    sbas_satelite_correction.d_fast_correction = fcorr;
                    sbas_satelite_correction.d_long_term_correction = lcorr;

                    if(fast_correction_updated)
                        {
                            ss << std::endl;
                            sbas_satelite_correction.print_fast_correction(ss << " ");
                        }

                    if(long_term_correction_updated)
                        {
                            ss << std::endl;
                            sbas_satelite_correction.print_long_term_correction(ss << " ");
                        }

                    if(fast_correction_updated || long_term_correction_updated)
                        {
                            //todo_Update to new GNURadio msg queue
                            //if(sat_corr_queue != nullptr) sat_corr_queue->push(sbas_satelite_correction);
                        }
                }
            VLOG(FLOW) << ss.str(); ss.str("");
        }
}



const double Sbas_Telemetry_Data::gpst0[] = {1980, 1, 6, 0, 0, 0}; /* gps time reference */

/* debug trace function -----------------------------------------------------*/
void Sbas_Telemetry_Data::trace(int level, const char *format, ...)
{
    va_list ap;
    char str[1000];
    va_start(ap, format);
    vsprintf(str, format, ap);
    va_end(ap);
    VLOG(FLOW) << "<<T>> " << std::string(str);
}



/* satellite system+prn/slot number to satellite number ------------------------
 * convert satellite system+prn/slot number to satellite number
 * args   : int    sys       I   satellite system (SYS_GPS,SYS_GLO,...)
 *          int    prn       I   satellite prn/slot number
 * return : satellite number (0:error)
 *-----------------------------------------------------------------------------*/
int Sbas_Telemetry_Data::satno(int sys, int prn)
{
    if (prn <= 0) return 0;
    switch (sys)
    {
    case SYS_GPS:
        if (prn < MINPRNGPS || MAXPRNGPS < prn) return 0;
        return prn - MINPRNGPS + 1;
    case SYS_GLO:
        if (prn < MINPRNGLO || MAXPRNGLO < prn) return 0;
        return NSATGPS + prn - MINPRNGLO + 1;
    case SYS_GAL:
        if (prn < MINPRNGAL || MAXPRNGAL < prn) return 0;
        return NSATGPS + NSATGLO + prn - MINPRNGAL + 1;
    case SYS_QZS:
        if (prn < MINPRNQZS || MAXPRNQZS < prn) return 0;
        return NSATGPS + NSATGLO + NSATGAL + prn - MINPRNQZS + 1;
    case SYS_CMP:
        if (prn < MINPRNCMP || MAXPRNCMP < prn) return 0;
        return NSATGPS + NSATGLO + NSATGAL + NSATQZS + prn - MINPRNCMP + 1;
    case SYS_SBS:
        if (prn < MINPRNSBS || MAXPRNSBS < prn) return 0;
        return NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + prn - MINPRNSBS + 1;
    }
    return 0;
}

/*!
 * \brief Extracts unsigned/signed bits from byte data
 * params : unsigned char *buff I byte data
 *          int    pos    I      bit position from start of data (bits)
 *          int    len    I      bit length (bits) (len<=32)
 * return : extracted unsigned/signed bits
 */
unsigned int Sbas_Telemetry_Data::getbitu(const unsigned char *buff, int pos, int len)
{
    unsigned int bits = 0;
    int i;
    for (i = pos; i < pos + len; i++) bits = (bits << 1) + ((buff[i/8] >> (7 - i % 8)) & 1u);
    return bits;
}



int Sbas_Telemetry_Data::getbits(const unsigned char *buff, int pos, int len)
{
    unsigned int bits = getbitu(buff,pos,len);
    if (len <= 0 || 32 <= len || !(bits & (1u << (len - 1)))) return (int)bits;
    return (int)(bits|(~0u << len)); /* extend sign */
}



/* convert calendar day/time to time -------------------------------------------
 * convert calendar day/time to gtime_t struct
 * args   : double *ep       I   day/time {year,month,day,hour,min,sec}
 * return : gtime_t struct
 * notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
 *-----------------------------------------------------------------------------*/
Sbas_Telemetry_Data::gtime_t Sbas_Telemetry_Data::epoch2time(const double *ep)
{
    const int doy[] = {1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
    gtime_t time = gtime_t();
    int days, sec, year = (int)ep[0], mon = (int)ep[1], day = (int)ep[2];

    if (year < 1970 || 2099 < year || mon < 1 || 12 < mon) return time;

    /* leap year if year%4==0 in 1901-2099 */
    days = (year - 1970)*365 + (year - 1969)/4 + doy[mon - 1] + day - 2 + (year % 4 == 0 && mon >= 3 ? 1 : 0);
    sec = (int)floor(ep[5]);
    time.time = (time_t)days*86400 + (int)ep[3]*3600 + (int)ep[4]*60 + sec;
    time.sec = ep[5] - sec;
    return time;
}




/* time difference -------------------------------------------------------------
 * difference between gtime_t structs
 * args   : gtime_t t1,t2    I   gtime_t structs
 * return : time difference (t1-t2) (s)
 *-----------------------------------------------------------------------------*/
double Sbas_Telemetry_Data::timediff(gtime_t t1, gtime_t t2)
{
    return difftime(t1.time, t2.time) + t1.sec - t2.sec;
}



/* gps time to time ------------------------------------------------------------
 * convert week and tow in gps time to gtime_t struct
 * args   : int    week      I   week number in gps time
 *          double sec       I   time of week in gps time (s)
 * return : gtime_t struct
 *-----------------------------------------------------------------------------*/
Sbas_Telemetry_Data::gtime_t Sbas_Telemetry_Data::gpst2time(int week, double sec)
{
    gtime_t t = epoch2time(gpst0);
    if (sec < -1E9 || 1E9 < sec) sec = 0.0;
    t.time += 86400*7*week + (int)sec;
    t.sec = sec - (int)sec;
    return t;
}



/* sbas igp definition -------------------------------------------------------*/
const short
Sbas_Telemetry_Data::x1[] = {-75,-65,-55,-50,-45,-40,-35,-30,-25,-20,-15,-10,- 5,  0,  5, 10, 15, 20,
                            25, 30, 35, 40, 45, 50, 55, 65, 75, 85},
Sbas_Telemetry_Data::x2[] = {-55,-50,-45,-40,-35,-30,-25,-20,-15,-10, -5,  0,  5, 10, 15, 20, 25, 30,
                            35, 40, 45, 50, 55},
Sbas_Telemetry_Data::x3[] = {-75,-65,-55,-50,-45,-40,-35,-30,-25,-20,-15,-10,- 5,  0,  5, 10, 15, 20,
                            25, 30, 35, 40, 45, 50, 55, 65, 75},
Sbas_Telemetry_Data::x4[] = {-85,-75,-65,-55,-50,-45,-40,-35,-30,-25,-20,-15,-10,- 5,  0,  5, 10, 15,
                            20, 25, 30, 35, 40, 45, 50, 55, 65, 75},
Sbas_Telemetry_Data::x5[] = {-180,-175,-170,-165,-160,-155,-150,-145,-140,-135,-130,-125,-120,-115,
                           -110,-105,-100,- 95,- 90,- 85,- 80,- 75,- 70,- 65,- 60,- 55,- 50,- 45,
                           - 40,- 35,- 30,- 25,- 20,- 15,- 10,-  5,   0,   5,  10,  15,  20,  25,
                             30,  35,  40,  45,  50,  55,  60,  65,  70,  75,  80,  85,  90,  95,
                            100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165,
                            170, 175},
Sbas_Telemetry_Data::x6[] = {-180,-170,-160,-150,-140,-130,-120,-110,-100,- 90,- 80,- 70,- 60,- 50,
                            -40, -30, -20, -10,   0,  10,  20,  30,  40,  50,  60,  70,  80,  90,
                            100, 110, 120, 130, 140, 150, 160, 170},
Sbas_Telemetry_Data::x7[] = {-180,-150,-120,- 90,- 60,- 30,   0,  30,  60,  90, 120, 150},
Sbas_Telemetry_Data::x8[] = {-170,-140,-110,- 80,- 50,- 20,  10,  40,  70, 100, 130, 160};

const Sbas_Telemetry_Data::sbsigpband_t Sbas_Telemetry_Data::igpband1[9][8] = { /* band 0-8 */
        {{-180,x1,  1, 28},{-175,x2, 29, 51},{-170,x3, 52, 78},{-165,x2, 79,101},
         {-160,x3,102,128},{-155,x2,129,151},{-150,x3,152,178},{-145,x2,179,201}},
        {{-140,x4,  1, 28},{-135,x2, 29, 51},{-130,x3, 52, 78},{-125,x2, 79,101},
         {-120,x3,102,128},{-115,x2,129,151},{-110,x3,152,178},{-105,x2,179,201}},
        {{-100,x3,  1, 27},{- 95,x2, 28, 50},{- 90,x1, 51, 78},{- 85,x2, 79,101},
         {- 80,x3,102,128},{- 75,x2,129,151},{- 70,x3,152,178},{- 65,x2,179,201}},
        {{- 60,x3,  1, 27},{- 55,x2, 28, 50},{- 50,x4, 51, 78},{- 45,x2, 79,101},
         {- 40,x3,102,128},{- 35,x2,129,151},{- 30,x3,152,178},{- 25,x2,179,201}},
        {{- 20,x3,  1, 27},{- 15,x2, 28, 50},{- 10,x3, 51, 77},{-  5,x2, 78,100},
         {   0,x1,101,128},{   5,x2,129,151},{  10,x3,152,178},{  15,x2,179,201}},
        {{  20,x3,  1, 27},{  25,x2, 28, 50},{  30,x3, 51, 77},{  35,x2, 78,100},
         {  40,x4,101,128},{  45,x2,129,151},{  50,x3,152,178},{  55,x2,179,201}},
        {{  60,x3,  1, 27},{  65,x2, 28, 50},{  70,x3, 51, 77},{  75,x2, 78,100},
         {  80,x3,101,127},{  85,x2,128,150},{  90,x1,151,178},{  95,x2,179,201}},
        {{ 100,x3,  1, 27},{ 105,x2, 28, 50},{ 110,x3, 51, 77},{ 115,x2, 78,100},
         { 120,x3,101,127},{ 125,x2,128,150},{ 130,x4,151,178},{ 135,x2,179,201}},
        {{ 140,x3,  1, 27},{ 145,x2, 28, 50},{ 150,x3, 51, 77},{ 155,x2, 78,100},
         { 160,x3,101,127},{ 165,x2,128,150},{ 170,x3,151,177},{ 175,x2,178,200}}
};



const Sbas_Telemetry_Data::sbsigpband_t Sbas_Telemetry_Data::igpband2[2][5] = { /* band 9-10 */
        {{  60,x5,  1, 72},{  65,x6, 73,108},{  70,x6,109,144},{  75,x6,145,180},
         {  85,x7,181,192}},
        {{- 60,x5,  1, 72},{- 65,x6, 73,108},{- 70,x6,109,144},{- 75,x6,145,180},
         {- 85,x8,181,192}}
};


/* decode type 1: prn masks --------------------------------------------------*/
int Sbas_Telemetry_Data::decode_sbstype1(const sbsmsg_t *msg, sbssat_t *sbssat)
{
    int i, n, sat;
    // see figure A-6: i corresponds to bit number (and for the GPS satellites is identically to the PRN), n to the PRN mask number

    trace(4, "decode_sbstype1:");

    for (i = 1, n = 0; i <= 210 && n < MAXSAT; i++)
        {
            if (getbitu(msg->msg, 13 + i, 1))
                {
                    if      (i <= 37) sat = satno(SYS_GPS, i);         /*   0 - 37: gps */
                    else if (i <= 61) sat = satno(SYS_GLO, i - 37);    /*  38 - 61: glonass */
                    else if (i <= 119) sat = 0;                        /*  62 - 119: future gnss */
                    else if (i <= 138) sat = satno(SYS_SBS, i);        /* 120 - 138: geo/waas */
                    else if (i <= 182) sat = 0;                        /* 139 - 182: reserved */
                    else if (i <= 192) sat = satno(SYS_SBS, i + 10);   /* 183 - 192: qzss ref [2] */
                    else if (i <= 202) sat = satno(SYS_QZS, i);        /* 193 - 202: qzss ref [2] */
                    else sat = 0;                                      /* 203 -   : reserved */
                    sbssat->sat[n++].sat = sat;
                }
        }
    // TODO consider the use of the old prn mask in the transition phase such that old data sets still can be used
    int new_iodp = getbitu(msg->msg, 224, 2);
    if (sbssat->iodp != new_iodp) prn_mask_changed(); // invalidate all satellite corrections
    sbssat->iodp = new_iodp;
    sbssat->nsat = n;

    trace(5, "decode_sbstype1: nprn=%d iodp=%d", n, sbssat->iodp);
    return 1;
}


/* decode type 2-5,0: fast corrections ---------------------------------------*/
int Sbas_Telemetry_Data::decode_sbstype2(const sbsmsg_t *msg, sbssat_t *sbssat)
{
    int i, j, iodf, type, udrei;
    double prc, dt;
    double t0_past;

    trace(4,"decode_sbstype2:");

    if (sbssat->iodp != (int)getbitu(msg->msg, 16, 2)) return 0;

    type = getbitu(msg->msg, 8, 6);
    iodf = getbitu(msg->msg, 14, 2);

    for (i=0; i<13; i++)
        {
            if ((j = 13*((type == 0 ? 2 : type) - 2) + i) >= sbssat->nsat) break;
            udrei = getbitu(msg->msg, 174 + 4*i, 4);
            t0_past = sbssat->sat[j].fcorr.t0;
            prc = sbssat->sat[j].fcorr.prc;
            sbssat->sat[j].fcorr.t0 = msg->sample_stamp;
            sbssat->sat[j].fcorr.prc = getbits(msg->msg, 18 + i*12, 12)*0.125f;
            sbssat->sat[j].fcorr.udre = udrei + 1;
            dt = sbssat->sat[j].fcorr.t0 - t0_past;
            if (!sbssat->sat[j].fcorr.valid || dt <= 0.0 || 18.0 < dt || sbssat->sat[j].fcorr.ai == 0)
                {
                    sbssat->sat[j].fcorr.rrc = 0.0;
                    sbssat->sat[j].fcorr.dt = 0.0;
                }
            else
                {
                    sbssat->sat[j].fcorr.rrc = (sbssat->sat[j].fcorr.prc - prc)/dt;
                    sbssat->sat[j].fcorr.dt = dt;
                }
            sbssat->sat[j].fcorr.valid = true;
            sbssat->sat[j].fcorr.iodf = iodf;
        }
    trace(5, "decode_sbstype2: type=%d iodf=%d", type, iodf);
    return 1;
}




/* decode type 6: integrity info ---------------------------------------------*/
int Sbas_Telemetry_Data::decode_sbstype6(const sbsmsg_t *msg, sbssat_t *sbssat)
{
    int i, iodf[4], udrei;

    trace(4, "decode_sbstype6:");

    if(sbssat->iodp < 0) return 0;

    for (i=0; i<4; i++)
        {
            iodf[i] = getbitu(msg->msg, 14 + i*2, 2);
        }
    for (i=0; i < sbssat->nsat && i < MAXSAT; i++)
        {
            if (!sbssat->sat[i].fcorr.valid || sbssat->sat[i].fcorr.iodf != iodf[i/13]) continue;
            udrei = getbitu(msg->msg, 22 + i*4, 4);
            sbssat->sat[i].fcorr.udre = udrei + 1;
        }
    trace(5, "decode_sbstype6: iodf=%d %d %d %d", iodf[0], iodf[1], iodf[2], iodf[3]);
    return 1;
}



/* decode type 7: fast correction degradation factor -------------------------*/
int Sbas_Telemetry_Data::decode_sbstype7(const sbsmsg_t *msg, sbssat_t *sbssat)
{
    int i;
    trace(4,"decode_sbstype7");
    if (sbssat->iodp != (int)getbitu(msg->msg, 18, 2)) return 0;
    sbssat->tlat = getbitu(msg->msg, 14, 4);
    for (i=0; i < sbssat->nsat && i < MAXSAT; i++)
        {
            sbssat->sat[i].fcorr.ai = getbitu(msg->msg, 22 + i*4, 4);
        }
    return 1;
}



/* decode type 9: geo navigation message -------------------------------------*/
int Sbas_Telemetry_Data::decode_sbstype9(const sbsmsg_t *msg, nav_t *nav)
{
    seph_t seph;
    int i;
    int sat;

    trace(4,"decode_sbstype9:");

    if (!(sat = satno(SYS_SBS, msg->prn)))
        {
            trace(2, "invalid prn in sbas type 9: prn=%3d", msg->prn);
            return 0;
        }
    /*t=(int)getbitu(msg->msg,22,13)*16-(int)msg->tow%86400;
    if      (t<=-43200) t+=86400;
    else if (t>  43200) t-=86400;*/
    //seph.t0 =gpst2time(msg->week,msg->tow+t);
    seph.sat = sat;
    seph.t0 = getbitu(msg->msg, 22, 13)*16;
    seph.tof = msg->sample_stamp;
    seph.sva = getbitu(msg->msg, 35, 4);
    seph.svh = seph.sva == 15 ? 1 : 0; /* unhealthy if ura==15 */

    seph.pos[0] = getbits(msg->msg, 39, 30)*0.08;
    seph.pos[1] = getbits(msg->msg, 69, 30)*0.08;
    seph.pos[2] = getbits(msg->msg, 99, 25)*0.4;
    seph.vel[0] = getbits(msg->msg, 124, 17)*0.000625;
    seph.vel[1] = getbits(msg->msg, 141, 17)*0.000625;
    seph.vel[2] = getbits(msg->msg, 158, 18)*0.004;
    seph.acc[0] = getbits(msg->msg, 176, 10)*0.0000125;
    seph.acc[1] = getbits(msg->msg, 186, 10)*0.0000125;
    seph.acc[2] = getbits(msg->msg, 196, 10)*0.0000625;

    seph.af0 = getbits(msg->msg, 206, 12)*P2_31;
    seph.af1 = getbits(msg->msg, 218, 8)*P2_39/2.0;

    i = msg->prn-MINPRNSBS;
    if (std::abs(nav->seph[i].t0 - seph.t0) < 1E-3)
        { /* not change */
            VLOG(FLOW) << "<<T>> no change in ephemeris -> won't parse";
            return 0;
        }
    nav->seph[NSATSBS + i] = nav->seph[i]; /* previous */
    nav->seph[i] = seph;                   /* current */

    trace(5, "decode_sbstype9: prn=%d", msg->prn);
    return 1;
}



/* decode type 18: ionospheric grid point masks ------------------------------*/
int Sbas_Telemetry_Data::decode_sbstype18(const sbsmsg_t *msg, sbsion_t *sbsion)
{
    const sbsigpband_t *p;
    int i, j, n, m, band = getbitu(msg->msg, 18, 4);

    trace(4, "decode_sbstype18:");

    if      (0 <= band && band <= 8) {p = igpband1[band]; m = 8;}
    else if (9 <= band && band <= 10) {p = igpband2[band - 9]; m = 5;}
    else return 0;

    short iodi_new = (short)getbitu(msg->msg, 22, 2);
    if(sbsion[band].iodi != iodi_new)
        {
            // IGP mask changed -> invalidate all IGPs in this band
            igp_mask_changed(band);
        }
    sbsion[band].iodi = iodi_new;

    for (i=1, n=0; i <= 201; i++)
        {
            if (!getbitu(msg->msg, 23 + i, 1)) continue;
            for (j = 0; j < m; j++)
                {
                    if (i < p[j].bits || p[j].bite < i) continue;
                    sbsion[band].igp[n].lat = band <= 8 ? p[j].y[i - p[j].bits] : p[j].x;
                    sbsion[band].igp[n++].lon = band <= 8 ? p[j].x : p[j].y[i - p[j].bits];
                    break;
                }
        }
    sbsion[band].nigp = n;

    trace(5, "decode_sbstype18: band=%d nigp=%d", band, n);
    return 1;
}




/* decode half long term correction (vel code=0) -----------------------------*/
int Sbas_Telemetry_Data::decode_longcorr0(const sbsmsg_t *msg, int p, sbssat_t *sbssat)
{
    int i, n = getbitu(msg->msg, p, 6);

    trace(4, "decode_longcorr0:");

    if (n == 0 || n > MAXSAT) return 0;

    sbssat->sat[n - 1].lcorr.iode = getbitu(msg->msg, p + 6, 8);

    for (i = 0; i < 3; i++)
        {
            sbssat->sat[n - 1].lcorr.dpos[i] = getbits(msg->msg, p + 14 + 9*i, 9)*0.125;
            sbssat->sat[n - 1].lcorr.dvel[i] = 0.0;
        }
    sbssat->sat[n - 1].lcorr.daf0 = getbits(msg->msg, p + 41, 10)*P2_31;
    sbssat->sat[n - 1].lcorr.daf1 = 0.0;
    //sbssat->sat[n-1].lcorr.t0=gpst2time(msg->week,msg->tow);
    sbssat->sat[n - 1].lcorr.trx = msg->sample_stamp; // vel=0 -> time of applicability is reception time
    sbssat->sat[n - 1].lcorr.tapp = 0;
    sbssat->sat[n - 1].lcorr.valid = true;

    trace(5, "decode_longcorr0:sat=%2d", sbssat->sat[n - 1].sat);
    return 1;
}



/* decode half long term correction (vel code=1) -----------------------------*/
int Sbas_Telemetry_Data::decode_longcorr1(const sbsmsg_t *msg, int p, sbssat_t *sbssat)
{
    int i;
    int n = getbitu(msg->msg, p, 6);

    trace(4,"decode_longcorr1:");

    if (n == 0 || n > MAXSAT) return 0;

    sbssat->sat[n - 1].lcorr.iode = getbitu(msg->msg, p + 6, 8);

    for (i=0; i<3; i++)
        {
            sbssat->sat[n - 1].lcorr.dpos[i] = getbits(msg->msg, p + 14 + i*11, 11)*0.125;
            sbssat->sat[n - 1].lcorr.dvel[i] = getbits(msg->msg, p + 58 + i*8, 8)*P2_11;
        }
    sbssat->sat[n - 1].lcorr.daf0 = getbits(msg->msg, p + 47, 11)*P2_31;
    sbssat->sat[n - 1].lcorr.daf1 = getbits(msg->msg, p + 82, 8)*P2_39;
    // vel=1 -> time of applicability is sent in message, needs to be corrected for rollover which can not be done here, since the absolute gps time is unknown. see IS-GPS-200G pdf page 116 for correction
    /*t=(int)getbitu(msg->msg,p+90,13)*16-(int)msg->tow%86400;
    if      (t<=-43200) t+=86400;
    else if (t>  43200) t-=86400;
    sbssat->sat[n-1].lcorr.t0=gpst2time(msg->week,msg->tow+t);*/
    sbssat->sat[n - 1].lcorr.trx = msg->sample_stamp;
    sbssat->sat[n - 1].lcorr.tapp = (int)getbitu(msg->msg, p + 90, 13)*16;
    sbssat->sat[n - 1].lcorr.vel = 1;
    sbssat->sat[n - 1].lcorr.valid = true;
    trace(5, "decode_longcorr1: sat=%2d", sbssat->sat[n - 1].sat);
    return 1;
}




/* decode half long term correction ------------------------------------------*/
int Sbas_Telemetry_Data::decode_longcorrh(const sbsmsg_t *msg, int p, sbssat_t *sbssat)
{
    trace(4,"decode_longcorrh:");

    if (getbitu(msg->msg, p, 1) == 0 )
        {
            /* vel code=0 */
            if (sbssat->iodp == (int)getbitu(msg->msg, p + 103, 2))
                {
                    return decode_longcorr0(msg, p + 1, sbssat) && decode_longcorr0(msg, p + 52, sbssat);
                }
        }
    else if (sbssat->iodp == (int)getbitu(msg->msg, p + 104, 2))
        {
            return decode_longcorr1(msg, p + 1, sbssat);
        }
    return 0;
}




/* decode type 24: mixed fast/long term correction ---------------------------*/
int Sbas_Telemetry_Data::decode_sbstype24(const sbsmsg_t *msg, sbssat_t *sbssat)
{
    int i, j, iodf, blk, udre;

    trace(4, "decode_sbstype24:");

    if (sbssat->iodp != (int)getbitu(msg->msg, 110, 2)) return 0; /* check IODP */

    blk = getbitu(msg->msg, 112, 2);
    iodf =getbitu(msg->msg, 114, 2);

    for (i=0; i<6; i++)
        {
            if ((j = 13*blk+i) >= sbssat->nsat) break;
            udre = getbitu(msg->msg, 86 + 4*i, 4);

            //sbssat->sat[j].fcorr.t0  =gpst2time(msg->week,msg->tow);
            sbssat->sat[j].fcorr.t0   = msg->sample_stamp;
            sbssat->sat[j].fcorr.prc  = getbits(msg->msg, 14 + i*12, 12)*0.125f;
            sbssat->sat[j].fcorr.udre = udre + 1;
            sbssat->sat[j].fcorr.iodf = iodf;
        }
    return decode_longcorrh(msg, 120, sbssat);
}




/* decode type 25: long term satellite error correction ----------------------*/
int Sbas_Telemetry_Data::decode_sbstype25(const sbsmsg_t *msg, sbssat_t *sbssat)
{
    trace(4,"decode_sbstype25:");
    return decode_longcorrh(msg, 14, sbssat) && decode_longcorrh(msg, 120, sbssat);
}




/* decode type 26: ionospheric delay corrections -----------------------------*/
int Sbas_Telemetry_Data::decode_sbstype26(const sbsmsg_t *msg, sbsion_t *sbsion)
{
    int i, j, block, delay, give, band = getbitu(msg->msg, 14, 4);

    trace(4, "decode_sbstype26:");

    if (band > MAXBAND || sbsion[band].iodi != (int)getbitu(msg->msg, 217, 2)) return 0;

    block = getbitu(msg->msg, 18, 4);

    for (i = 0; i < 15; i++)
        {
            if ((j = block*15 + i) >= sbsion[band].nigp) continue;
            give = getbitu(msg->msg, 2 + i*13 + 9, 4);
            delay = getbitu(msg->msg, 22 + i*13, 9);
            //sbsion[band].igp[j].t0=gpst2time(msg->week,msg->tow);
            sbsion[band].igp[j].t0 = msg->sample_stamp;
            sbsion[band].igp[j].valid = true;
            sbsion[band].igp[j].delay = delay == 0x1FF ? 0.0f : delay*0.125f;
            sbsion[band].igp[j].give = give;

            if(sbsion[band].igp[j].give > 15) sbsion[band].igp[j].give = 15; // give is not higher than 15, but to be sure
        }
    trace(5, "decode_sbstype26: band=%d block=%d", band, block);
    return 1;
}




/* update sbas corrections -----------------------------------------------------
 * update sbas correction parameters in navigation data with a sbas message
 * args   : sbsmg_t  *msg    I   sbas message
 *          nav_t    *nav    IO  navigation data
 * return : message type (-1: error or not supported type)
 * notes  : nav->seph must point to seph[NSATSBS*2] (array of seph_t)
 *               seph[prn-MINPRNSBS+1]          : sat prn current epehmeris
 *               seph[prn-MINPRNSBS+1+MAXPRNSBS]: sat prn previous epehmeris
 *-----------------------------------------------------------------------------*/
int Sbas_Telemetry_Data::sbsupdatecorr(const sbsmsg_t *msg, nav_t *nav)
{
    int type = getbitu(msg->msg, 8, 6), stat = -1;

    trace(3,"sbsupdatecorr: type=%d",type);

    //if (msg->week==0) return -1;

    switch (type)
    {
    case  0: stat = decode_sbstype2(msg, &nav->sbssat); break;
    case  1: stat = decode_sbstype1(msg, &nav->sbssat); break;
    case  2:
    case  3:
    case  4:
    case  5: stat = decode_sbstype2(msg, &nav->sbssat); break;
    case  6: stat = decode_sbstype6(msg, &nav->sbssat); break;
    case  7: stat = decode_sbstype7(msg, &nav->sbssat); break;
    case  9: stat = decode_sbstype9(msg, nav);          break;
    case 10: trace(2, "unsupported sbas message: type=%d", type); break;
    case 12: trace(2, "unsupported sbas message: type=%d", type); break;
    case 17: trace(2, "unsupported sbas message: type=%d", type); break;
    case 18: stat = decode_sbstype18(msg, nav ->sbsion); break;
    case 24: stat = decode_sbstype24(msg, &nav->sbssat); break;
    case 25: stat = decode_sbstype25(msg, &nav->sbssat); break;
    case 26: stat = decode_sbstype26(msg, nav ->sbsion); break;
    case 63: break; /* null message */

    default: trace(2, "Unsupported SBAS message: type=%d", type); break;
    }
    return stat ? type : -1;
}


void Sbas_Telemetry_Data::prn_mask_changed()
{
    d_nav.sbssat.tlat = -1;
    // for each satellite in the RTKLIB structure
    for (int i_sat = 0; i_sat < d_nav.sbssat.nsat; i_sat++)
        {
            d_nav.sbssat.sat[i_sat].fcorr.valid = false;
            d_nav.sbssat.sat[i_sat].lcorr.valid = false;
        }
}


bool Sbas_Telemetry_Data::is_rtklib_sat_correction_valid(int sat)
{
    return d_nav.sbssat.tlat > -1
            && d_nav.sbssat.sat[sat].fcorr.valid
            && d_nav.sbssat.sat[sat].lcorr.valid;
}


void Sbas_Telemetry_Data::igp_mask_changed(int band)
{
    for(int i_igp = 0; i_igp < d_nav.sbsion[band].nigp; i_igp++)
        d_nav.sbsion[band].igp[i_igp].valid = false;
}
