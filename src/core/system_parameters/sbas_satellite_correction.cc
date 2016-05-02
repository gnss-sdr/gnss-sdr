/*!
 * \file sbas_satellite_correction.cc
 * \brief Implementation of the SBAS satellite correction set based on SBAS RTKLIB functions
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

#include "sbas_satellite_correction.h"
#include <stdarg.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <glog/logging.h>


#define EVENT 2 // logs important events which don't occur every update() call
#define FLOW 3  // logs the function calls of block processing functions

#define CLIGHT      299792458.0         /* speed of light (m/s) */
#define MAXSBSAGEF  30.0                /* max age of SBAS fast correction (s) */
#define MAXSBSAGEL  1800.0              /* max age of SBAS long term corr (s) */


void Sbas_Satellite_Correction::print(std::ostream &out)
{
    out << "<<S>> Sbas satellite corrections for PRN" << d_prn << ":" << std::endl;
    print_fast_correction(out);
    print_long_term_correction(out);
}


void Sbas_Satellite_Correction::print_fast_correction(std::ostream &out)
{
    Fast_Correction fcorr = d_fast_correction;
    out << "<<S>> Fast PRN" << d_prn << ":";
    if(fcorr.d_tof.is_related())
    {
        int gps_week;
        double gps_sec;
        fcorr.d_tof.get_gps_time(gps_week, gps_sec);
        out << "  d_t0=(week=" <<  gps_week << ",sec=" << gps_sec << ")";
    }
    else
    {
        out << "  d_t0=" <<  fcorr.d_tof.get_time_stamp();
    }
    out << "  d_prc=" <<  fcorr.d_prc;
    out << "  d_rrc=" <<  fcorr.d_rrc;
    out << "  d_dt=" <<  fcorr.d_dt;
    out << "  d_udre=" <<  fcorr.d_udre;
    out << "  d_ai=" <<  fcorr.d_ai;
    out << "  d_tlat=" <<  fcorr.d_tlat;
}


void Sbas_Satellite_Correction::print_long_term_correction(std::ostream &out)
{
    Long_Term_Correction lcorr = d_long_term_correction;
    out << "<<S>> Long PRN" << d_prn << ":";
    out << "  d_trx=" <<  lcorr.d_trx;
    out << "  i_tapp=" <<  lcorr.i_tapp;
    out << "  i_vel=" <<  lcorr.i_vel;
    double *p = lcorr.d_dpos;
    out << "  d_dpos=(x=" <<  p[0] << ", y=" << p[1] << ", z=" << p[2] << ")" ;
    double *v = lcorr.d_dvel;
    out << "  d_dvel=(x=" <<  v[0] << ", y=" << v[1] << ", z=" << v[2] << ")" ;
    out << "  d_daf0=" <<  lcorr.d_daf0;
    out << "  d_daf1=" <<  lcorr.d_daf1;
}


int Sbas_Satellite_Correction::apply_fast(double sample_stamp, double &pr, double &var)
{
    int result;
    double prc = 0; // pseudo range correction
    result = sbsfastcorr(sample_stamp, &prc, &var);
    pr += prc;
    VLOG(FLOW) << "<<S>> fast correction applied: sample_stamp=" << sample_stamp << " prc=" << prc << " corr. pr=" << pr;
    return result;
}



int Sbas_Satellite_Correction::apply_long_term_sv_pos(double sample_stamp, double sv_pos[], double &var)
{
    int result;
    double drs[3] = {0};
    double ddts = 0;
    result = sbslongcorr(sample_stamp, drs, &ddts);
    for (int i = 0; i < 3; i++) sv_pos[i] += drs[i];
    VLOG(FLOW) << "<<S>> long term sv pos correction applied: sample_stamp=" << sample_stamp << " drs=(x=" << drs[0] << " y=" << drs[1] << " z=" << drs[2] << ")";
    return result;
}



int Sbas_Satellite_Correction::apply_long_term_sv_clk(double sample_stamp, double &dts, double &var)
{
    int result;
    double drs[3] = {0};
    double ddts = 0;
    result = sbslongcorr(sample_stamp, drs, &ddts);
    dts += ddts;
    VLOG(FLOW) << "<<S>> long term sv clock correction correction applied: sample_stamp=" << sample_stamp << " ddts=" << ddts;
    return result;
}


bool Sbas_Satellite_Correction::alarm()
{
    return this->d_fast_correction.d_udre == 16;
}



/* debug trace function -----------------------------------------------------*/
void Sbas_Satellite_Correction::trace(int level, const char *format, ...)
{
    va_list ap;
    char str[1000];
    va_start(ap,format);
    vsprintf(str,format,ap);
    va_end(ap);
    VLOG(FLOW) << "<<S>> " << std::string(str);
}


/* variance of fast correction (udre=UDRE+1) ---------------------------------*/
double Sbas_Satellite_Correction::varfcorr(int udre)
{
    const double var[14] = {
        0.052, 0.0924, 0.1444, 0.283, 0.4678, 0.8315, 1.2992, 1.8709, 2.5465, 3.326,
        5.1968, 20.7870, 230.9661, 2078.695
    };
    return 0 < udre && udre <= 14 ? var[udre - 1] : 0.0;
}


/* fast correction degradation -----------------------------------------------*/
double Sbas_Satellite_Correction::degfcorr(int ai)
{
    const double degf[16] = {
        0.00000, 0.00005, 0.00009, 0.00012, 0.00015, 0.00020, 0.00030, 0.00045,
        0.00060, 0.00090, 0.00150, 0.00210, 0.00270, 0.00330, 0.00460, 0.00580
    };
    return 0 < ai && ai <= 15 ? degf[ai] : 0.0058;
}



/* long term correction ------------------------------------------------------*/
int Sbas_Satellite_Correction::sbslongcorr(double time_stamp, double *drs, double *ddts)
{
    double t = 0.0;
    int i;
    Long_Term_Correction lcorr = d_long_term_correction;
    trace(3, "sbslongcorr: prn=%2d", this->d_prn);
    // if (p->sat!=sat||p->lcorr.t0.time==0) continue;
    // compute time of applicability
    if(d_long_term_correction.i_vel == 1)
        {
            // time of applicability is the one sent, i.e., tapp
            // TODO: adapt for vel==1 case
            // t = tow-d_long_term_correction.i_tapp;
            // vel=1 -> time of applicability is sent in message, needs to be corrected for rollover which can not be done here, since the absolute gps time is unknown. see IS-GPS-200G pdf page 116 for correction
            /* t = (int)getbitu(msg->msg, p + 90, 13)*16 - (int)msg->tow%86400;
            if (t <= -43200) t += 86400;
            else if (t >  43200) t -= 86400;
            sbssat->sat[n-1].lcorr.t0 = gpst2time(msg->week, msg->tow + t);*/
        }
    else
        {
            // time of applicability is time of reception
            t = time_stamp - lcorr.d_trx; // should not have any impact if vel==0 since d_dvel and d_daf1 are zero, is only used to check outdating
        }
    //t=time_stamp-lcorr.d_t0;
    if (fabs(t) > MAXSBSAGEL)
        {
            trace(2,"sbas long-term correction expired: sat=%2d time_stamp=%5.0f", d_prn, time_stamp);
            return 0;
        }
    // sv position correction
    for (i=0; i<3; i++) drs[i] = lcorr.d_dpos[i] + lcorr.d_dvel[i]*t;
    // sv clock correction correction
    *ddts = lcorr.d_daf0 + lcorr.d_daf1*t;
    trace(5, "sbslongcorr: sat=%2d drs=%7.2f%7.2f%7.2f ddts=%7.2f", d_prn, drs[0], drs[1], drs[2], *ddts * CLIGHT);
    return 1;
    /* if sbas satellite without correction, no correction applied */
    //if (satsys(sat,NULL)==SYS_SBS) return 1;
    //trace(2,"no sbas long-term correction: %s sat=%2d\n",time_str(time,0),sat);
    //return 0;
}




/* fast correction -----------------------------------------------------------*/
int Sbas_Satellite_Correction::sbsfastcorr(double time_stamp, double *prc, double *var)
#define RRCENA
{
    double t;
    Fast_Correction fcorr = d_fast_correction;
    trace(3, "sbsfastcorr: sat=%2d", this->d_prn);
    //if (p->fcorr.t0.time==0) break; // time==0is only true if t0 hasn't been initialised -> it checks if the correction is valid
    t = (time_stamp - fcorr.d_tof.get_time_stamp()) + fcorr.d_tlat; // delta t between now and tof
    /* expire age of correction? */
    if (fabs(t) > MAXSBSAGEF)
        {
            trace(2, "no sbas fast correction (expired): time_stamp=%f prn=%2d", time_stamp, d_prn);
            return 0;
        }
    /* UDRE==14 (not monitored)? */
    else if(fcorr.d_udre == 15)
        {
            trace(2,"no sbas fast correction (not monitored): time_stamp=%f prn=%2d", time_stamp, d_prn);
            return 0;
        }
    else if(fcorr.d_udre == 16)
        {
            trace(2,"SV is marked as unhealthy: time_stamp=%f prn=%2d", time_stamp, d_prn);
            return 0;
        }
    *prc = fcorr.d_prc;
#ifdef RRCENA
    if (fcorr.d_ai > 0 && fabs(t) <= 8.0*fcorr.d_dt)
        {
            *prc += fcorr.d_rrc*t;
        }
#endif
    *var = varfcorr(fcorr.d_udre) + degfcorr(fcorr.d_ai) * t * t / 2.0;
    trace(5, "sbsfastcorr: sat=%3d prc=%7.2f sig=%7.2f t=%5.0f", d_prn, *prc, sqrt(*var), t);
    return 1;
}



/* sbas satellite ephemeris and clock correction -------------------------------
* correct satellite position and clock bias with sbas satellite corrections
* args   : long time_stamp     I   reception time stamp
*          double *rs       IO  sat position and corrected {x,y,z} (ecef) (m)
*          double *dts      IO  sat clock bias and corrected (s)
*          double *var      O   sat position and clock variance (m^2)
* return : status (1:ok,0:no correction)
* notes  : before calling the function, sbas satellite correction parameters
*          in navigation data (nav->sbssat) must be set by callig
*          sbsupdatecorr().
*          satellite clock correction include long-term correction and fast
*          correction.
*          sbas clock correction is usually based on L1C/A code. TGD or DCB has
*          to be considered for other codes
*-----------------------------------------------------------------------------*/
int Sbas_Satellite_Correction::sbssatcorr(double time_stamp, double *rs, double *dts, double *var)
{
    double drs[3] = {0}, dclk = 0.0, prc = 0.0;
    int i;
    trace(3,"sbssatcorr : sat=%2d",d_prn);
    /* sbas long term corrections */
    if (!sbslongcorr(time_stamp, drs, &dclk))
        {
            return 0;
        }
    /* sbas fast corrections */
    if (!sbsfastcorr(time_stamp, &prc, var))
        {
            return 0;
        }
    for (i = 0; i < 3; i++) rs[i] += drs[i];
    dts[0] += dclk + prc/CLIGHT;
    trace(5, "sbssatcorr: sat=%2d drs=%6.3f %6.3f %6.3f dclk=%.3f %.3f var=%.3f",
            d_prn, drs[0], drs[1], drs[2], dclk,prc/CLIGHT, *var);
    return 1;
}

