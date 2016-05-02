/*!
 * \file sbas_satellite_correction.h
 * \brief Interface of the SBAS satellite correction set based on SBAS RTKLIB functions
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

#ifndef GNSS_SDR_SBAS_SATELLITE_CORRECTION_H_
#define GNSS_SDR_SBAS_SATELLITE_CORRECTION_H_

#include "sbas_time.h"

struct Fast_Correction
{
    Sbas_Time d_tof; // for fast corrections the time of applicability (tof) is defined as the time when the corresponding message was send
    double d_prc;
    double d_rrc;
    double d_dt;
    int d_udre;         // UDRE
    int d_ai;
    int d_tlat;
};


struct Long_Term_Correction
{
    double d_trx;     //!< Time when message was received
    int i_tapp;       //!< Time of applicability (only valid if vel=1, equals the sent t0)
    int i_vel;        //!< Use velocity corrections if vel=1
    int d_iode;
    double d_dpos[3]; //!< position correction
    double d_dvel[3]; //!< velocity correction
    double d_daf0;    //!< clock offset correction
    double d_daf1;    //!< clock drift correction
};


/*!
 * \brief Valid long and fast term SBAS corrections for one SV
 */
class Sbas_Satellite_Correction
{
public:
    int d_prn;
    Fast_Correction d_fast_correction;
    Long_Term_Correction d_long_term_correction;
    void print(std::ostream &out);
    void print_fast_correction(std::ostream &out);
    void print_long_term_correction(std::ostream &out);
    int apply_fast(double sample_stamp, double &pr, double &var);
    int apply_long_term_sv_pos(double sample_stamp, double sv_pos[], double &var);
    int apply_long_term_sv_clk(double sample_stamp, double &dts, double &var);
    bool alarm();
private:
    /* debug trace functions -----------------------------------------------------*/
    void trace(int level, const char *format, ...);
    /* variance of fast correction (udre=UDRE+1) ---------------------------------*/
    double varfcorr(int udre);
    /* fast correction degradation -----------------------------------------------*/
    double degfcorr(int ai);
    /* long term correction ------------------------------------------------------*/
    int sbslongcorr(double time_stamp, double *drs, double *ddts);
    /* fast correction -----------------------------------------------------------*/
    int sbsfastcorr(double time_stamp, double *prc, double *var);
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
    int sbssatcorr(double time_stamp, double *rs, double *dts, double *var);
};


#endif /* GNSS_SDR_SBAS_SATELLITE_CORRECTION_H_ */
