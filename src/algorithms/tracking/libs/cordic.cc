/*!
 * \file cordic.cc
 * \brief Implementation of the CORDIC (COordinate Rotation DIgital Computer) algorithm.
 * This implementation is NOT OPTIMIZED, only for demonstration purposes
 * \author Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
 * This is a modified implementation of the one found at
 * http://www.dspguru.com/dsp/faqs/cordic
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

#include <stdlib.h>
//#include <math.h>
#include <cmath>
#include "cordic.h"



const double HALF_PI = 3.1415926535898 / 2;
const int INVALID_K = -1;



Cordic::Cordic(int max_L)
{
    double K, dummy;
    int L;
    //mp_cordic_table = (CORDIC_TABLE *) calloc(max_L + 1, sizeof(CORDIC_TABLE));
    mp_cordic_table = (CORDIC_TABLE *) malloc((max_L + 1) * sizeof(CORDIC_TABLE));
    if (!mp_cordic_table)
        {
             /* failed to calloc table */
        }

    K = 1.0;
    for (L = 0; L <= max_L; L++)
        {
            mp_cordic_table[L].K = K;
            mp_cordic_table[L].phase_rads = (double) atan(K);
            K *= 0.5;
        }

    m_max_L = max_L;

    /* get m_mag_scale by getting the cordic magnitude with m_mag_scale = 1.0 */
    m_mag_scale = 1.0;
    Cordic::cordic_get_mag_phase(1.0, 0.0, m_mag_scale, dummy);
    m_mag_scale = 1.0 / m_mag_scale;
}




Cordic::~Cordic ()
{
    free(mp_cordic_table);
    m_max_L = INVALID_K;
}






void Cordic::cordic_get_mag_phase(double I, double Q, double &p_mag, double &p_phase_rads)
{
    int L;
    double tmp_I, K, phase_rads, acc_phase_rads;

    if (I < 0)
        {
            /* rotate by an initial +/- 90 degrees */
            tmp_I = I;
            if (Q > 0.0)
                {
                    I = Q;           /* subtract 90 degrees */
                    Q = -tmp_I;
                    acc_phase_rads = -HALF_PI;
                }
            else
                {
                    I = -Q;          /* add 90 degrees */
                    Q = tmp_I;
                    acc_phase_rads = HALF_PI;
                }
        }
    else
        {
            acc_phase_rads = 0.0;
        }

    /* rotate using "1 + jK" factors */
    for (L = 0; L <= m_max_L; L++)
        {
            K = mp_cordic_table[L].K;
            phase_rads = mp_cordic_table[L].phase_rads;
            tmp_I = I;
            if (Q >= 0.0)
                {
                    /* phase is positive: do negative rotation */
                    I += Q * K;
                    Q -= tmp_I * K;
                    acc_phase_rads -= phase_rads;
                }
            else
                {
                    /* phase is negative: do positive rotation */
                    I -= Q * K;
                    Q += tmp_I * K;
                    acc_phase_rads += phase_rads;
                }
        }

    p_phase_rads = -acc_phase_rads;
    p_mag = I * m_mag_scale;
}




void Cordic::cordic_get_cos_sin(double desired_phase_rads, double &p_cos, double &p_sin)
{
    double I, Q, tmp_I;
    double acc_phase_rads, phase_rads, K;
    int L;

    /* start with +90, -90, or 0 degrees */
    if (desired_phase_rads > HALF_PI)
        {
            I = 0.0;
            Q = 1.0;
            acc_phase_rads = HALF_PI;
        }
    else if (desired_phase_rads < -HALF_PI)
        {
            I = 0.0;
            Q = -1.0;
            acc_phase_rads = -HALF_PI;
        }
    else
        {
            I = 1.0;
            Q = 0.0;
            acc_phase_rads = 0.0;
        }

    /* rotate using "1 + jK" factors */
    for (L = 0; L <= m_max_L; L++)
        {
            K = mp_cordic_table[L].K;
            phase_rads = mp_cordic_table[L].phase_rads;
            tmp_I = I;
            if (desired_phase_rads - acc_phase_rads < 0.0)
                {
                    /* do negative rotation */
                    I += Q * K;
                    Q -= tmp_I * K;
                    acc_phase_rads -= phase_rads;
                }
            else
                {
                    /* do positive rotation */
                    I -= Q * K;
                    Q += tmp_I * K;
                    acc_phase_rads += phase_rads;
                }
        }

    p_cos = I * m_mag_scale;
    p_sin = Q * m_mag_scale;
}
