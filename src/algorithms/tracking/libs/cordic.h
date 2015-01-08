/*!
 * \file cordic.h
 * \brief Interface of the CORDIC (COordinate Rotation DIgital Computer) algorithm.
 * \author Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
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




#ifndef GNSS_SDR_CORDIC_H_
#define GNSS_SDR_CORDIC_H_

typedef struct tagCORDIC_TABLE {
    double K;
    double phase_rads;
} CORDIC_TABLE;


/*!
 * \brief Implementation of the CORDIC (COordinate Rotation DIgital Computer) algorithm.
 * This implementation is NOT OPTIMIZED, only for demonstration purposes
 */
class Cordic
{
public:
    /*!
     * \brief construct the CORDIC table which will be of size "largest_k + 1".
     */
    Cordic(int largest_k);

    /*!
     * \brief Frees the CORDIC table's memory
     */
    ~Cordic();

    /*!
     * \brief Calculates the magnitude and phase of "I + jQ".  p_phase_rads is in radians
     */
    void cordic_get_mag_phase(double I, double Q, double &p_mag, double &p_phase_rads);
    /* calculate the magnitude and phase of "I + jQ".  phase is in radians */

    /*!
     * \brief Calculates the cosine and sine of the desired phase in radians
     */
    void cordic_get_cos_sin(double desired_phase_rads, double &p_cos, double &p_sin);

private:

    CORDIC_TABLE *mp_cordic_table;
    int m_max_L;
    double m_mag_scale;

};

#endif
