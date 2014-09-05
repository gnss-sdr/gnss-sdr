/*!
 * \file galileo_almanac.h
 * \brief  Interface of a Galileo ALMANAC storage
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2013  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

#ifndef GNSS_SDR_GALILEO_ALMANAC_H_
#define GNSS_SDR_GALILEO_ALMANAC_H_


/*!
 * \brief This class is a storage for the GALILEO ALMANAC data as described in GALILEO ICD
 *
 * See http://ec.europa.eu/enterprise/policies/satnav/galileo/files/galileo-os-sis-icd-issue1-revision1_en.pdf paragraph 5.1.10
 */
class Galileo_Almanac
{
public:
    /*Word type 7: Almanac for SVID1 (1/2), almanac reference time and almanac reference week number*/
    int IOD_a_7;
    double WN_a_7;
    double t0a_7;
    int SVID1_7;
    double DELTA_A_7;
    double e_7;
    double omega_7;
    double delta_i_7;
    double Omega0_7;
    double Omega_dot_7;
    double M0_7;

    /*Word type 8: Almanac for SVID1 (2/2) and SVID2 (1/2)*/
    int IOD_a_8;
    double af0_8;
    double af1_8;
    double E5b_HS_8;
    double E1B_HS_8;
    double E5a_HS_8;
    int SVID2_8;
    double DELTA_A_8;
    double e_8;
    double omega_8;
    double delta_i_8;
    double Omega0_8;
    double Omega_dot_8;

    /*Word type 9: Almanac for SVID2 (2/2) and SVID3 (1/2)*/
    int IOD_a_9;
    double WN_a_9;
    double t0a_9;
    double M0_9;
    double af0_9;
    double af1_9;
    double E5b_HS_9;
    double E1B_HS_9;
    double E5a_HS_9;
    int SVID3_9;
    double DELTA_A_9;
    double e_9;
    double omega_9;
    double delta_i_9;

    /*Word type 10: Almanac for SVID3 (2/2)*/
    int IOD_a_10;
    double Omega0_10;
    double Omega_dot_10;
    double M0_10;
    double af0_10;
    double af1_10;
    double E5b_HS_10;
    double E1B_HS_10;
    double E5a_HS_10;

    Galileo_Almanac();  //!< Default constructor
};

#endif
