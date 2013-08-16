/*!
 * \file gps_utc_model.h
 * \brief  Interface of a GPS UTC MODEL storage
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 *
 *
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


#ifndef GNSS_SDR_GALILEO_UTC_MODEL_H_
#define GNSS_SDR_GALILEO_UTC_MODEL_H_

#include "Galileo_E1.h"


/*!
 * \brief This class is a storage for the GALILEO UTC MODEL data as described in Galileo ICD
 * http://ec.europa.eu/enterprise/policies/satnav/galileo/files/galileo-os-sis-icd-issue1-revision1_en.pdf
 * paragraph 5.1.7
 */

class Galileo_Utc_Model
{
public:

	//bool valid;
	/*Word type 6: GST-UTC conversion parameters*/
	double A0_6;
	double A1_6;
	double Delta_tLS_6;
	double t0t_6;
	double WNot_6;
	double WN_LSF_6;
	double DN_6;
	double Delta_tLSF_6;
	//double TOW_6;

	/*GST*/
	//double WN_5; //Week number
	//double TOW_5; //Time of Week

	//this is from gps
	/*bool valid;
    // UTC parameters
    double d_A1;          //!< 1st order term of a model that relates GPS and UTC time (ref. 20.3.3.5.2.4 IS-GPS-200E) [s/s]
    double d_A0;          //!< Constant of a model that relates GPS and UTC time (ref. 20.3.3.5.2.4 IS-GPS-200E) [s]
    double d_t_OT;        //!< Reference time for UTC data (reference 20.3.4.5 and 20.3.3.5.2.4 IS-GPS-200E) [s]
    int i_WN_T;           //!< UTC reference week number [weeks]
    double d_DeltaT_LS;   //!< delta time due to leap seconds [s]. Number of leap seconds since 6-Jan-1980 as transmitted by the GPS almanac.
    int i_WN_LSF;         //!< Week number at the end of which the leap second becomes effective [weeks]
    int i_DN;             //!< Day number (DN) at the end of which the leap second becomes effective [days]
    double d_DeltaT_LSF;  //!< Scheduled future or recent past (relative to NAV message upload) value of the delta time due to leap seconds [s]
*/


	double GST_to_UTC_time(double t_e, int WN);
     /*!
     * Default constructor
     */

    Galileo_Utc_Model();

};

#endif
