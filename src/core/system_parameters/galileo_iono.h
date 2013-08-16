/*!
 * \file gps_iono.h
 * \brief  Interface of a GPS IONOSPHERIC MODEL storage
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


#ifndef GNSS_SDR_GALILEO_IONO_H_
#define GNSS_SDR_GALILEO_IONO_H_


/*!
 * \brief This class is a storage for the GALILEO IONOSPHERIC data as described in Galileo ICD paragraph 5.1.6
 *
 * See http://ec.europa.eu/enterprise/policies/satnav/galileo/files/galileo-os-sis-icd-issue1-revision1_en.pdf
 */
class Galileo_Iono
{
private:

public:

	// valid flag
    bool valid;

    /*Ionospheric correction*/
    /*Az*/
    double ai0_5;		//Effective Ionisation Level 1st order parameter [sfu]
    double ai1_5;		//Effective Ionisation Level 2st order parameter [sfu/degree]
    double ai2_5;		//Effective Ionisation Level 3st order parameter [sfu/degree]

    /*Ionospheric disturbance flag*/
    bool Region1_flag_5;	// Ionospheric Disturbance Flag for region 1
    bool Region2_flag_5;	// Ionospheric Disturbance Flag for region 2
    bool Region3_flag_5;	// Ionospheric Disturbance Flag for region 3
    bool Region4_flag_5;	// Ionospheric Disturbance Flag for region 4
    bool Region5_flag_5;	// Ionospheric Disturbance Flag for region 5

    // Ionospheric parameters GPS
    /*double d_alpha0;      //!< Coefficient 0 of a cubic equation representing the amplitude of the vertical delay [s]
    double d_alpha1;      //!< Coefficient 1 of a cubic equation representing the amplitude of the vertical delay [s/semi-circle]
    double d_alpha2;      //!< Coefficient 2 of a cubic equation representing the amplitude of the vertical delay [s(semi-circle)^2]
    double d_alpha3;      //!< Coefficient 3 of a cubic equation representing the amplitude of the vertical delay [s(semi-circle)^3]
    double d_beta0;       //!< Coefficient 0 of a cubic equation representing the period of the model [s]
    double d_beta1;       //!< Coefficient 1 of a cubic equation representing the period of the model [s/semi-circle]
    double d_beta2;       //!< Coefficient 2 of a cubic equation representing the period of the model [s(semi-circle)^2]
    double d_beta3;       //!< Coefficient 3 of a cubic equation representing the period of the model [s(semi-circle)^3]*/
    /*!
     * Default constructor
     */
    Galileo_Iono();
};

#endif
