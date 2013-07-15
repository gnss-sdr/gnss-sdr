/*!
 * \file gps_iono.h
 * \brief  Interface of a GPS IONOSPHERIC MODEL storage
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_GALILEO_IONO_H_
#define GNSS_SDR_GALILEO_IONO_H_


/*!
 * \brief This class is a storage for the GALILEO IONOSPHERIC data as described in Galileo ICD
 *
 * See [Update ref]
 */

/*
 * ToDo: Rewrite the class to include all the parameters described in Galileo ICD (this is just a copy of GPS code!)
 */
class Galileo_Iono
{
private:

public:
    // valid flag
    bool valid;
    // Ionospheric parameters
    double d_alpha0;      //!< Coefficient 0 of a cubic equation representing the amplitude of the vertical delay [s]
    double d_alpha1;      //!< Coefficient 1 of a cubic equation representing the amplitude of the vertical delay [s/semi-circle]
    double d_alpha2;      //!< Coefficient 2 of a cubic equation representing the amplitude of the vertical delay [s(semi-circle)^2]
    double d_alpha3;      //!< Coefficient 3 of a cubic equation representing the amplitude of the vertical delay [s(semi-circle)^3]
    double d_beta0;       //!< Coefficient 0 of a cubic equation representing the period of the model [s]
    double d_beta1;       //!< Coefficient 1 of a cubic equation representing the period of the model [s/semi-circle]
    double d_beta2;       //!< Coefficient 2 of a cubic equation representing the period of the model [s(semi-circle)^2]
    double d_beta3;       //!< Coefficient 3 of a cubic equation representing the period of the model [s(semi-circle)^3]
    /*!
     * Default constructor
     */
    Galileo_Iono();
};

#endif
