/*!
 * \file galileo_iono.h
 * \brief  Interface of a Galileo Ionospheric Model storage
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GALILEO_IONO_H_
#define GNSS_SDR_GALILEO_IONO_H_


/*!
 * \brief This class is a storage for the GALILEO IONOSPHERIC data as described in Galileo ICD paragraph 5.1.6
 *
 * See https://www.gsc-europa.eu/system/files/galileo_documents/Galileo_OS_SIS_ICD.pdf
 */
class Galileo_Iono
{
public:
    /*Ionospheric correction*/
    double ai0_5;  //!< Effective Ionisation Level 1st order parameter [sfu]
    double ai1_5;  //!< Effective Ionisation Level 2st order parameter [sfu/degree]
    double ai2_5;  //!< Effective Ionisation Level 3st order parameter [sfu/degree]

    /*Ionospheric disturbance flag*/
    bool Region1_flag_5;  //!<  Ionospheric Disturbance Flag for region 1
    bool Region2_flag_5;  //!< Ionospheric Disturbance Flag for region 2
    bool Region3_flag_5;  //!< Ionospheric Disturbance Flag for region 3
    bool Region4_flag_5;  //!< Ionospheric Disturbance Flag for region 4
    bool Region5_flag_5;  //!< Ionospheric Disturbance Flag for region 5

    /*from page 5 (UTC) to have a timestamp*/
    double TOW_5;  //!< UTC data reference Time of Week [s]
    double WN_5;   //!< UTC data reference Week number [week]

    /*!
     * Default constructor
     */
    Galileo_Iono();
};

#endif
