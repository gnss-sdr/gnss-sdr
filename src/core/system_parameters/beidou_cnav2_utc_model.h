/*!
 * \file beidou_cnav2_utc_model.h
 * \brief  Interface of a BEIDOU CNAV2 UTC MODEL storage
 * \note Code added as part of GSoC 2018 program
 * \author Dong Kyeong Lee, 2018. dole7890(at)colorado.edu
 * \see <a href="http://m.beidou.gov.cn/xt/gfxz/201712/P020171226742357364174.pdf">BEIDOU ICD</a>
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


#ifndef GNSS_SDR_BEIDOU_CNAV2_UTC_MODEL_H_
#define GNSS_SDR_BEIDOU_CNAV2_UTC_MODEL_H_

#include <boost/assign.hpp>
#include <boost/serialization/nvp.hpp>
#include "beidou_cnav2_ephemeris.h"
#include "beidou_cnav2_almanac.h"

//!!! Check
/*!
 * \brief This class is a storage for the BEIDOU CNAV2 UTC MODEL data as described in BEIDOU ICD
 * \note Code added as part of GSoC 2018 program
 * \see <a href="http://m.beidou.gov.cn/xt/gfxz/201712/P020171226742357364174.pdf">BEIDOU ICD</a>
 */
class Beidou_Cnav2_Utc_Model
{
public:
    bool valid;

    // BDT-UTC Time Offset Parameters
    double A_0UTC;	//Bias coefficient of BDT time scale relative to UTC time scale [s]
	double A_1UTC;	//Drift coefficient of BDT time scale relative to UTC time scale [s/s]
	double A_2UTC;	//Drift rate coefficient of BDT time scale relative to UTC time scale [s/s^2]
	double dt_LS;	//Current of past leap second count [s]
	double t_ot;	//Reference time of week [s]
	double WN_ot;	//Reference week number [week]
	double WN_LSF;	//Leap second reference week number [week]
	double DN;		//Leap second reference day number [day]
	double dt_LSF;	//Current of future leap second count [s]


	// BDT_GNSS Time Offset Parameters
	double GNSS_ID;		//GNSS type identification [dimensionless]
	double WN_0BGTO;	//Reference week number [week]
	double t_0BGTO;		//Reference time of week [s]
	double A_0BGTO;		//Bias coefficient of BDT time scale relative to GNSS time scale [s]
	double A_1BGTO;		//Drift coefficient of BDT time scale relative to GNSS time scale [s/s]
	double A_2BGTO;		//Drift rate coefficient of BDT time scale relative to GNSS time scale [s/s^2]

    template <class Archive>
    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML serialization. Here is used to save the almanac data on disk file.
     */
    void serialize(Archive& archive, const unsigned int version)
    {
        using boost::serialization::make_nvp;
        if (version)
            {
            };
        archive& make_nvp("valid", valid);

        // BDT-UTC Time Offset Parameters
        archive& make_nvp("A_0UTC", A_0UTC);
        archive& make_nvp("A_1UTC", A_1UTC);
        archive& make_nvp("A_2UTC", A_2UTC);
        archive& make_nvp("dt_LS", dt_LS);
        archive& make_nvp("t_ot", t_ot);
        archive& make_nvp("WN_ot", WN_ot);
        archive& make_nvp("WN_LSF", WN_LSF);
        archive& make_nvp("DN", DN);
        archive& make_nvp("dt_LSF", dt_LSF);

        // BDT_GNSS Time Offset Parameters
        archive& make_nvp("GNSS_ID", GNSS_ID);
        archive& make_nvp("WN_0BGTO", WN_0BGTO);
        archive& make_nvp("t_0BGTO", t_0BGTO);
        archive& make_nvp("A_0BGTO", A_0BGTO);
        archive& make_nvp("A_1BGTO", A_1BGTO);
        archive& make_nvp("A_2BGTO", A_2BGTO);

    }

    double time_of_transmission(Beidou_Cnav2_Almanac const&alm, Beidou_Cnav2_Ephemeris const&eph, double t_sv);

    double beidt_to_gpst(Beidou_Cnav2_Ephemeris const&eph, double beidou_time);

    /*!
     * \brief Converts from BEIDOUT to GPST
     */
    double beidt_to_utc(Beidou_Cnav2_Ephemeris const&eph, double time_bds);


    /*!
     * \brief Converts from BEIDOUT to UTC
     * The t_E, WN_E, DN_E are the BDT estimated by the user
     */

    double utc_time(double beidou_time);

    /*!
     * Default constructor
     */

    Beidou_Cnav2_Utc_Model();

};

#endif