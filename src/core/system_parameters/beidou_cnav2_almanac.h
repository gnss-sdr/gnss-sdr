/*!
 * \file beidou_cnav2_almanac.h
 * \brief  Interface of a beidou cnav2 ALMANAC storage
 * \note Code added as part of GSoC 2018 program
 * \author Dong Kyeong Lee, 2018. dole7890(at)colorado.edu
 * \see <a href="http://m.beidou.gov.cn/xt/gfxz/201712/P020171226742357364174.pdf">beidou ICD</a>
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


#ifndef GNSS_SDR_BEIDOU_CNAV2_ALMANAC_H_
#define GNSS_SDR_BEIDOU_CNAV2_ALMANAC_H_

#include <boost/serialization/nvp.hpp>

/*!
 * \brief This class is a storage for the beidou SV ALMANAC data as described BEIDOU ICD
 * \note Code added as part of GSoC 2018 program
 * \see <a href="http://m.beidou.gov.cn/xt/gfxz/201712/P020171226742357364174.pdf">beidou ICD</a>
 */
class Beidou_Cnav2_Almanac
{
public:
	// Midi Almanac Parameters

	unsigned int i_satellite_PRN;				//!< PRN number of the corresponding almanac data [dimnsionless]
	double SatType;				//!< Satellite orbit type [dimnsionless]
	int i_BDS_week;				//!< Alamanc reference week number [week]
	double t_oa;				//!< Almanac reference time [s]
	double e;					//!< Eccentricity [dimnsionless]
	double delta_i;				//!< Correction of inclination angle relative to reference value at reference time [pi]
	double sqrt_A;				//!< Square root of semi-major axis [m^1/2]
	double Omega_0;				//!< Longitude of ascending node of orbital plane at weekly epoch [pi]
	double Omega_dot;			//!< Rate of right ascension [pi/s]
	double omega;				//!< Argument of perigee [pi]
	double M_0;					//!< Mean anomaly at reference time [pi]
	double a_f0;				//!< Satellite clock time bias correction coefficient [s]
	double a_f1;				//!< Satellite clock time drift correction coefficient [s/s]
	double Health;				//!< Satellite health information [dimnsionless]

	// Reduced Almanac Parameters

	double delta_A;				//!< Correction of semi-major axis relative to reference value at reference time
	double Phi_0;				//!< Argument of latitude at reference time

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

        archive& make_nvp("i_satellite_PRN", i_satellite_PRN);
        archive& make_nvp("SatType", SatType);
        archive& make_nvp("WN", i_BDS_week);
        archive& make_nvp("t_oa", t_oa);
        archive& make_nvp("e", e);
        archive& make_nvp("delta_i", delta_i);
        archive& make_nvp("sqrt_A", sqrt_A);
        archive& make_nvp("Omega_0", Omega_0);
        archive& make_nvp("Omega_dot", Omega_dot);
        archive& make_nvp("omega", omega);
        archive& make_nvp("M_0", M_0);
        archive& make_nvp("a_f0", a_f0);
        archive& make_nvp("a_f1", a_f1);
        archive& make_nvp("Health", Health);

        archive& make_nvp("PRN", i_satellite_PRN);
        archive& make_nvp("SatType", SatType);
        archive& make_nvp("delta_A", delta_A);
        archive& make_nvp("Omega_0", Omega_0);
        archive& make_nvp("Phi_0", Phi_0);
        archive& make_nvp("Health", Health);
    }

    double BDS_time_of_transmission(double t_sv);
    /*!
     * Default constructor
     */
    Beidou_Cnav2_Almanac();
};

#endif