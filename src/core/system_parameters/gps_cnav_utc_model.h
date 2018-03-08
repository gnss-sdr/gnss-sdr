/*!
 * \file gps_utc_model.h
 * \brief  Interface of a GPS UTC MODEL storage
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_GPS_CNAV_UTC_MODEL_H_
#define GNSS_SDR_GPS_CNAV_UTC_MODEL_H_

#include <boost/assign.hpp>
#include <boost/serialization/nvp.hpp>


/*!
 * \brief This class is a storage for the GPS UTC MODEL data as described in in IS-GPS-200H
 *
 * See http://www.gps.gov/technical/icwg/IS-GPS-200H.pdf Appendix III
 */
class Gps_CNAV_Utc_Model
{
public:
    bool valid;
    // UTC parameters
    double d_A2;          //!< 2nd order term of a model that relates GPS and UTC time (ref. 20.3.3.5.2.4 IS-GPS-200H) [s/s]
    double d_A1;          //!< 1st order term of a model that relates GPS and UTC time (ref. 20.3.3.5.2.4 IS-GPS-200H) [s/s]
    double d_A0;          //!< Constant of a model that relates GPS and UTC time (ref. 20.3.3.5.2.4 IS-GPS-200H) [s]
    double d_t_OT;        //!< Reference time for UTC data (reference 20.3.4.5 and 20.3.3.5.2.4 IS-GPS-200H) [s]
    int i_WN_T;           //!< UTC reference week number [weeks]
    double d_DeltaT_LS;   //!< delta time due to leap seconds [s]. Number of leap seconds since 6-Jan-1980 as transmitted by the GPS almanac.
    int i_WN_LSF;         //!< Week number at the end of which the leap second becomes effective [weeks]
    int i_DN;             //!< Day number (DN) at the end of which the leap second becomes effective [days]
    double d_DeltaT_LSF;  //!< Scheduled future or recent past (relative to NAV message upload) value of the delta time due to leap seconds [s]

    /*!
     * Default constructor
     */
    Gps_CNAV_Utc_Model();

    /*!
     * \brief Computes the Coordinated Universal Time (UTC) and
     * returns it in [s] (IS-GPS-200E, 20.3.3.5.2.4 + 30.3.3.6.2)
     */
    double utc_time(double gpstime_corrected, int i_GPS_week);

    template <class Archive>
    /*
     * \brief Serialize is a boost standard method to be called by the boost XML serialization. Here is used to save the ephemeris data on disk file.
     */
    inline void serialize(Archive& archive, const unsigned int version)
    {
        using boost::serialization::make_nvp;
        if (version)
            {
            };
        archive& make_nvp("valid", valid);
        archive& make_nvp("d_A1", d_A1);
        archive& make_nvp("d_A0", d_A0);
        archive& make_nvp("d_t_OT", d_t_OT);
        archive& make_nvp("i_WN_T", i_WN_T);
        archive& make_nvp("d_DeltaT_LS", d_DeltaT_LS);
        archive& make_nvp("i_WN_LSF", i_WN_LSF);
        archive& make_nvp("i_DN", i_DN);
        archive& make_nvp("d_DeltaT_LSF", d_DeltaT_LSF);
    }
};

#endif
