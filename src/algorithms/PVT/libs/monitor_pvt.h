/*!
 * \file monitor_pvt.h
 * \brief  Interface of the Monitor_Pvt class
 * \author
 *  Álvaro Cebrián Juan, 2019. acebrianjuan(at)gmail.com
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_MONITOR_PVT_H_
#define GNSS_SDR_MONITOR_PVT_H_

#include <boost/serialization/nvp.hpp>
#include <cstdint>

/*!
 * \brief This class contains parameters and outputs of the PVT block
 */
class Monitor_Pvt
{
public:
    // TOW
    uint32_t TOW_at_current_symbol_ms;
    // WEEK
    uint32_t week;
    // PVT GPS time
    double RX_time;
    // User clock offset [s]
    double user_clk_offset;

    // ECEF POS X,Y,X [m] + ECEF VEL X,Y,X [m/s] (6 x double)
    double pos_x;
    double pos_y;
    double pos_z;
    double vel_x;
    double vel_y;
    double vel_z;

    // position variance/covariance (m^2) {c_xx,c_yy,c_zz,c_xy,c_yz,c_zx} (6 x double)
    double cov_xx;
    double cov_yy;
    double cov_zz;
    double cov_xy;
    double cov_yz;
    double cov_zx;

    // GEO user position Latitude [deg]
    double latitude;
    // GEO user position Longitude [deg]
    double longitude;
    // GEO user position Height [m]
    double height;

    // NUMBER OF VALID SATS
    uint8_t valid_sats;
    // RTKLIB solution status
    uint8_t solution_status;
    // RTKLIB solution type (0:xyz-ecef,1:enu-baseline)
    uint8_t solution_type;
    // AR ratio factor for validation
    float AR_ratio_factor;
    // AR ratio threshold for validation
    float AR_ratio_threshold;

    // GDOP / PDOP/ HDOP/ VDOP
    double gdop;
    double pdop;
    double hdop;
    double vdop;

    /*!
     * \brief This member function serializes and restores
     * Monitor_Pvt objects from a byte stream.
     */
    template <class Archive>

    void serialize(Archive& ar, const unsigned int version)
    {
        if (version)
            {
            };

        ar& BOOST_SERIALIZATION_NVP(TOW_at_current_symbol_ms);
        ar& BOOST_SERIALIZATION_NVP(week);
        ar& BOOST_SERIALIZATION_NVP(RX_time);
        ar& BOOST_SERIALIZATION_NVP(user_clk_offset);

        ar& BOOST_SERIALIZATION_NVP(pos_x);
        ar& BOOST_SERIALIZATION_NVP(pos_y);
        ar& BOOST_SERIALIZATION_NVP(pos_z);
        ar& BOOST_SERIALIZATION_NVP(vel_x);
        ar& BOOST_SERIALIZATION_NVP(vel_y);
        ar& BOOST_SERIALIZATION_NVP(vel_z);

        ar& BOOST_SERIALIZATION_NVP(cov_xx);
        ar& BOOST_SERIALIZATION_NVP(cov_yy);
        ar& BOOST_SERIALIZATION_NVP(cov_zz);
        ar& BOOST_SERIALIZATION_NVP(cov_xy);
        ar& BOOST_SERIALIZATION_NVP(cov_yz);
        ar& BOOST_SERIALIZATION_NVP(cov_zx);

        ar& BOOST_SERIALIZATION_NVP(latitude);
        ar& BOOST_SERIALIZATION_NVP(longitude);
        ar& BOOST_SERIALIZATION_NVP(height);

        ar& BOOST_SERIALIZATION_NVP(valid_sats);
        ar& BOOST_SERIALIZATION_NVP(solution_status);
        ar& BOOST_SERIALIZATION_NVP(solution_type);
        ar& BOOST_SERIALIZATION_NVP(AR_ratio_factor);
        ar& BOOST_SERIALIZATION_NVP(AR_ratio_threshold);

        ar& BOOST_SERIALIZATION_NVP(gdop);
        ar& BOOST_SERIALIZATION_NVP(pdop);
        ar& BOOST_SERIALIZATION_NVP(hdop);
        ar& BOOST_SERIALIZATION_NVP(vdop);
    }
};

#endif /* GNSS_SDR_MONITOR_PVT_H_ */
