/*!
 * \file gps_cnav_navigation_message.h
 * \brief  Interface of a GPS CNAV Data message decoder
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_GPS_CNAV_NAVIGATION_MESSAGE_H_
#define GNSS_SDR_GPS_CNAV_NAVIGATION_MESSAGE_H_


#include "GPS_CNAV.h"
#include "gps_cnav_ephemeris.h"
#include "gps_cnav_iono.h"
#include "gps_cnav_utc_model.h"
#include <bitset>
#include <map>
#include <string>
#include <vector>
#include <utility>

//TODO: Create GPS CNAV almanac
//#include "gps_almanac.h"



/*!
 * \brief This class decodes a GPS CNAV Data message as described in IS-GPS-200H
 *
 * See http://www.gps.gov/technical/icwg/IS-GPS-200H.pdf Appendix III
 */
class Gps_CNAV_Navigation_Message
{
private:
    unsigned long int read_navigation_unsigned(std::bitset<GPS_CNAV_DATA_PAGE_BITS> bits, const std::vector<std::pair<int,int>> parameter);
    signed long int read_navigation_signed(std::bitset<GPS_CNAV_DATA_PAGE_BITS> bits, const std::vector<std::pair<int,int>> parameter);
    bool read_navigation_bool(std::bitset<GPS_CNAV_DATA_PAGE_BITS> bits, const std::vector<std::pair<int,int>> parameter);

    Gps_CNAV_Ephemeris ephemeris_record;
    Gps_CNAV_Iono iono_record;
    Gps_CNAV_Utc_Model utc_model_record;

public:
    double d_TOW;
    bool b_flag_ephemeris_1;
    bool b_flag_ephemeris_2;
    bool b_flag_iono_valid; //!< If set, it indicates that the ionospheric parameters are filled and are not yet readed by the get_iono
    bool b_flag_utc_valid; //!< If set, it indicates that the utc parameters are filled and are not yet readed by the get_utc_model

    std::map<int,std::string> satelliteBlock; //!< Map that stores to which block the PRN belongs http://www.navcen.uscg.gov/?Do=constellationStatus

    // satellite positions
    double d_satpos_X;       //!< Earth-fixed coordinate x of the satellite [m]. Intersection of the IERS Reference Meridian (IRM) and the plane passing through the origin and normal to the Z-axis.
    double d_satpos_Y;       //!< Earth-fixed coordinate y of the satellite [m]. Completes a right-handed, Earth-Centered, Earth-Fixed orthogonal coordinate system.
    double d_satpos_Z;       //!< Earth-fixed coordinate z of the satellite [m]. The direction of the IERS (International Earth Rotation and Reference Systems Service) Reference Pole (IRP).

    // satellite identification info
    int i_channel_ID;
    unsigned int i_satellite_PRN;

    // Satellite velocity
    double d_satvel_X;    //!< Earth-fixed velocity coordinate x of the satellite [m]
    double d_satvel_Y;    //!< Earth-fixed velocity coordinate y of the satellite [m]
    double d_satvel_Z;    //!< Earth-fixed velocity coordinate z of the satellite [m]

    // public functions
    void reset();

    void decode_page(std::bitset<GPS_CNAV_DATA_PAGE_BITS> data_bits);
    /*!
     * \brief Obtain a GPS SV Ephemeris class filled with current SV data
     */
    Gps_CNAV_Ephemeris get_ephemeris();
    /*!
     * \brief Check if we have a new iono record stored in the GPS ephemeris class
     */
    bool have_new_iono();
    /*!
     * \brief Obtain a GPS ionospheric correction parameters class filled with current SV data
     */
    Gps_CNAV_Iono get_iono();

    /*!
     * \brief Obtain a GPS UTC model parameters class filled with current SV data
     */
    Gps_CNAV_Utc_Model get_utc_model();

    /*!
     * \briefCheck if we have a new GPS UTC model record stored in the GPS ephemeris class
     */
    bool have_new_utc_model();

    /*!
     * \brief Check if we have a new ephemeris stored in the GPS ephemeris class
     */
    bool have_new_ephemeris();

    /*!
     * Default constructor
     */
    Gps_CNAV_Navigation_Message();
};

#endif
