/*!
 * \file beidou_cnav2_iono.h
 * \brief  Interface of a BeiDou CNAV2 Ionospheric Model storage
 * \author Damian Miralles, 2018. dmiralles2009(at)gmail.com
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


#ifndef GNSS_SDR_BEIDOU_CNAV2_IONO_H_
#define GNSS_SDR_BEIDOU_CNAV2_IONO_H_

#include <boost/serialization/nvp.hpp>

/*!
 * \brief This class is a storage for the BeiDou CNAV2 IONOSPHERIC data as described in its ICD paragraph 6.2.3
 *
 * See http://m.beidou.gov.cn/xt/gfxz/201712/P020171226742357364174.pdf
 */
class Beidou_Cnav2_Iono
{
public:
    // Ionospheric correction
    bool valid;  //!< Valid flag

    // BeiDou Global Ionospheric delay correction Model (BDGIM) parameters
    double alpha1;  //!< Coefficient 1 of the BDGIM model [TECu]
    double alpha2;  //!< Coefficient 2 of the BDGIM model [TECu]
    double alpha3;  //!< Coefficient 3 of the BDGIM model [TECu]
    double alpha4;  //!< Coefficient 3 of the BDGIM model [TECu]
    double alpha5;  //!< Coefficient 3 of the BDGIM model [TECu]
    double alpha6;  //!< Coefficient 3 of the BDGIM model [TECu]
    double alpha7;  //!< Coefficient 3 of the BDGIM model [TECu]
    double alpha8;  //!< Coefficient 3 of the BDGIM model [TECu]
    double alpha9;  //!< Coefficient 3 of the BDGIM model [TECu]

    // from message type 30, get the iono correction parameters
    double d_TOW_30;  //!< BDT data reference Time of Week [s]
    double d_WN_30;   //!< BDT data reference Week number [week]

    /*!
     * Default constructor
     */
    Beidou_Cnav2_Iono();

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML serialization.
     Here is used to save the iono data on disk file.
     */
    inline void serialize(Archive& archive, const unsigned int version)
    {
        using boost::serialization::make_nvp;
        if (version)
            {
            };
        archive& make_nvp("d_alpha1", alpha1);
        archive& make_nvp("d_alpha2", alpha2);
        archive& make_nvp("d_alpha3", alpha3);
        archive& make_nvp("d_alpha4", alpha4);
        archive& make_nvp("d_alpha5", alpha5);
        archive& make_nvp("d_alpha6", alpha6);
        archive& make_nvp("d_alpha7", alpha7);
        archive& make_nvp("d_alpha8", alpha8);
        archive& make_nvp("d_alpha9", alpha9);
        archive& make_nvp("TOW_30", d_TOW_30);
        archive& make_nvp("WN_30", d_WN_30);
    }
};

#endif