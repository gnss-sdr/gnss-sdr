/*!
 * \file beidou_dnav_utc_model.h
 * \brief  Interface of a BeiDou UTC MODEL storage
 * \author Damian Miralles, 2018. dmiralles2009@gmail.com
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
 *
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


#ifndef GNSS_SDR_BEIDOU_DNAV_UTC_MODEL_H_
#define GNSS_SDR_BEIDOU_DNAV_UTC_MODEL_H_

#include <boost/serialization/nvp.hpp>


/*!
 * \brief This class is a storage for the BeiDou DNAV UTC Model.
 * \details Implementation follows the interface described in the Open Service Signal (Version 2.1)
 *
 */
class Beidou_Dnav_Utc_Model
{
public:
    bool valid;

    // BeiDou UTC parameters
    double d_A0_UTC;      //!< BDT clock bias relative to UTC [s]
    double d_A1_UTC;      //!< BDT clock rate relative to UTC [s/s]
    double d_DeltaT_LS;   //!< Delta time due to leap seconds before the new leap second effective
    int i_WN_LSF;         //!< Week number of the new leap second
    int i_DN;             //!< Day number of week of the new leap second
    double d_DeltaT_LSF;  //!< Delta time due to leap seconds after the new leap second effective [s]

    // BeiDou to GPS time corrections
    double d_A0_GPS;  //!< BDT clock bias relative to GPS time [s]
    double d_A1_GPS;  //!< BDT clock rate relative to GPS time [s/s]

    // BeiDou to Galileo time corrections
    double d_A0_GAL;  //!< BDT clock bias relative to GAL time [s]
    double d_A1_GAL;  //!< BDT clock rate relative to GAL time [s/s]

    // BeiDou to GLONASS time corrections
    double d_A0_GLO;  //!< BDT clock bias relative to GLO time [s]
    double d_A1_GLO;  //!< BDT clock rate relative to GLO time [s/s]

    Beidou_Dnav_Utc_Model();

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
        archive& make_nvp("d_A1", d_A1_UTC);
        archive& make_nvp("d_A0", d_A0_UTC);
        archive& make_nvp("d_DeltaT_LS", d_DeltaT_LS);
        archive& make_nvp("i_WN_LSF", i_WN_LSF);
        archive& make_nvp("i_DN", i_DN);
        archive& make_nvp("d_DeltaT_LSF", d_DeltaT_LSF);
        archive& make_nvp("d_A0_GPS", d_A0_GPS);
        archive& make_nvp("d_A0_GPS", d_A1_GPS);
        archive& make_nvp("d_A0_GPS", d_A0_GAL);
        archive& make_nvp("d_A0_GPS", d_A1_GAL);
        archive& make_nvp("d_A0_GPS", d_A0_GLO);
        archive& make_nvp("d_A0_GPS", d_A1_GLO);
    }
};

#endif
