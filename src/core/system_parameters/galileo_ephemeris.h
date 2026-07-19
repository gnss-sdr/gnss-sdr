/*!
 * \file galileo_ephemeris.h
 * \brief  Interface of a Galileo EPHEMERIS storage
 * \author Javier Arribas, 2013. jarribas(at)cttc.es,
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GALILEO_EPHEMERIS_H
#define GNSS_SDR_GALILEO_EPHEMERIS_H

#include "gnss_ephemeris.h"
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/version.hpp>
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage and orbital model functions for the Galileo SV
 * ephemeris data as described in Galileo ICD paragraph 5.1.1
 *
 *  (See https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo_OS_SIS_ICD_v2.0.pdf )
 *
 */
enum class Galileo_Nav_Message_Type : uint8_t
{
    Unknown = 0,
    INAV = 1,
    FNAV = 2
};


class Galileo_Ephemeris : public Gnss_Ephemeris
{
public:
    Galileo_Ephemeris()
    {
        this->System = 'E';
    }

    double Galileo_System_Time(double week_number, double TOW);  //!< Galileo System Time (GST), ICD paragraph 5.1.2

    int32_t IOD_ephemeris{};
    int32_t IOD_nav{};

    // SV status
    int32_t SISA{-1};    //!< Signal in space accuracy index (-1 if unavailable in the source data)
    int32_t E5a_HS{};    //!< E5a Signal Health Status
    int32_t E5b_HS{};    //!< E5b Signal Health Status
    int32_t E1B_HS{};    //!< E1B Signal Health Status
    bool E5a_DVS{};      //!< E5a Data Validity Status
    bool E5b_DVS{};      //!< E5b Data Validity Status
    bool E1B_DVS{};      //!< E1B Data Validity Status
    double BGD_E1E5a{};  //!< E1-E5a Broadcast Group Delay [s]
    double BGD_E1E5b{};  //!< E1-E5b Broadcast Group Delay [s]
    //! Navigation message carrying this record
    Galileo_Nav_Message_Type nav_message_type{Galileo_Nav_Message_Type::Unknown};

    bool flag_all_ephemeris{};

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML
     * serialization. Here is used to save the ephemeris data on disk file.
     */
    inline void serialize(Archive& archive, const uint32_t version)
    {
        if (version)
            {
            };

        archive& BOOST_SERIALIZATION_NVP(PRN);
        archive& BOOST_SERIALIZATION_NVP(M_0);
        archive& BOOST_SERIALIZATION_NVP(delta_n);
        archive& BOOST_SERIALIZATION_NVP(ecc);
        archive& BOOST_SERIALIZATION_NVP(sqrtA);
        archive& BOOST_SERIALIZATION_NVP(OMEGA_0);
        archive& BOOST_SERIALIZATION_NVP(i_0);
        archive& BOOST_SERIALIZATION_NVP(omega);
        archive& BOOST_SERIALIZATION_NVP(OMEGAdot);
        archive& BOOST_SERIALIZATION_NVP(idot);
        archive& BOOST_SERIALIZATION_NVP(Cuc);
        archive& BOOST_SERIALIZATION_NVP(Cus);
        archive& BOOST_SERIALIZATION_NVP(Crc);
        archive& BOOST_SERIALIZATION_NVP(Crs);
        archive& BOOST_SERIALIZATION_NVP(Cic);
        archive& BOOST_SERIALIZATION_NVP(Cis);
        archive& BOOST_SERIALIZATION_NVP(toe);
        archive& BOOST_SERIALIZATION_NVP(toc);
        archive& BOOST_SERIALIZATION_NVP(af0);
        archive& BOOST_SERIALIZATION_NVP(af1);
        archive& BOOST_SERIALIZATION_NVP(af2);
        archive& BOOST_SERIALIZATION_NVP(WN);
        archive& BOOST_SERIALIZATION_NVP(tow);
        archive& BOOST_SERIALIZATION_NVP(satClkDrift);
        archive& BOOST_SERIALIZATION_NVP(dtr);

        archive& BOOST_SERIALIZATION_NVP(IOD_ephemeris);
        archive& BOOST_SERIALIZATION_NVP(IOD_nav);
        archive& BOOST_SERIALIZATION_NVP(SISA);
        archive& BOOST_SERIALIZATION_NVP(E5a_HS);
        archive& BOOST_SERIALIZATION_NVP(E5b_HS);
        archive& BOOST_SERIALIZATION_NVP(E1B_HS);
        archive& BOOST_SERIALIZATION_NVP(E5a_DVS);
        archive& BOOST_SERIALIZATION_NVP(E5b_DVS);
        archive& BOOST_SERIALIZATION_NVP(E1B_DVS);
        archive& BOOST_SERIALIZATION_NVP(BGD_E1E5a);
        archive& BOOST_SERIALIZATION_NVP(BGD_E1E5b);
        archive& BOOST_SERIALIZATION_NVP(flag_all_ephemeris);
        if (version > 0)
            {
                uint32_t nav_message_type_value = static_cast<uint32_t>(nav_message_type);
                archive& boost::serialization::make_nvp("nav_message_type", nav_message_type_value);
                if (nav_message_type_value <= static_cast<uint32_t>(Galileo_Nav_Message_Type::FNAV))
                    {
                        nav_message_type = static_cast<Galileo_Nav_Message_Type>(nav_message_type_value);
                    }
                else
                    {
                        nav_message_type = Galileo_Nav_Message_Type::Unknown;
                    }
            }
    }
};


BOOST_CLASS_VERSION(Galileo_Ephemeris, 1)


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_EPHEMERIS_H
