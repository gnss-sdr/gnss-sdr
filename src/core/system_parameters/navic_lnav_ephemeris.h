/*!
 * \file navic_lnav_ephemeris.h
 * \brief  Interface of a NavIC (IRNSS) LNAV EPHEMERIS storage
 * \author Pradyumna Byppanahalli Suresha, 2025.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2025  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_NAVIC_LNAV_EPHEMERIS_H
#define GNSS_SDR_NAVIC_LNAV_EPHEMERIS_H

#include "gnss_ephemeris.h"
#include <boost/serialization/nvp.hpp>
#include <map>
#include <string>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This is a storage class for the NavIC (IRNSS) SV ephemeris data
 * as described in the IRNSS SIS ICD for SPS, Version 1.1, August 2017
 * (ISRO-IRNSS-ICD-SPS-1.1)
 *
 * Ephemeris parameters are decoded from subframes 1 and 2.
 * IRNSS uses WGS-84 coordinate system with:
 *   mu = 3.986005e14 m^3/s^2
 *   omega_e_dot = 7.2921151467e-5 rad/s
 */
class Navic_Lnav_Ephemeris : public Gnss_Ephemeris
{
public:
    /*!
     * Default constructor
     */
    Navic_Lnav_Ephemeris();

    // Subframe 1 parameters (NavIC-specific)
    int URA{};       //!< User Range Accuracy index (4 bits unsigned)
    double TGD{};    //!< Total Group Delay [s], 8-bit signed, scale 2^-31
    int IODEC{};     //!< Issue of Data, Ephemeris and Clock (8 bits unsigned)
    int L5_flag{};   //!< L5 signal health flag (1 bit)
    int S_flag{};    //!< S-band signal health flag (1 bit)

    std::map<int, std::string> satelliteBlock;  //!< Map that stores to which block the PRN belongs

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML
     * serialization. Here is used to save the ephemeris data on disk file.
     */
    void serialize(Archive& archive, const unsigned int version)
    {
        using boost::serialization::make_nvp;
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

        archive& BOOST_SERIALIZATION_NVP(URA);
        archive& BOOST_SERIALIZATION_NVP(TGD);
        archive& BOOST_SERIALIZATION_NVP(IODEC);
        archive& BOOST_SERIALIZATION_NVP(L5_flag);
        archive& BOOST_SERIALIZATION_NVP(S_flag);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_NAVIC_LNAV_EPHEMERIS_H
