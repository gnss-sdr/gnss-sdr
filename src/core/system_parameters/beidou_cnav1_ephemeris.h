/*!
 * \file beidou_cnav1_ephemeris.h
 * \brief BeiDou B-CNAV1 ephemeris storage (BDS-SIS-ICD-B1C-1.0 §7)
 * \author GNSS-SDR contributors
 *
 * -----------------------------------------------------------------------------
 * SPDX-License-Identifier: GPL-3.0-or-later
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_BEIDOU_CNAV1_EPHEMERIS_H
#define GNSS_SDR_BEIDOU_CNAV1_EPHEMERIS_H

#include "gnss_ephemeris.h"
#include <boost/serialization/nvp.hpp>

class Beidou_Cnav1_Ephemeris : public Gnss_Ephemeris
{
public:
    Beidou_Cnav1_Ephemeris() = default;

    double TGD_B1Cp{};    //!< §7.6 B1C pilot group delay
    double TGD_B2ap{};    //!< §7.6 B2a pilot group delay
    double ISC_B1Cd{};    //!< §7.6 B1C data-to-pilot intra-frequency delay correction
    double IODC{};        //!< §7.4.2
    double IODE{};        //!< §7.4.1
    int32_t sig_type{7};  //!< B1C data component identifier for PVT
    int32_t nav_type{1};  //!< 0: GEO, 1: MEO/IGSO

    template <class Archive>
    void serialize(Archive& archive, const unsigned int version)
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
        archive& BOOST_SERIALIZATION_NVP(TGD_B1Cp);
        archive& BOOST_SERIALIZATION_NVP(TGD_B2ap);
        archive& BOOST_SERIALIZATION_NVP(ISC_B1Cd);
        archive& BOOST_SERIALIZATION_NVP(IODC);
        archive& BOOST_SERIALIZATION_NVP(IODE);
        archive& BOOST_SERIALIZATION_NVP(sig_type);
        archive& BOOST_SERIALIZATION_NVP(nav_type);
    }
};

#endif  // GNSS_SDR_BEIDOU_CNAV1_EPHEMERIS_H
