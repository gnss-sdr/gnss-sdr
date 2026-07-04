/*!
 * \file gps_cnav2_ephemeris.h
 * \brief  Interface of a GPS CNAV2 EPHEMERIS storage
 * \author José Antonio Mayo, 2015. contact(at)tatjam.eu
 *
 * The CNAV2 ephemeris is extremely similar to CNAV, dropping legacy functions
 * and including new integrity fields for the new signal.
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


#ifndef GNSS_SDR_GPS_CNAV2_EPHEMERIS_H
#define GNSS_SDR_GPS_CNAV2_EPHEMERIS_H

#include "gnss_ephemeris.h"
#include "gps_cnav_ephemeris.h"
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/version.hpp>
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This is a storage class for the GPS CNAV2 ephemeris data as described
 * in IS-GPS-800J
 *
 * See https://www.gps.gov/sites/default/files/2025-07/IS-GPS-800J.pdf Section 3.5.3
 */
class Gps_CNAV2_Ephemeris : public Gps_CNAV_Ephemeris
{
public:
    /*!
     * Constructor
     */
    Gps_CNAV2_Ephemeris()
    {
        this->System = 'G';
    }

    int32_t l1c_signal_health{};  //!< Signal health (L1C)
    double ISCL1CP{};             //!< Inter-signal Correciton
    double ISCL1CD{};             //!< Inter-signal Correction
    double ISCL2C{};              //!< Inter-signal Correction
    double ISCL5I5{};             //!< Inter-signal Correction
    double ISCL5Q5{};             //!< Inter-signal Correction

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML
     * serialization. Here is used to save the ephemeris data on disk file.
     */
    inline void serialize(Archive& archive, const uint32_t /* version */)
    {
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

        archive& BOOST_SERIALIZATION_NVP(toe);
        archive& BOOST_SERIALIZATION_NVP(WNop);
        archive& BOOST_SERIALIZATION_NVP(top);
        archive& BOOST_SERIALIZATION_NVP(URAED);
        archive& BOOST_SERIALIZATION_NVP(URANED0);
        archive& BOOST_SERIALIZATION_NVP(URANED1);
        archive& BOOST_SERIALIZATION_NVP(URANED2);
        archive& BOOST_SERIALIZATION_NVP(TGD);
        archive& BOOST_SERIALIZATION_NVP(ISCL1CP);
        archive& BOOST_SERIALIZATION_NVP(ISCL1CD);
        archive& BOOST_SERIALIZATION_NVP(ISCL2C);
        archive& BOOST_SERIALIZATION_NVP(ISCL5I5);
        archive& BOOST_SERIALIZATION_NVP(ISCL5Q5);
        archive& BOOST_SERIALIZATION_NVP(delta_A);
        archive& BOOST_SERIALIZATION_NVP(Adot);
        archive& BOOST_SERIALIZATION_NVP(delta_OMEGAdot);
        archive& BOOST_SERIALIZATION_NVP(integrity_status_flag);
    }
};

BOOST_CLASS_VERSION(Gps_CNAV2_Ephemeris, 1)


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_CNAV2_EPHEMERIS_H
