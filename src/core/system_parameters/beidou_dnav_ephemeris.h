/*!
 * \file beidou_dnav_ephemeris.h
 * \brief  Interface of a BEIDOU EPHEMERIS storage
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
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


#ifndef GNSS_SDR_BEIDOU_DNAV_EPHEMERIS_H
#define GNSS_SDR_BEIDOU_DNAV_EPHEMERIS_H

#include "gnss_ephemeris.h"
#include <boost/serialization/nvp.hpp>
#include <map>
#include <string>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This is a storage class for the Beidou SV ephemeris data as described in
 * BeiDou Navigation Satellite System Signal In Space Interface Control Document
 * Open Service Signal B1I (Version 3.0)
 *
 * See http://en.beidou.gov.cn/SYSTEMS/Officialdocument/201902/P020190227601370045731.pdf
 */
class Beidou_Dnav_Ephemeris : public Gnss_Ephemeris
{
public:
    /*!
     * Default constructor
     */
    Beidou_Dnav_Ephemeris();

    int SV_accuracy{};  //!< User Range Accuracy (URA) index of the SV (reference paragraph 5.2.4.5) for the standard positioning service user
    int SV_health{};
    double TGD1{};  //!< Estimated Group Delay Differential on B1I [s]
    double TGD2{};  //!< Estimated Group Delay Differential on B2I [s]
    double AODC{};  //!< Age of Data, Clock
    double AODE{};  //!< Age of Data, Ephemeris
    int AODO{};     //!< Age of Data Offset (AODO) term for the navigation message correction table (NMCT) contained in subframe 4 (reference paragraph 20.3.3.5.1.9) [s]

    int sig_type{};  //!< BDS: data source (0:unknown,1:B1I,2:B1Q,3:B2I,4:B2Q,5:B3I,6:B3Q) */
    int nav_type{};  //!< BDS: nav type (0:unknown,1:IGSO/MEO,2:GEO) */

    bool fit_interval_flag{};  //!< Curve-fit interval used by the CS (Block II/IIA/IIR/IIR-M/IIF) and SS (Block IIIA) in determining the ephemeris parameters, as follows: 0 = 4 hours, 1 = greater than 4 hours.
    double spare1{};
    double spare2{};

    /*! \brief If true, enhanced level of integrity assurance.
     *
     *  If false, indicates that the conveying signal is provided with the
     *  legacy level of integrity assurance. That is, the probability that the
     *  instantaneous URE of the conveying signal exceeds 4.42 times the upper
     *  bound value of the current broadcast URA index, for more than 5.2
     *  seconds, without an accompanying alert, is less than 1E-5 per hour. If
     *  true, indicates that the conveying signal is provided with an enhanced
     *  level of integrity assurance. That is, the probability that the
     *  instantaneous URE of the conveying signal exceeds 5.73 times the upper
     *  bound value of the current broadcast URA index, for more than 5.2
     *  seconds, without an accompanying alert, is less than 1E-8 per hour.
     */
    bool integrity_status_flag{};
    bool alert_flag{};         //!< If true, indicates that the SV URA may be worse than indicated in d_SV_accuracy, use that SV at our own risk.
    bool antispoofing_flag{};  //!< If true, the AntiSpoofing mode is ON in that SV

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

        archive& BOOST_SERIALIZATION_NVP(AODE);
        archive& BOOST_SERIALIZATION_NVP(SV_accuracy);
        archive& BOOST_SERIALIZATION_NVP(SV_health);
        archive& BOOST_SERIALIZATION_NVP(AODC);
        archive& BOOST_SERIALIZATION_NVP(TGD1);
        archive& BOOST_SERIALIZATION_NVP(TGD2);
        archive& BOOST_SERIALIZATION_NVP(sig_type);
        archive& BOOST_SERIALIZATION_NVP(nav_type);
        archive& BOOST_SERIALIZATION_NVP(AODO);
        archive& BOOST_SERIALIZATION_NVP(fit_interval_flag);
        archive& BOOST_SERIALIZATION_NVP(spare1);
        archive& BOOST_SERIALIZATION_NVP(spare2);
        archive& BOOST_SERIALIZATION_NVP(integrity_status_flag);
        archive& BOOST_SERIALIZATION_NVP(alert_flag);
        archive& BOOST_SERIALIZATION_NVP(antispoofing_flag);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_DNAV_EPHEMERIS_H
