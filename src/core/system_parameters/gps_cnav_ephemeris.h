/*!
 * \file gps_cnav_ephemeris.h
 * \brief  Interface of a GPS CNAV EPHEMERIS storage
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_GPS_CNAV_EPHEMERIS_H
#define GNSS_SDR_GPS_CNAV_EPHEMERIS_H

#include "gnss_ephemeris.h"
#include <boost/serialization/nvp.hpp>
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This is a storage class for the GPS CNAV ephemeris data as described
 * in IS-GPS-200M
 *
 * See https://www.gps.gov/technical/icwg/IS-GPS-200M.pdf Appendix III
 */
class Gps_CNAV_Ephemeris : public Gnss_Ephemeris
{
public:
    /*!
     * Constructor
     */
    Gps_CNAV_Ephemeris()
    {
        this->System = 'G';
    }

    double delta_A{};         //!< Semi-major axis difference at reference time
    double Adot{};            //!< Change rate in semi-major axis
    double delta_ndot{};      //!< Rate of mean motion difference from computed value
    double delta_OMEGAdot{};  //!< Rate of Right Ascension  difference [semi-circles/s]
    int32_t toe1{};           //!< Ephemeris data reference time of week (Ref. 20.3.3.4.3 IS-GPS-200M) [s]
    int32_t toe2{};           //!< Ephemeris data reference time of week (Ref. 20.3.3.4.3 IS-GPS-200M) [s]
    int32_t signal_health{};  //!< Signal health (L1/L2/L5)
    int32_t top{};            //!< Data predict time of week
    int32_t URA{};            //!< ED Accuracy Index

    double URA0{};  //!< NED Accuracy Index
    double URA1{};  //!< NED Accuracy Change Index
    double URA2{};  //!< NED Accuracy Change Rate Index

    // Group Delay Differential Parameters
    double TGD{};  //!< Estimated Group Delay Differential: L1-L2 correction term only for the benefit of "L1 P(Y)" or "L2 P(Y)" s users [s]
    double ISCL1{};
    double ISCL2{};
    double ISCL5I{};
    double ISCL5Q{};

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
    bool l2c_phasing_flag{};
    bool alert_flag{};         //!< If true, indicates that the SV URA may be worse than indicated in d_SV_accuracy, use that SV at our own risk.
    bool antispoofing_flag{};  //!< If true, the AntiSpoofing mode is ON in that SV

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML
     * serialization. Here is used to save the ephemeris data on disk file.
     */
    inline void serialize(Archive& archive, const uint32_t version)
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

        archive& BOOST_SERIALIZATION_NVP(toe1);
        archive& BOOST_SERIALIZATION_NVP(toe2);
        archive& BOOST_SERIALIZATION_NVP(TGD);
        archive& BOOST_SERIALIZATION_NVP(ISCL1);
        archive& BOOST_SERIALIZATION_NVP(ISCL2);
        archive& BOOST_SERIALIZATION_NVP(ISCL5I);
        archive& BOOST_SERIALIZATION_NVP(ISCL5Q);
        archive& BOOST_SERIALIZATION_NVP(delta_A);
        archive& BOOST_SERIALIZATION_NVP(Adot);
        archive& BOOST_SERIALIZATION_NVP(delta_OMEGAdot);
        archive& BOOST_SERIALIZATION_NVP(integrity_status_flag);
        archive& BOOST_SERIALIZATION_NVP(l2c_phasing_flag);
        archive& BOOST_SERIALIZATION_NVP(alert_flag);
        archive& BOOST_SERIALIZATION_NVP(antispoofing_flag);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_CNAV_EPHEMERIS_H
