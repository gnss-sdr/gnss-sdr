/*!
 * \file gps_ephemeris.h
 * \brief  Interface of a GPS EPHEMERIS storage
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_GPS_EPHEMERIS_H
#define GNSS_SDR_GPS_EPHEMERIS_H


#include "gnss_ephemeris.h"
#include <boost/serialization/nvp.hpp>
#include <cstdint>
#include <map>
#include <string>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage and orbital model functions for the GPS SV
 * ephemeris data as described in IS-GPS-200M
 *
 * See https://www.gps.gov/technical/icwg/IS-GPS-200M.pdf Appendix II
 */
class Gps_Ephemeris : public Gnss_Ephemeris
{
public:
    /*!
     * Default constructor
     */
    Gps_Ephemeris();

    int32_t code_on_L2{};   //!< If 1, P code ON in L2;  if 2, C/A code ON in L2;
    bool L2_P_data_flag{};  //!< When true, indicates that the NAV data stream was commanded OFF on the P-code of the L2 channel
    int32_t SV_accuracy{};  //!< User Range Accuracy (URA) index of the SV (reference paragraph 6.2.1) for the standard positioning service user (Ref 20.3.3.3.1.3 IS-GPS-200M)
    int32_t SV_health{};    //!< Satellite heath status
    double TGD{};           //!< Estimated Group Delay Differential: L1-L2 correction term only for the benefit of "L1 P(Y)" or "L2 P(Y)" s users [s]
    int32_t IODC{};         //!< Issue of Data, Clock
    int32_t IODE_SF2{};     //!< Issue of Data, Ephemeris (IODE), subframe 2
    int32_t IODE_SF3{};     //!< Issue of Data, Ephemeris (IODE), subframe 3
    int32_t AODO{};         //!< Age of Data Offset (AODO) term for the navigation message correction table (NMCT) contained in subframe 4 (reference paragraph 20.3.3.5.1.9) [s]

    bool fit_interval_flag{};  //!< indicates the curve-fit interval used by the CS (Block II/IIA/IIR/IIR-M/IIF) and SS (Block IIIA) in determining the ephemeris parameters, as follows: 0 = 4 hours, 1 = greater than 4 hours.
    double spare1{};
    double spare2{};

    // Flags

    /*! \brief If true, enhanced level of integrity assurance.
     *
     *  If false, indicates that the conveying signal is provided with the legacy level of integrity assurance.
     *  That is, the probability that the instantaneous URE of the conveying signal exceeds 4.42 times the upper bound
     *  value of the current broadcast URA index, for more than 5.2 seconds, without an accompanying alert, is less
     *  than 1E-5 per hour. If true, indicates that the conveying signal is provided with an enhanced level of
     *  integrity assurance. That is, the probability that the instantaneous URE of the conveying signal exceeds 5.73
     *  times the upper bound value of the current broadcast URA index, for more than 5.2 seconds, without an
     *  accompanying alert, is less than 1E-8 per hour.
     */
    bool integrity_status_flag{};
    bool alert_flag{};         //!< If true, indicates that the SV URA may be worse than indicated in d_SV_accuracy, use that SV at our own risk.
    bool antispoofing_flag{};  //!< If true, the AntiSpoofing mode is ON in that SV

    std::map<int, std::string> satelliteBlock;  //!< Map that stores to which block the PRN belongs https://www.navcen.uscg.gov/?Do=constellationStatus

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

        archive& BOOST_SERIALIZATION_NVP(IODE_SF2);
        archive& BOOST_SERIALIZATION_NVP(IODE_SF3);
        archive& BOOST_SERIALIZATION_NVP(code_on_L2);
        archive& BOOST_SERIALIZATION_NVP(L2_P_data_flag);
        archive& BOOST_SERIALIZATION_NVP(SV_accuracy);
        archive& BOOST_SERIALIZATION_NVP(SV_health);
        archive& BOOST_SERIALIZATION_NVP(TGD);
        archive& BOOST_SERIALIZATION_NVP(IODC);
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
#endif  // GNSS_SDR_GPS_EPHEMERIS_H
