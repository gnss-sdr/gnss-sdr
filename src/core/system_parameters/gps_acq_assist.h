/*!
 * \file gps_acq_assist.h
 * \brief  Interface of a GPS RRLL ACQUISITION ASSISTACE storage
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


#ifndef GNSS_SDR_GPS_ACQ_ASSIST_H
#define GNSS_SDR_GPS_ACQ_ASSIST_H

#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the GPS GSM RRLL acquisition assistance data as described in
 * Digital cellular telecommunications system (Phase 2+);
 * Location Services (LCS);
 * Mobile Station (MS) - Serving Mobile Location Centre (SMLC)
 * Radio Resource LCS Protocol (RRLP)
 * (3GPP TS 44.031 version 5.12.0 Release 5)
 */
class Gps_Acq_Assist
{
public:
    /*!
     * Default constructor
     */
    Gps_Acq_Assist() = default;

    uint32_t i_satellite_PRN{};   //!< SV PRN NUMBER
    double d_TOW{};               //!< Time Of Week assigned to the acquisition data
    double d_Doppler0{};          //!< Doppler (0 order term) [Hz]
    double d_Doppler1{};          //!< Doppler (1 order term) [Hz]
    double dopplerUncertainty{};  //!< Doppler Uncertainty [Hz]
    double Code_Phase{};          //!< Code phase [chips]
    double Code_Phase_int{};      //!< Integer Code Phase [1 C/A code period]
    double GPS_Bit_Number{};      //!< GPS Bit Number
    double Code_Phase_window{};   //!< Code Phase search window [chips]
    double Azimuth{};             //!< Satellite Azimuth [deg]
    double Elevation{};           //!< Satellite Elevation [deg]
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_ACQ_ASSIST_H
