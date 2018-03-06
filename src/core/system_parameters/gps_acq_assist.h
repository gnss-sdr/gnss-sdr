/*!
 * \file gps_acq_assist.h
 * \brief  Interface of a GPS RRLL ACQUISITION ASSISTACE storage
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GPS_ACQ_ASSIST_H_
#define GNSS_SDR_GPS_ACQ_ASSIST_H_


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
    unsigned int i_satellite_PRN;  //!< SV PRN NUMBER
    double d_TOW;                  //!< Time Of Week assigned to the acquisition data
    double d_Doppler0;             //!< Doppler (0 order term) [Hz]
    double d_Doppler1;             //!< Doppler (1 order term) [Hz]
    double dopplerUncertainty;     //!< Doppler Uncertainty [Hz]
    double Code_Phase;             //!< Code phase [chips]
    double Code_Phase_int;         //!< Integer Code Phase [1 C/A code period]
    double GPS_Bit_Number;         //!< GPS Bit Number
    double Code_Phase_window;      //!< Code Phase search window [chips]
    double Azimuth;                //!< Satellite Azimuth [deg]
    double Elevation;              //!< Satellite Elevation [deg]

    /*!
     * Default constructor
     */
    Gps_Acq_Assist();
};

#endif
