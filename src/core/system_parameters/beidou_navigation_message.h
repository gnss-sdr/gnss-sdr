/*!
 * \file beidou_navigation_message.h
 * \brief  Interface of a BeiDou D1 NAV Data message decoder
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
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


#ifndef GNSS_SDR_BEIDOU_NAVIGATION_MESSAGE_H_
#define GNSS_SDR_BEIDOU_NAVIGATION_MESSAGE_H_


#include <bitset>
#include <map>
#include <string>
#include <utility>
#include <vector>
#include "beidou_b1I.h"
#include "beidou_ephemeris.h"
#include "beidou_iono.h"
#include "beidou_almanac.h"
#include "beidou_utc_model.h"



/*!
 * \brief This class decodes a BeiDou D1 NAV Data message as described in IS-GPS-200E
 *
 * See http://www.gps.gov/technical/icwg/IS-GPS-200E.pdf Appendix II
 */
class Beidou_Navigation_Message_D1
{
private:
    unsigned long int read_navigation_unsigned(std::bitset<BEIDOU_SUBFRAME_BITS> bits, const std::vector<std::pair<int,int>> parameter);
    signed long int read_navigation_signed(std::bitset<BEIDOU_SUBFRAME_BITS> bits, const std::vector<std::pair<int,int>> parameter);
    bool read_navigation_bool(std::bitset<BEIDOU_SUBFRAME_BITS> bits, const std::vector<std::pair<int,int>> parameter);
    void print_beidou_word_bytes(unsigned int BEIDOU_word);
    /*
     * Accounts for the beginning or end of week crossover
     *
     * See paragraph 20.3.3.3.3.1 (IS-GPS-200E)
     * \param[in]  -  time in seconds
     * \param[out] -  corrected time, in seconds
     */
    double check_t(double time);

public:
    bool b_valid_ephemeris_set_flag; // flag indicating that this ephemeris set have passed the validation check
    //broadcast orbit 1
    double d_SOW; //!< Time of BeiDou Week of the ephemeris set (taken from subframes SOW) [s]
    double d_SOW_SF1;            //!< Time of BeiDou Week from HOW word of Subframe 1 [s]
    double d_SOW_SF2;            //!< Time of BeiDou Week from HOW word of Subframe 2 [s]
    double d_SOW_SF3;            //!< Time of BeiDou Week from HOW word of Subframe 3 [s]
    double d_SOW_SF4;            //!< Time of BeiDou Week from HOW word of Subframe 4 [s]
    double d_SOW_SF5;            //!< Time of BeiDou Week from HOW word of Subframe 5 [s]

    double d_AODE;
    double d_Crs;            //!< Amplitude of the Sine Harmonic Correction Term to the Orbit Radius [m]
    double d_Delta_n;        //!< Mean Motion Difference From Computed Value [semi-circles/s]
    double d_M_0;            //!< Mean Anomaly at Reference Time [semi-circles]
    //broadcast orbit 2
    double d_Cuc;            //!< Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude [rad]
    double d_e_eccentricity; //!< Eccentricity [dimensionless]
    double d_Cus;            //!< Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude [rad]
    double d_sqrt_A;         //!< Square Root of the Semi-Major Axis [sqrt(m)]
    //broadcast orbit 3
    double d_Toe;            //!< Ephemeris data reference time of week (Ref. 20.3.3.4.3 IS-GPS-200E) [s]
    double d_Toe2;
    double d_Toc;            //!< clock data reference time (Ref. 20.3.3.3.3.1 IS-GPS-200E) [s]
    double d_Cic;            //!< Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination [rad]
    double d_OMEGA0;         //!< Longitude of Ascending Node of Orbit Plane at Weekly Epoch [semi-circles]
    double d_Cis;            //!< Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination [rad]
    //broadcast orbit 4
    double d_i_0;            //!< Inclination Angle at Reference Time [semi-circles]
    double d_Crc;            //!< Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius [m]
    double d_OMEGA;          //!< Argument of Perigee [semi-cicles]
    double d_OMEGA_DOT;      //!< Rate of Right Ascension [semi-circles/s]
    //broadcast orbit 5
    double d_IDOT;           //!< Rate of Inclination Angle [semi-circles/s]
    int i_BEIDOU_week;          //!< BeiDou week number, aka WN [week]
    //broadcast orbit 6
    int i_SV_accuracy;       //!< User Range Accuracy (URA) index of the SV 
    int i_SV_health;
    double d_TGD1;            //!< Estimated Group Delay Differential in B1
    double d_TGD2;            //!< Estimated Group Delay Differential in B2
    double d_AODC;           //!< Age of Data, Clock
    //broadcast orbit 7
//    int i_AODO;              //!< Age of Data Offset (AODO) term for the navigation message correction table (NMCT) contained in subframe 4 (reference paragraph 20.3.3.5.1.9) [s]

    bool b_fit_interval_flag;//!< indicates the curve-fit interval used by the CS (Block II/IIA/IIR/IIR-M/IIF) and SS (Block IIIA) in determining the ephemeris parameters, as follows: 0 = 4 hours, 1 = greater than 4 hours.
    double d_spare1;
    double d_spare2;

    double d_A_f0;          //!< Coefficient 0 of code phase offset model [s]
    double d_A_f1;          //!< Coefficient 1 of code phase offset model [s/s]
    double d_A_f2;          //!< Coefficient 2 of code phase offset model [s/s^2]

    double d_A0;
    double d_A1;
    double d_A2;

    // Almanac
    double d_Toa;           //!< Almanac reference time [s]
    int i_WN_A;             //!< Modulo 256 of the GPS week number to which the almanac reference time (d_Toa) is referenced
    std::map<int,int> almanacHealth; //!< Map that stores the health information stored in the almanac

    std::map<int,std::string> satelliteBlock; //!< Map that stores to which block the PRN belongs http://www.navcen.uscg.gov/?Do=constellationStatus

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
    bool b_integrity_status_flag;
    bool b_alert_flag;      //!< If true, indicates  that the SV URA may be worse than indicated in d_SV_accuracy, use that SV at our own risk.
    bool b_antispoofing_flag;  //!<  If true, the AntiSpoofing mode is ON in that SV

    // clock terms
    //double d_master_clock;  // GPS transmission time
    double d_satClkCorr;     // GPS clock error
    double d_dtr;            // relativistic clock correction term
    double d_satClkDrift;

    // satellite positions
    double d_satpos_X;       //!< Earth-fixed coordinate x of the satellite [m]. Intersection of the IERS Reference Meridian (IRM) and the plane passing through the origin and normal to the Z-axis.
    double d_satpos_Y;       //!< Earth-fixed coordinate y of the satellite [m]. Completes a right-handed, Earth-Centered, Earth-Fixed orthogonal coordinate system.
    double d_satpos_Z;       //!< Earth-fixed coordinate z of the satellite [m]. The direction of the IERS (International Earth Rotation and Reference Systems Service) Reference Pole (IRP).

    // satellite identification info
    int i_channel_ID;
    unsigned int i_satellite_PRN;

    // time synchro
    double d_subframe_timestamp_ms; //[ms]

    // Ionospheric parameters
    bool flag_iono_valid; //!< If set, it indicates that the ionospheric parameters are filled (page 18 has arrived and decoded)
    double d_alpha0;      //!< Coefficient 0 of a cubic equation representing the amplitude of the vertical delay [s]
    double d_alpha1;      //!< Coefficient 1 of a cubic equation representing the amplitude of the vertical delay [s/semi-circle]
    double d_alpha2;      //!< Coefficient 2 of a cubic equation representing the amplitude of the vertical delay [s(semi-circle)^2]
    double d_alpha3;      //!< Coefficient 3 of a cubic equation representing the amplitude of the vertical delay [s(semi-circle)^3]
    double d_beta0;       //!< Coefficient 0 of a cubic equation representing the period of the model [s]
    double d_beta1;       //!< Coefficient 1 of a cubic equation representing the period of the model [s/semi-circle]
    double d_beta2;       //!< Coefficient 2 of a cubic equation representing the period of the model [s(semi-circle)^2]
    double d_beta3;       //!< Coefficient 3 of a cubic equation representing the period of the model [s(semi-circle)^3]

    // UTC parameters
    bool flag_utc_model_valid; //!< If set, it indicates that the UTC model parameters are filled
    double d_A2UTC; 
    double d_A1UTC;          //!< 1st order term of a model that relates GPS and UTC time (ref. 20.3.3.5.2.4 IS-GPS-200E) [s/s]
    double d_A0UTC;          //!< Constant of a model that relates GPS and UTC time (ref. 20.3.3.5.2.4 IS-GPS-200E) [s]
    double d_t_OT;        //!< Reference time for UTC data (reference 20.3.4.5 and 20.3.3.5.2.4 IS-GPS-200E) [s]
    int i_WN_T;           //!< UTC reference week number [weeks]
    double d_DeltaT_LS;   //!< delta time due to leap seconds [s]. Number of leap seconds since 6-Jan-1980 as transmitted by the GPS almanac.
    int i_WN_LSF;         //!< Week number at the end of which the leap second becomes effective [weeks]
    int i_DN;             //!< Day number (DN) at the end of which the leap second becomes effective [days]
    double d_DeltaT_LSF;  //!< Scheduled future or recent past (relative to NAV message upload) value of the delta time due to leap seconds [s]
    double d_A1GPS;
    double d_A0GPS;
    double d_A1GAL;
    double d_A0GAL;
    double d_A1GLO;
    double d_A0GLO;

    double d_AODE_SF1;
    double d_SQRT_A_ALMANAC;
    double d_A1_ALMANAC;
    double d_A0_ALMANAC;
    double d_OMEGA0_ALMANAC;
    double d_E_ALMANAC;
    double d_DELTA_I;
    double d_TOA;
    double d_OMEGA_DOT_ALMANAC;
    double d_OMEGA_ALMANAC;
    double d_M0_ALMANAC;
    int    almanac_WN;
    double d_toa2;







    // Satellite velocity
    double d_satvel_X;    //!< Earth-fixed velocity coordinate x of the satellite [m]
    double d_satvel_Y;    //!< Earth-fixed velocity coordinate y of the satellite [m]
    double d_satvel_Z;    //!< Earth-fixed velocity coordinate z of the satellite [m]


    // public functions
    void reset();

    /*!
     * \brief Obtain a GPS SV Ephemeris class filled with current SV data
     */
    Beidou_Ephemeris get_ephemeris();

    /*!
     * \brief Obtain a GPS ionospheric correction parameters class filled with current SV data
     */
    Beidou_Iono get_iono();

    /*!
     * \brief Obtain a GPS UTC model parameters class filled with current SV data
     */
    Beidou_Utc_Model get_utc_model();


    /*!
     * \brief Decodes the GPS NAV message
     */
    int subframe_decoder(char *subframe);

    /*!
     * \brief Computes the position of the satellite
     *
     * Implementation of Table 20-IV (IS-GPS-200E)
     */
    void satellitePosition(double transmitTime);

    /*!
     * \brief Sets (\a d_satClkCorr) according to the User Algorithm for SV Clock Correction
     * and returns the corrected clock (IS-GPS-200E,  20.3.3.3.3.1)
     */
    double sv_clock_correction(double transmitTime);

    /*!
     * \brief Computes the Coordinated Universal Time (UTC) and
     * returns it in [s] (IS-GPS-200E, 20.3.3.5.2.4)
     */
    double utc_time(const double gpstime_corrected) const;

    bool satellite_validation();

    /*!
     * Default constructor
     */
    Beidou_Navigation_Message_D1();
};

#endif
