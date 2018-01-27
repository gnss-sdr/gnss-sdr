/*!
 * \file glonass_gnav_ephemeris.h
 * \brief  Interface of a GLONASS EPHEMERIS storage
 * \note Code added as part of GSoC 2017 program
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
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


#ifndef GNSS_SDR_GLONASS_GNAV_EPHEMERIS_H_
#define GNSS_SDR_GLONASS_GNAV_EPHEMERIS_H_


#include <boost/serialization/nvp.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>



/*!
 * \brief This class is a storage and orbital model functions for the GLONASS SV ephemeris data as described in GLONASS ICD (Edition 5.1)
 * \note Code added as part of GSoC 2017 program
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
 */
class Glonass_Gnav_Ephemeris
{
private:
    /*
     * Accounts for the beginning or end of week crossover
     *
     * See paragraph 20.3.3.3.3.1 (IS-GPS-200E)
     * \param[in]  -  time in seconds
     * \param[out] -  corrected time, in seconds
     */
    double check_t(double time);

public:
    double d_m;             //!< String number within frame [dimensionless]
    double d_t_k;           //!< GLONASS Time (UTC(SU) + 3 h) referenced to the beginning of the frame within the current day [s]
    double d_t_b;           //!< Reference ephemeris relative time in GLONASS Time (UTC(SU) + 3 h). Index of a time interval within current day according to UTC(SU) + 03 hours 00 min. [s]
    double d_M;             //!< Type of satellite transmitting navigation signal [dimensionless]
    double d_gamma_n;       //!< Relative deviation of predicted carrier frequency value of n- satellite from nominal value at the instant tb [dimensionless]
    double d_tau_n;         //!< Correction to the nth satellite time (tn) relative to GLONASS time (te),
    double d_Xn;            //!< Earth-fixed coordinate x of the satellite in PZ-90.02 coordinate system [km].
    double d_Yn;            //!< Earth-fixed coordinate y of the satellite in PZ-90.02 coordinate system [km]
    double d_Zn;            //!< Earth-fixed coordinate z of the satellite in PZ-90.02 coordinate system [km]
    double d_VXn;           //!< Earth-fixed velocity coordinate x of the satellite in PZ-90.02 coordinate system [km/s]
    double d_VYn;           //!< Earth-fixed velocity coordinate y of the satellite in PZ-90.02 coordinate system [km/s]
    double d_VZn;           //!< Earth-fixed velocity coordinate z of the satellite in PZ-90.02 coordinate system [km/s]
    double d_AXn;           //!< Earth-fixed acceleration coordinate x of the satellite in PZ-90.02 coordinate system [km/s^2]
    double d_AYn;           //!< Earth-fixed acceleration coordinate y of the satellite in PZ-90.02 coordinate system [km/s^2]
    double d_AZn;           //!< Earth-fixed acceleration coordinate z of the satellite in PZ-90.02 coordinate system [km/s^2]
    double d_B_n;           //!< Health flag [dimensionless]
    double d_P;             //!< Technological parameter of control segment, indication the satellite operation mode in respect of time parameters [dimensionless]
    double d_N_T;           //!< Current date, calendar number of day within four-year interval starting from the 1-st of January in a leap year [days]
    double d_F_T;           //!< Parameter that provides the predicted satellite user range accuracy at time tb [dimensionless]
    double d_n;             //!< Index of the satellite transmitting given navigation signal. It corresponds to a slot number within GLONASS constellation
    double d_Delta_tau_n;   //!< Time difference between navigation RF signal transmitted in L2 sub- band and aviation RF signal transmitted in L1 sub-band by nth satellite. [dimensionless]
    double d_E_n;           //!< Characterises "age" of a current information [days]
    double d_P_1;           //!< Flag of the immediate data updating [minutes]
    bool d_P_2;             //!< Flag of oddness ("1") or evenness ("0") of the value of (tb) [dimensionless]
    bool d_P_3;             //!< Flag indicating a number of satellites for which almanac is transmitted within given frame: "1" corresponds to 5 satellites and "0" corresponds to 4 satellites [dimensionless]
    bool d_P_4;             //!< Flag to show that ephemeris parameters are present. "1" indicates that updated ephemeris or frequency/time parameters have been uploaded by the control segment [dimensionless]
    bool d_l3rd_n;          //!< Health flag for nth satellite; ln = 0 indicates the n-th satellite is healthy, ln = 1 indicates malfunction of this nth satellite [dimensionless]
    bool d_l5th_n;          //!< Health flag for nth satellite; ln = 0 indicates the n-th satellite is healthy, ln = 1 indicates malfunction of this nth satellite [dimensionless]

    // Inmediate deliverables of ephemeris information
    // Satellite Identification Information
    int i_satellite_freq_channel;           //!< SV Frequency Channel Number
    unsigned int i_satellite_PRN;           //!< SV PRN Number, equivalent to slot number for compatibility with GPS
    unsigned int i_satellite_slot_number;   //!< SV Slot Number
    double d_yr;                            //!< Current year
    double d_satClkDrift;                   //!< GLONASS clock error
    double d_dtr;                           //!< relativistic clock correction term
    double d_iode;                          //!< Issue of data, ephemeris (Bit 0-6 of tb)
    double d_tau_c;                         //!< GLONASST 2 UTC correction (todo) may be eliminated
    double d_TOW;                           //!< GLONASST IN GPST seconds of week
    double d_WN;                            //!< GLONASST IN GPST week number of the start of frame
    double d_tod;                           //!< Time of Day since ephemeris where decoded

    template<class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML serialization. Here is used to save the ephemeris data on disk file.
     */
    void serialize(Archive& archive, const unsigned int version)
    {
        using boost::serialization::make_nvp;
        if(version){};

        archive & make_nvp("i_satellite_freq_channel", i_satellite_freq_channel); //!< SV PRN frequency channel number
        archive & make_nvp("i_satellite_PRN", i_satellite_PRN);
        archive & make_nvp("i_satellite_slot_number", i_satellite_slot_number);
        archive & make_nvp("d_m", d_m);             //!< String number within frame [dimensionless]
        archive & make_nvp("d_t_k", d_t_k);         //!< Time referenced to the beginning of the frame within the current day [hours, minutes, seconds]
        archive & make_nvp("d_t_b", d_t_b);         //!< Index of a time interval within current day according to UTC(SU) + 03 hours 00 min. [minutes]
        archive & make_nvp("d_M", d_M);             //!< Type of satellite transmitting navigation signal [dimensionless]
        archive & make_nvp("d_gamma_n", d_gamma_n); //!< Relative deviation of predicted carrier frequency value of n- satellite from nominal value at the instant tb [dimensionless]
        archive & make_nvp("d_tau_n", d_tau_n);     //!< Correction to the nth satellite time (tn) relative to GLONASS time (te)
        archive & make_nvp("d_Xn", d_Xn);           //!< Earth-fixed coordinate x of the satellite in PZ-90.02 coordinate system [km].
        archive & make_nvp("d_Yn", d_Yn);           //!< Earth-fixed coordinate y of the satellite in PZ-90.02 coordinate system [km]
        archive & make_nvp("d_Zn", d_Zn);           //!< Earth-fixed coordinate z of the satellite in PZ-90.02 coordinate system [km]
        archive & make_nvp("d_VXn", d_VXn);         //!< Earth-fixed velocity coordinate x of the satellite in PZ-90.02 coordinate system [km/s]
        archive & make_nvp("d_VYn", d_VYn);         //!< Earth-fixed velocity coordinate y of the satellite in PZ-90.02 coordinate system [km/s]
        archive & make_nvp("d_VZn", d_VZn);         //!< Earth-fixed velocity coordinate z of the satellite in PZ-90.02 coordinate system [km/s]
        archive & make_nvp("d_AXn", d_AXn);         //!< Earth-fixed acceleration coordinate x of the satellite in PZ-90.02 coordinate system [km/s^2]
        archive & make_nvp("d_AYn", d_AYn);         //!< Earth-fixed acceleration coordinate y of the satellite in PZ-90.02 coordinate system [km/s^2]
        archive & make_nvp("d_AZn", d_AZn);         //!< Earth-fixed acceleration coordinate z of the satellite in PZ-90.02 coordinate system [km/s^2]
        archive & make_nvp("d_B_n", d_B_n);         //!< Health flag [dimensionless]
        archive & make_nvp("d_P", d_P);             //!< Technological parameter of control segment, indication the satellite operation mode in respect of time parameters [dimensionless]
        archive & make_nvp("d_N_T", d_N_T);         //!< Current date, calendar number of day within four-year interval starting from the 1-st of January in a leap year [days]
        archive & make_nvp("d_F_T", d_F_T);         //!< Parameter that provides the predicted satellite user range accuracy at time tb [dimensionless]
        archive & make_nvp("d_n", d_n);             //!< Index of the satellite transmitting given navigation signal. It corresponds to a slot number within GLONASS constellation
        archive & make_nvp("d_Delta_tau_n", d_Delta_tau_n); //!< Time difference between navigation RF signal transmitted in L2 sub- band and aviation RF signal transmitted in L1 sub-band by nth satellite. [dimensionless]
        archive & make_nvp("d_E_n", d_E_n);         //!< Characterises "age" of a current information [days]
        archive & make_nvp("d_P_1", d_P_1);         //!< Flag of the immediate data updating.
        archive & make_nvp("d_P_2", d_P_2);         //!< Flag of oddness ("1") or evenness ("0") of the value of (tb) [dimensionless]
        archive & make_nvp("d_P_3", d_P_3);         //!< Flag indicating a number of satellites for which almanac is transmitted within given frame: "1" corresponds to 5 satellites and "0" corresponds to 4 satellites [dimensionless]
        archive & make_nvp("d_P_4", d_P_4);         //!< Flag to show that ephemeris parameters are present. "1" indicates that updated ephemeris or frequency/time parameters have been uploaded by the control segment [dimensionless]
        archive & make_nvp("d_l3rd_n", d_l3rd_n);   //!< Health flag for nth satellite; ln = 0 indicates the n-th satellite is helthy, ln = 1 indicates malfunction of this nth satellite [dimensionless]
        archive & make_nvp("d_l5th_n", d_l5th_n);   //!< Health flag for nth satellite; ln = 0 indicates the n-th satellite is helthy, ln = 1 indicates malfunction of this nth satellite [dimensionless]
    }

    /*!
     * \brief Sets (\a d_satClkDrift)and returns the clock drift in seconds according to the User Algorithm for SV Clock Correction
     *  (IS-GPS-200E,  20.3.3.3.3.1)
     */
    double sv_clock_drift(double transmitTime, double timeCorrUTC);

    /*!
     *  \brief Computes the GLONASS System Time and returns a boost::posix_time::ptime object
     * \ param offset_time Is the start of day offset to compute the time
     */
    boost::posix_time::ptime compute_GLONASS_time(const double offset_time) const;

    /*!
     * \brief Converts from GLONASST to UTC
     * \details The function simply adjust for the 6 hrs offset between GLONASST and UTC
     * \param[in] offset_time Is the start of day offset
     * \param[in] glot2utc_corr Correction from GLONASST to UTC
     * \returns UTC time as a boost::posix_time::ptime object
     */
    boost::posix_time::ptime glot_to_utc(const double offset_time, const double glot2utc_corr) const;

    /*!
     * \brief Converts from GLONASST to GPST
     * \details Converts from GLONASST to GPST in time of week (TOW) and week number (WN) format
     * \param[in] tod_offset Is the start of day offset
     * \param[in] glot2utc_corr Correction from GLONASST to UTC
     * \param[in] glot2gpst_corr Correction from GLONASST to GPST
     * \param[out] WN Week Number, not in mod(1024) format
     * \param[out] TOW Time of Week in seconds of week
     */
    void glot_to_gpst(double tod_offset, double glot2utc_corr, double glot2gpst_corr, double * WN, double * TOW) const;

    /*!
     * Default constructor
     */
    Glonass_Gnav_Ephemeris();
};

#endif
