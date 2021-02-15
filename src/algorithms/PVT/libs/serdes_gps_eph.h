/*!
 * \file serdes_monitor_pvt.h
 * \brief Serialization / Deserialization of Monitor_Pvt objects using
 * Protocol Buffers
 * \author Carles Fernandez-Prades, 2019. cfernandez(at)cttc.es
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

#ifndef GNSS_SDR_SERDES_GPS_EPH_H
#define GNSS_SDR_SERDES_GPS_EPH_H

#include "gps_ephemeris.h"
#include "monitor_gps_ephemeris.pb.h"  // file created by Protocol Buffers at compile time
#include <memory>
#include <string>
#include <utility>

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */


/*!
 * \brief This class implements serialization and deserialization of
 * Monitor_Pvt objects using Protocol Buffers.
 */
class Serdes_Gps_Eph
{
public:
    Serdes_Gps_Eph()
    {
        // Verify that the version of the library that we linked against is
        // compatible with the version of the headers we compiled against.
        GOOGLE_PROTOBUF_VERIFY_VERSION;
    }

    ~Serdes_Gps_Eph()
    {
        // google::protobuf::ShutdownProtobufLibrary();
    }

    inline Serdes_Gps_Eph(const Serdes_Gps_Eph& other) noexcept  //!< Copy constructor
    {
        this->monitor_ = other.monitor_;
    }

    inline Serdes_Gps_Eph& operator=(const Serdes_Gps_Eph& rhs) noexcept  //!< Copy assignment operator
    {
        this->monitor_ = rhs.monitor_;
        return *this;
    }

    inline Serdes_Gps_Eph(Serdes_Gps_Eph&& other) noexcept  //!< Move constructor
    {
        this->monitor_ = std::move(other.monitor_);
    }

    inline Serdes_Gps_Eph& operator=(Serdes_Gps_Eph&& other) noexcept  //!< Move assignment operator
    {
        if (this != &other)
            {
                this->monitor_ = std::move(other.monitor_);
            }
        return *this;
    }

    inline std::string createProtobuffer(const std::shared_ptr<Gps_Ephemeris> monitor)  //!< Serialization into a string
    {
        monitor_.Clear();
        std::string data;
        monitor_.set_i_satellite_prn(monitor->i_satellite_PRN);    // SV PRN NUMBER
        monitor_.set_d_tow(monitor->d_TOW);                        //!< time of gps week of the ephemeris set (taken from subframes tow) [s]
        monitor_.set_d_crs(monitor->d_Crs);                        //!< amplitude of the sine harmonic correction term to the orbit radius [m]
        monitor_.set_d_delta_n(monitor->d_Delta_n);                //!< mean motion difference from computed value [semi-circles/s]
        monitor_.set_d_m_0(monitor->d_M_0);                        //!< mean anomaly at reference time [semi-circles]
        monitor_.set_d_cuc(monitor->d_Cuc);                        //!< amplitude of the cosine harmonic correction term to the argument of latitude [rad]
        monitor_.set_d_e_eccentricity(monitor->d_e_eccentricity);  //!< eccentricity [dimensionless]
        monitor_.set_d_cus(monitor->d_Cus);                        //!< amplitude of the sine harmonic correction term to the argument of latitude [rad]
        monitor_.set_d_sqrt_a(monitor->d_sqrt_A);                  //!< square root of the semi-major axis [sqrt(m)]
        monitor_.set_d_toe(monitor->d_Toe);                        //!< ephemeris data reference time of week (ref. 20.3.3.4.3 is-gps-200k) [s]
        monitor_.set_d_toc(monitor->d_Toc);                        //!< clock data reference time (ref. 20.3.3.3.3.1 is-gps-200k) [s]
        monitor_.set_d_cic(monitor->d_Cic);                        //!< amplitude of the cosine harmonic correction term to the angle of inclination [rad]
        monitor_.set_d_omega0(monitor->d_OMEGA0);                  //!< longitude of ascending node of orbit plane at weekly epoch [semi-circles]
        monitor_.set_d_cis(monitor->d_Cis);                        //!< amplitude of the sine harmonic correction term to the angle of inclination [rad]
        monitor_.set_d_i_0(monitor->d_i_0);                        //!< inclination angle at reference time [semi-circles]
        monitor_.set_d_crc(monitor->d_Crc);                        //!< amplitude of the cosine harmonic correction term to the orbit radius [m]
        monitor_.set_d_omega(monitor->d_OMEGA);                    //!< argument of perigee [semi-cicles]
        monitor_.set_d_omega_dot(monitor->d_OMEGA_DOT);            //!< rate of right ascension [semi-circles/s]
        monitor_.set_d_idot(monitor->d_IDOT);                      //!< rate of inclination angle [semi-circles/s]
        monitor_.set_i_code_on_l2(monitor->i_code_on_L2);          //!< if 1, p code on in l2;  if 2, c/a code on in l2;
        monitor_.set_i_gps_week(monitor->i_GPS_week);              //!< gps week number, aka wn [week]
        monitor_.set_b_l2_p_data_flag(monitor->b_L2_P_data_flag);  //!< when true, indicates that the nav data stream was commanded off on the p-code of the l2 channel
        monitor_.set_i_sv_accuracy(monitor->i_SV_accuracy);        //!< user range accuracy (ura) index of the sv (reference paragraph 6.2.1) for the standard positioning service user (ref 20.3.3.3.1.3 is-gps-200k)
        monitor_.set_i_sv_health(monitor->i_SV_health);
        monitor_.set_d_tgd(monitor->d_TGD);            //!< estimated group delay differential: l1-l2 correction term only for the benefit of "l1 p(y)" or "l2 p(y)" s users [s]
        monitor_.set_d_iodc(monitor->d_IODC);          //!< issue of data, clock
        monitor_.set_d_iode_sf2(monitor->d_IODE_SF2);  //!< issue of data, ephemeris (iode), subframe 2
        monitor_.set_d_iode_sf3(monitor->d_IODE_SF3);  //!< issue of data, ephemeris(iode), subframe 3
        monitor_.set_i_aodo(monitor->i_AODO);          //!< age of data offset (aodo) term for the navigation message correction table (nmct) contained in subframe 4 (reference paragraph 20.3.3.5.1.9) [s]

        monitor_.set_b_fit_interval_flag(monitor->b_fit_interval_flag);  //!< indicates the curve-fit interval used by the cs (block ii/iia/iir/iir-m/iif) and ss (block iiia) in determining the ephemeris parameters, as follows: 0 = 4 hours, 1 = greater than 4 hours.
        monitor_.set_d_spare1(monitor->d_spare1);
        monitor_.set_d_spare2(monitor->d_spare2);

        monitor_.set_d_a_f0(monitor->d_A_f0);  //!< coefficient 0 of code phase offset model [s]
        monitor_.set_d_a_f1(monitor->d_A_f1);  //!< coefficient 1 of code phase offset model [s/s]
        monitor_.set_d_a_f2(monitor->d_A_f2);  //!< coefficient 2 of code phase offset model [s/s^2]

        monitor_.set_b_integrity_status_flag(monitor->b_integrity_status_flag);
        monitor_.set_b_alert_flag(monitor->b_alert_flag);                //!< if true, indicates  that the sv ura may be worse than indicated in d_sv_accuracy, use that sv at our own risk.
        monitor_.set_b_antispoofing_flag(monitor->b_antispoofing_flag);  //!<  if true, the antispoofing mode is on in that sv

        monitor_.SerializeToString(&data);
        return data;
    }

    inline Gps_Ephemeris readProtobuffer(const gnss_sdr::MonitorGpsEphemeris& mon) const  //!< Deserialization
    {
        Gps_Ephemeris monitor;

        monitor.i_satellite_PRN = monitor_.i_satellite_prn();    // SV PRN NUMBER
        monitor.d_TOW = monitor_.d_tow();                        //!< time of gps week of the ephemeris set (taken from subframes tow) [s]
        monitor.d_Crs = monitor_.d_crs();                        //!< amplitude of the sine harmonic correction term to the orbit radius [m]
        monitor.d_Delta_n = monitor_.d_delta_n();                //!< mean motion difference from computed value [semi-circles/s]
        monitor.d_M_0 = monitor_.d_m_0();                        //!< mean anomaly at reference time [semi-circles]
        monitor.d_Cuc = monitor_.d_cuc();                        //!< amplitude of the cosine harmonic correction term to the argument of latitude [rad]
        monitor.d_e_eccentricity = monitor_.d_e_eccentricity();  //!< eccentricity [dimensionless]
        monitor.d_Cus = monitor_.d_cus();                        //!< amplitude of the sine harmonic correction term to the argument of latitude [rad]
        monitor.d_sqrt_A = monitor_.d_sqrt_a();                  //!< square root of the semi-major axis [sqrt(m)]
        monitor.d_Toe = monitor_.d_toe();                        //!< ephemeris data reference time of week (ref. 20.3.3.4.3 is-gps-200k) [s]
        monitor.d_Toc = monitor_.d_toc();                        //!< clock data reference time (ref. 20.3.3.3.3.1 is-gps-200k) [s]
        monitor.d_Cic = monitor_.d_cic();                        //!< amplitude of the cosine harmonic correction term to the angle of inclination [rad]
        monitor.d_OMEGA0 = monitor_.d_omega0();                  //!< longitude of ascending node of orbit plane at weekly epoch [semi-circles]
        monitor.d_Cis = monitor_.d_cis();                        //!< amplitude of the sine harmonic correction term to the angle of inclination [rad]
        monitor.d_i_0 = monitor_.d_i_0();                        //!< inclination angle at reference time [semi-circles]
        monitor.d_Crc = monitor_.d_crc();                        //!< amplitude of the cosine harmonic correction term to the orbit radius [m]
        monitor.d_OMEGA = monitor_.d_omega();                    //!< argument of perigee [semi-cicles]
        monitor.d_OMEGA_DOT = monitor_.d_omega_dot();            //!< rate of right ascension [semi-circles/s]
        monitor.d_IDOT = monitor_.d_idot();                      //!< rate of inclination angle [semi-circles/s]
        monitor.i_code_on_L2 = monitor_.i_code_on_l2();          //!< if 1, p code on in l2;  if 2, c/a code on in l2;
        monitor.i_GPS_week = monitor_.i_gps_week();              //!< gps week number, aka wn [week]
        monitor.b_L2_P_data_flag = monitor_.b_l2_p_data_flag();  //!< when true, indicates that the nav data stream was commanded off on the p-code of the l2 channel
        monitor.i_SV_accuracy = monitor_.i_sv_accuracy();        //!< user range accuracy (ura) index of the sv (reference paragraph 6.2.1) for the standard positioning service user (ref 20.3.3.3.1.3 is-gps-200k)
        monitor.i_SV_health = monitor_.i_sv_health();
        monitor.d_TGD = monitor_.d_tgd();            //!< estimated group delay differential: l1-l2 correction term only for the benefit of "l1 p(y)" or "l2 p(y)" s users [s]
        monitor.d_IODC = monitor_.d_iodc();          //!< issue of data, clock
        monitor.d_IODE_SF2 = monitor_.d_iode_sf2();  //!< issue of data, ephemeris (iode), subframe 2
        monitor.d_IODE_SF3 = monitor_.d_iode_sf3();  //!< issue of data, ephemeris(iode), subframe 3
        monitor.i_AODO = monitor_.i_aodo();          //!< age of data offset (aodo) term for the navigation message correction table (nmct) contained in subframe 4 (reference paragraph 20.3.3.5.1.9) [s]

        monitor.b_fit_interval_flag = monitor_.b_fit_interval_flag();  //!< indicates the curve-fit interval used by the cs (block ii/iia/iir/iir-m/iif) and ss (block iiia) in determining the ephemeris parameters, as follows: 0 = 4 hours, 1 = greater than 4 hours.
        monitor.d_spare1 = monitor_.d_spare1();
        monitor.d_spare2 = monitor_.d_spare2();

        monitor.d_A_f0 = monitor_.d_a_f0();  //!< coefficient 0 of code phase offset model [s]
        monitor.d_A_f1 = monitor_.d_a_f1();  //!< coefficient 1 of code phase offset model [s/s]
        monitor.d_A_f2 = monitor_.d_a_f2();  //!< coefficient 2 of code phase offset model [s/s^2]

        monitor.b_integrity_status_flag = monitor_.b_integrity_status_flag();
        monitor.b_alert_flag = monitor_.b_alert_flag();                //!< if true, indicates  that the sv ura may be worse than indicated in d_sv_accuracy, use that sv at our own risk.
        monitor.b_antispoofing_flag = monitor_.b_antispoofing_flag();  //!<  if true, the antispoofing mode is on in that sv

        return monitor;
    }

private:
    gnss_sdr::MonitorGpsEphemeris monitor_{};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_SERDES_GPS_EPH_H
