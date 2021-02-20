/*!
 * \file serdes_gps_eph.h
 * \brief Serialization / Deserialization of Gps_Ephemeris objects using
 * Protocol Buffers
 * \author Javier Arribas, 2021. jarribas(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
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
 * Gps_Ephemeris objects using Protocol Buffers.
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
        monitor_.set_i_satellite_prn(monitor->PRN);              //!< SV PRN NUMBER
        monitor_.set_d_tow(monitor->tow);                        //!< time of gps week of the ephemeris set (taken from subframes tow) [s]
        monitor_.set_d_crs(monitor->Crs);                        //!< amplitude of the sine harmonic correction term to the orbit radius [m]
        monitor_.set_d_delta_n(monitor->delta_n);                //!< mean motion difference from computed value [semi-circles/s]
        monitor_.set_d_m_0(monitor->M_0);                        //!< mean anomaly at reference time [semi-circles]
        monitor_.set_d_cuc(monitor->Cuc);                        //!< amplitude of the cosine harmonic correction term to the argument of latitude [rad]
        monitor_.set_d_e_eccentricity(monitor->ecc);             //!< eccentricity [dimensionless]
        monitor_.set_d_cus(monitor->Cus);                        //!< amplitude of the sine harmonic correction term to the argument of latitude [rad]
        monitor_.set_d_sqrt_a(monitor->sqrtA);                   //!< square root of the semi-major axis [sqrt(m)]
        monitor_.set_d_toe(monitor->toe);                        //!< ephemeris data reference time of week (ref. 20.3.3.4.3 is-gps-200k) [s]
        monitor_.set_d_toc(monitor->toc);                        //!< clock data reference time (ref. 20.3.3.3.3.1 is-gps-200k) [s]
        monitor_.set_d_cic(monitor->Cic);                        //!< amplitude of the cosine harmonic correction term to the angle of inclination [rad]
        monitor_.set_d_omega0(monitor->OMEGA_0);                 //!< longitude of ascending node of orbit plane at weekly epoch [semi-circles]
        monitor_.set_d_cis(monitor->Cis);                        //!< amplitude of the sine harmonic correction term to the angle of inclination [rad]
        monitor_.set_d_i_0(monitor->i_0);                        //!< inclination angle at reference time [semi-circles]
        monitor_.set_d_crc(monitor->Crc);                        //!< amplitude of the cosine harmonic correction term to the orbit radius [m]
        monitor_.set_d_omega(monitor->omega);                    //!< argument of perigee [semi-cicles]
        monitor_.set_d_omega_dot(monitor->OMEGAdot);             //!< rate of right ascension [semi-circles/s]
        monitor_.set_d_idot(monitor->idot);                      //!< rate of inclination angle [semi-circles/s]
        monitor_.set_i_code_on_l2(monitor->code_on_L2);          //!< if 1, p code on in l2;  if 2, c/a code on in l2;
        monitor_.set_i_gps_week(monitor->WN);                    //!< gps week number, aka wn [week]
        monitor_.set_b_l2_p_data_flag(monitor->L2_P_data_flag);  //!< when true, indicates that the nav data stream was commanded off on the p-code of the l2 channel
        monitor_.set_i_sv_accuracy(monitor->SV_accuracy);        //!< user range accuracy (ura) index of the sv (reference paragraph 6.2.1) for the standard positioning service user (ref 20.3.3.3.1.3 is-gps-200k)
        monitor_.set_i_sv_health(monitor->SV_health);
        monitor_.set_d_tgd(monitor->TGD);            //!< estimated group delay differential: l1-l2 correction term only for the benefit of "l1 p(y)" or "l2 p(y)" s users [s]
        monitor_.set_d_iodc(monitor->IODC);          //!< issue of data, clock
        monitor_.set_d_iode_sf2(monitor->IODE_SF2);  //!< issue of data, ephemeris (iode), subframe 2
        monitor_.set_d_iode_sf3(monitor->IODE_SF3);  //!< issue of data, ephemeris(iode), subframe 3
        monitor_.set_i_aodo(monitor->AODO);          //!< age of data offset (aodo) term for the navigation message correction table (nmct) contained in subframe 4 (reference paragraph 20.3.3.5.1.9) [s]

        monitor_.set_b_fit_interval_flag(monitor->b_fit_interval_flag);  //!< indicates the curve-fit interval used by the cs (block ii/iia/iir/iir-m/iif) and ss (block iiia) in determining the ephemeris parameters, as follows: 0 = 4 hours, 1 = greater than 4 hours.
        monitor_.set_d_spare1(monitor->d_spare1);
        monitor_.set_d_spare2(monitor->d_spare2);

        monitor_.set_d_a_f0(monitor->af0);  //!< coefficient 0 of code phase offset model [s]
        monitor_.set_d_a_f1(monitor->af1);  //!< coefficient 1 of code phase offset model [s/s]
        monitor_.set_d_a_f2(monitor->af2);  //!< coefficient 2 of code phase offset model [s/s^2]

        monitor_.set_b_integrity_status_flag(monitor->b_integrity_status_flag);
        monitor_.set_b_alert_flag(monitor->b_alert_flag);                //!< if true, indicates that the sv ura may be worse than indicated in d_sv_accuracy, use that sv at our own risk.
        monitor_.set_b_antispoofing_flag(monitor->b_antispoofing_flag);  //!< if true, the antispoofing mode is on in that sv

        monitor_.SerializeToString(&data);
        return data;
    }

    inline Gps_Ephemeris readProtobuffer(const gnss_sdr::MonitorGpsEphemeris& mon) const  //!< Deserialization
    {
        Gps_Ephemeris monitor;

        monitor.PRN = mon.i_satellite_prn();              //!< SV PRN NUMBER
        monitor.tow = mon.d_tow();                        //!< time of gps week of the ephemeris set (taken from subframes tow) [s]
        monitor.Crs = mon.d_crs();                        //!< amplitude of the sine harmonic correction term to the orbit radius [m]
        monitor.delta_n = mon.d_delta_n();                //!< mean motion difference from computed value [semi-circles/s]
        monitor.M_0 = mon.d_m_0();                        //!< mean anomaly at reference time [semi-circles]
        monitor.Cuc = mon.d_cuc();                        //!< amplitude of the cosine harmonic correction term to the argument of latitude [rad]
        monitor.ecc = mon.d_e_eccentricity();             //!< eccentricity [dimensionless]
        monitor.Cus = mon.d_cus();                        //!< amplitude of the sine harmonic correction term to the argument of latitude [rad]
        monitor.sqrtA = mon.d_sqrt_a();                   //!< square root of the semi-major axis [sqrt(m)]
        monitor.toe = mon.d_toe();                        //!< ephemeris data reference time of week (ref. 20.3.3.4.3 is-gps-200k) [s]
        monitor.toc = mon.d_toc();                        //!< clock data reference time (ref. 20.3.3.3.3.1 is-gps-200k) [s]
        monitor.Cic = mon.d_cic();                        //!< amplitude of the cosine harmonic correction term to the angle of inclination [rad]
        monitor.OMEGA_0 = mon.d_omega0();                 //!< longitude of ascending node of orbit plane at weekly epoch [semi-circles]
        monitor.Cis = mon.d_cis();                        //!< amplitude of the sine harmonic correction term to the angle of inclination [rad]
        monitor.i_0 = mon.d_i_0();                        //!< inclination angle at reference time [semi-circles]
        monitor.Crc = mon.d_crc();                        //!< amplitude of the cosine harmonic correction term to the orbit radius [m]
        monitor.omega = mon.d_omega();                    //!< argument of perigee [semi-cicles]
        monitor.OMEGAdot = mon.d_omega_dot();             //!< rate of right ascension [semi-circles/s]
        monitor.idot = mon.d_idot();                      //!< rate of inclination angle [semi-circles/s]
        monitor.code_on_L2 = mon.i_code_on_l2();          //!< if 1, p code on in l2;  if 2, c/a code on in l2;
        monitor.WN = mon.i_gps_week();                    //!< gps week number, aka wn [week]
        monitor.L2_P_data_flag = mon.b_l2_p_data_flag();  //!< when true, indicates that the nav data stream was commanded off on the p-code of the l2 channel
        monitor.SV_accuracy = mon.i_sv_accuracy();        //!< user range accuracy (ura) index of the sv (reference paragraph 6.2.1) for the standard positioning service user (ref 20.3.3.3.1.3 is-gps-200k)
        monitor.SV_health = mon.i_sv_health();
        monitor.TGD = mon.d_tgd();            //!< estimated group delay differential: l1-l2 correction term only for the benefit of "l1 p(y)" or "l2 p(y)" s users [s]
        monitor.IODC = mon.d_iodc();          //!< issue of data, clock
        monitor.IODE_SF2 = mon.d_iode_sf2();  //!< issue of data, ephemeris (iode), subframe 2
        monitor.IODE_SF3 = mon.d_iode_sf3();  //!< issue of data, ephemeris(iode), subframe 3
        monitor.AODO = mon.i_aodo();          //!< age of data offset (aodo) term for the navigation message correction table (nmct) contained in subframe 4 (reference paragraph 20.3.3.5.1.9) [s]

        monitor.b_fit_interval_flag = mon.b_fit_interval_flag();  //!< indicates the curve-fit interval used by the cs (block ii/iia/iir/iir-m/iif) and ss (block iiia) in determining the ephemeris parameters, as follows: 0 = 4 hours, 1 = greater than 4 hours.
        monitor.d_spare1 = mon.d_spare1();
        monitor.d_spare2 = mon.d_spare2();

        monitor.af0 = mon.d_a_f0();  //!< coefficient 0 of code phase offset model [s]
        monitor.af1 = mon.d_a_f1();  //!< coefficient 1 of code phase offset model [s/s]
        monitor.af2 = mon.d_a_f2();  //!< coefficient 2 of code phase offset model [s/s^2]

        monitor.b_integrity_status_flag = mon.b_integrity_status_flag();
        monitor.b_alert_flag = mon.b_alert_flag();                //!< if true, indicates that the sv ura may be worse than indicated in d_sv_accuracy, use that sv at our own risk.
        monitor.b_antispoofing_flag = mon.b_antispoofing_flag();  //!< if true, the antispoofing mode is on in that sv

        return monitor;
    }

private:
    gnss_sdr::MonitorGpsEphemeris monitor_{};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_SERDES_GPS_EPH_H
