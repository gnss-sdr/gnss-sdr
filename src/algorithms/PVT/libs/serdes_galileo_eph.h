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

#ifndef GNSS_SDR_SERDESGAL_EPH_H
#define GNSS_SDR_SERDESGAL_EPH_H

#include "galileo_ephemeris.h"
#include "monitor_galileo_ephemeris.pb.h"  // file created by Protocol Buffers at compile time
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
class Serdes_Galileo_Eph
{
public:
    Serdes_Galileo_Eph()
    {
        // Verify that the version of the library that we linked against is
        // compatible with the version of the headers we compiled against.
        GOOGLE_PROTOBUF_VERIFY_VERSION;
    }

    ~Serdes_Galileo_Eph()
    {
        // google::protobuf::ShutdownProtobufLibrary();
    }

    inline Serdes_Galileo_Eph(const Serdes_Galileo_Eph& other) noexcept  //!< Copy constructor
    {
        this->monitor_ = other.monitor_;
    }

    inline Serdes_Galileo_Eph& operator=(const Serdes_Galileo_Eph& rhs) noexcept  //!< Copy assignment operator
    {
        this->monitor_ = rhs.monitor_;
        return *this;
    }

    inline Serdes_Galileo_Eph(Serdes_Galileo_Eph&& other) noexcept  //!< Move constructor
    {
        this->monitor_ = std::move(other.monitor_);
    }

    inline Serdes_Galileo_Eph& operator=(Serdes_Galileo_Eph&& other) noexcept  //!< Move assignment operator
    {
        if (this != &other)
            {
                this->monitor_ = std::move(other.monitor_);
            }
        return *this;
    }

    inline std::string createProtobuffer(const std::shared_ptr<Galileo_Ephemeris> monitor)  //!< Serialization into a string
    {
        monitor_.Clear();

        std::string data;

        monitor_.set_i_satellite_prn(monitor->i_satellite_PRN);
        monitor_.set_iod_ephemeris(monitor->IOD_ephemeris);
        monitor_.set_iod_nav_1(monitor->IOD_nav_1);
        monitor_.set_m0_1(monitor->M0_1);                //!< Mean anomaly at reference time [semi-circles]
        monitor_.set_delta_n_3(monitor->delta_n_3);      //!< Mean motion difference from computed value [semi-circles/sec]
        monitor_.set_e_1(monitor->e_1);                  //!< Eccentricity
        monitor_.set_a_1(monitor->A_1);                  //!< Square root of the semi-major axis [meters^1/2]
        monitor_.set_omega_0_2(monitor->OMEGA_0_2);      //!< Longitude of ascending node of orbital plane at weekly epoch [semi-circles]
        monitor_.set_i_0_2(monitor->i_0_2);              //!< Inclination angle at reference time  [semi-circles]
        monitor_.set_omega_2(monitor->omega_2);          //!< Argument of perigee [semi-circles]
        monitor_.set_omega_dot_3(monitor->OMEGA_dot_3);  //!< Rate of right ascension [semi-circles/sec]
        monitor_.set_idot_2(monitor->iDot_2);            //!< Rate of inclination angle [semi-circles/sec]
        monitor_.set_c_uc_3(monitor->C_uc_3);            //!< Amplitude of the cosine harmonic correction term to the argument of latitude [radians]
        monitor_.set_c_us_3(monitor->C_us_3);            //!< Amplitude of the sine harmonic correction term to the argument of latitude [radians]
        monitor_.set_c_rc_3(monitor->C_rc_3);            //!< Amplitude of the cosine harmonic correction term to the orbit radius [meters]
        monitor_.set_c_rs_3(monitor->C_rs_3);            //!< Amplitude of the sine harmonic correction term to the orbit radius [meters]
        monitor_.set_c_ic_4(monitor->C_ic_4);            //!< Amplitude of the cosine harmonic correction term to the angle of inclination [radians]
        monitor_.set_c_is_4(monitor->C_is_4);            //!< Amplitude of the sine harmonic correction term to the angle of inclination [radians]
        monitor_.set_d_toe(monitor->t0e_1);              // Ephemeris reference time

        /*Clock correction parameters*/
        monitor_.set_d_toc(monitor->t0c_4);  // Clock correction data reference Time of Week
        monitor_.set_af0_4(monitor->af0_4);  //!< SV clock bias correction coefficient [s]
        monitor_.set_af1_4(monitor->af1_4);  //!< SV clock drift correction coefficient [s/s]
        monitor_.set_af2_4(monitor->af2_4);  //!< SV clock drift rate correction coefficient [s/s^2]

        /*GST*/
        // Not belong to ephemeris set (page 1 to 4)
        monitor_.set_wn_5(monitor->WN_5);    //!< Week number
        monitor_.set_tow_5(monitor->TOW_5);  //!< Time of Week
        monitor_.set_galileo_satclkdrift(monitor->Galileo_satClkDrift);
        monitor_.set_galileo_dtr(monitor->Galileo_dtr);  //!< relativistic clock correction term

        // SV status
        monitor_.set_sisa_3(monitor->SISA_3);
        monitor_.set_e5a_hs(monitor->E5a_HS);        //!< E5a Signal Health Status
        monitor_.set_e5b_hs_5(monitor->E5b_HS_5);    //!< E5b Signal Health Status
        monitor_.set_e1b_hs_5(monitor->E1B_HS_5);    //!< E1B Signal Health Status
        monitor_.set_e5a_dvs(monitor->E5a_DVS);      //!< E5a Data Validity Status
        monitor_.set_e5b_dvs_5(monitor->E5b_DVS_5);  //!< E5b Data Validity Status
        monitor_.set_e1b_dvs_5(monitor->E1B_DVS_5);  //!< E1B Data Validity Status

        monitor_.set_bgd_e1e5a_5(monitor->BGD_E1E5a_5);  //!< E1-E5a Broadcast Group Delay [s]
        monitor_.set_bgd_e1e5b_5(monitor->BGD_E1E5b_5);  //!< E1-E5b Broadcast Group Delay [s]

        monitor_.SerializeToString(&data);
        return data;
    }

    inline Galileo_Ephemeris readProtobuffer(const gnss_sdr::MonitorGalileoEphemeris& mon) const  //!< Deserialization
    {
        Galileo_Ephemeris monitor;

        monitor.i_satellite_PRN = mon.i_satellite_prn();

        monitor.IOD_ephemeris = mon.iod_ephemeris();
        monitor.IOD_nav_1 = mon.iod_nav_1();
        monitor.M0_1 = mon.m0_1();                //!< Mean anomaly at reference time [semi-circles]
        monitor.delta_n_3 = mon.delta_n_3();      //!< Mean motion difference from computed value [semi-circles/sec]
        monitor.e_1 = mon.e_1();                  //!< Eccentricity
        monitor.A_1 = mon.a_1();                  //!< Square root of the semi-major axis [meters^1/2]
        monitor.OMEGA_0_2 = mon.omega_0_2();      //!< Longitude of ascending node of orbital plane at weekly epoch [semi-circles]
        monitor.i_0_2 = mon.i_0_2();              //!< Inclination angle at reference time  [semi-circles]
        monitor.omega_2 = mon.omega_2();          //!< Argument of perigee [semi-circles]
        monitor.OMEGA_dot_3 = mon.omega_dot_3();  //!< Rate of right ascension [semi-circles/sec]
        monitor.iDot_2 = mon.idot_2();            //!< Rate of inclination angle [semi-circles/sec]
        monitor.C_uc_3 = mon.c_uc_3();            //!< Amplitude of the cosine harmonic correction term to the argument of latitude [radians]
        monitor.C_us_3 = mon.c_us_3();            //!< Amplitude of the sine harmonic correction term to the argument of latitude [radians]
        monitor.C_rc_3 = mon.c_rc_3();            //!< Amplitude of the cosine harmonic correction term to the orbit radius [meters]
        monitor.C_rs_3 = mon.c_rs_3();            //!< Amplitude of the sine harmonic correction term to the orbit radius [meters]
        monitor.C_ic_4 = mon.c_ic_4();            //!< Amplitude of the cosine harmonic correction term to the angle of inclination [radians]
        monitor.C_is_4 = mon.c_is_4();            //!< Amplitude of the sine harmonic correction term to the angle of inclination [radians]
        monitor.t0e_1 = mon.d_toe();              // Ephemeris reference time

        /*Clock correction parameters*/
        monitor.t0c_4 = mon.d_toc();  // Clock correction data reference Time of Week
        monitor.af0_4 = mon.af0_4();  //!< SV clock bias correction coefficient [s]
        monitor.af1_4 = mon.af1_4();  //!< SV clock drift correction coefficient [s/s]
        monitor.af2_4 = mon.af2_4();  //!< SV clock drift rate correction coefficient [s/s^2]

        /*GST*/
        // Not belong to ephemeris set (page 1 to 4)
        monitor.WN_5 = mon.wn_5();    //!< Week number
        monitor.TOW_5 = mon.tow_5();  //!< Time of Week
        monitor.Galileo_satClkDrift = mon.galileo_satclkdrift();
        monitor.Galileo_dtr = mon.galileo_dtr();  //!< relativistic clock correction term

        // SV status
        monitor.SISA_3 = mon.sisa_3();
        monitor.E5a_HS = mon.e5a_hs();        //!< E5a Signal Health Status
        monitor.E5b_HS_5 = mon.e5b_hs_5();    //!< E5b Signal Health Status
        monitor.E1B_HS_5 = mon.e1b_hs_5();    //!< E1B Signal Health Status
        monitor.E5a_DVS = mon.e5a_dvs();      //!< E5a Data Validity Status
        monitor.E5b_DVS_5 = mon.e5b_dvs_5();  //!< E5b Data Validity Status
        monitor.E1B_DVS_5 = mon.e1b_dvs_5();  //!< E1B Data Validity Status

        monitor.BGD_E1E5a_5 = mon.bgd_e1e5a_5();  //!< E1-E5a Broadcast Group Delay [s]
        monitor.BGD_E1E5b_5 = mon.bgd_e1e5b_5();  //!< E1-E5b Broadcast Group Delay [s]

        return monitor;
    }

private:
    gnss_sdr::MonitorGalileoEphemeris monitor_{};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_SERDESGAL_EPH_H
