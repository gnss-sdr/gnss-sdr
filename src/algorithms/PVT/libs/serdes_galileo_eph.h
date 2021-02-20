/*!
 * \file serdes_galileo_eph.h
 * \brief Serialization / Deserialization of Galileo_Ephemeris objects using
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

#ifndef GNSS_SDR_SERDES_GALILEO_EPH_H
#define GNSS_SDR_SERDES_GALILEO_EPH_H

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
 * Galileo_Ephemeris using Protocol Buffers.
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

        monitor_.set_i_satellite_prn(monitor->PRN);
        monitor_.set_iod_ephemeris(monitor->IOD_ephemeris);
        monitor_.set_iod_nav_1(monitor->IOD_nav);
        monitor_.set_m0_1(monitor->M_0);              //!< Mean anomaly at reference time [semi-circles]
        monitor_.set_delta_n_3(monitor->delta_n);     //!< Mean motion difference from computed value [semi-circles/sec]
        monitor_.set_e_1(monitor->ecc);               //!< Eccentricity
        monitor_.set_a_1(monitor->sqrtA);             //!< Square root of the semi-major axis [meters^1/2]
        monitor_.set_omega_0_2(monitor->OMEGA_0);     //!< Longitude of ascending node of orbital plane at weekly epoch [semi-circles]
        monitor_.set_i_0_2(monitor->i_0);             //!< Inclination angle at reference time  [semi-circles]
        monitor_.set_omega_2(monitor->omega);         //!< Argument of perigee [semi-circles]
        monitor_.set_omega_dot_3(monitor->OMEGAdot);  //!< Rate of right ascension [semi-circles/sec]
        monitor_.set_idot_2(monitor->idot);           //!< Rate of inclination angle [semi-circles/sec]
        monitor_.set_c_uc_3(monitor->Cuc);            //!< Amplitude of the cosine harmonic correction term to the argument of latitude [radians]
        monitor_.set_c_us_3(monitor->Cus);            //!< Amplitude of the sine harmonic correction term to the argument of latitude [radians]
        monitor_.set_c_rc_3(monitor->Crc);            //!< Amplitude of the cosine harmonic correction term to the orbit radius [meters]
        monitor_.set_c_rs_3(monitor->Crs);            //!< Amplitude of the sine harmonic correction term to the orbit radius [meters]
        monitor_.set_c_ic_4(monitor->Cic);            //!< Amplitude of the cosine harmonic correction term to the angle of inclination [radians]
        monitor_.set_c_is_4(monitor->Cis);            //!< Amplitude of the sine harmonic correction term to the angle of inclination [radians]
        monitor_.set_d_toe(monitor->toe);             //!< Ephemeris reference time

        /*Clock correction parameters*/
        monitor_.set_d_toc(monitor->toc);  //!< Clock correction data reference Time of Week
        monitor_.set_af0_4(monitor->af0);  //!< SV clock bias correction coefficient [s]
        monitor_.set_af1_4(monitor->af1);  //!< SV clock drift correction coefficient [s/s]
        monitor_.set_af2_4(monitor->af2);  //!< SV clock drift rate correction coefficient [s/s^2]

        /*GST*/
        // Not belong to ephemeris set (page 1 to 4)
        monitor_.set_wn_5(monitor->WN);    //!< Week number
        monitor_.set_tow_5(monitor->tow);  //!< Time of Week
        monitor_.set_galileo_satclkdrift(monitor->satClkDrift);
        monitor_.set_galileo_dtr(monitor->dtr);  //!< relativistic clock correction term

        // SV status
        monitor_.set_sisa_3(monitor->SISA);
        monitor_.set_e5a_hs(monitor->E5a_HS);      //!< E5a Signal Health Status
        monitor_.set_e5b_hs_5(monitor->E5b_HS);    //!< E5b Signal Health Status
        monitor_.set_e1b_hs_5(monitor->E1B_HS);    //!< E1B Signal Health Status
        monitor_.set_e5a_dvs(monitor->E5a_DVS);    //!< E5a Data Validity Status
        monitor_.set_e5b_dvs_5(monitor->E5b_DVS);  //!< E5b Data Validity Status
        monitor_.set_e1b_dvs_5(monitor->E1B_DVS);  //!< E1B Data Validity Status

        monitor_.set_bgd_e1e5a_5(monitor->BGD_E1E5a);  //!< E1-E5a Broadcast Group Delay [s]
        monitor_.set_bgd_e1e5b_5(monitor->BGD_E1E5b);  //!< E1-E5b Broadcast Group Delay [s]

        monitor_.SerializeToString(&data);
        return data;
    }

    inline Galileo_Ephemeris readProtobuffer(const gnss_sdr::MonitorGalileoEphemeris& mon) const  //!< Deserialization
    {
        Galileo_Ephemeris monitor;

        monitor.PRN = mon.i_satellite_prn();

        monitor.IOD_ephemeris = mon.iod_ephemeris();
        monitor.IOD_nav = mon.iod_nav_1();
        monitor.M_0 = mon.m0_1();              //!< Mean anomaly at reference time [semi-circles]
        monitor.delta_n = mon.delta_n_3();     //!< Mean motion difference from computed value [semi-circles/sec]
        monitor.ecc = mon.e_1();               //!< Eccentricity
        monitor.sqrtA = mon.a_1();             //!< Square root of the semi-major axis [meters^1/2]
        monitor.OMEGA_0 = mon.omega_0_2();     //!< Longitude of ascending node of orbital plane at weekly epoch [semi-circles]
        monitor.i_0 = mon.i_0_2();             //!< Inclination angle at reference time  [semi-circles]
        monitor.omega = mon.omega_2();         //!< Argument of perigee [semi-circles]
        monitor.OMEGAdot = mon.omega_dot_3();  //!< Rate of right ascension [semi-circles/sec]
        monitor.idot = mon.idot_2();           //!< Rate of inclination angle [semi-circles/sec]
        monitor.Cuc = mon.c_uc_3();            //!< Amplitude of the cosine harmonic correction term to the argument of latitude [radians]
        monitor.Cus = mon.c_us_3();            //!< Amplitude of the sine harmonic correction term to the argument of latitude [radians]
        monitor.Crc = mon.c_rc_3();            //!< Amplitude of the cosine harmonic correction term to the orbit radius [meters]
        monitor.Crs = mon.c_rs_3();            //!< Amplitude of the sine harmonic correction term to the orbit radius [meters]
        monitor.Cic = mon.c_ic_4();            //!< Amplitude of the cosine harmonic correction term to the angle of inclination [radians]
        monitor.Cis = mon.c_is_4();            //!< Amplitude of the sine harmonic correction term to the angle of inclination [radians]
        monitor.toe = mon.d_toe();             // Ephemeris reference time

        /*Clock correction parameters*/
        monitor.toc = mon.d_toc();  // Clock correction data reference Time of Week
        monitor.af0 = mon.af0_4();  //!< SV clock bias correction coefficient [s]
        monitor.af1 = mon.af1_4();  //!< SV clock drift correction coefficient [s/s]
        monitor.af2 = mon.af2_4();  //!< SV clock drift rate correction coefficient [s/s^2]

        /*GST*/
        // Not belong to ephemeris set (page 1 to 4)
        monitor.WN = mon.wn_5();    //!< Week number
        monitor.tow = mon.tow_5();  //!< Time of Week
        monitor.satClkDrift = mon.galileo_satclkdrift();
        monitor.dtr = mon.galileo_dtr();  //!< relativistic clock correction term

        // SV status
        monitor.SISA = mon.sisa_3();
        monitor.E5a_HS = mon.e5a_hs();      //!< E5a Signal Health Status
        monitor.E5b_HS = mon.e5b_hs_5();    //!< E5b Signal Health Status
        monitor.E1B_HS = mon.e1b_hs_5();    //!< E1B Signal Health Status
        monitor.E5a_DVS = mon.e5a_dvs();    //!< E5a Data Validity Status
        monitor.E5b_DVS = mon.e5b_dvs_5();  //!< E5b Data Validity Status
        monitor.E1B_DVS = mon.e1b_dvs_5();  //!< E1B Data Validity Status

        monitor.BGD_E1E5a = mon.bgd_e1e5a_5();  //!< E1-E5a Broadcast Group Delay [s]
        monitor.BGD_E1E5b = mon.bgd_e1e5b_5();  //!< E1-E5b Broadcast Group Delay [s]

        return monitor;
    }

private:
    gnss_sdr::MonitorGalileoEphemeris monitor_{};
};


/** \} */
/** \} */
#endif  // GGNSS_SDR_SERDES_GALILEO_EPH_H
