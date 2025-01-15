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
#include "galileo_ephemeris.pb.h"  // file created by Protocol Buffers at compile time
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

    inline Serdes_Galileo_Eph(const Serdes_Galileo_Eph& other) noexcept : monitor_(other.monitor_)  //!< Copy constructor
    {
    }

    inline Serdes_Galileo_Eph& operator=(const Serdes_Galileo_Eph& rhs) noexcept  //!< Copy assignment operator
    {
        Serdes_Galileo_Eph temp(rhs);
        std::swap(this->monitor_, temp.monitor_);
        return *this;
    }

    inline Serdes_Galileo_Eph(Serdes_Galileo_Eph&& other) noexcept : monitor_(std::move(other.monitor_))  //!< Move constructor
    {
    }

    inline Serdes_Galileo_Eph& operator=(Serdes_Galileo_Eph&& other) noexcept  //!< Move assignment operator
    {
        std::swap(this->monitor_, other.monitor_);
        return *this;
    }

    inline std::string createProtobuffer(const std::shared_ptr<Galileo_Ephemeris> monitor)  //!< Serialization into a string
    {
        monitor_.Clear();

        std::string data;

        monitor_.set_prn(monitor->PRN);
        monitor_.set_m_0(monitor->M_0);
        monitor_.set_delta_n(monitor->delta_n);
        monitor_.set_ecc(monitor->ecc);
        monitor_.set_sqrta(monitor->sqrtA);
        monitor_.set_omega_0(monitor->OMEGA_0);
        monitor_.set_i_0(monitor->i_0);
        monitor_.set_omega(monitor->omega);
        monitor_.set_omegadot(monitor->OMEGAdot);
        monitor_.set_idot(monitor->idot);
        monitor_.set_cuc(monitor->Cuc);
        monitor_.set_cus(monitor->Cus);
        monitor_.set_crc(monitor->Crc);
        monitor_.set_crs(monitor->Crs);
        monitor_.set_cic(monitor->Cic);
        monitor_.set_cis(monitor->Cis);
        monitor_.set_toe(monitor->toe);
        monitor_.set_toc(monitor->toc);
        monitor_.set_af0(monitor->af0);
        monitor_.set_af1(monitor->af1);
        monitor_.set_af2(monitor->af2);
        monitor_.set_satclkdrift(monitor->satClkDrift);
        monitor_.set_dtr(monitor->dtr);
        monitor_.set_wn(monitor->WN);
        monitor_.set_tow(monitor->tow);

        // Galileo-specific parameters
        monitor_.set_iod_ephemeris(monitor->IOD_ephemeris);
        monitor_.set_iod_nav(monitor->IOD_nav);
        monitor_.set_sisa(monitor->SISA);
        monitor_.set_e5a_hs(monitor->E5a_HS);
        monitor_.set_e5b_hs(monitor->E5b_HS);
        monitor_.set_e1b_hs(monitor->E1B_HS);
        monitor_.set_e5a_dvs(monitor->E5a_DVS);
        monitor_.set_e5b_dvs(monitor->E5b_DVS);
        monitor_.set_e1b_dvs(monitor->E1B_DVS);
        monitor_.set_bgd_e1e5a(monitor->BGD_E1E5a);
        monitor_.set_bgd_e1e5b(monitor->BGD_E1E5b);

        monitor_.SerializeToString(&data);
        return data;
    }

    inline Galileo_Ephemeris readProtobuffer(const gnss_sdr::GalileoEphemeris& mon) const  //!< Deserialization
    {
        Galileo_Ephemeris monitor;

        monitor.PRN = mon.prn();
        monitor.M_0 = mon.m_0();
        monitor.delta_n = mon.delta_n();
        monitor.ecc = mon.ecc();
        monitor.sqrtA = mon.sqrta();
        monitor.OMEGA_0 = mon.omega_0();
        monitor.i_0 = mon.i_0();
        monitor.omega = mon.omega();
        monitor.OMEGAdot = mon.omegadot();
        monitor.idot = mon.idot();
        monitor.Cuc = mon.cuc();
        monitor.Cus = mon.cus();
        monitor.Crc = mon.crc();
        monitor.Crs = mon.crs();
        monitor.Cic = mon.cic();
        monitor.Cis = mon.cis();
        monitor.toe = mon.toe();
        monitor.toc = mon.toc();
        monitor.af0 = mon.af0();
        monitor.af1 = mon.af1();
        monitor.af2 = mon.af2();
        monitor.satClkDrift = mon.satclkdrift();
        monitor.dtr = mon.dtr();
        monitor.WN = mon.wn();
        monitor.tow = mon.tow();

        // Galileo-specific parameters
        monitor.IOD_ephemeris = mon.iod_ephemeris();
        monitor.IOD_nav = mon.iod_nav();
        monitor.SISA = mon.sisa();
        monitor.E5a_HS = mon.e5a_hs();
        monitor.E5b_HS = mon.e5b_hs();
        monitor.E1B_HS = mon.e1b_hs();
        monitor.E5a_DVS = mon.e5a_dvs();
        monitor.E5b_DVS = mon.e5b_dvs();
        monitor.E1B_DVS = mon.e1b_dvs();
        monitor.BGD_E1E5a = mon.bgd_e1e5a();
        monitor.BGD_E1E5b = mon.bgd_e1e5b();

        return monitor;
    }

private:
    gnss_sdr::GalileoEphemeris monitor_{};
};


/** \} */
/** \} */
#endif  // GGNSS_SDR_SERDES_GALILEO_EPH_H
