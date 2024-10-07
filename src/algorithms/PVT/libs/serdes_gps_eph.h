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
#include "gps_ephemeris.pb.h"  // file created by Protocol Buffers at compile time
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

    inline Serdes_Gps_Eph(const Serdes_Gps_Eph& other) noexcept : monitor_(other.monitor_)  //!< Copy constructor
    {
    }

    inline Serdes_Gps_Eph& operator=(const Serdes_Gps_Eph& rhs) noexcept  //!< Copy assignment operator
    {
        Serdes_Gps_Eph temp(rhs);
        std::swap(this->monitor_, temp.monitor_);
        return *this;
    }

    inline Serdes_Gps_Eph(Serdes_Gps_Eph&& other) noexcept : monitor_(std::move(other.monitor_))  //!< Move constructor
    {
    }

    inline Serdes_Gps_Eph& operator=(Serdes_Gps_Eph&& other) noexcept  //!< Move assignment operator
    {
        std::swap(this->monitor_, other.monitor_);
        return *this;
    }

    inline std::string createProtobuffer(const std::shared_ptr<Gps_Ephemeris> monitor)  //!< Serialization into a string
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

        // GPS-specific parameters
        monitor_.set_code_on_l2(monitor->code_on_L2);
        monitor_.set_l2_p_data_flag(monitor->L2_P_data_flag);
        monitor_.set_sv_accuracy(monitor->SV_accuracy);
        monitor_.set_sv_health(monitor->SV_health);
        monitor_.set_tgd(monitor->TGD);
        monitor_.set_iodc(monitor->IODC);
        monitor_.set_iode_sf2(monitor->IODE_SF2);
        monitor_.set_iode_sf3(monitor->IODE_SF3);
        monitor_.set_aodo(monitor->AODO);
        monitor_.set_fit_interval_flag(monitor->fit_interval_flag);
        monitor_.set_spare1(monitor->spare1);
        monitor_.set_spare2(monitor->spare2);
        monitor_.set_integrity_status_flag(monitor->integrity_status_flag);
        monitor_.set_alert_flag(monitor->alert_flag);
        monitor_.set_antispoofing_flag(monitor->antispoofing_flag);

        monitor_.SerializeToString(&data);
        return data;
    }

    inline Gps_Ephemeris readProtobuffer(const gnss_sdr::GpsEphemeris& mon) const  //!< Deserialization
    {
        Gps_Ephemeris monitor;

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

        // GPS-specific parameters
        monitor.code_on_L2 = mon.code_on_l2();
        monitor.L2_P_data_flag = mon.l2_p_data_flag();
        monitor.SV_accuracy = mon.sv_accuracy();
        monitor.SV_health = mon.sv_health();
        monitor.TGD = mon.tgd();
        monitor.IODC = mon.iodc();
        monitor.IODE_SF2 = mon.iode_sf2();
        monitor.IODE_SF3 = mon.iode_sf3();
        monitor.AODO = mon.aodo();
        monitor.fit_interval_flag = mon.fit_interval_flag();
        monitor.spare1 = mon.spare1();
        monitor.spare2 = mon.spare2();
        monitor.integrity_status_flag = mon.integrity_status_flag();
        monitor.alert_flag = mon.alert_flag();
        monitor.antispoofing_flag = mon.antispoofing_flag();

        return monitor;
    }

private:
    gnss_sdr::GpsEphemeris monitor_{};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_SERDES_GPS_EPH_H
