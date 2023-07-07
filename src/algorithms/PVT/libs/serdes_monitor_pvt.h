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

#ifndef GNSS_SDR_SERDES_MONITOR_PVT_H
#define GNSS_SDR_SERDES_MONITOR_PVT_H

#include "monitor_pvt.h"
#include "monitor_pvt.pb.h"  // file created by Protocol Buffers at compile time
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
class Serdes_Monitor_Pvt
{
public:
    Serdes_Monitor_Pvt()
    {
        // Verify that the version of the library that we linked against is
        // compatible with the version of the headers we compiled against.
        GOOGLE_PROTOBUF_VERIFY_VERSION;
    }

    ~Serdes_Monitor_Pvt()
    {
        // google::protobuf::ShutdownProtobufLibrary();
    }

    inline Serdes_Monitor_Pvt(const Serdes_Monitor_Pvt& other) noexcept : monitor_(other.monitor_)  //!< Copy constructor
    {
    }

    inline Serdes_Monitor_Pvt& operator=(const Serdes_Monitor_Pvt& rhs) noexcept  //!< Copy assignment operator
    {
        if (this != &rhs)
            {
                this->monitor_.CopyFrom(rhs.monitor_);
            }
        return *this;
    }

    inline Serdes_Monitor_Pvt(Serdes_Monitor_Pvt&& other) noexcept : monitor_(std::move(other.monitor_))  //!< Move constructor
    {
        // Set the other object's monitor_ to a default-constructed state
        other.monitor_ = gnss_sdr::MonitorPvt{};
    }

    inline Serdes_Monitor_Pvt& operator=(Serdes_Monitor_Pvt&& other) noexcept  //!< Move assignment operator
    {
        if (this != &other)
            {
                this->monitor_ = std::move(other.monitor_);
            }
        return *this;
    }

    inline std::string createProtobuffer(const Monitor_Pvt* const monitor)  //!< Serialization into a string
    {
        monitor_.Clear();

        std::string data;

        monitor_.set_tow_at_current_symbol_ms(monitor->TOW_at_current_symbol_ms);
        monitor_.set_week(monitor->week);
        monitor_.set_rx_time(monitor->RX_time);
        monitor_.set_user_clk_offset(monitor->user_clk_offset);
        monitor_.set_pos_x(monitor->pos_x);
        monitor_.set_pos_y(monitor->pos_y);
        monitor_.set_pos_z(monitor->pos_z);
        monitor_.set_vel_x(monitor->vel_x);
        monitor_.set_vel_y(monitor->vel_y);
        monitor_.set_vel_z(monitor->vel_z);
        monitor_.set_cov_xx(monitor->cov_xx);
        monitor_.set_cov_yy(monitor->cov_yy);
        monitor_.set_cov_zz(monitor->cov_zz);
        monitor_.set_cov_xy(monitor->cov_xy);
        monitor_.set_cov_yz(monitor->cov_yz);
        monitor_.set_cov_zx(monitor->cov_zx);
        monitor_.set_latitude(monitor->latitude);
        monitor_.set_longitude(monitor->longitude);
        monitor_.set_height(monitor->height);
        monitor_.set_valid_sats(monitor->valid_sats);
        monitor_.set_solution_status(monitor->solution_status);
        monitor_.set_solution_type(monitor->solution_type);
        monitor_.set_ar_ratio_factor(monitor->AR_ratio_factor);
        monitor_.set_ar_ratio_threshold(monitor->AR_ratio_threshold);
        monitor_.set_gdop(monitor->gdop);
        monitor_.set_pdop(monitor->pdop);
        monitor_.set_hdop(monitor->hdop);
        monitor_.set_vdop(monitor->vdop);
        monitor_.set_user_clk_drift_ppm(monitor->user_clk_drift_ppm);

        monitor_.SerializeToString(&data);
        return data;
    }

    inline Monitor_Pvt readProtobuffer(const gnss_sdr::MonitorPvt& mon) const  //!< Deserialization
    {
        Monitor_Pvt monitor;

        monitor.TOW_at_current_symbol_ms = mon.tow_at_current_symbol_ms();
        monitor.week = mon.week();
        monitor.RX_time = mon.rx_time();
        monitor.user_clk_offset = mon.user_clk_offset();
        monitor.pos_x = mon.pos_x();
        monitor.pos_y = mon.pos_y();
        monitor.pos_z = mon.pos_z();
        monitor.vel_x = mon.vel_x();
        monitor.vel_y = mon.vel_y();
        monitor.vel_z = mon.vel_z();
        monitor.cov_xx = mon.cov_xx();
        monitor.cov_yy = mon.cov_yy();
        monitor.cov_zz = mon.cov_zz();
        monitor.cov_xy = mon.cov_xy();
        monitor.cov_yz = mon.cov_yz();
        monitor.cov_zx = mon.cov_zx();
        monitor.latitude = mon.latitude();
        monitor.longitude = mon.longitude();
        monitor.height = mon.height();
        monitor.valid_sats = static_cast<uint8_t>(mon.valid_sats());
        monitor.solution_status = static_cast<uint8_t>(mon.solution_status());
        monitor.solution_type = static_cast<uint8_t>(mon.solution_type());
        monitor.AR_ratio_factor = mon.ar_ratio_factor();
        monitor.AR_ratio_threshold = mon.ar_ratio_threshold();
        monitor.gdop = mon.gdop();
        monitor.pdop = mon.pdop();
        monitor.hdop = mon.hdop();
        monitor.vdop = mon.vdop();
        monitor.user_clk_drift_ppm = mon.user_clk_drift_ppm();

        return monitor;
    }

private:
    gnss_sdr::MonitorPvt monitor_{};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_SERDES_MONITOR_PVT_H
