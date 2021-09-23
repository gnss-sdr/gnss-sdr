/*!
 * \file rtklib_pvt.h
 * \brief Interface of a Position Velocity and Time computation block
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_RTKLIB_PVT_H
#define GNSS_SDR_RTKLIB_PVT_H

#include "gnss_synchro.h"
#include "pvt_interface.h"           // for PvtInterface
#include "rtklib.h"                  // for rtk_t
#include "rtklib_pvt_gs.h"           // for rtklib_pvt_gs_sptr
#include <gnuradio/gr_complex.h>     // for gr_complex
#include <gnuradio/runtime_types.h>  // for basic_block_sptr, top_block_sptr
#include <cstddef>                   // for size_t
#include <ctime>                     // for time_t
#include <map>                       // for map
#include <string>                    // for string

/** \addtogroup PVT
 * Computation of Position, Velocity and Time from GNSS observables.
 * \{ */
/** \addtogroup PVT_adapters pvt_adapters
 * Wrap GNU Radio PVT solvers with a PvtInterface
 * \{ */

class ConfigurationInterface;
class Galileo_Almanac;
class Galileo_Ephemeris;
class Gps_Almanac;
class Gps_Ephemeris;

/*!
 * \brief This class implements a PvtInterface for the RTKLIB PVT block
 *
 * Global configuration options used:
 *
 * GNSS-SDR.pre_2009_file - flag indicating a file older than 2009 rollover should be processed (false)
 * GNSS-SDR.observable_interval_ms - (20)
 *
 * It supports the following configuration options:
 *
 *  .dump - (false)
 *  .dump_filename - ("./pvt.dat")
 *  .dump_mat - (true)
 *  .rtk_trace_level - debug level for the RTKLIB methods (0)
 *
 *  .output_rate_ms - (500)
 *                    Note that the actual rate is the least common multiple of this value and GNSS-SDR.observable_interval_ms
 *  .display_rate_ms - (500)
 *
 *  .flag_nmea_tty_port - (false)
 *  .nmea_dump_filename - ("./nmea_pvt.nmea")
 *  .nmea_dump_devname - ("/dev/tty1")
 *
 *  .rinex_version - (3) overridden by -RINEX_version=n.nn command line argument
 *  .rinexobs_rate_ms - rate at which RINEX observations are written (1000).  Note that
 *                      the actual rate is the least common multiple of this value and
 *                      .output_rate_ms
 *  .rinex_name - (-RINEX_name command-line argument)
 *
 *  .flag_rtcm_tty_port - (false)
 *  .rtcm_dump_devname - ("/dev/pts/1")
 *  .flag_rtcm_server - (false)
 *  .rtcm_tcp_port - (2101)
 *  .rtcm_station_id - (1234)
 * Output rates ... all values are LCM with the computed output rate (above)
 *  .rtcm_MT1019_rate_ms - (5000)
 *  .rtcm_MT1020_rate_ms - (5000)
 *  .rtcm_MT1045_rate_ms - (5000)
 *  .rtcm_MSM_rate_ms - (1000)
 *  .rtcm_MT1077_rate_ms - (.rtcm_MSM_rate_ms)
 *  .rtcm_MT1087_rate_ms - (.rtcm_MSM_rate_ms)
 *  .rtcm_MT1097_rate_ms - (.rtcm_MSM_rate_ms)
 *
 *  .kml_rate_ms - (1000)
 *  .gpx_rate_ms - (1000)
 *  .geojson_rate_ms - (1000)
 *  .nmea_rate_ms - (1000)
 *
 *  .positioning_mode - The RTKLIB positioning mode. ("Single") Supported values are "Single",
 *                      "Static", "Kinematic", "PPP_Static" and "PPP_Kinematic". Unsupported modes
 *                      include DGPS/DGNSS, Moving Baseline, Fixed, and PPP-fixed
 *  .num_bands - number of frequencies to use, between 1 and 3. Default is based on the channels configured
 *  .elevation_mask - (15.0). Value must be in the range [0,90.0]
 *  .dynamics_model - (0) 0:none, 1:velocity, 2:acceleration

 *  .iono_model - ("OFF"). Supported values are "OFF", "Broadcast", "SBAS", "Iono-Free-LC",
 *                      "Estimate_STEC", "IONEX". Unsupported values include QZSS broadcast, QZSS
 *                      LEX, and SLANT TEC.
 *  .trop_model - ("OFF"). Supported values are "OFF", "Saastamoinen", "SBAS", "Estimate_ZTD", and
 *                      "Estimate_ZTD_Grad". Unsupported values include ZTD correction and ZTD+grad
 *                      correction
 *  .phwindup - phase windup correction for PPP modes (0)
 *  .reject_GPS_IIA - whether the GPS Block IIA satellites in eclipse are excluded (0). Only applies in PPP-* modes
 *  .raim_fde - whether RAIM (receiver autonomous integrity monitoring) FDE (fault detection and exclusion) is enabled (0)
 *  .earth_tide - (0)
 *  .navigation_system - mask of navigation systems to use. Default based on configured channels
 *                      0x01:GPS, 0x02:SBAS, 0x04:GLONASS, 0x08:Galileo, 0x10:QZSS, 0x20:BeiDou,
 *                      0x40:IRNS, 0x80:LEO
 *
 *  .AR_GPS - Ambiguity Resolution mode for GPS ("Continuous"). Supported values are "OFF",
 *                      "Continuous", "Instantaneous", "Fix-and-Hold", "PPP-AR". Unsupported values
 *                      include PPP-AR ILS, WLNL, and TCAR.
 *  .AR_GLO - Ambiguity Resolution mode for GLONASS (1). Value must be in the range [0,3]. (0:off,1:on,2:auto cal,3:ext cal)
 *  .AR_DBS - Ambiguity Resolution Mode for BeiDou (1). Value must be in the range [0,1]. (0:off,1:on)
 *  .min_ratio_to_fix_ambiguity - (3.0)
 *  .min_lock_to_fix_ambiguity - (0)
 *  .min_elevation_to_fix_ambiguity - minimum elevation (deg) to fix integer ambiguity (0.0)
 *  .outage_reset_ambiguity - (5)
 *  .slip_threshold - (0.05)
 *  .threshold_reject_gdop - if GDOP is over this value, the observable is excluded (30.0)
 *  .threshold_reject_innovation - if innovation is over this value, the observable is excluded (30.0)
 *  .number_filter_iter - number of iterations for the estimation filter (1)
 *  .bias_0 - (30.0)
 *  .iono_0 - (0.03)
 *  .trop_0 - (0.3)
 *  .sigma_bias - process noise stddev of carrier-phase bias(ambiguity)(cycle/sqrt(s)) (1e-4)
 *  .sigma_iono - process noise stddev of vertical ionospheric delay per 10km baseline (m/sqrt(s)) (1e-3)
 *  .sigma_trop - process noise stddev of zenith tropospheric delay (m/sqrt(s)) (1e-4)
 *  .sigma_acch - process noise stddev of the receiver acceleration horizontal component (m/s2/sqrt(s)) (1e-1)
 *  .sigma_accv - process noise stddev of the receiver acceleration vertical component (m/s2/sqrt(s)) (1e-2)
 *  .sigma_pos - (0.0)
 *  .code_phase_error_ratio_l1 - (100.0)
 *  .code_phase_error_ratio_l2 - (100.0)
 *  .code_phase_error_ratio_l5 - (100.0)
 *  .carrier_phase_error_factor_a - (0.003)
 *  .carrier_phase_error_factor_b - (0.003)
 *
 *  .output_enabled - (true)
 *  .rinex_output_enabled - (.output_enabled)
 *  .gpx_output_enabled - (.output_enabled)
 *  .geojson_output_enabled - (.output_enabled)
 *  .kml_output_enabled - (.output_enabled)
 *  .xml_output_enabled - (.output_enabled)
 *  .nmea_output_enabled - (.output_enabled)
 *  .rtcm_output_enabled - (false)

 *  .output_path - directory to which output files are written (".")
 *  .rinex_output_path - (.output_path)
 *  .gpx_output_path - (.output_path)
 *  .geojson_output_path - (.output_path)
 *  .kml_output_path - (.output_path)
 *  .xml_output_path - (.output_path)
 *  .nmea_output_path - (.output_path)
 *  .rtcm_output_path - (.output_path)
 *
 *  .enable_monitor - enable the PVT monitor (false)
 *  .monitor_client_addresses - ("127.0.0.1")
 *  .monitor_udp_port - DO NOT USE THE DEFAULT (1234)
 *  .enable_protobuf - serialize using protocol buffers (true). Monitor.enable_protobuf if true, sets this to true
 *
 *  .enable_monitor_ephemeris - enable the ephemeris monitor (false)
 *  .monitor_ephemeris_client_addresses - ("127.0.0.1")
 *  .monitor_ephemeris_udp_port - DO NOT USE THE DEFAULT (1234)
 *
 *  .show_local_time_zone - (false)
 *  .enable_rx_clock_correction - (false)
 *  .max_clock_offset_ms - (40)
 */
class Rtklib_Pvt : public PvtInterface
{
public:
    Rtklib_Pvt(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    virtual ~Rtklib_Pvt();

    inline std::string role() override
    {
        return role_;
    }

    //!  Returns "RTKLIB_PVT"
    inline std::string implementation() override
    {
        return "RTKLIB_PVT";
    }

    void clear_ephemeris() override;
    std::map<int, Gps_Ephemeris> get_gps_ephemeris() const override;
    std::map<int, Galileo_Ephemeris> get_galileo_ephemeris() const override;
    std::map<int, Gps_Almanac> get_gps_almanac() const override;
    std::map<int, Galileo_Almanac> get_galileo_almanac() const override;

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    inline void reset() override
    {
        return;
    }

    //! All blocks must have an item_size() function implementation
    inline size_t item_size() override
    {
        return sizeof(Gnss_Synchro);
    }

    bool get_latest_PVT(double* longitude_deg,
        double* latitude_deg,
        double* height_m,
        double* ground_speed_kmh,
        double* course_over_ground_deg,
        time_t* UTC_time) override;

private:
    rtklib_pvt_gs_sptr pvt_;
    rtk_t rtk{};
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_RTKLIB_PVT_H
