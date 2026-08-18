/*!
 * \file rtklib_pvt.cc
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


#include "rtklib_pvt.h"
#include "MATH_CONSTANTS.h"            // for D2R
#include "SBAS_L1.h"                   // for SBAS PRN limits
#include "configuration_interface.h"   // for ConfigurationInterface
#include "galileo_almanac.h"           // for Galileo_Almanac
#include "galileo_ephemeris.h"         // for Galileo_Ephemeris
#include "gnss_sdr_flags.h"            // for FLAGS_RINEX_version
#include "gnss_sdr_string_literals.h"  // for std::string_literals in C++11
#include "gps_almanac.h"               // for Gps_Almanac
#include "gps_ephemeris.h"             // for Gps_Ephemeris
#include "gps_week_rollover.h"         // for gps_ref_week_from_config
#include "ntrip_rtcm_client.h"         // for make_ntrip_rtcm_client_config, secure_clear_string
#include "ntrip_rtk_signals.h"         // for the NTRIP RTK signal table and derived masks
#include "pvt_conf.h"                  // for Pvt_Conf
#include "rtklib_rtkpos.h"             // for rtkfree, rtkinit
#include "signal_enabled_flags.h"      // for signal_enabled_flags
#include <algorithm>                   // for any_of
#include <cctype>                      // for isspace, iscntrl
#include <cmath>                       // for isfinite
#include <cstdlib>                     // for getenv
#include <iostream>                    // for std::cout
#include <stdexcept>                   // for invalid_argument
#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif
#if USE_STD_COMMON_FACTOR
#include <numeric>
namespace bc = std;
#else
#if USE_OLD_BOOST_MATH_COMMON_FACTOR
#include <boost/math/common_factor_rt.hpp>
namespace bc = boost::math;
#else
#include <boost/integer/common_factor_rt.hpp>
namespace bc = boost::integer;
#endif
#endif

using namespace std::string_literals;

namespace
{
class Ntrip_Credential_Guard
{
public:
    explicit Ntrip_Credential_Guard(Pvt_Conf* configuration) noexcept
        : d_configuration(configuration)
    {
    }

    ~Ntrip_Credential_Guard()
    {
        secure_clear_ntrip_credentials(d_configuration);
    }

    Ntrip_Credential_Guard(const Ntrip_Credential_Guard&) = delete;
    Ntrip_Credential_Guard& operator=(const Ntrip_Credential_Guard&) = delete;
    Ntrip_Credential_Guard(Ntrip_Credential_Guard&&) = delete;
    Ntrip_Credential_Guard& operator=(Ntrip_Credential_Guard&&) = delete;

private:
    Pvt_Conf* d_configuration;
};
}  // namespace


Rtklib_Pvt::Rtklib_Pvt(const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : role_(role),
                                in_streams_(in_streams),
                                out_streams_(out_streams)
{
    Pvt_Conf pvt_output_parameters = Pvt_Conf();
    Ntrip_Credential_Guard ntrip_credential_guard(&pvt_output_parameters);
    // dump parameters
    const std::string default_dump_filename("./pvt.dat");
    const std::string default_nmea_dump_filename("./nmea_pvt.nmea");
    const std::string default_nmea_dump_devname("/dev/tty1");
    const std::string default_rtcm_dump_devname("/dev/pts/1");
    DLOG(INFO) << "role " << role;
    pvt_output_parameters.dump = configuration->property(role + ".dump", false);
    pvt_output_parameters.dump_filename = configuration->property(role + ".dump_filename", default_dump_filename);
    pvt_output_parameters.dump_mat = configuration->property(role + ".dump_mat", true);

    pvt_output_parameters.rtk_trace_level = configuration->property(role + ".rtk_trace_level"s, 0);

    // Approximate date of the signal capture, used to resolve the GPS mod-1024
    // week-number rollover when post-processing recorded files
    pvt_output_parameters.ref_gps_week = gps_ref_week_from_config(
        configuration->property("GNSS-SDR.observation_date", std::string("")),
        configuration->property("GNSS-SDR.pre_2009_file", false));

    // output rate
    pvt_output_parameters.observable_interval_ms = configuration->property("GNSS-SDR.observable_interval_ms", pvt_output_parameters.observable_interval_ms);

    pvt_output_parameters.output_rate_ms = bc::lcm(static_cast<int>(pvt_output_parameters.observable_interval_ms), configuration->property(role + ".output_rate_ms", 500));

    // display rate
    pvt_output_parameters.display_rate_ms = bc::lcm(pvt_output_parameters.output_rate_ms, configuration->property(role + ".display_rate_ms", 500));

    // PVT KF settings
    pvt_output_parameters.enable_pvt_kf = configuration->property(role + ".enable_pvt_kf", false);
    pvt_output_parameters.measures_ecef_pos_sd_m = configuration->property(role + ".kf_measures_ecef_pos_sd_m", 1.0);
    pvt_output_parameters.measures_ecef_vel_sd_ms = configuration->property(role + ".kf_measures_ecef_vel_sd_ms", 0.1);
    pvt_output_parameters.system_ecef_pos_sd_m = configuration->property(role + ".kf_system_ecef_pos_sd_m", 2.0);
    pvt_output_parameters.system_ecef_vel_sd_ms = configuration->property(role + ".kf_system_ecef_vel_sd_ms", 0.5);

    // PVT Sensor data settings
    SensorDataSourceConfiguration sensor_data_configuration{configuration};
    pvt_output_parameters.kf_use_imu_vel = configuration->property(role + ".kf_use_imu_vel", false);

    // NMEA Printer settings
    pvt_output_parameters.flag_nmea_tty_port = configuration->property(role + ".flag_nmea_tty_port", false);
    pvt_output_parameters.nmea_dump_filename = configuration->property(role + ".nmea_dump_filename", default_nmea_dump_filename);
    pvt_output_parameters.nmea_dump_devname = configuration->property(role + ".nmea_dump_devname", default_nmea_dump_devname);

    // RINEX version
    pvt_output_parameters.rinex_version = configuration->property(role + ".rinex_version", 3);
#if USE_GLOG_AND_GFLAGS
    if (FLAGS_RINEX_version == "4.00" || FLAGS_RINEX_version == "4.01" || FLAGS_RINEX_version == "4.02" || FLAGS_RINEX_version == "4")
        {
            pvt_output_parameters.rinex_version = 4;
        }
    else if (FLAGS_RINEX_version == "3.01" || FLAGS_RINEX_version == "3.02" || FLAGS_RINEX_version == "3")
        {
            pvt_output_parameters.rinex_version = 3;
        }
    else if (FLAGS_RINEX_version == "2.10" || FLAGS_RINEX_version == "2.11" || FLAGS_RINEX_version == "2")
        {
            pvt_output_parameters.rinex_version = 2;
        }
#else
    if (absl::GetFlag(FLAGS_RINEX_version) == "4.00" || absl::GetFlag(FLAGS_RINEX_version) == "4.01" || absl::GetFlag(FLAGS_RINEX_version) == "4.02" || absl::GetFlag(FLAGS_RINEX_version) == "4")
        {
            pvt_output_parameters.rinex_version = 4;
        }
    else if (absl::GetFlag(FLAGS_RINEX_version) == "3.01" || absl::GetFlag(FLAGS_RINEX_version) == "3.02" || absl::GetFlag(FLAGS_RINEX_version) == "3")
        {
            pvt_output_parameters.rinex_version = 3;
        }
    else if (absl::GetFlag(FLAGS_RINEX_version) == "2.10" || absl::GetFlag(FLAGS_RINEX_version) == "2.11" || absl::GetFlag(FLAGS_RINEX_version) == "2")
        {
            pvt_output_parameters.rinex_version = 2;
        }
#endif
    pvt_output_parameters.rinexobs_rate_ms = bc::lcm(configuration->property(role + ".rinexobs_rate_ms", 1000), pvt_output_parameters.output_rate_ms);
    pvt_output_parameters.rinex_name = configuration->property(role + ".rinex_name", std::string("-"));
#if USE_GLOG_AND_GFLAGS
    if (FLAGS_RINEX_name != "-")
        {
            pvt_output_parameters.rinex_name = FLAGS_RINEX_name;
        }
#else
    if (absl::GetFlag(FLAGS_RINEX_name) != "-")
        {
            pvt_output_parameters.rinex_name = absl::GetFlag(FLAGS_RINEX_name);
        }
#endif
    // RTCM Printer settings
    pvt_output_parameters.flag_rtcm_tty_port = configuration->property(role + ".flag_rtcm_tty_port", false);
    pvt_output_parameters.rtcm_dump_devname = configuration->property(role + ".rtcm_dump_devname", default_rtcm_dump_devname);
    pvt_output_parameters.flag_rtcm_server = configuration->property(role + ".flag_rtcm_server", false);
    pvt_output_parameters.rtcm_tcp_port = configuration->property(role + ".rtcm_tcp_port", 2101);
    pvt_output_parameters.rtcm_station_id = configuration->property(role + ".rtcm_station_id", 1234);
    // RTCM message rates: least common multiple with output_rate_ms
    const int rtcm_MT1019_rate_ms = bc::lcm(configuration->property(role + ".rtcm_MT1019_rate_ms", 5000), pvt_output_parameters.output_rate_ms);
    const int rtcm_MT1020_rate_ms = bc::lcm(configuration->property(role + ".rtcm_MT1020_rate_ms", 5000), pvt_output_parameters.output_rate_ms);
    const int rtcm_MT1045_rate_ms = bc::lcm(configuration->property(role + ".rtcm_MT1045_rate_ms", 5000), pvt_output_parameters.output_rate_ms);
    const int rtcm_MSM_rate_ms = bc::lcm(configuration->property(role + ".rtcm_MSM_rate_ms", 1000), pvt_output_parameters.output_rate_ms);
    const int rtcm_MT1077_rate_ms = bc::lcm(configuration->property(role + ".rtcm_MT1077_rate_ms", rtcm_MSM_rate_ms), pvt_output_parameters.output_rate_ms);
    const int rtcm_MT1087_rate_ms = bc::lcm(configuration->property(role + ".rtcm_MT1087_rate_ms", rtcm_MSM_rate_ms), pvt_output_parameters.output_rate_ms);
    const int rtcm_MT1097_rate_ms = bc::lcm(configuration->property(role + ".rtcm_MT1097_rate_ms", rtcm_MSM_rate_ms), pvt_output_parameters.output_rate_ms);

    pvt_output_parameters.rtcm_msg_rate_ms[1019] = rtcm_MT1019_rate_ms;
    pvt_output_parameters.rtcm_msg_rate_ms[1020] = rtcm_MT1020_rate_ms;
    pvt_output_parameters.rtcm_msg_rate_ms[1045] = rtcm_MT1045_rate_ms;
    for (int k = 1071; k < 1078; k++)  // All GPS MSM
        {
            pvt_output_parameters.rtcm_msg_rate_ms[k] = rtcm_MT1077_rate_ms;
        }
    for (int k = 1081; k < 1088; k++)  // All GLONASS MSM
        {
            pvt_output_parameters.rtcm_msg_rate_ms[k] = rtcm_MT1087_rate_ms;
        }
    for (int k = 1091; k < 1098; k++)  // All Galileo MSM
        {
            pvt_output_parameters.rtcm_msg_rate_ms[k] = rtcm_MT1097_rate_ms;
        }

    // NTRIP client settings. These properties are intentionally ignored unless
    // the client is enabled so existing configurations retain their exact
    // startup and data-flow behavior.
    pvt_output_parameters.ntrip_client_enabled = configuration->property(role + ".ntrip_client_enabled", false);
    if (pvt_output_parameters.ntrip_client_enabled)
        {
            pvt_output_parameters.ntrip_caster_address = configuration->property(role + ".ntrip_caster_address", std::string(""));
            pvt_output_parameters.ntrip_mountpoint = configuration->property(role + ".ntrip_mountpoint", std::string(""));
            pvt_output_parameters.ntrip_username = configuration->property(role + ".ntrip_username", std::string(""));
            pvt_output_parameters.ntrip_password = configuration->property(role + ".ntrip_password", std::string(""));
            pvt_output_parameters.ntrip_password_env = configuration->property(role + ".ntrip_password_env", std::string(""));

            const int ntrip_port = configuration->property(role + ".ntrip_caster_port", 2101);
            const int ntrip_station_id = configuration->property(role + ".ntrip_station_id", 0);
            pvt_output_parameters.ntrip_timeout_ms = configuration->property(role + ".ntrip_inactivity_timeout_ms", pvt_output_parameters.ntrip_timeout_ms);
            pvt_output_parameters.ntrip_reconnect_interval_ms = configuration->property(role + ".ntrip_reconnect_interval_ms", pvt_output_parameters.ntrip_reconnect_interval_ms);
            pvt_output_parameters.ntrip_max_correction_age_s = configuration->property(role + ".ntrip_max_correction_age_s", pvt_output_parameters.ntrip_max_correction_age_s);
            pvt_output_parameters.ntrip_fallback_to_single = configuration->property(role + ".ntrip_fallback_to_single", pvt_output_parameters.ntrip_fallback_to_single);
            pvt_output_parameters.ntrip_tls_enabled = configuration->property(role + ".ntrip_tls_enabled", pvt_output_parameters.ntrip_tls_enabled);
            pvt_output_parameters.ntrip_version = configuration->property(role + ".ntrip_version", pvt_output_parameters.ntrip_version);
            pvt_output_parameters.ntrip_send_gga = configuration->property(role + ".ntrip_send_gga", pvt_output_parameters.ntrip_send_gga);
            pvt_output_parameters.ntrip_gga_period_ms = configuration->property(role + ".ntrip_gga_period_ms", pvt_output_parameters.ntrip_gga_period_ms);

            if (ntrip_port < 1 || ntrip_port > 65535)
                {
                    throw std::invalid_argument(role + ".ntrip_caster_port must be in the range 1..65535");
                }
            if (ntrip_station_id < 0 || ntrip_station_id > 4095)
                {
                    throw std::invalid_argument(role + ".ntrip_station_id must be in the range 0..4095");
                }
            pvt_output_parameters.ntrip_port = static_cast<uint16_t>(ntrip_port);
            pvt_output_parameters.ntrip_station_id = static_cast<uint16_t>(ntrip_station_id);

            const auto has_space_or_control = [](const std::string& value) {
                return std::any_of(value.cbegin(), value.cend(), [](char character) {
                    const auto byte = static_cast<unsigned char>(character);
                    return std::isspace(byte) != 0 || std::iscntrl(byte) != 0;
                });
            };
            if (!pvt_output_parameters.ntrip_password_env.empty())
                {
                    if (!pvt_output_parameters.ntrip_password.empty())
                        {
                            throw std::invalid_argument(role + ".ntrip_password and .ntrip_password_env are mutually exclusive");
                        }
                    if (has_space_or_control(pvt_output_parameters.ntrip_password_env.value()))
                        {
                            throw std::invalid_argument(role + ".ntrip_password_env must be one environment-variable name");
                        }
                    const char* password = std::getenv(pvt_output_parameters.ntrip_password_env.value().c_str());
                    if (password == nullptr)
                        {
                            throw std::invalid_argument(role + ".ntrip_password_env names an environment variable that is not set");
                        }
                    pvt_output_parameters.ntrip_password = password;
                }
            // the client library owns the configuration rules; the adapter
            // only turns the verdict into a constructor-time throw
            Ntrip_Rtcm_Client_Config ntrip_probe = make_ntrip_rtcm_client_config(pvt_output_parameters);
            const std::string ntrip_error = ntrip_probe.validate();
            secure_clear_string(&ntrip_probe.username);
            secure_clear_string(&ntrip_probe.password);
            if (!ntrip_error.empty())
                {
                    throw std::invalid_argument(role + " NTRIP configuration: " + ntrip_error);
                }
        }

    // Advanced Nativation Protocol Printer settings
    pvt_output_parameters.an_output_enabled = configuration->property(role + ".an_output_enabled", pvt_output_parameters.an_output_enabled);
    pvt_output_parameters.an_dump_devname = configuration->property(role + ".an_dump_devname", default_nmea_dump_devname);
    if (pvt_output_parameters.an_output_enabled && pvt_output_parameters.flag_nmea_tty_port)
        {
            if (pvt_output_parameters.nmea_dump_devname == pvt_output_parameters.an_dump_devname)
                {
                    std::cerr << "Warning: NMEA an Advanced Nativation printers set to write to the same serial port.\n"
                              << "Please make sure that PVT.an_dump_devname and PVT.an_dump_devname are different.\n"
                              << "Shutting down the NMEA serial output.\n";
                    pvt_output_parameters.flag_nmea_tty_port = false;
                }
        }
    if (pvt_output_parameters.an_output_enabled && pvt_output_parameters.flag_rtcm_tty_port)
        {
            if (pvt_output_parameters.rtcm_dump_devname == pvt_output_parameters.an_dump_devname)
                {
                    std::cerr << "Warning: RTCM an Advanced Nativation printers set to write to the same serial port.\n"
                              << "Please make sure that PVT.an_dump_devname and .rtcm_dump_devname are different.\n"
                              << "Shutting down the RTCM serial output.\n";
                    pvt_output_parameters.flag_rtcm_tty_port = false;
                }
        }

    pvt_output_parameters.kml_rate_ms = bc::lcm(configuration->property(role + ".kml_rate_ms", pvt_output_parameters.kml_rate_ms), pvt_output_parameters.output_rate_ms);
    pvt_output_parameters.gpx_rate_ms = bc::lcm(configuration->property(role + ".gpx_rate_ms", pvt_output_parameters.gpx_rate_ms), pvt_output_parameters.output_rate_ms);
    pvt_output_parameters.geojson_rate_ms = bc::lcm(configuration->property(role + ".geojson_rate_ms", pvt_output_parameters.geojson_rate_ms), pvt_output_parameters.output_rate_ms);
    pvt_output_parameters.nmea_rate_ms = bc::lcm(configuration->property(role + ".nmea_rate_ms", pvt_output_parameters.nmea_rate_ms), pvt_output_parameters.output_rate_ms);
    pvt_output_parameters.an_rate_ms = configuration->property(role + ".an_rate_ms", pvt_output_parameters.an_rate_ms);

    const Signal_Enabled_Flags signal_enabled_flags(configuration);
    pvt_output_parameters.signal_enabled_flags = signal_enabled_flags.flags;

    // RTKLIB PVT solver options
    // Settings 1
    int positioning_mode = -1;
    const std::string default_pos_mode("Single");
    const std::string positioning_mode_str = configuration->property(role + ".positioning_mode", default_pos_mode);  // (PMODE_XXX) see src/algorithms/libs/rtklib/rtklib.h
    if (positioning_mode_str == "Single")
        {
            positioning_mode = PMODE_SINGLE;
        }
    if (positioning_mode_str == "Static")
        {
            positioning_mode = PMODE_STATIC;
        }
    if (positioning_mode_str == "Kinematic")
        {
            positioning_mode = PMODE_KINEMA;
        }
    if (positioning_mode_str == "PPP_Static")
        {
            positioning_mode = PMODE_PPP_STATIC;
        }
    if (positioning_mode_str == "PPP_Kinematic")
        {
            positioning_mode = PMODE_PPP_KINEMA;
        }

    if (positioning_mode == -1)
        {
            // warn user and set the default
            std::cout << "WARNING: Bad specification of positioning mode.\n"
                      << "positioning_mode possible values: Single / Static / Kinematic / PPP_Static / PPP_Kinematic\n"
                      << "positioning_mode specified value: " << positioning_mode_str << "\n"
                      << "Setting positioning_mode to Single\n"
                      << std::flush;
            positioning_mode = PMODE_SINGLE;
        }

    // Per-system NTRIP RTK channel rule: GPS is L1 C/A alone or with one
    // second band (L2C or L5); Galileo is E1 alone or with E5a; BeiDou is
    // B1C alone. Single-band sets run single-frequency RTK.
    const bool ntrip_gps_l1 = signal_enabled_flags.check_any_enabled(GPS_1C);
    const bool ntrip_gps_l2 = signal_enabled_flags.check_any_enabled(GPS_2S);
    const bool ntrip_gps_l5 = signal_enabled_flags.check_any_enabled(GPS_L5);
    const bool ntrip_gal_e1 = signal_enabled_flags.check_any_enabled(GAL_1B);
    const bool ntrip_gal_e5a = signal_enabled_flags.check_any_enabled(GAL_E5a);
    const bool ntrip_bds_b1c = signal_enabled_flags.check_any_enabled(BDS_B1C);
    const bool ntrip_only_supported_signals =
        (signal_enabled_flags.flags & ~ntrip_rtk_supported_signal_mask()) == 0;
    const bool ntrip_valid_channel_set =
        ntrip_only_supported_signals &&
        (ntrip_gps_l1 || ntrip_gal_e1 || ntrip_bds_b1c) &&
        !(ntrip_gps_l2 && ntrip_gps_l5) &&
        (!(ntrip_gps_l2 || ntrip_gps_l5) || ntrip_gps_l1) &&
        (!ntrip_gal_e5a || ntrip_gal_e1);

    if (pvt_output_parameters.ntrip_client_enabled)
        {
            if (positioning_mode != PMODE_STATIC && positioning_mode != PMODE_KINEMA)
                {
                    throw std::invalid_argument(role + ".ntrip_client_enabled requires positioning_mode=Static or positioning_mode=Kinematic");
                }
            if (!ntrip_valid_channel_set)
                {
                    throw std::invalid_argument("NTRIP RTK channels must be GPS L1 C/A alone or with one of L2C or L5, and/or Galileo E1 alone or with E5a, and/or BeiDou B1C");
                }
        }

    int num_bands = 0;

    if (signal_enabled_flags.check_any_enabled(GPS_1C, GAL_1B, GLO_1G, BDS_B1, BDS_B1C, QZS_J1))
        {
            num_bands += 1;
        }
    if (signal_enabled_flags.check_any_enabled(GPS_2S, GLO_2G, BDS_B3))
        {
            num_bands += 1;
        }
    if (signal_enabled_flags.check_any_enabled(GAL_E6))
        {
            num_bands += 1;
        }
    if (signal_enabled_flags.check_any_enabled(GAL_E5a, GPS_L5, QZS_J5))
        {
            num_bands += 1;
        }
    if (signal_enabled_flags.check_any_enabled(GAL_E5b))
        {
            num_bands += 1;
        }
    if (num_bands > 3)
        {
            LOG(WARNING) << "Too much bands: The PVT engine can only handle 3 bands, but " << num_bands << " were set";
            num_bands = 3;
        }

    int number_of_frequencies = configuration->property(role + ".num_bands", num_bands); /* (1:L1, 2:L1+L2, 3:L1+L2+L5) */
    if ((number_of_frequencies < 1) || (number_of_frequencies > 3))
        {
            // warn user and set the default
            number_of_frequencies = num_bands;
        }
    if (pvt_output_parameters.ntrip_client_enabled)
        {
            // The slot count is fully determined by the channel set (see the
            // slot column of ntrip_rtk_signals()): single-band sets run on the
            // first slot alone
            const int ntrip_required_bands = ntrip_rtk_required_bands(signal_enabled_flags);
            if (number_of_frequencies != ntrip_required_bands)
                {
                    LOG(INFO) << "NTRIP RTK: setting the number of frequency slots to " << ntrip_required_bands;
                    number_of_frequencies = ntrip_required_bands;
                }
        }

    double elevation_mask = configuration->property(role + ".elevation_mask", 15.0);
    if ((elevation_mask < 0.0) || (elevation_mask > 90.0))
        {
            // warn user and set the default
            LOG(WARNING) << "Erroneous Elevation Mask. Setting to default value of 15.0 degrees";
            elevation_mask = 15.0;
        }

    int dynamics_model = configuration->property(role + ".dynamics_model", 0); /*  dynamics model (0:none, 1:velocity, 2:accel) */
    if ((dynamics_model < 0) || (dynamics_model > 2))
        {
            // warn user and set the default
            LOG(WARNING) << "Erroneous Dynamics Model configuration. Setting to default value of (0:none)";
            dynamics_model = 0;
        }

    const std::string default_satellite_ephemeris("Broadcast");
    const std::string satellite_ephemeris_str = configuration->property(
        role + ".satellite_ephemeris", default_satellite_ephemeris);
    int satellite_ephemeris = -1;
    if (satellite_ephemeris_str == "Broadcast")
        {
            satellite_ephemeris = EPHOPT_BRDC;
        }
    if (satellite_ephemeris_str == "SBAS")
        {
            satellite_ephemeris = EPHOPT_SBAS;
        }
    if (satellite_ephemeris == -1)
        {
            std::cout << "WARNING: Bad specification of satellite ephemeris source.\n"
                      << "satellite_ephemeris possible values: Broadcast / SBAS\n"
                      << "satellite_ephemeris specified value: " << satellite_ephemeris_str << "\n"
                      << "Setting satellite_ephemeris to Broadcast\n"
                      << std::flush;
            satellite_ephemeris = EPHOPT_BRDC;
        }

    int sbas_satellite = configuration->property(role + ".sbas_satellite", 0);
    if (sbas_satellite != 0 &&
        (sbas_satellite < static_cast<int>(SBAS_L1_PRN_MIN) ||
            sbas_satellite > static_cast<int>(SBAS_L1_PRN_MAX)))
        {
            LOG(WARNING) << "Erroneous SBAS satellite PRN. Selecting the first received SBAS stream";
            sbas_satellite = 0;
        }

    const std::string default_iono_model("OFF");
    const std::string iono_model_str = configuration->property(role + ".iono_model", default_iono_model); /*  (IONOOPT_XXX) see src/algorithms/libs/rtklib/rtklib.h */
    int iono_model = -1;
    if (iono_model_str == "OFF")
        {
            iono_model = IONOOPT_OFF;
        }
    if (iono_model_str == "Broadcast")
        {
            iono_model = IONOOPT_BRDC;
        }
    if (iono_model_str == "SBAS")
        {
            iono_model = IONOOPT_SBAS;
        }
    if (iono_model_str == "Iono-Free-LC")
        {
            iono_model = IONOOPT_IFLC;
        }
    if (iono_model_str == "Estimate_STEC")
        {
            iono_model = IONOOPT_EST;
        }
    if (iono_model_str == "IONEX")
        {
            iono_model = IONOOPT_TEC;
        }
    if (iono_model == -1)
        {
            // warn user and set the default
            std::cout << "WARNING: Bad specification of ionospheric model.\n"
                      << "iono_model possible values: OFF / Broadcast / SBAS / Iono-Free-LC / Estimate_STEC / IONEX\n"
                      << "iono_model specified value: " << iono_model_str << "\n"
                      << "Setting iono_model to OFF\n"
                      << std::flush;
            iono_model = IONOOPT_OFF; /* 0: ionosphere option: correction off */
        }

    const std::string default_trop_model("OFF");
    int trop_model = -1;
    const std::string trop_model_str = configuration->property(role + ".trop_model", default_trop_model); /*  (TROPOPT_XXX) see src/algorithms/libs/rtklib/rtklib.h */
    if (trop_model_str == "OFF")
        {
            trop_model = TROPOPT_OFF;
        }
    if (trop_model_str == "Saastamoinen")
        {
            trop_model = TROPOPT_SAAS;
        }
    if (trop_model_str == "SBAS")
        {
            trop_model = TROPOPT_SBAS;
        }
    if (trop_model_str == "Estimate_ZTD")
        {
            trop_model = TROPOPT_EST;
        }
    if (trop_model_str == "Estimate_ZTD_Grad")
        {
            trop_model = TROPOPT_ESTG;
        }
    if (trop_model == -1)
        {
            // warn user and set the default
            std::cout << "WARNING: Bad specification of tropospheric model.\n"
                      << "trop_model possible values: OFF / Saastamoinen / SBAS / Estimate_ZTD / Estimate_ZTD_Grad\n"
                      << "trop_model specified value: " << trop_model_str << "\n"
                      << "Setting trop_model to OFF\n"
                      << std::flush;
            trop_model = TROPOPT_OFF;
        }

    /* RTKLIB positioning options */
    int sat_PCV = 0; /*  Set whether the satellite antenna PCV (phase center variation) model is used or not. This feature requires a Satellite Antenna PCV File. */
    int rec_PCV = 0; /*  Set whether the receiver antenna PCV (phase center variation) model is used or not. This feature requires a Receiver Antenna PCV File. */

    /* Set whether the phase windup correction for PPP modes is applied or not. Only applicable to PPP‐* modes.*/
    const int phwindup = configuration->property(role + ".phwindup", 0);

    /* Set whether the GPS Block IIA satellites in eclipse are excluded or not.
    The eclipsing Block IIA satellites often degrade the PPP solutions due to unpredicted behavior of yaw‐attitude. Only applicable to PPP‐* modes.*/
    const int reject_GPS_IIA = configuration->property(role + ".reject_GPS_IIA", 0);

    /* Set whether RAIM (receiver autonomous integrity monitoring) FDE (fault detection and exclusion) feature is enabled or not.
    In case of RAIM FDE enabled, a satellite is excluded if SSE (sum of squared errors) of residuals is over a threshold.
    The excluded satellite is selected to indicate the minimum SSE. */
    const int raim_fde = configuration->property(role + ".raim_fde", 0);

    const int earth_tide = configuration->property(role + ".earth_tide", 0);

    int nsys = 0;
    if (signal_enabled_flags.check_any_enabled(GPS_1C, GPS_2S, GPS_L5))
        {
            nsys += SYS_GPS;
        }
    if (signal_enabled_flags.check_any_enabled(GAL_1B, GAL_E5a, GAL_E5b, GAL_E6))
        {
            nsys += SYS_GAL;
        }
    if (signal_enabled_flags.check_any_enabled(GLO_1G, GLO_2G))
        {
            nsys += SYS_GLO;
        }
    if (signal_enabled_flags.check_any_enabled(BDS_B1, BDS_B3, BDS_B1C))
        {
            nsys += SYS_BDS;
        }
    if (signal_enabled_flags.check_any_enabled(QZS_J1, QZS_J5))
        {
            nsys += SYS_QZS;
        }

    int navigation_system = configuration->property(role + ".navigation_system", nsys); /* (SYS_XXX) see src/algorithms/libs/rtklib/rtklib.h */
    if ((navigation_system < 1) || (navigation_system > 255))                           /* GPS: 1   SBAS: 2   GPS+SBAS: 3 Galileo: 8  Galileo+GPS: 9 GPS+SBAS+Galileo: 11 All: 255 */
        {
            // warn user and set the default
            LOG(WARNING) << "Erroneous Navigation System. Setting to default value of (0:none)";
            navigation_system = nsys;
        }
    if (pvt_output_parameters.ntrip_client_enabled)
        {
            const int expected_navigation_system = ntrip_rtk_navigation_systems(signal_enabled_flags);
            if (navigation_system != expected_navigation_system)
                {
                    throw std::invalid_argument(role + ".navigation_system must select exactly the constellations of the enabled RTK channels");
                }
        }

    // Settings 2
    const std::string default_gps_ar("Continuous");
    const std::string integer_ambiguity_resolution_gps_str = configuration->property(role + ".AR_GPS", default_gps_ar); /* Integer Ambiguity Resolution mode for GPS (0:off,1:continuous,2:instantaneous,3:fix and hold,4:ppp-ar) */
    int integer_ambiguity_resolution_gps = -1;
    if (integer_ambiguity_resolution_gps_str == "OFF")
        {
            integer_ambiguity_resolution_gps = ARMODE_OFF;
        }
    if (integer_ambiguity_resolution_gps_str == "Continuous")
        {
            integer_ambiguity_resolution_gps = ARMODE_CONT;
        }
    if (integer_ambiguity_resolution_gps_str == "Instantaneous")
        {
            integer_ambiguity_resolution_gps = ARMODE_INST;
        }
    if (integer_ambiguity_resolution_gps_str == "Fix-and-Hold")
        {
            integer_ambiguity_resolution_gps = ARMODE_FIXHOLD;
        }
    if (integer_ambiguity_resolution_gps_str == "PPP-AR")
        {
            integer_ambiguity_resolution_gps = ARMODE_PPPAR;
        }
    if (integer_ambiguity_resolution_gps == -1)
        {
            // warn user and set the default
            std::cout << "WARNING: Bad specification of GPS ambiguity resolution method.\n"
                      << "AR_GPS possible values: OFF / Continuous / Instantaneous / Fix-and-Hold / PPP-AR\n"
                      << "AR_GPS specified value: " << integer_ambiguity_resolution_gps_str << "\n"
                      << "Setting AR_GPS to OFF\n"
                      << std::flush;
            integer_ambiguity_resolution_gps = ARMODE_OFF;
        }

    int integer_ambiguity_resolution_glo = configuration->property(role + ".AR_GLO", 1); /* Integer Ambiguity Resolution mode for GLONASS (0:off,1:on,2:auto cal,3:ext cal) */
    if ((integer_ambiguity_resolution_glo < 0) || (integer_ambiguity_resolution_glo > 3))
        {
            // warn user and set the default
            LOG(WARNING) << "Erroneous Integer Ambiguity Resolution for GLONASS . Setting to default value of (1:on)";
            integer_ambiguity_resolution_glo = 1;
        }

    int integer_ambiguity_resolution_bds = configuration->property(role + ".AR_DBS", 1); /* Integer Ambiguity Resolution mode for BEIDOU (0:off,1:on) */
    if ((integer_ambiguity_resolution_bds < 0) || (integer_ambiguity_resolution_bds > 1))
        {
            // warn user and set the default
            LOG(WARNING) << "Erroneous Integer Ambiguity Resolution for BEIDOU . Setting to default value of (1:on)";
            integer_ambiguity_resolution_bds = 1;
        }

    const double min_ratio_to_fix_ambiguity = configuration->property(role + ".min_ratio_to_fix_ambiguity", 3.0); /* Set the integer ambiguity validation threshold for ratio‐test,
                                                                                                               which uses the ratio of squared residuals of the best integer vector to the second‐best vector. */

    const int min_lock_to_fix_ambiguity = configuration->property(role + ".min_lock_to_fix_ambiguity", 0); /* Set the minimum lock count to fix integer ambiguity.FLAGS_RINEX_version.
                                                                                                         If the lock count is less than the value, the ambiguity is excluded from the fixed integer vector. */

    const double min_elevation_to_fix_ambiguity = configuration->property(role + ".min_elevation_to_fix_ambiguity", 0.0); /* Set the minimum elevation (deg) to fix integer ambiguity.
                                                                                                                        If the elevation of the satellite is less than the value, the ambiguity is excluded from the fixed integer vector. */

    const int outage_reset_ambiguity = configuration->property(role + ".outage_reset_ambiguity", 5); /* Set the outage count to reset ambiguity. If the data outage count is over the value, the estimated ambiguity is reset to the initial value.  */

    const double slip_threshold = configuration->property(role + ".slip_threshold", 0.05); /* set the cycle‐slip threshold (m) of geometry‐free LC carrier‐phase difference between epochs */

    const double slip_threshold_doppler = configuration->property(role + ".slip_threshold_doppler", 0.0); /* cycle-slip threshold (m/s) of the phase-doppler difference, after removing the median common range-rate error over all satellites. 0 disables the test. */

    const double threshold_reject_gdop = configuration->property(role + ".threshold_reject_gdop", 30.0); /* reject threshold of GDOP. If the GDOP is over the value, the observable is excluded for the estimation process as an outlier. */

    const double threshold_reject_innovation = configuration->property(role + ".threshold_reject_innovation", 30.0); /* reject threshold of innovation (m) for code observables. If the innovation is over the value, the observable is excluded for the estimation process as an outlier. */

    const double threshold_reject_innovation_phase = configuration->property(role + ".threshold_reject_innovation_phase", threshold_reject_innovation); /* reject threshold of innovation (m) for carrier-phase observables. Defaults to threshold_reject_innovation; the demo5 RTKLIB fork recommends 5.0 for RTK. */

    const int number_filter_iter = configuration->property(role + ".number_filter_iter", 1); /* Set the number of iteration in the measurement update of the estimation filter.
                                                                                         If the baseline length is very short like 1 m, the iteration may be effective to handle
                                                                                         the nonlinearity of measurement equation. */

    // Statistics
    const double bias_0 = configuration->property(role + ".bias_0", 30.0);

    const double iono_0 = configuration->property(role + ".iono_0", 0.03);

    const double trop_0 = configuration->property(role + ".trop_0", 0.3);

    const double sigma_bias = configuration->property(role + ".sigma_bias", 1e-4); /* Set the process noise standard deviation of carrier‐phase
                                                                                bias (ambiguity) (cycle/sqrt(s)) */

    const double sigma_iono = configuration->property(role + ".sigma_iono", 1e-3); /* Set the process noise standard deviation of vertical ionospheric delay per 10 km baseline (m/sqrt(s)). */

    const double sigma_trop = configuration->property(role + ".sigma_trop", 1e-4); /* Set the process noise standard deviation of zenith tropospheric delay (m/sqrt(s)). */

    const double sigma_acch = configuration->property(role + ".sigma_acch", 1e-1); /* Set the process noise standard deviation of the receiver acceleration as
                                                                                the horizontal component. (m/s2/sqrt(s)). If Receiver Dynamics is set to OFF, they are not used. */

    const double sigma_accv = configuration->property(role + ".sigma_accv", 1e-2); /* Set the process noise standard deviation of the receiver acceleration as
                                                                                the vertical component. (m/s2/sqrt(s)). If Receiver Dynamics is set to OFF, they are not used. */

    const double sigma_pos = configuration->property(role + ".sigma_pos", 0.0);

    const double code_phase_error_ratio_l1 = configuration->property(role + ".code_phase_error_ratio_l1", 100.0);
    const double code_phase_error_ratio_l2 = configuration->property(role + ".code_phase_error_ratio_l2", 100.0);
    const double code_phase_error_ratio_l5 = configuration->property(role + ".code_phase_error_ratio_l5", 100.0);
    const double carrier_phase_error_factor_a = configuration->property(role + ".carrier_phase_error_factor_a", 0.003);
    const double carrier_phase_error_factor_b = configuration->property(role + ".carrier_phase_error_factor_b", 0.003);

    /* SNR-dependent and receiver-reported-stdev observation weighting (demo5). Both terms default to
       0.0 (disabled), preserving the elevation-only error model */
    const double error_snr_max = configuration->property(role + ".error_snr_max", 52.0);              /* SNR reference (dB-Hz) above which no SNR penalty is applied */
    const double error_factor_snr = configuration->property(role + ".error_factor_snr", 0.0);         /* SNR error term (m); demo5 uses 0.005 for its receivers */
    const double error_factor_rcv_std = configuration->property(role + ".error_factor_rcv_std", 0.0); /* receiver-reported stdev error term; requires Pstd/Lstd filled in the observations */

    /* AR management options (demo5) */
    const bool ar_filter = configuration->property(role + ".ar_filter", true);                                 /* reject newly-risen satellites from AR and retry when the AR ratio degrades */
    const int min_fix_sats = configuration->property(role + ".min_fix_sats", 4);                               /* min satellites required to fix integer ambiguities */
    const int min_hold_sats = configuration->property(role + ".min_hold_sats", 5);                             /* min satellites required to hold integer ambiguities */
    const int min_drop_sats = configuration->property(role + ".min_drop_sats", 10);                            /* min satellites to enable single-satellite exclusion cycling in AR (0 disables) */
    const double var_holdamb = configuration->property(role + ".var_holdamb", 0.1);                            /* variance (cycle^2) of the fix-and-hold pseudo measurements */
    const double ar_max_position_variance = configuration->property(role + ".ar_max_position_variance", 0.25); /* max float-position variance (m^2) to attempt AR; avoids false fixes while the filter converges (0 disables the gate) */
    const double ar_ratio_min = configuration->property(role + ".ar_ratio_min", 0.0);                          /* lower bound for the satellite-count-dependent AR ratio threshold; leave equal to ar_ratio_max to use the fixed min_ratio_to_fix_ambiguity */
    const double ar_ratio_max = configuration->property(role + ".ar_ratio_max", 0.0);                          /* upper bound for the satellite-count-dependent AR ratio threshold */

    const bool bancroft_init = configuration->property(role + ".bancroft_init", true);

    const bool estimate_qzss_isb = configuration->property(role + ".estimate_qzss_isb", false); /* estimate a separate QZS-GPS inter-system bias in single-point positioning. It requires one extra satellite in mixed GPS+QZSS epochs, so keep disabled under degraded visibility. */

    snrmask_t snrmask = {{}, {{}, {}}};
    const double max_time_difference_s = pvt_output_parameters.ntrip_client_enabled ? pvt_output_parameters.ntrip_max_correction_age_s : 30.0;
    const int output_single_on_outage = (!pvt_output_parameters.ntrip_client_enabled || pvt_output_parameters.ntrip_fallback_to_single) ? 1 : 0;

    prcopt_t rtklib_configuration_options = {
        positioning_mode,                                                                  /* positioning mode (PMODE_XXX) see src/algorithms/libs/rtklib/rtklib.h */
        0,                                                                                 /* solution type (0:forward,1:backward,2:combined) */
        number_of_frequencies,                                                             /* number of frequencies (1:L1, 2:L1+L2, 3:L1+L2+L5)*/
        navigation_system,                                                                 /* navigation system  */
        elevation_mask * D2R,                                                              /* elevation mask angle (degrees) */
        snrmask,                                                                           /* snrmask_t snrmask    SNR mask */
        satellite_ephemeris,                                                               /* satellite ephemeris/clock (EPHOPT_XXX) */
        integer_ambiguity_resolution_gps,                                                  /* AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold,4:ppp-ar) */
        integer_ambiguity_resolution_glo,                                                  /* GLONASS AR mode (0:off,1:on,2:auto cal,3:ext cal) */
        integer_ambiguity_resolution_bds,                                                  /* BeiDou AR mode (0:off,1:on) */
        outage_reset_ambiguity,                                                            /* obs outage count to reset bias */
        min_lock_to_fix_ambiguity,                                                         /* min lock count to fix ambiguity */
        10,                                                                                /* min fix count to hold ambiguity */
        1,                                                                                 /* max iteration to resolve ambiguity */
        iono_model,                                                                        /* ionosphere option (IONOOPT_XXX) */
        trop_model,                                                                        /* troposphere option (TROPOPT_XXX) */
        dynamics_model,                                                                    /* dynamics model (0:none, 1:velocity, 2:accel) */
        earth_tide,                                                                        /* earth tide correction (0:off,1:solid,2:solid+otl+pole) */
        number_filter_iter,                                                                /* number of filter iteration */
        0,                                                                                 /* code smoothing window size (0:none) */
        0,                                                                                 /* interpolate reference obs (for post mission) */
        0,                                                                                 /* SBAS correction options */
        sbas_satellite,                                                                    /* SBAS satellite selection (0:first received) */
        0,                                                                                 /* rover position for fixed mode */
        0,                                                                                 /* base position for relative mode */
                                                                                           /*    0:pos in prcopt,  1:average of single pos, */
                                                                                           /*    2:read from file, 3:rinex header, 4:rtcm pos */
        {code_phase_error_ratio_l1, code_phase_error_ratio_l2, code_phase_error_ratio_l5}, /* eratio[NFREQ] code/phase error ratio */
        {100.0, carrier_phase_error_factor_a, carrier_phase_error_factor_b, 0.0, 1.0,
            error_snr_max, error_factor_snr, error_factor_rcv_std},                             /* err[8]:  measurement error factor [0]:reserved, [1-3]:error factor a/b/c of phase (m), [4]:doppler frequency (hz), [5]:SNR reference (dBHz), [6]:SNR error term (m), [7]:receiver stdev term */
        {bias_0, iono_0, trop_0},                                                               /* std[3]: initial-state std [0]bias,[1]iono [2]trop*/
        {sigma_bias, sigma_iono, sigma_trop, sigma_acch, sigma_accv, sigma_pos},                /* prn[6] process-noise std */
        5e-12,                                                                                  /* sclkstab: satellite clock stability (sec/sec) */
        {min_ratio_to_fix_ambiguity, 0.9999, 0.25, 0.1, 0.05, ar_ratio_min, ar_ratio_max, 0.0}, /* thresar[8]: AR validation threshold; [5]/[6]: min/max of the sat-count-dependent AR ratio threshold (equal: fixed threshold) */
        min_elevation_to_fix_ambiguity,                                                         /* elevation mask of AR for rising satellite (deg) */
        0.0,                                                                                    /* elevation mask to hold ambiguity (deg) */
        slip_threshold,                                                                         /* slip threshold of geometry-free phase (m) */
        slip_threshold_doppler,                                                                 /* slip threshold of doppler (m/s) (0:disabled) */
        max_time_difference_s,                                                                  /* max difference of time (sec) */
        {threshold_reject_innovation_phase, threshold_reject_innovation},                       /* reject threshold of innovation {phase, code} (m) */
        threshold_reject_gdop,                                                                  /* reject threshold of gdop */
        {},                                                                                     /* double baseline[2] baseline length constraint {const,sigma} (m) */
        {},                                                                                     /* double ru[3]  rover position for fixed mode {x,y,z} (ecef) (m) */
        {},                                                                                     /* double rb[3]  base position for relative mode {x,y,z} (ecef) (m) */
        {"", ""},                                                                               /* char anttype[2][MAXANT]  antenna types {rover,base}  */
        {{}, {}},                                                                               /* double antdel[2][3]   antenna delta {{rov_e,rov_n,rov_u},{ref_e,ref_n,ref_u}} */
        {},                                                                                     /* pcv_t pcvr[2]   receiver antenna parameters {rov,base} */
        {},                                                                                     /* unsigned char exsats[MAXSAT]  excluded satellites (1:excluded, 2:included) */
        0,                                                                                      /* max averaging epoches */
        0,                                                                                      /* initialize by restart */
        output_single_on_outage,                                                                /* output single by dgps/float/fix/ppp outage */
        {"", ""},                                                                               /* char rnxopt[2][256]   rinex options {rover,base} */
        {sat_PCV, rec_PCV, phwindup, reject_GPS_IIA, raim_fde},                                 /* posopt[6] positioning options [0]: satellite and receiver antenna PCV model; [1]: interpolate antenna parameters; [2]: apply phase wind-up correction for PPP modes; [3]: exclude measurements of GPS Block IIA satellites satellite [4]: RAIM FDE (fault detection and exclusion) [5]: handle day-boundary clock jump */
        0,                                                                                      /* solution sync mode (0:off,1:on) */
        {{}, {}},                                                                               /* odisp[2][6*11] ocean tide loading parameters {rov,base} */
        {{}, {{}, {}}, {{}, {}}, {}, {}},                                                       /* exterr_t exterr   extended receiver error model */
        0,                                                                                      /* disable L2-AR */
        {},                                                                                     /* char pppopt[256]   ppp option   "-GAP_RESION="  default gap to reset iono parameters (ep) */
        bancroft_init,                                                                          /* enable Bancroft initialization for the first iteration of the PVT computation, useful in some geometries */
        estimate_qzss_isb,                                                                      /* estimate a separate QZS-GPS inter-system bias (needs one extra satellite in mixed GPS+QZSS epochs) */
        ar_filter ? 1 : 0,                                                                      /* AR filtering to reject newly-added sats on AR ratio degradation */
        min_fix_sats,                                                                           /* min sats to fix integer ambiguities */
        min_hold_sats,                                                                          /* min sats to hold integer ambiguities */
        min_drop_sats,                                                                          /* min sats to enable single-sat exclusion cycling in AR */
        var_holdamb,                                                                            /* variance of the fix-and-hold pseudo measurements (cycle^2) */
        ar_max_position_variance                                                                /* max position variance to attempt AR (m^2) */
    };

    if (integer_ambiguity_resolution_gps > 0 &&
        (ar_filter || min_fix_sats > 0 || min_hold_sats > 0 || min_drop_sats > 0 || ar_max_position_variance > 0.0))
        {
            // These gates default ON (demo5 behavior), unlike the RTKLIB
            // PRCOPT_DEFAULT zeros: say so where the user can see why AR may
            // silently be skipped (e.g. float variance above the gate)
            LOG(INFO) << "RTK AR management gates active: ar_filter=" << (ar_filter ? "true" : "false")
                      << ", min_fix_sats=" << min_fix_sats
                      << ", min_hold_sats=" << min_hold_sats
                      << ", min_drop_sats=" << min_drop_sats
                      << ", ar_max_position_variance=" << ar_max_position_variance
                      << " m^2 (AR is skipped while the float position variance exceeds it;"
                      << " set these options to 0/false for the classic RTKLIB behavior)";
        }

    // Outputs
    const bool default_output_enabled = configuration->property(role + ".output_enabled", true);
    pvt_output_parameters.output_enabled = default_output_enabled;
    pvt_output_parameters.rinex_output_enabled = configuration->property(role + ".rinex_output_enabled", default_output_enabled);
    pvt_output_parameters.gpx_output_enabled = configuration->property(role + ".gpx_output_enabled", default_output_enabled);
    pvt_output_parameters.geojson_output_enabled = configuration->property(role + ".geojson_output_enabled", default_output_enabled);
    pvt_output_parameters.kml_output_enabled = configuration->property(role + ".kml_output_enabled", default_output_enabled);
    pvt_output_parameters.xml_output_enabled = configuration->property(role + ".xml_output_enabled", default_output_enabled);
    pvt_output_parameters.nmea_output_file_enabled = configuration->property(role + ".nmea_output_file_enabled", default_output_enabled);
    pvt_output_parameters.rtcm_output_file_enabled = configuration->property(role + ".rtcm_output_file_enabled", false);

    const std::string default_output_path = configuration->property(role + ".output_path", std::string("."));
    pvt_output_parameters.output_path = default_output_path;
    pvt_output_parameters.rinex_output_path = configuration->property(role + ".rinex_output_path", default_output_path);
    pvt_output_parameters.gpx_output_path = configuration->property(role + ".gpx_output_path", default_output_path);
    pvt_output_parameters.geojson_output_path = configuration->property(role + ".geojson_output_path", default_output_path);
    pvt_output_parameters.kml_output_path = configuration->property(role + ".kml_output_path", default_output_path);
    pvt_output_parameters.xml_output_path = configuration->property(role + ".xml_output_path", default_output_path);
    pvt_output_parameters.nmea_output_file_path = configuration->property(role + ".nmea_output_file_path", default_output_path);
    pvt_output_parameters.rtcm_output_file_path = configuration->property(role + ".rtcm_output_file_path", default_output_path);
    pvt_output_parameters.has_output_file_path = configuration->property(role + ".has_output_file_path", default_output_path);

    // Read PVT MONITOR Configuration
    pvt_output_parameters.monitor_enabled = configuration->property(role + ".enable_monitor", false);
    pvt_output_parameters.udp_addresses = configuration->property(role + ".monitor_client_addresses", std::string("127.0.0.1"));
    pvt_output_parameters.udp_ports = configuration->property(role + ".monitor_udp_port", std::string("1234"));
    pvt_output_parameters.protobuf_enabled = configuration->property(role + ".enable_protobuf", true);
    if (configuration->property("Monitor.enable_protobuf", false) == true)
        {
            pvt_output_parameters.protobuf_enabled = true;
        }

    // Read EPHEMERIS MONITOR Configuration
    pvt_output_parameters.monitor_ephemeris_enabled = configuration->property(role + ".enable_monitor_ephemeris", false);
    pvt_output_parameters.udp_eph_addresses = configuration->property(role + ".monitor_ephemeris_client_addresses", std::string("127.0.0.1"));
    pvt_output_parameters.udp_eph_port = configuration->property(role + ".monitor_ephemeris_udp_port", 1234);

    // Show time in local zone
    pvt_output_parameters.show_local_time_zone = configuration->property(role + ".show_local_time_zone", false);

    // Enable or disable rx clock correction in observables
    pvt_output_parameters.enable_rx_clock_correction = configuration->property(role + ".enable_rx_clock_correction", false);

    // Set maximum clock offset allowed if pvt_output_parameters.enable_rx_clock_correction = false
    pvt_output_parameters.max_obs_block_rx_clock_offset_ms = configuration->property(role + ".max_clock_offset_ms", pvt_output_parameters.max_obs_block_rx_clock_offset_ms);

    // Source timetag
    pvt_output_parameters.log_source_timetag = configuration->property(role + ".log_timetag", pvt_output_parameters.log_source_timetag);
    pvt_output_parameters.log_source_timetag_file = configuration->property(role + ".log_source_timetag_file", pvt_output_parameters.log_source_timetag_file);

    // Use E6 for PVT
    pvt_output_parameters.use_e6_for_pvt = configuration->property(role + ".use_e6_for_pvt", pvt_output_parameters.use_e6_for_pvt);
    pvt_output_parameters.use_has_corrections = configuration->property(role + ".use_has_corrections", pvt_output_parameters.use_has_corrections);

    // Use unhealthy satellites
    pvt_output_parameters.use_unhealthy_sats = configuration->property(role + ".use_unhealthy_sats", pvt_output_parameters.use_unhealthy_sats);

    // Apply the GLONASS Bn-word MSB health bit in addition to the ln flag when
    // deciding whether to exclude a GLONASS satellite from the PVT.
    pvt_output_parameters.glonass_strict_health = configuration->property(role + ".glonass_strict_health", pvt_output_parameters.glonass_strict_health);

    // OSNMA
    if (signal_enabled_flags.check_any_enabled(GAL_1B) > 0)
        {
            std::string osnma_mode = configuration->property("GNSS-SDR.osnma_mode", std::string(""));
            bool enable_osnma = configuration->property("GNSS-SDR.osnma_enable", true);
            if (enable_osnma && osnma_mode == "strict")
                {
                    pvt_output_parameters.osnma_strict = true;
                }
        }

    // make PVT object. rtk_t owns raw filter buffers, so allocate the adapter's
    // seed only after all potentially throwing C++ configuration work, and
    // release it once each solver has initialized its independent state.
    rtkinit(&rtk, &rtklib_configuration_options);

    // Release the seed as well if block construction fails.
    try
        {
            pvt_ = rtklib_make_pvt_gs(in_streams_, pvt_output_parameters, rtk, sensor_data_configuration);
        }
    catch (...)
        {
            rtkfree(&rtk);
            throw;
        }
    rtkfree(&rtk);
    DLOG(INFO) << "pvt(" << pvt_->unique_id() << ")";
    if (out_streams_ > 0)
        {
            LOG(ERROR) << "The PVT block does not have an output stream";
        }
}


Rtklib_Pvt::~Rtklib_Pvt()
{
    DLOG(INFO) << "PVT adapter destructor called.";
    rtkfree(&rtk);
}


bool Rtklib_Pvt::get_latest_PVT(double* longitude_deg,
    double* latitude_deg,
    double* height_m,
    double* ground_speed_kmh,
    double* course_over_ground_deg,
    time_t* UTC_time)
{
    return pvt_->get_latest_PVT(longitude_deg,
        latitude_deg,
        height_m,
        ground_speed_kmh,
        course_over_ground_deg,
        UTC_time);
}


void Rtklib_Pvt::clear_ephemeris()
{
    pvt_->clear_ephemeris();
}


std::map<int, Gps_Ephemeris> Rtklib_Pvt::get_gps_ephemeris() const
{
    return pvt_->get_gps_ephemeris_map();
}


std::map<int, Galileo_Ephemeris> Rtklib_Pvt::get_galileo_ephemeris() const
{
    return pvt_->get_galileo_ephemeris_map();
}


std::map<int, Beidou_Dnav_Ephemeris> Rtklib_Pvt::get_beidou_dnav_ephemeris() const
{
    return pvt_->get_beidou_dnav_ephemeris_map();
}


std::map<int, Gps_Almanac> Rtklib_Pvt::get_gps_almanac() const
{
    return pvt_->get_gps_almanac_map();
}


std::map<int, Galileo_Almanac> Rtklib_Pvt::get_galileo_almanac() const
{
    return pvt_->get_galileo_almanac_map();
}


std::map<int, Beidou_Dnav_Almanac> Rtklib_Pvt::get_beidou_dnav_almanac() const
{
    return pvt_->get_beidou_dnav_almanac_map();
}


void Rtklib_Pvt::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to connect internally
    DLOG(INFO) << "nothing to connect internally";
}


void Rtklib_Pvt::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to disconnect
}


gr::basic_block_sptr Rtklib_Pvt::get_left_block()
{
    return pvt_;
}


gr::basic_block_sptr Rtklib_Pvt::get_right_block()
{
    return nullptr;  // this is a sink, nothing downstream
}
