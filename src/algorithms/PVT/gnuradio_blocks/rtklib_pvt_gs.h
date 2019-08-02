/*!
 * \file rtklib_pvt_gs.h
 * \brief Interface of a Position Velocity and Time computation block
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_RTKLIB_PVT_GS_H
#define GNSS_SDR_RTKLIB_PVT_GS_H

#include "gnss_synchro.h"
#include "rtklib.h"
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/shared_ptr.hpp>   // for boost::shared_ptr
#include <gnuradio/sync_block.h>  // for sync_block
#include <gnuradio/types.h>       // for gr_vector_const_void_star
#include <pmt/pmt.h>              // for pmt_t
#include <chrono>                 // for system_clock
#include <cstdint>                // for int32_t
#include <ctime>                  // for time_t
#include <map>                    // for map
#include <memory>                 // for shared_ptr, unique_ptr
#include <string>                 // for string
#include <sys/types.h>            // for key_t
#include <utility>                // for pair
#include <vector>                 // for vector

class Beidou_Dnav_Almanac;
class Beidou_Dnav_Ephemeris;
class Galileo_Almanac;
class Galileo_Ephemeris;
class GeoJSON_Printer;
class Gps_Almanac;
class Gps_Ephemeris;
class Gpx_Printer;
class Kml_Printer;
class Monitor_Pvt_Udp_Sink;
class Nmea_Printer;
class Pvt_Conf;
class Rinex_Printer;
class Rtcm_Printer;
class Rtklib_Solver;
class rtklib_pvt_gs;

using rtklib_pvt_gs_sptr = boost::shared_ptr<rtklib_pvt_gs>;

rtklib_pvt_gs_sptr rtklib_make_pvt_gs(uint32_t nchannels,
    const Pvt_Conf& conf_,
    const rtk_t& rtk);

/*!
 * \brief This class implements a block that computes the PVT solution using the RTKLIB integrated library
 */
class rtklib_pvt_gs : public gr::sync_block
{
public:
    ~rtklib_pvt_gs();  //!< Default destructor

    /*!
     * \brief Get latest set of GPS ephemeris from PVT block
     */
    std::map<int, Gps_Ephemeris> get_gps_ephemeris_map() const;

    /*!
     * \brief Get latest set of GPS almanac from PVT block
     */
    std::map<int, Gps_Almanac> get_gps_almanac_map() const;

    /*!
     * \brief Get latest set of Galileo ephemeris from PVT block
     */
    std::map<int, Galileo_Ephemeris> get_galileo_ephemeris_map() const;

    /*!
     * \brief Get latest set of Galileo almanac from PVT block
     */
    std::map<int, Galileo_Almanac> get_galileo_almanac_map() const;

    /*!
     * \brief Get latest set of BeiDou DNAV ephemeris from PVT block
     */
    std::map<int, Beidou_Dnav_Ephemeris> get_beidou_dnav_ephemeris_map() const;

    /*!
     * \brief Get latest set of BeiDou DNAV almanac from PVT block
     */
    std::map<int, Beidou_Dnav_Almanac> get_beidou_dnav_almanac_map() const;

    /*!
     * \brief Clear all ephemeris information and the almanacs for GPS and Galileo
     */
    void clear_ephemeris();

    /*!
     * \brief Get the latest Position WGS84 [deg], Ground Velocity, Course over Ground, and UTC Time, if available
     */
    bool get_latest_PVT(double* longitude_deg,
        double* latitude_deg,
        double* height_m,
        double* ground_speed_kmh,
        double* course_over_ground_deg,
        time_t* UTC_time) const;

    int work(int noutput_items, gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items);  //!< PVT Signal Processing

private:
    friend rtklib_pvt_gs_sptr rtklib_make_pvt_gs(uint32_t nchannels,
        const Pvt_Conf& conf_,
        const rtk_t& rtk);

    rtklib_pvt_gs(uint32_t nchannels,
        const Pvt_Conf& conf_,
        const rtk_t& rtk);

    void msg_handler_telemetry(const pmt::pmt_t& msg);

    enum StringValue
    {
        evGPS_1C,
        evGPS_2S,
        evGPS_L5,
        evSBAS_1C,
        evGAL_1B,
        evGAL_5X,
        evGLO_1G,
        evGLO_2G,
        evBDS_B1,
        evBDS_B2,
        evBDS_B3
    };

    std::map<std::string, StringValue> mapStringValues_;

    void apply_rx_clock_offset(std::map<int, Gnss_Synchro>& observables_map,
        double rx_clock_offset_s);

    std::map<int, Gnss_Synchro> interpolate_observables(std::map<int, Gnss_Synchro>& observables_map_t0,
        std::map<int, Gnss_Synchro>& observables_map_t1,
        double rx_time_s);

    bool d_dump;
    bool d_dump_mat;
    bool b_rinex_output_enabled;
    bool b_rinex_header_written;
    bool b_rinex_header_updated;
    double d_rinex_version;
    int32_t d_rinexobs_rate_ms;

    bool b_rtcm_writing_started;
    bool b_rtcm_enabled;
    int32_t d_rtcm_MT1045_rate_ms;  // Galileo Broadcast Ephemeris
    int32_t d_rtcm_MT1019_rate_ms;  // GPS Broadcast Ephemeris (orbits)
    int32_t d_rtcm_MT1020_rate_ms;  // GLONASS Broadcast Ephemeris (orbits)
    int32_t d_rtcm_MT1077_rate_ms;  // The type 7 Multiple Signal Message format for the USA’s GPS system, popular
    int32_t d_rtcm_MT1087_rate_ms;  // GLONASS MSM7. The type 7 Multiple Signal Message format for the Russian GLONASS system
    int32_t d_rtcm_MT1097_rate_ms;  // Galileo MSM7. The type 7 Multiple Signal Message format for Europe’s Galileo system
    int32_t d_rtcm_MSM_rate_ms;

    int32_t d_kml_rate_ms;
    int32_t d_gpx_rate_ms;
    int32_t d_geojson_rate_ms;
    int32_t d_nmea_rate_ms;

    int32_t d_last_status_print_seg;  // for status printer

    uint32_t d_nchannels;
    std::string d_dump_filename;

    int32_t d_output_rate_ms;
    int32_t d_display_rate_ms;
    int32_t d_report_rate_ms;

    std::shared_ptr<Rinex_Printer> rp;
    std::shared_ptr<Kml_Printer> d_kml_dump;
    std::shared_ptr<Gpx_Printer> d_gpx_dump;
    std::shared_ptr<Nmea_Printer> d_nmea_printer;
    std::shared_ptr<GeoJSON_Printer> d_geojson_printer;
    std::shared_ptr<Rtcm_Printer> d_rtcm_printer;
    double d_rx_time;

    bool d_geojson_output_enabled;
    bool d_gpx_output_enabled;
    bool d_kml_output_enabled;
    bool d_nmea_output_file_enabled;

    std::shared_ptr<Rtklib_Solver> d_internal_pvt_solver;
    std::shared_ptr<Rtklib_Solver> d_user_pvt_solver;

    int32_t max_obs_block_rx_clock_offset_ms;
    bool d_waiting_obs_block_rx_clock_offset_correction_msg;
    std::map<int, Gnss_Synchro> gnss_observables_map;
    std::map<int, Gnss_Synchro> gnss_observables_map_t0;
    std::map<int, Gnss_Synchro> gnss_observables_map_t1;

    uint32_t type_of_rx;

    bool first_fix;
    key_t sysv_msg_key;
    int sysv_msqid;
    typedef struct
    {
        long mtype;  // NOLINT(google-runtime-int) required by SysV queue messaging
        double ttff;
    } ttff_msgbuf;
    bool send_sys_v_ttff_msg(ttff_msgbuf ttff);
    std::chrono::time_point<std::chrono::system_clock> start, end;

    bool save_gnss_synchro_map_xml(const std::string& file_name);  // debug helper function

    bool load_gnss_synchro_map_xml(const std::string& file_name);  // debug helper function

    bool d_xml_storage;
    std::string xml_base_path;

    inline std::time_t convert_to_time_t(const boost::posix_time::ptime pt) const
    {
        return (pt - boost::posix_time::ptime(boost::gregorian::date(1970, 1, 1))).total_seconds();
    }

    bool flag_monitor_pvt_enabled;
    std::unique_ptr<Monitor_Pvt_Udp_Sink> udp_sink_ptr;
    std::vector<std::string> split_string(const std::string& s, char delim) const;

    bool d_show_local_time_zone;
    std::string d_local_time_str;
    boost::posix_time::time_duration d_utc_diff_time;
};

#endif
