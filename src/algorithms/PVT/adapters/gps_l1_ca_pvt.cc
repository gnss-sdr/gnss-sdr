/*!
 * \file gps_l1_ca_pvt.cc
 * \brief  Implementation of an adapter of a GPS L1 C/A PVT solver block to a
 * PvtInterface
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include "gps_l1_ca_pvt.h"
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/math/common_factor_rt.hpp>
#include <boost/serialization/map.hpp>
#include <glog/logging.h>
#include "configuration_interface.h"

using google::LogMessage;

GpsL1CaPvt::GpsL1CaPvt(ConfigurationInterface* configuration,
        std::string role,
        unsigned int in_streams,
        unsigned int out_streams) :
                role_(role),
                in_streams_(in_streams),
                out_streams_(out_streams)
{
    // dump parameters
    std::string default_dump_filename = "./pvt.dat";
    std::string default_nmea_dump_filename = "./nmea_pvt.nmea";
    std::string default_nmea_dump_devname = "/dev/tty1";
    std::string default_rtcm_dump_devname = "/dev/pts/1";
    DLOG(INFO) << "role " << role;
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_filename);
    // moving average depth parameters
    int averaging_depth = configuration->property(role + ".averaging_depth", 10);
    bool flag_averaging = configuration->property(role + ".flag_averaging", false);

    // output rate
    int output_rate_ms = configuration->property(role + ".output_rate_ms", 500);

    // display rate
    int display_rate_ms = configuration->property(role + ".display_rate_ms", 500);

    // NMEA Printer settings
    bool flag_nmea_tty_port = configuration->property(role + ".flag_nmea_tty_port", false);
    std::string nmea_dump_filename = configuration->property(role + ".nmea_dump_filename", default_nmea_dump_filename);
    std::string nmea_dump_devname = configuration->property(role + ".nmea_dump_devname", default_nmea_dump_devname);

    // RTCM Printer settings
    bool flag_rtcm_tty_port = configuration->property(role + ".flag_rtcm_tty_port", false);
    std::string rtcm_dump_devname = configuration->property(role + ".rtcm_dump_devname", default_rtcm_dump_devname);
    bool flag_rtcm_server = configuration->property(role + ".flag_rtcm_server", false);
    unsigned short rtcm_tcp_port = configuration->property(role + ".rtcm_tcp_port", 2101);
    unsigned short rtcm_station_id = configuration->property(role + ".rtcm_station_id", 1234);
    // RTCM message rates: least common multiple with output_rate_ms
    int rtcm_MT1019_rate_ms = boost::math::lcm(configuration->property(role + ".rtcm_MT1019_rate_ms", 5000), output_rate_ms);
    int rtcm_MSM_rate_ms = boost::math::lcm(configuration->property(role + ".rtcm_MSM_rate_ms", 1000), output_rate_ms);
    std::map<int,int> rtcm_msg_rate_ms;
    rtcm_msg_rate_ms[1019] = rtcm_MT1019_rate_ms;
    for (int k = 1071; k < 1078; k++) // All GPS MSM
        {
            rtcm_msg_rate_ms[k] = rtcm_MSM_rate_ms;
        }

    // getting names from the config file, if available
    // default filename for assistance data
    const std::string eph_default_xml_filename = "./gps_ephemeris.xml";
    eph_xml_filename_= configuration->property("GNSS-SDR.SUPL_gps_ephemeris_xml", eph_default_xml_filename);

    //const std::string utc_default_xml_filename = "./gps_utc_model.xml";
    //const std::string iono_default_xml_filename = "./gps_iono.xml";
    //const std::string ref_time_default_xml_filename = "./gps_ref_time.xml";
    //const std::string ref_location_default_xml_filename = "./gps_ref_location.xml";

    //std::string utc_xml_filename = configuration_->property("GNSS-SDR.SUPL_gps_utc_model.xml", utc_default_xml_filename);
    //std::string iono_xml_filename = configuration_->property("GNSS-SDR.SUPL_gps_iono_xml", iono_default_xml_filename);
    //std::string ref_time_xml_filename = configuration_->property("GNSS-SDR.SUPL_gps_ref_time_xml", ref_time_default_xml_filename);
    //std::string ref_location_xml_filename = configuration_->property("GNSS-SDR.SUPL_gps_ref_location_xml", ref_location_default_xml_filename);

    // RINEX version
    int conf_rinex_version;
    conf_rinex_version = configuration->property(role + ".rinex_version", 0);

    // make PVT object
    pvt_ = gps_l1_ca_make_pvt_cc(in_streams_,
            dump_,
            dump_filename_,
            averaging_depth,
            flag_averaging,
            output_rate_ms,
            display_rate_ms,
            flag_nmea_tty_port,
            nmea_dump_filename,
            nmea_dump_devname,
            flag_rtcm_server,
            flag_rtcm_tty_port,
            rtcm_tcp_port,
            rtcm_station_id,
            rtcm_msg_rate_ms,
            rtcm_dump_devname,
            conf_rinex_version );

    DLOG(INFO) << "pvt(" << pvt_->unique_id() << ")";
}


bool GpsL1CaPvt::save_assistance_to_XML()
{
    // return variable (true == succeeded)
    bool ret = false;

    LOG(INFO) << "SUPL: Try to save GPS ephemeris to XML file " << eph_xml_filename_;
    std::map<int,Gps_Ephemeris> eph_map = pvt_->get_GPS_L1_ephemeris_map();

    if (eph_map.size() > 0)
        {
            try
                {
                    std::ofstream ofs(eph_xml_filename_.c_str(), std::ofstream::trunc | std::ofstream::out);
                    boost::archive::xml_oarchive xml(ofs);
                    xml << boost::serialization::make_nvp("GNSS-SDR_ephemeris_map", eph_map);
                    ofs.close();
                    LOG(INFO) << "Saved GPS L1 Ephemeris map data";
                }
            catch (std::exception& e)
                {
                    LOG(WARNING) << e.what();
                    return false;
                }
            return true;
        }
    else
        {
            LOG(WARNING) << "Failed to save Ephemeris, map is empty";
            return false;
        }
    // Only try to save {utc, iono, ref time, ref location} if SUPL is enabled
    //    bool enable_gps_supl_assistance = configuration_->property("GNSS-SDR.SUPL_gps_enabled", false);
    //    if (enable_gps_supl_assistance == true)
    //        {
    //            // try to save utc model xml file
    //            std::map<int, Gps_Utc_Model> utc_copy = global_gps_utc_model_map.get_map_copy();
    //            if (supl_client_acquisition_.save_utc_map_xml(utc_xml_filename, utc_copy) == true)
    //                {
    //                    LOG(INFO) << "SUPL: Successfully saved UTC Model XML file";
    //                    //ret = true;
    //                }
    //            else
    //                {
    //                    LOG(INFO) << "SUPL: Error while trying to save utc XML file";
    //                    //ret = false;
    //                }
    //            // try to save iono model xml file
    //            std::map<int, Gps_Iono> iono_copy = global_gps_iono_map.get_map_copy();
    //            if (supl_client_acquisition_.save_iono_map_xml(iono_xml_filename, iono_copy) == true)
    //                {
    //                    LOG(INFO) << "SUPL: Successfully saved IONO Model XML file";
    //                    //ret = true;
    //                }
    //            else
    //                {
    //                    LOG(INFO) << "SUPL: Error while trying to save iono XML file";
    //                    //ret = false;
    //                }
    //            // try to save ref time xml file
    //            std::map<int, Gps_Ref_Time> ref_time_copy = global_gps_ref_time_map.get_map_copy();
    //            if (supl_client_acquisition_.save_ref_time_map_xml(ref_time_xml_filename, ref_time_copy) == true)
    //                {
    //                    LOG(INFO) << "SUPL: Successfully saved Ref Time XML file";
    //                    //ret = true;
    //                }
    //            else
    //                {
    //                    LOG(INFO) << "SUPL: Error while trying to save ref time XML file";
    //                    //ref = false;
    //                }
    //            // try to save ref location xml file
    //            std::map<int, Gps_Ref_Location> ref_location_copy = global_gps_ref_location_map.get_map_copy();
    //            if (supl_client_acquisition_.save_ref_location_map_xml(ref_location_xml_filename, ref_location_copy) == true)
    //                {
    //                    LOG(INFO) << "SUPL: Successfully saved Ref Location XML file";
    //                    //ref = true;
    //                }
    //            else
    //                {
    //                    LOG(INFO) << "SUPL: Error while trying to save ref location XML file";
    //                    //ret = false;
    //                }
    //        }
    return ret;
}


GpsL1CaPvt::~GpsL1CaPvt()
{
    save_assistance_to_XML();
}


void GpsL1CaPvt::connect(gr::top_block_sptr top_block)
{
    if(top_block) { /* top_block is not null */};
    // Nothing to connect internally
    DLOG(INFO) << "nothing to connect internally";
}


void GpsL1CaPvt::disconnect(gr::top_block_sptr top_block)
{
    if(top_block) { /* top_block is not null */};
    // Nothing to disconnect
}


gr::basic_block_sptr GpsL1CaPvt::get_left_block()
{
    return pvt_;
}


gr::basic_block_sptr GpsL1CaPvt::get_right_block()
{
    return pvt_;
}
