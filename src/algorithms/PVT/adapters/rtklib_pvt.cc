/*!
 * \file rtklib_pvt.cc
 * \brief Interface of a Position Velocity and Time computation block
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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


#include "rtklib_pvt.h"
#include "pvt_conf.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <glog/logging.h>
#if OLD_BOOST
#include <boost/math/common_factor_rt.hpp>
namespace bc = boost::math;
#else
#include <boost/integer/common_factor_rt.hpp>
namespace bc = boost::integer;
#endif


using google::LogMessage;


bool RtklibPvt::get_latest_PVT(double* longitude_deg,
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

void RtklibPvt::clear_ephemeris()
{
    pvt_->clear_ephemeris();
}

RtklibPvt::RtklibPvt(ConfigurationInterface* configuration,
    std::string role,
    unsigned int in_streams,
    unsigned int out_streams) : role_(role),
                                in_streams_(in_streams),
                                out_streams_(out_streams)
{
    Pvt_Conf pvt_output_parameters = Pvt_Conf();
    // dump parameters
    std::string default_dump_filename = "./pvt.dat";
    std::string default_nmea_dump_filename = "./nmea_pvt.nmea";
    std::string default_nmea_dump_devname = "/dev/tty1";
    std::string default_rtcm_dump_devname = "/dev/pts/1";
    DLOG(INFO) << "role " << role;
    pvt_output_parameters.dump = configuration->property(role + ".dump", false);
    pvt_output_parameters.dump_filename = configuration->property(role + ".dump_filename", default_dump_filename);
    pvt_output_parameters.dump_mat = configuration->property(role + ".dump_mat", true);

    // output rate
    pvt_output_parameters.output_rate_ms = configuration->property(role + ".output_rate_ms", 500);

    // display rate
    pvt_output_parameters.display_rate_ms = configuration->property(role + ".display_rate_ms", 500);

    // NMEA Printer settings
    pvt_output_parameters.flag_nmea_tty_port = configuration->property(role + ".flag_nmea_tty_port", false);
    pvt_output_parameters.nmea_dump_filename = configuration->property(role + ".nmea_dump_filename", default_nmea_dump_filename);
    std::string nmea_dump_devname = configuration->property(role + ".nmea_dump_devname", default_nmea_dump_devname);

    // RINEX version
    pvt_output_parameters.rinex_version = configuration->property(role + ".rinex_version", 3);
    if (FLAGS_RINEX_version.compare("3.01") == 0)
        {
            pvt_output_parameters.rinex_version = 3;
        }
    else if (FLAGS_RINEX_version.compare("3.02") == 0)
        {
            pvt_output_parameters.rinex_version = 3;
        }
    else if (FLAGS_RINEX_version.compare("3") == 0)
        {
            pvt_output_parameters.rinex_version = 3;
        }
    else if (FLAGS_RINEX_version.compare("2.11") == 0)
        {
            pvt_output_parameters.rinex_version = 2;
        }
    else if (FLAGS_RINEX_version.compare("2.10") == 0)
        {
            pvt_output_parameters.rinex_version = 2;
        }
    else if (FLAGS_RINEX_version.compare("2") == 0)
        {
            pvt_output_parameters.rinex_version = 2;
        }
    pvt_output_parameters.rinexobs_rate_ms = bc::lcm(configuration->property(role + ".rinexobs_rate_ms", 1000), pvt_output_parameters.output_rate_ms);
    pvt_output_parameters.rinexnav_rate_ms = bc::lcm(configuration->property(role + ".rinexnav_rate_ms", 6000), pvt_output_parameters.output_rate_ms);

    // RTCM Printer settings
    pvt_output_parameters.flag_rtcm_tty_port = configuration->property(role + ".flag_rtcm_tty_port", false);
    pvt_output_parameters.rtcm_dump_devname = configuration->property(role + ".rtcm_dump_devname", default_rtcm_dump_devname);
    pvt_output_parameters.flag_rtcm_server = configuration->property(role + ".flag_rtcm_server", false);
    pvt_output_parameters.rtcm_tcp_port = configuration->property(role + ".rtcm_tcp_port", 2101);
    pvt_output_parameters.rtcm_station_id = configuration->property(role + ".rtcm_station_id", 1234);
    // RTCM message rates: least common multiple with output_rate_ms
    int rtcm_MT1019_rate_ms = bc::lcm(configuration->property(role + ".rtcm_MT1019_rate_ms", 5000), pvt_output_parameters.output_rate_ms);
    int rtcm_MT1020_rate_ms = bc::lcm(configuration->property(role + ".rtcm_MT1020_rate_ms", 5000), pvt_output_parameters.output_rate_ms);
    int rtcm_MT1045_rate_ms = bc::lcm(configuration->property(role + ".rtcm_MT1045_rate_ms", 5000), pvt_output_parameters.output_rate_ms);
    int rtcm_MSM_rate_ms = bc::lcm(configuration->property(role + ".rtcm_MSM_rate_ms", 1000), pvt_output_parameters.output_rate_ms);
    int rtcm_MT1077_rate_ms = bc::lcm(configuration->property(role + ".rtcm_MT1077_rate_ms", rtcm_MSM_rate_ms), pvt_output_parameters.output_rate_ms);
    int rtcm_MT1087_rate_ms = bc::lcm(configuration->property(role + ".rtcm_MT1087_rate_ms", rtcm_MSM_rate_ms), pvt_output_parameters.output_rate_ms);
    int rtcm_MT1097_rate_ms = bc::lcm(configuration->property(role + ".rtcm_MT1097_rate_ms", rtcm_MSM_rate_ms), pvt_output_parameters.output_rate_ms);
    //std::map<int, int> rtcm_msg_rate_ms;
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

    // Infer the type of receiver
    /*
     *   TYPE  |  RECEIVER
     *     0   |  Unknown
     *     1   |  GPS L1 C/A
     *     2   |  GPS L2C
     *     3   |  GPS L5
     *     4   |  Galileo E1B
     *     5   |  Galileo E5a
     *     6   |  Galileo E5b
     *     7   |  GPS L1 C/A + GPS L2C
     *     8   |  GPS L1 C/A + GPS L5
     *     9   |  GPS L1 C/A + Galileo E1B
     *    10   |  GPS L1 C/A + Galileo E5a
     *    11   |  GPS L1 C/A + Galileo E5b
     *    12   |  Galileo E1B + GPS L2C
     *    13   |  Galileo E1B + GPS L5
     *    14   |  Galileo E1B + Galileo E5a
     *    15   |  Galileo E1B + Galileo E5b
     *    16   |  GPS L2C + GPS L5
     *    17   |  GPS L2C + Galileo E5a
     *    18   |  GPS L2C + Galileo E5b
     *    19   |  GPS L5 + Galileo E5a
     *    20   |  GPS L5 + Galileo E5b
     *    21   |  GPS L1 C/A + Galileo E1B + GPS L2C
     *    22   |  GPS L1 C/A + Galileo E1B + GPS L5
     *    23   |  GLONASS L1 C/A
     *    24   |  GLONASS L2 C/A
     *    25   |  GLONASS L1 C/A + GLONASS L2 C/A
     *    26   |  GPS L1 C/A + GLONASS L1 C/A
     *    27   |  Galileo E1B + GLONASS L1 C/A
     *    28   |  GPS L2C + GLONASS L1 C/A
     */
    int gps_1C_count = configuration->property("Channels_1C.count", 0);
    int gps_2S_count = configuration->property("Channels_2S.count", 0);
    int gps_L5_count = configuration->property("Channels_L5.count", 0);
    int gal_1B_count = configuration->property("Channels_1B.count", 0);
    int gal_E5a_count = configuration->property("Channels_5X.count", 0);
    int gal_E5b_count = configuration->property("Channels_7X.count", 0);
    int glo_1G_count = configuration->property("Channels_1G.count", 0);
    int glo_2G_count = configuration->property("Channels_2G.count", 0);

    if ((gps_1C_count != 0) && (gps_2S_count == 0) && (gps_L5_count == 0) && (gal_1B_count == 0) && (gal_E5a_count == 0) && (gal_E5b_count == 0) && (glo_1G_count == 0) && (glo_2G_count == 0)) pvt_output_parameters.type_of_receiver = 1;  // L1
    if ((gps_1C_count == 0) && (gps_2S_count != 0) && (gps_L5_count == 0) && (gal_1B_count == 0) && (gal_E5a_count == 0) && (gal_E5b_count == 0) && (glo_1G_count == 0) && (glo_2G_count == 0)) pvt_output_parameters.type_of_receiver = 2;
    if ((gps_1C_count == 0) && (gps_2S_count == 0) && (gps_L5_count != 0) && (gal_1B_count == 0) && (gal_E5a_count == 0) && (gal_E5b_count == 0) && (glo_1G_count == 0) && (glo_2G_count == 0)) pvt_output_parameters.type_of_receiver = 3;  // L5
    if ((gps_1C_count == 0) && (gps_2S_count == 0) && (gps_L5_count == 0) && (gal_1B_count != 0) && (gal_E5a_count == 0) && (gal_E5b_count == 0) && (glo_1G_count == 0) && (glo_2G_count == 0)) pvt_output_parameters.type_of_receiver = 4;  // E1
    if ((gps_1C_count == 0) && (gps_2S_count == 0) && (gps_L5_count == 0) && (gal_1B_count == 0) && (gal_E5a_count != 0) && (gal_E5b_count == 0) && (glo_1G_count == 0) && (glo_2G_count == 0)) pvt_output_parameters.type_of_receiver = 5;  // E5a
    if ((gps_1C_count == 0) && (gps_2S_count == 0) && (gps_L5_count == 0) && (gal_1B_count == 0) && (gal_E5a_count == 0) && (gal_E5b_count != 0) && (glo_1G_count == 0) && (glo_2G_count == 0)) pvt_output_parameters.type_of_receiver = 6;

    if ((gps_1C_count != 0) && (gps_2S_count != 0) && (gps_L5_count == 0) && (gal_1B_count == 0) && (gal_E5a_count == 0) && (gal_E5b_count == 0) && (glo_1G_count == 0) && (glo_2G_count == 0)) pvt_output_parameters.type_of_receiver = 7;
    if ((gps_1C_count != 0) && (gps_2S_count == 0) && (gps_L5_count != 0) && (gal_1B_count == 0) && (gal_E5a_count == 0) && (gal_E5b_count == 0) && (glo_1G_count == 0) && (glo_2G_count == 0)) pvt_output_parameters.type_of_receiver = 8;  // L1+L5
    if ((gps_1C_count != 0) && (gps_2S_count == 0) && (gps_L5_count == 0) && (gal_1B_count != 0) && (gal_E5a_count == 0) && (gal_E5b_count == 0) && (glo_1G_count == 0) && (glo_2G_count == 0)) pvt_output_parameters.type_of_receiver = 9;  // L1+E1
    if ((gps_1C_count != 0) && (gps_2S_count == 0) && (gps_L5_count == 0) && (gal_1B_count == 0) && (gal_E5a_count != 0) && (gal_E5b_count == 0) && (glo_1G_count == 0) && (glo_2G_count == 0)) pvt_output_parameters.type_of_receiver = 10;
    if ((gps_1C_count != 0) && (gps_2S_count == 0) && (gps_L5_count == 0) && (gal_1B_count == 0) && (gal_E5a_count == 0) && (gal_E5b_count != 0) && (glo_1G_count == 0) && (glo_2G_count == 0)) pvt_output_parameters.type_of_receiver = 11;
    if ((gps_1C_count == 0) && (gps_2S_count != 0) && (gps_L5_count == 0) && (gal_1B_count != 0) && (gal_E5a_count == 0) && (gal_E5b_count == 0) && (glo_1G_count == 0) && (glo_2G_count == 0)) pvt_output_parameters.type_of_receiver = 12;
    if ((gps_1C_count == 0) && (gps_2S_count == 0) && (gps_L5_count != 0) && (gal_1B_count == 0) && (gal_E5a_count != 0) && (gal_E5b_count == 0) && (glo_1G_count == 0) && (glo_2G_count == 0)) pvt_output_parameters.type_of_receiver = 13;  // L5+E5a
    if ((gps_1C_count == 0) && (gps_2S_count == 0) && (gps_L5_count == 0) && (gal_1B_count != 0) && (gal_E5a_count != 0) && (gal_E5b_count == 0) && (glo_1G_count == 0) && (glo_2G_count == 0)) pvt_output_parameters.type_of_receiver = 14;
    if ((gps_1C_count == 0) && (gps_2S_count == 0) && (gps_L5_count == 0) && (gal_1B_count != 0) && (gal_E5a_count == 0) && (gal_E5b_count != 0) && (glo_1G_count == 0) && (glo_2G_count == 0)) pvt_output_parameters.type_of_receiver = 15;
    //if( (gps_1C_count == 0) && (gps_2S_count == 0)  && (gps_L5_count == 0) && (gal_1B_count == 0) && (gal_E5a_count == 0) && (gal_E5b_count == 0)) pvt_output_parameters.type_of_receiver = 16;
    if ((gps_1C_count == 0) && (gps_2S_count != 0) && (gps_L5_count == 0) && (gal_1B_count == 0) && (gal_E5a_count != 0) && (gal_E5b_count == 0) && (glo_1G_count == 0) && (glo_2G_count == 0)) pvt_output_parameters.type_of_receiver = 17;
    if ((gps_1C_count == 0) && (gps_2S_count != 0) && (gps_L5_count == 0) && (gal_1B_count == 0) && (gal_E5a_count == 0) && (gal_E5b_count != 0) && (glo_1G_count == 0) && (glo_2G_count == 0)) pvt_output_parameters.type_of_receiver = 18;
    //if( (gps_1C_count == 0) && (gps_2S_count == 0) && (gps_L5_count == 0) && (gal_1B_count == 0) && (gal_E5a_count == 0) && (gal_E5b_count == 0)) pvt_output_parameters.type_of_receiver = 19;
    //if( (gps_1C_count == 0) && (gps_2S_count == 0) && (gps_L5_count == 0) && (gal_1B_count == 0) && (gal_E5a_count == 0) && (gal_E5b_count == 0)) pvt_output_parameters.type_of_receiver = 20;
    if ((gps_1C_count != 0) && (gps_2S_count != 0) && (gps_L5_count == 0) && (gal_1B_count != 0) && (gal_E5a_count == 0) && (gal_E5b_count == 0) && (glo_1G_count == 0) && (glo_2G_count == 0)) pvt_output_parameters.type_of_receiver = 21;
    //if( (gps_1C_count == 0) && (gps_2S_count == 0) && (gps_L5_count == 0) && (gal_1B_count == 0) && (gal_E5a_count == 0) && (gal_E5b_count = 0)) pvt_output_parameters.type_of_receiver = 22;
    if ((gps_1C_count == 0) && (gps_2S_count == 0) && (gps_L5_count == 0) && (gal_1B_count == 0) && (gal_E5a_count == 0) && (gal_E5b_count == 0) && (glo_1G_count != 0)) pvt_output_parameters.type_of_receiver = 23;
    if ((gps_1C_count == 0) && (gps_2S_count == 0) && (gps_L5_count == 0) && (gal_1B_count == 0) && (gal_E5a_count == 0) && (gal_E5b_count == 0) && (glo_1G_count == 0) && (glo_2G_count != 0)) pvt_output_parameters.type_of_receiver = 24;
    if ((gps_1C_count == 0) && (gps_2S_count == 0) && (gps_L5_count == 0) && (gal_1B_count == 0) && (gal_E5a_count == 0) && (gal_E5b_count == 0) && (glo_1G_count != 0) && (glo_2G_count != 0)) pvt_output_parameters.type_of_receiver = 25;
    if ((gps_1C_count != 0) && (gps_2S_count == 0) && (gps_L5_count == 0) && (gal_1B_count == 0) && (gal_E5a_count == 0) && (gal_E5b_count == 0) && (glo_1G_count != 0) && (glo_2G_count == 0)) pvt_output_parameters.type_of_receiver = 26;
    if ((gps_1C_count == 0) && (gps_2S_count == 0) && (gps_L5_count == 0) && (gal_1B_count != 0) && (gal_E5a_count == 0) && (gal_E5b_count == 0) && (glo_1G_count != 0) && (glo_2G_count == 0)) pvt_output_parameters.type_of_receiver = 27;
    if ((gps_1C_count == 0) && (gps_2S_count != 0) && (gps_L5_count == 0) && (gal_1B_count == 0) && (gal_E5a_count == 0) && (gal_E5b_count == 0) && (glo_1G_count != 0) && (glo_2G_count == 0)) pvt_output_parameters.type_of_receiver = 28;
    if ((gps_1C_count != 0) && (gps_2S_count == 0) && (gps_L5_count == 0) && (gal_1B_count == 0) && (gal_E5a_count == 0) && (gal_E5b_count == 0) && (glo_1G_count == 0) && (glo_2G_count != 0)) pvt_output_parameters.type_of_receiver = 29;
    if ((gps_1C_count == 0) && (gps_2S_count == 0) && (gps_L5_count == 0) && (gal_1B_count != 0) && (gal_E5a_count == 0) && (gal_E5b_count == 0) && (glo_1G_count == 0) && (glo_2G_count != 0)) pvt_output_parameters.type_of_receiver = 30;
    if ((gps_1C_count == 0) && (gps_2S_count != 0) && (gps_L5_count == 0) && (gal_1B_count == 0) && (gal_E5a_count == 0) && (gal_E5b_count == 0) && (glo_1G_count == 0) && (glo_2G_count != 0)) pvt_output_parameters.type_of_receiver = 31;

    if ((gps_1C_count != 0) && (gps_2S_count == 0) && (gps_L5_count != 0) && (gal_1B_count != 0) && (gal_E5a_count != 0) && (gal_E5b_count == 0) && (glo_1G_count == 0) && (glo_2G_count == 0)) pvt_output_parameters.type_of_receiver = 32;  // L1+E1+L5+E5a

    // RTKLIB PVT solver options
    // Settings 1
    int positioning_mode = -1;
    std::string default_pos_mode("Single");
    std::string positioning_mode_str = configuration->property(role + ".positioning_mode", default_pos_mode);  //  (PMODE_XXX) see src/algorithms/libs/rtklib/rtklib.h
    if (positioning_mode_str.compare("Single") == 0) positioning_mode = PMODE_SINGLE;
    if (positioning_mode_str.compare("Static") == 0) positioning_mode = PMODE_STATIC;
    if (positioning_mode_str.compare("Kinematic") == 0) positioning_mode = PMODE_KINEMA;
    if (positioning_mode_str.compare("PPP_Static") == 0) positioning_mode = PMODE_PPP_STATIC;
    if (positioning_mode_str.compare("PPP_Kinematic") == 0) positioning_mode = PMODE_PPP_KINEMA;

    if (positioning_mode == -1)
        {
            //warn user and set the default
            std::cout << "WARNING: Bad specification of positioning mode." << std::endl;
            std::cout << "positioning_mode possible values: Single / Static / Kinematic / PPP_Static / PPP_Kinematic" << std::endl;
            std::cout << "positioning_mode specified value: " << positioning_mode_str << std::endl;
            std::cout << "Setting positioning_mode to Single" << std::endl;
            positioning_mode = PMODE_SINGLE;
        }

    int num_bands = 0;

    if ((gps_1C_count > 0) || (gal_1B_count > 0) || (glo_1G_count > 0)) num_bands = 1;
    if (((gps_1C_count > 0) || (gal_1B_count > 0) || (glo_1G_count > 0)) && ((gps_2S_count > 0) || (glo_2G_count > 0))) num_bands = 2;
    if (((gps_1C_count > 0) || (gal_1B_count > 0) || (glo_1G_count > 0)) && ((gal_E5a_count > 0) || (gal_E5b_count > 0) || (gps_L5_count > 0))) num_bands = 2;
    if (((gps_1C_count > 0) || (gal_1B_count > 0) || (glo_1G_count > 0)) && ((gps_2S_count > 0) || (glo_2G_count > 0)) && ((gal_E5a_count > 0) || (gal_E5b_count > 0) || (gps_L5_count > 0))) num_bands = 3;

    int number_of_frequencies = configuration->property(role + ".num_bands", num_bands); /* (1:L1, 2:L1+L2, 3:L1+L2+L5) */
    if ((number_of_frequencies < 1) || (number_of_frequencies > 3))
        {
            //warn user and set the default
            number_of_frequencies = num_bands;
        }

    double elevation_mask = configuration->property(role + ".elevation_mask", 15.0);
    if ((elevation_mask < 0.0) || (elevation_mask > 90.0))
        {
            //warn user and set the default
            LOG(WARNING) << "Erroneous Elevation Mask. Setting to default value of 15.0 degrees";
            elevation_mask = 15.0;
        }

    int dynamics_model = configuration->property(role + ".dynamics_model", 0); /*  dynamics model (0:none, 1:velocity, 2:accel) */
    if ((dynamics_model < 0) || (dynamics_model > 2))
        {
            //warn user and set the default
            LOG(WARNING) << "Erroneous Dynamics Model configuration. Setting to default value of (0:none)";
            dynamics_model = 0;
        }

    std::string default_iono_model("OFF");
    std::string iono_model_str = configuration->property(role + ".iono_model", default_iono_model); /*  (IONOOPT_XXX) see src/algorithms/libs/rtklib/rtklib.h */
    int iono_model = -1;
    if (iono_model_str.compare("OFF") == 0) iono_model = IONOOPT_OFF;
    if (iono_model_str.compare("Broadcast") == 0) iono_model = IONOOPT_BRDC;
    if (iono_model_str.compare("SBAS") == 0) iono_model = IONOOPT_SBAS;
    if (iono_model_str.compare("Iono-Free-LC") == 0) iono_model = IONOOPT_IFLC;
    if (iono_model_str.compare("Estimate_STEC") == 0) iono_model = IONOOPT_EST;
    if (iono_model_str.compare("IONEX") == 0) iono_model = IONOOPT_TEC;
    if (iono_model == -1)
        {
            //warn user and set the default
            std::cout << "WARNING: Bad specification of ionospheric model." << std::endl;
            std::cout << "iono_model possible values: OFF / Broadcast / SBAS / Iono-Free-LC / Estimate_STEC / IONEX" << std::endl;
            std::cout << "iono_model specified value: " << iono_model_str << std::endl;
            std::cout << "Setting iono_model to OFF" << std::endl;
            iono_model = IONOOPT_OFF; /* 0: ionosphere option: correction off */
        }

    std::string default_trop_model("OFF");
    int trop_model = -1;
    std::string trop_model_str = configuration->property(role + ".trop_model", default_trop_model); /*  (TROPOPT_XXX) see src/algorithms/libs/rtklib/rtklib.h */
    if (trop_model_str.compare("OFF") == 0) trop_model = TROPOPT_OFF;
    if (trop_model_str.compare("Saastamoinen") == 0) trop_model = TROPOPT_SAAS;
    if (trop_model_str.compare("SBAS") == 0) trop_model = TROPOPT_SBAS;
    if (trop_model_str.compare("Estimate_ZTD") == 0) trop_model = TROPOPT_EST;
    if (trop_model_str.compare("Estimate_ZTD_Grad") == 0) trop_model = TROPOPT_ESTG;
    if (trop_model == -1)
        {
            //warn user and set the default
            std::cout << "WARNING: Bad specification of tropospheric model." << std::endl;
            std::cout << "trop_model possible values: OFF / Saastamoinen / SBAS / Estimate_ZTD / Estimate_ZTD_Grad" << std::endl;
            std::cout << "trop_model specified value: " << trop_model_str << std::endl;
            std::cout << "Setting trop_model to OFF" << std::endl;
            trop_model = TROPOPT_OFF;
        }

    /* RTKLIB positioning options */
    int sat_PCV = 0; /*  Set whether the satellite antenna PCV (phase center variation) model is used or not. This feature requires a Satellite Antenna PCV File. */
    int rec_PCV = 0; /*  Set whether the receiver antenna PCV (phase center variation) model is used or not. This feature requires a Receiver Antenna PCV File. */

    /* Set whether the phase windup correction for PPP modes is applied or not. Only applicable to PPP‐* modes.*/
    int phwindup = configuration->property(role + ".phwindup", 0);

    /* Set whether the GPS Block IIA satellites in eclipse are excluded or not.
    The eclipsing Block IIA satellites often degrade the PPP solutions due to unpredicted behavior of yaw‐attitude. Only applicable to PPP‐* modes.*/
    int reject_GPS_IIA = configuration->property(role + ".reject_GPS_IIA", 0);

    /* Set whether RAIM (receiver autonomous integrity monitoring) FDE (fault detection and exclusion) feature is enabled or not.
    In case of RAIM FDE enabled, a satellite is excluded if SSE (sum of squared errors) of residuals is over a threshold.
    The excluded satellite is selected to indicate the minimum SSE. */
    int raim_fde = configuration->property(role + ".raim_fde", 0);

    int earth_tide = configuration->property(role + ".earth_tide", 0);

    int nsys = 0;
    if ((gps_1C_count > 0) || (gps_2S_count > 0) || (gps_L5_count > 0)) nsys += SYS_GPS;
    if ((gal_1B_count > 0) || (gal_E5a_count > 0) || (gal_E5b_count > 0)) nsys += SYS_GAL;
    if ((glo_1G_count > 0) || (glo_2G_count > 0)) nsys += SYS_GLO;
    int navigation_system = configuration->property(role + ".navigation_system", nsys); /* (SYS_XXX) see src/algorithms/libs/rtklib/rtklib.h */
    if ((navigation_system < 1) || (navigation_system > 255))                           /* GPS: 1   SBAS: 2   GPS+SBAS: 3 Galileo: 8  Galileo+GPS: 9 GPS+SBAS+Galileo: 11 All: 255 */
        {
            //warn user and set the default
            LOG(WARNING) << "Erroneous Navigation System. Setting to default value of (0:none)";
            navigation_system = nsys;
        }

    // Settings 2
    std::string default_gps_ar("Continuous");
    std::string integer_ambiguity_resolution_gps_str = configuration->property(role + ".AR_GPS", default_gps_ar); /* Integer Ambiguity Resolution mode for GPS (0:off,1:continuous,2:instantaneous,3:fix and hold,4:ppp-ar) */
    int integer_ambiguity_resolution_gps = -1;
    if (integer_ambiguity_resolution_gps_str.compare("OFF") == 0) integer_ambiguity_resolution_gps = ARMODE_OFF;
    if (integer_ambiguity_resolution_gps_str.compare("Continuous") == 0) integer_ambiguity_resolution_gps = ARMODE_CONT;
    if (integer_ambiguity_resolution_gps_str.compare("Instantaneous") == 0) integer_ambiguity_resolution_gps = ARMODE_INST;
    if (integer_ambiguity_resolution_gps_str.compare("Fix-and-Hold") == 0) integer_ambiguity_resolution_gps = ARMODE_FIXHOLD;
    if (integer_ambiguity_resolution_gps_str.compare("PPP-AR") == 0) integer_ambiguity_resolution_gps = ARMODE_PPPAR;
    if (integer_ambiguity_resolution_gps == -1)
        {
            //warn user and set the default
            std::cout << "WARNING: Bad specification of GPS ambiguity resolution method." << std::endl;
            std::cout << "AR_GPS possible values: OFF / Continuous / Instantaneous / Fix-and-Hold / PPP-AR" << std::endl;
            std::cout << "AR_GPS specified value: " << integer_ambiguity_resolution_gps_str << std::endl;
            std::cout << "Setting AR_GPS to OFF" << std::endl;
            integer_ambiguity_resolution_gps = ARMODE_OFF;
        }

    int integer_ambiguity_resolution_glo = configuration->property(role + ".AR_GLO", 1); /* Integer Ambiguity Resolution mode for GLONASS (0:off,1:on,2:auto cal,3:ext cal) */
    if ((integer_ambiguity_resolution_glo < 0) || (integer_ambiguity_resolution_glo > 3))
        {
            //warn user and set the default
            LOG(WARNING) << "Erroneous Integer Ambiguity Resolution for GLONASS . Setting to default value of (1:on)";
            integer_ambiguity_resolution_glo = 1;
        }

    int integer_ambiguity_resolution_bds = configuration->property(role + ".AR_DBS", 1); /* Integer Ambiguity Resolution mode for BEIDOU (0:off,1:on) */
    if ((integer_ambiguity_resolution_bds < 0) || (integer_ambiguity_resolution_bds > 1))
        {
            //warn user and set the default
            LOG(WARNING) << "Erroneous Integer Ambiguity Resolution for BEIDOU . Setting to default value of (1:on)";
            integer_ambiguity_resolution_bds = 1;
        }

    double min_ratio_to_fix_ambiguity = configuration->property(role + ".min_ratio_to_fix_ambiguity", 3.0); /* Set the integer ambiguity validation threshold for ratio‐test,
                                                                                                               which uses the ratio of squared residuals of the best integer vector to the second‐best vector. */

    int min_lock_to_fix_ambiguity = configuration->property(role + ".min_lock_to_fix_ambiguity", 0); /* Set the minimum lock count to fix integer ambiguity.
                                                                                                         If the lock count is less than the value, the ambiguity is excluded from the fixed integer vector. */

    double min_elevation_to_fix_ambiguity = configuration->property(role + ".min_elevation_to_fix_ambiguity", 0.0); /* Set the minimum elevation (deg) to fix integer ambiguity.
                                                                                                                        If the elevation of the satellite is less than the value, the ambiguity is excluded from the fixed integer vector. */

    int outage_reset_ambiguity = configuration->property(role + ".outage_reset_ambiguity", 5); /* Set the outage count to reset ambiguity. If the data outage count is over the value, the estimated ambiguity is reset to the initial value.  */

    double slip_threshold = configuration->property(role + ".slip_threshold", 0.05); /* set the cycle‐slip threshold (m) of geometry‐free LC carrier‐phase difference between epochs */

    double threshold_reject_gdop = configuration->property(role + ".threshold_reject_gdop", 30.0); /* reject threshold of GDOP. If the GDOP is over the value, the observable is excluded for the estimation process as an outlier. */

    double threshold_reject_innovation = configuration->property(role + ".threshold_reject_innovation", 30.0); /* reject threshold of innovation (m). If the innovation is over the value, the observable is excluded for the estimation process as an outlier. */

    int number_filter_iter = configuration->property(role + ".number_filter_iter", 1); /* Set the number of iteration in the measurement update of the estimation filter.
                                                                                         If the baseline length is very short like 1 m, the iteration may be effective to handle
                                                                                         the nonlinearity of measurement equation. */

    /// Statistics
    double bias_0 = configuration->property(role + ".bias_0", 30.0);

    double iono_0 = configuration->property(role + ".iono_0", 0.03);

    double trop_0 = configuration->property(role + ".trop_0", 0.3);

    double sigma_bias = configuration->property(role + ".sigma_bias", 1e-4); /* Set the process noise standard deviation of carrier‐phase
                                                                                bias (ambiguity) (cycle/sqrt(s)) */

    double sigma_iono = configuration->property(role + ".sigma_iono", 1e-3); /* Set the process noise standard deviation of vertical ionospheric delay per 10 km baseline (m/sqrt(s)). */

    double sigma_trop = configuration->property(role + ".sigma_trop", 1e-4); /* Set the process noise standard deviation of zenith tropospheric delay (m/sqrt(s)). */

    double sigma_acch = configuration->property(role + ".sigma_acch", 1e-1); /* Set the process noise standard deviation of the receiver acceleration as
                                                                                the horizontal component. (m/s2/sqrt(s)). If Receiver Dynamics is set to OFF, they are not used. */

    double sigma_accv = configuration->property(role + ".sigma_accv", 1e-2); /* Set the process noise standard deviation of the receiver acceleration as
                                                                                the vertical component. (m/s2/sqrt(s)). If Receiver Dynamics is set to OFF, they are not used. */

    double sigma_pos = configuration->property(role + ".sigma_pos", 0.0);

    double code_phase_error_ratio_l1 = configuration->property(role + ".code_phase_error_ratio_l1", 100.0);
    double code_phase_error_ratio_l2 = configuration->property(role + ".code_phase_error_ratio_l2", 100.0);
    double code_phase_error_ratio_l5 = configuration->property(role + ".code_phase_error_ratio_l5", 100.0);
    double carrier_phase_error_factor_a = configuration->property(role + ".carrier_phase_error_factor_a", 0.003);
    double carrier_phase_error_factor_b = configuration->property(role + ".carrier_phase_error_factor_b", 0.003);

    snrmask_t snrmask = {{}, {{}, {}}};

    prcopt_t rtklib_configuration_options = {
        positioning_mode,                                                                  /* positioning mode (PMODE_XXX) see src/algorithms/libs/rtklib/rtklib.h */
        0,                                                                                 /* solution type (0:forward,1:backward,2:combined) */
        number_of_frequencies,                                                             /* number of frequencies (1:L1, 2:L1+L2, 3:L1+L2+L5)*/
        navigation_system,                                                                 /* navigation system  */
        elevation_mask * D2R,                                                              /* elevation mask angle (degrees) */
        snrmask,                                                                           /* snrmask_t snrmask    SNR mask */
        0,                                                                                 /* satellite ephemeris/clock (EPHOPT_XXX) */
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
        0,                                                                                 /* sbssat_t sbssat  SBAS correction options */
        0,                                                                                 /* sbsion_t sbsion[MAXBAND+1] SBAS satellite selection (0:all) */
        0,                                                                                 /* rover position for fixed mode */
        0,                                                                                 /* base position for relative mode */
                                                                                           /*    0:pos in prcopt,  1:average of single pos, */
                                                                                           /*    2:read from file, 3:rinex header, 4:rtcm pos */
        {code_phase_error_ratio_l1, code_phase_error_ratio_l2, code_phase_error_ratio_l5}, /* eratio[NFREQ] code/phase error ratio */
        {100.0, carrier_phase_error_factor_a, carrier_phase_error_factor_b, 0.0, 1.0},     /* err[5]:  measurement error factor [0]:reserved, [1-3]:error factor a/b/c of phase (m) , [4]:doppler frequency (hz) */
        {bias_0, iono_0, trop_0},                                                          /* std[3]: initial-state std [0]bias,[1]iono [2]trop*/
        {sigma_bias, sigma_iono, sigma_trop, sigma_acch, sigma_accv, sigma_pos},           /* prn[6] process-noise std */
        5e-12,                                                                             /* sclkstab: satellite clock stability (sec/sec) */
        {min_ratio_to_fix_ambiguity, 0.9999, 0.25, 0.1, 0.05, 0.0, 0.0, 0.0},              /* thresar[8]: AR validation threshold */
        min_elevation_to_fix_ambiguity,                                                    /* elevation mask of AR for rising satellite (deg) */
        0.0,                                                                               /* elevation mask to hold ambiguity (deg) */
        slip_threshold,                                                                    /* slip threshold of geometry-free phase (m) */
        30.0,                                                                              /* max difference of time (sec) */
        threshold_reject_innovation,                                                       /* reject threshold of innovation (m) */
        threshold_reject_gdop,                                                             /* reject threshold of gdop */
        {},                                                                                /* double baseline[2] baseline length constraint {const,sigma} (m) */
        {},                                                                                /* double ru[3]  rover position for fixed mode {x,y,z} (ecef) (m) */
        {},                                                                                /* double rb[3]  base position for relative mode {x,y,z} (ecef) (m) */
        {"", ""},                                                                          /* char anttype[2][MAXANT]  antenna types {rover,base}  */
        {{}, {}},                                                                          /* double antdel[2][3]   antenna delta {{rov_e,rov_n,rov_u},{ref_e,ref_n,ref_u}} */
        {},                                                                                /* pcv_t pcvr[2]   receiver antenna parameters {rov,base} */
        {},                                                                                /* unsigned char exsats[MAXSAT]  excluded satellites (1:excluded, 2:included) */
        0,                                                                                 /* max averaging epoches */
        0,                                                                                 /* initialize by restart */
        1,                                                                                 /* output single by dgps/float/fix/ppp outage */
        {"", ""},                                                                          /* char rnxopt[2][256]   rinex options {rover,base} */
        {sat_PCV, rec_PCV, phwindup, reject_GPS_IIA, raim_fde},                            /* posopt[6] positioning options [0]: satellite and receiver antenna PCV model; [1]: interpolate antenna parameters; [2]: apply phase wind-up correction for PPP modes; [3]: exclude measurements of GPS Block IIA satellites satellite [4]: RAIM FDE (fault detection and exclusion) [5]: handle day-boundary clock jump */
        0,                                                                                 /* solution sync mode (0:off,1:on) */
        {{}, {}},                                                                          /* odisp[2][6*11] ocean tide loading parameters {rov,base} */
        {{}, {{}, {}}, {{}, {}}, {}, {}},                                                  /* exterr_t exterr   extended receiver error model */
        0,                                                                                 /* disable L2-AR */
        {}                                                                                 /* char pppopt[256]   ppp option   "-GAP_RESION="  default gap to reset iono parameters (ep) */
    };

    rtkinit(&rtk, &rtklib_configuration_options);

    // Outputs
    bool default_output_enabled = configuration->property(role + ".output_enabled", true);
    pvt_output_parameters.output_enabled = default_output_enabled;
    pvt_output_parameters.rinex_output_enabled = configuration->property(role + ".rinex_output_enabled", default_output_enabled);
    pvt_output_parameters.gpx_output_enabled = configuration->property(role + ".gpx_output_enabled", default_output_enabled);
    pvt_output_parameters.geojson_output_enabled = configuration->property(role + ".geojson_output_enabled", default_output_enabled);
    pvt_output_parameters.kml_output_enabled = configuration->property(role + ".kml_output_enabled", default_output_enabled);
    pvt_output_parameters.xml_output_enabled = configuration->property(role + ".xml_output_enabled", default_output_enabled);
    pvt_output_parameters.nmea_output_file_enabled = configuration->property(role + ".nmea_output_file_enabled", default_output_enabled);
    pvt_output_parameters.rtcm_output_file_enabled = configuration->property(role + ".rtcm_output_file_enabled", default_output_enabled);

    std::string default_output_path = configuration->property(role + ".output_path", std::string("."));
    pvt_output_parameters.output_path = default_output_path;
    pvt_output_parameters.rinex_output_path = configuration->property(role + ".rinex_output_path", default_output_path);
    pvt_output_parameters.gpx_output_path = configuration->property(role + ".gpx_output_path", default_output_path);
    pvt_output_parameters.geojson_output_path = configuration->property(role + ".geojson_output_path", default_output_path);
    pvt_output_parameters.kml_output_path = configuration->property(role + ".kml_output_path", default_output_path);
    pvt_output_parameters.xml_output_path = configuration->property(role + ".xml_output_path", default_output_path);
    pvt_output_parameters.nmea_output_file_path = configuration->property(role + ".nmea_output_file_path", default_output_path);
    pvt_output_parameters.rtcm_output_file_path = configuration->property(role + ".rtcm_output_file_path", default_output_path);

    // make PVT object
    pvt_ = rtklib_make_pvt_cc(in_streams_, pvt_output_parameters, rtk);
    DLOG(INFO) << "pvt(" << pvt_->unique_id() << ")";
    if (out_streams_ > 0)
        {
            LOG(ERROR) << "The PVT block does not have an output stream";
        }
}


RtklibPvt::~RtklibPvt()
{
    rtkfree(&rtk);
}

std::map<int, Gps_Ephemeris> RtklibPvt::get_gps_ephemeris()
{
    return pvt_->get_gps_ephemeris_map();
}
std::map<int, Galileo_Ephemeris> RtklibPvt::get_galileo_ephemeris()
{
    return pvt_->get_galileo_ephemeris_map();
}
std::map<int, Gps_Almanac> RtklibPvt::get_gps_almanac()
{
    return pvt_->get_gps_almanac_map();
}
std::map<int, Galileo_Almanac> RtklibPvt::get_galileo_almanac()
{
    return pvt_->get_galileo_almanac_map();
}

void RtklibPvt::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to connect internally
    DLOG(INFO) << "nothing to connect internally";
}


void RtklibPvt::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to disconnect
}


gr::basic_block_sptr RtklibPvt::get_left_block()
{
    return pvt_;
}


gr::basic_block_sptr RtklibPvt::get_right_block()
{
    return pvt_;  // this is a sink, nothing downstream
}
