/*!
 * \file gps_l1_ca_pvt_cc.h
 * \brief Interface of a Position Velocity and Time computation block for GPS L1 C/A
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_GPS_L1_CA_PVT_CC_H
#define GNSS_SDR_GPS_L1_CA_PVT_CC_H

#include <fstream>
#include <string>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <gnuradio/block.h>
#include "nmea_printer.h"
#include "kml_printer.h"
#include "rinex_printer.h"
#include "geojson_printer.h"
#include "rtcm_printer.h"
#include "gps_l1_ca_ls_pvt.h"


class gps_l1_ca_pvt_cc;

typedef boost::shared_ptr<gps_l1_ca_pvt_cc> gps_l1_ca_pvt_cc_sptr;

gps_l1_ca_pvt_cc_sptr gps_l1_ca_make_pvt_cc(unsigned int n_channels,
                                            bool dump,
                                            std::string dump_filename,
                                            int averaging_depth,
                                            bool flag_averaging,
                                            int output_rate_ms,
                                            int display_rate_ms,
                                            bool flag_nmea_tty_port,
                                            std::string nmea_dump_filename,
                                            std::string nmea_dump_devname,
                                            bool flag_rtcm_server,
                                            bool flag_rtcm_tty_port,
                                            unsigned short rtcm_tcp_port,
                                            unsigned short rtcm_station_id,
                                            std::map<int,int> rtcm_msg_rate_ms,
                                            std::string rtcm_dump_devname,
                                            int rinex_version
);

/*!
 * \brief This class implements a block that computes the PVT solution
 */
class gps_l1_ca_pvt_cc : public gr::block
{
private:
    friend gps_l1_ca_pvt_cc_sptr gps_l1_ca_make_pvt_cc(unsigned int nchannels,
                                                       bool dump,
                                                       std::string dump_filename,
                                                       int averaging_depth,
                                                       bool flag_averaging,
                                                       int output_rate_ms,
                                                       int display_rate_ms,
                                                       bool flag_nmea_tty_port,
                                                       std::string nmea_dump_filename,
                                                       std::string nmea_dump_devname,
                                                       bool flag_rtcm_server,
                                                       bool flag_rtcm_tty_port,
                                                       unsigned short rtcm_tcp_port,
                                                       unsigned short rtcm_station_id,
                                                       std::map<int,int> rtcm_msg_rate_ms,
                                                       std::string rtcm_dump_devname,
                                                       int rinex_version);
    gps_l1_ca_pvt_cc(unsigned int nchannels,
                     bool dump,
                     std::string dump_filename,
                     int averaging_depth,
                     bool flag_averaging,
                     int output_rate_ms,
                     int display_rate_ms,
                     bool flag_nmea_tty_port,
                     std::string nmea_dump_filename,
                     std::string nmea_dump_devname,
                     bool flag_rtcm_server,
                     bool flag_rtcm_tty_port,
                     unsigned short rtcm_tcp_port,
                     unsigned short rtcm_station_id,
                     std::map<int,int> rtcm_msg_rate_ms,
                     std::string rtcm_dump_devname,
                     int rinex_version);

    void msg_handler_telemetry(pmt::pmt_t msg);

    bool d_dump;
    bool b_rinex_header_written;
    bool b_rinex_sbs_header_written;
    bool b_rinex_header_updated;
    bool b_rtcm_writing_started;
    unsigned short d_rtcm_tcp_port;
    unsigned short d_rtcm_station_id;
    int d_rtcm_MT1019_rate_ms;
    int d_rtcm_MSM_rate_ms;

    void print_receiver_status(Gnss_Synchro** channels_synchronization_data);
    int d_last_status_print_seg; //for status printer

    unsigned int d_nchannels;
    std::string d_dump_filename;
    std::ofstream d_dump_file;
    int d_averaging_depth;
    bool d_flag_averaging;
    int d_output_rate_ms;
    int d_display_rate_ms;
    long unsigned int d_sample_counter;
    long unsigned int d_last_sample_nav_output;

    std::shared_ptr<Rinex_Printer> rp;
    std::shared_ptr<Kml_Printer> d_kml_printer;
    std::shared_ptr<Nmea_Printer> d_nmea_printer;
    std::shared_ptr<GeoJSON_Printer> d_geojson_printer;
    std::shared_ptr<Rtcm_Printer> d_rtcm_printer;
    double d_rx_time;
    std::shared_ptr<gps_l1_ca_ls_pvt> d_ls_pvt;

    std::map<int,Gnss_Synchro> gnss_observables_map;

    bool first_fix;
    key_t sysv_msg_key;
    int sysv_msqid;
    typedef struct  {
        long mtype;//required by sys v message
        double ttff;
    } ttff_msgbuf;
    bool send_sys_v_ttff_msg(ttff_msgbuf ttff);

public:

    /*!
     * \brief Get latest set of GPS L1 ephemeris from PVT block
     *
     * It is used to save the assistance data at the receiver shutdown
     */
    std::map<int,Gps_Ephemeris> get_GPS_L1_ephemeris_map();

    ~gps_l1_ca_pvt_cc (); //!< Default destructor

    int general_work (int noutput_items, gr_vector_int &ninput_items,
            gr_vector_const_void_star &input_items, gr_vector_void_star &output_items); //!< PVT Signal Processing
};

#endif
