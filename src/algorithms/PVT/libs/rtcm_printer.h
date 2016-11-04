/*!
 * \file rtcm_printer.h
 * \brief Interface of a RTCM 3.2 printer for GNSS-SDR
 * This class provides a implementation of a subset of the RTCM Standard 10403.2
 * for Differential GNSS Services
 *
 * \author Carles Fernandez-Prades, 2014. cfernandez(at)cttc.es
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

#ifndef GNSS_SDR_RTCM_PRINTER_H_
#define GNSS_SDR_RTCM_PRINTER_H_

#include <fstream>  // std::ofstream
#include <iostream> // std::cout
#include <memory>   // std::shared_ptr
#include "rtcm.h"

/*!
 * \brief This class provides a implementation of a subset of the RTCM Standard 10403.2 messages
 */
class Rtcm_Printer
{
public:
    /*!
     * \brief Default constructor.
     */
    Rtcm_Printer(std::string filename, bool flag_rtcm_server, bool flag_rtcm_tty_port, unsigned short rtcm_tcp_port, unsigned short rtcm_station_id, std::string rtcm_dump_filename, bool time_tag_name = true);

    /*!
     * \brief Default destructor.
     */
    ~Rtcm_Printer();

    bool Print_Rtcm_MT1001(const Gps_Ephemeris& gps_eph, double obs_time, const std::map<int, Gnss_Synchro> & observables);
    bool Print_Rtcm_MT1002(const Gps_Ephemeris& gps_eph, double obs_time, const std::map<int, Gnss_Synchro> & observables);
    bool Print_Rtcm_MT1003(const Gps_Ephemeris& gps_eph, const Gps_CNAV_Ephemeris& cnav_eph, double obs_time, const std::map<int, Gnss_Synchro> & observables);
    bool Print_Rtcm_MT1004(const Gps_Ephemeris& gps_eph, const Gps_CNAV_Ephemeris& cnav_eph, double obs_time, const std::map<int, Gnss_Synchro> & observables);
    bool Print_Rtcm_MT1019(const Gps_Ephemeris & gps_eph); //<! GPS Ephemeris, should be broadcast in the event that the IODC does not match the IODE, and every 2 minutes.
    bool Print_Rtcm_MT1045(const Galileo_Ephemeris & gal_eph); //<! Galileo Ephemeris, should be broadcast every 2 minutes

    bool Print_Rtcm_MSM(unsigned int msm_number, const Gps_Ephemeris & gps_eph,
            const Gps_CNAV_Ephemeris & gps_cnav_eph,
            const Galileo_Ephemeris & gal_eph,
            double obs_time,
            const std::map<int, Gnss_Synchro> & observables,
            unsigned int clock_steering_indicator,
            unsigned int external_clock_indicator,
            int smooth_int,
            bool divergence_free,
            bool more_messages);

    std::string print_MT1005_test(); //<!  For testing purposes
    unsigned int lock_time(const Gps_Ephemeris& eph, double obs_time, const Gnss_Synchro & gnss_synchro);
    unsigned int lock_time(const Gps_CNAV_Ephemeris& eph, double obs_time, const Gnss_Synchro & gnss_synchro);
    unsigned int lock_time(const Galileo_Ephemeris& eph, double obs_time, const Gnss_Synchro & gnss_synchro);

private:
    std::string rtcm_filename; // String with the RTCM log filename
    std::ofstream rtcm_file_descriptor; // Output file stream for RTCM log file
    std::string rtcm_devname;
    unsigned short port;
    unsigned short station_id;
    int rtcm_dev_descriptor; // RTCM serial device descriptor (i.e. COM port)
    int init_serial (std::string serial_device); //serial port control
    void close_serial ();
    std::shared_ptr<Rtcm> rtcm;
    bool Print_Message(const std::string & message);
};

#endif
