/*!
 * \file nmea_printer.h
 * \brief Interface of a NMEA 2.1 printer for GNSS-SDR
 * This class provides a implementation of a subset of the NMEA-0183 standard for interfacing
 * marine electronic devices as defined by the National Marine Electronics Association (NMEA).
 * See https://www.nmea.org/ for the NMEA 183 standard
 *
 * \author Javier Arribas, 2012. jarribas(at)cttc.es
 *
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
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_NMEA_PRINTER_H_
#define GNSS_SDR_NMEA_PRINTER_H_

#include <boost/date_time/posix_time/ptime.hpp>  // for ptime
#include <fstream>                               // for ofstream
#include <memory>                                // for shared_ptr
#include <string>                                // for string

class Rtklib_Solver;

/*!
 * \brief This class provides a implementation of a subset of the NMEA-0183 standard for interfacing
 * marine electronic devices as defined by the National Marine Electronics Association (NMEA).
 *
 * See https://en.wikipedia.org/wiki/NMEA_0183
 */
class Nmea_Printer
{
public:
    /*!
     * \brief Default constructor.
     */
    Nmea_Printer(const std::string& filename, bool flag_nmea_output_file, bool flag_nmea_tty_port, std::string nmea_dump_devname, const std::string& base_path = ".");

    /*!
     * \brief Print NMEA PVT and satellite info to the initialized device
     */
    bool Print_Nmea_Line(const std::shared_ptr<Rtklib_Solver>& pvt_data, bool print_average_values);

    /*!
     * \brief Default destructor.
     */
    ~Nmea_Printer();

private:
    std::string nmea_filename;  // String with the NMEA log filename
    std::string nmea_base_path;
    std::ofstream nmea_file_descriptor;  // Output file stream for NMEA log file
    std::string nmea_devname;
    int nmea_dev_descriptor;  // NMEA serial device descriptor (i.e. COM port)
    std::shared_ptr<Rtklib_Solver> d_PVT_data;
    int init_serial(const std::string& serial_device);  // serial port control
    void close_serial();
    std::string get_GPGGA();  // fix data
    std::string get_GPGSV();  // satellite data
    std::string get_GPGSA();  // overall satellite reception data
    std::string get_GPRMC();  // minimum recommended data
    std::string get_UTC_NMEA_time(boost::posix_time::ptime d_position_UTC_time);
    std::string longitude_to_hm(double longitude);
    std::string latitude_to_hm(double lat);
    char checkSum(const std::string& sentence);
    bool print_avg_pos;
    bool d_flag_nmea_output_file;
};

#endif
