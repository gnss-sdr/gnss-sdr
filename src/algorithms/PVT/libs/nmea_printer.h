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

#ifndef GNSS_SDR_NMEA_PRINTER_H
#define GNSS_SDR_NMEA_PRINTER_H

#include <boost/date_time/posix_time/ptime.hpp>  // for ptime
#include <fstream>                               // for ofstream
#include <memory>                                // for shared_ptr
#include <string>                                // for string

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */


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
     * \brief Default destructor.
     */
    ~Nmea_Printer();

    /*!
     * \brief Print NMEA PVT and satellite info to the initialized device
     */
    bool Print_Nmea_Line(const Rtklib_Solver* const pvt_data);

private:
    int init_serial(const std::string& serial_device);  // serial port control
    void close_serial() const;
    std::string get_GPGGA() const;  // fix data
    std::string get_GPGSV() const;  // satellite data
    std::string get_GPGSA() const;  // overall satellite reception data
    std::string get_GPRMC() const;  // minimum recommended data
    std::string get_UTC_NMEA_time(const boost::posix_time::ptime d_position_UTC_time) const;
    std::string longitude_to_hm(double longitude) const;
    std::string latitude_to_hm(double lat) const;
    char checkSum(const std::string& sentence) const;

    const Rtklib_Solver* d_PVT_data;

    std::ofstream nmea_file_descriptor;  // Output file stream for NMEA log file

    std::string nmea_filename;  // String with the NMEA log filename
    std::string nmea_base_path;
    std::string nmea_devname;

    int nmea_dev_descriptor;  // NMEA serial device descriptor (i.e. COM port)
    bool d_flag_nmea_output_file;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_NMEA_PRINTER_H
