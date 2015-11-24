/*!
 * \file rtcm_printer.cc
 * \brief Implementation of a RTCM 3.2 printer for GNSS-SDR
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

#include "rtcm_printer.h"
#include <fcntl.h>    // for O_RDWR
#include <termios.h>  // for tcgetattr
#include <gflags/gflags.h>
#include <glog/logging.h>

using google::LogMessage;


Rtcm_Printer::Rtcm_Printer(std::string filename, bool flag_rtcm_tty_port, std::string rtcm_dump_devname)
{
    rtcm_filename = filename;
    rtcm_file_descriptor.open(rtcm_filename.c_str(), std::ios::out);
    if (rtcm_file_descriptor.is_open())
        {
            DLOG(INFO) << "RTCM printer writing on " << rtcm_filename.c_str();
        }

    rtcm_devname = rtcm_dump_devname;
    if (flag_rtcm_tty_port == true)
        {
            rtcm_dev_descriptor = init_serial(rtcm_devname.c_str());
            if (rtcm_dev_descriptor != -1)
                {
                    DLOG(INFO) << "RTCM printer writing on " << rtcm_devname.c_str();
                }
        }
    else
        {
            rtcm_dev_descriptor = -1;
        }
    rtcm = std::make_shared<Rtcm>();
}




Rtcm_Printer::~Rtcm_Printer()
{
    if (rtcm_file_descriptor.is_open())
        {
            long pos;
            pos = rtcm_file_descriptor.tellp();
            rtcm_file_descriptor.close();
            if (pos == 0)
                {
                    if(remove(rtcm_filename.c_str()) != 0) LOG(INFO) << "Error deleting temporary RTCM file";
                }
        }
    close_serial();
}


bool Rtcm_Printer::Print_Rtcm_MT1001(const Gps_Ephemeris& gps_eph, double obs_time, const std::map<int, Gnss_Synchro> & pseudoranges)
{
    std::string m1001 = rtcm->print_MT1001(gps_eph, obs_time, pseudoranges);
    Rtcm_Printer::Print_Message(m1001);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MT1019(const Gps_Ephemeris & gps_eph)
{
    std::string m1019 = rtcm->print_MT1019(gps_eph);
    Rtcm_Printer::Print_Message(m1019);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MT1045(const Galileo_Ephemeris & gal_eph)
{
    std::string m1045 = rtcm->print_MT1045(gal_eph);
    Rtcm_Printer::Print_Message(m1045);
    return true;
}


int Rtcm_Printer::init_serial(std::string serial_device)
{
    /*
     * Opens the serial device and sets the default baud rate for a RTCM transmission (9600,8,N,1)
     */
    int fd = 0;
    struct termios options;
    long BAUD;
    long DATABITS;
    long STOPBITS;
    long PARITYON;
    long PARITY;

    fd = open(serial_device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) return fd;  // failed to open TTY port

    if(fcntl(fd, F_SETFL, 0) == -1) LOG(INFO) << "Error enabling direct I/O";    // clear all flags on descriptor, enable direct I/O
    tcgetattr(fd, &options);  // read serial port options

    BAUD  = B9600;
    //BAUD  =  B38400;
    DATABITS = CS8;
    STOPBITS = 0;
    PARITYON = 0;
    PARITY = 0;

    options.c_cflag = BAUD | DATABITS | STOPBITS | PARITYON | PARITY | CLOCAL | CREAD;
    // enable receiver, set 8 bit data, ignore control lines
    //options.c_cflag |= (CLOCAL | CREAD | CS8);
    options.c_iflag = IGNPAR;

    // set the new port options
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}



void Rtcm_Printer::close_serial()
{
    if (rtcm_dev_descriptor != -1)
        {
            close(rtcm_dev_descriptor);
        }
}



bool Rtcm_Printer::Print_Message(std::string message)
{
    //write to file
    try
    {
            rtcm_file_descriptor << message << std::endl;
    }
    catch(std::exception ex)
    {
            DLOG(INFO) << "RTCM printer can not write on output file" << rtcm_filename.c_str();
            return false;
    }

    //write to serial device
    if (rtcm_dev_descriptor != -1)
        {
            if(write(rtcm_dev_descriptor, message.c_str(), message.length()) == -1)
                {
                    DLOG(INFO) << "RTCM printer cannot write on serial device" << rtcm_devname.c_str();
                    std::cout << "RTCM printer cannot write on serial device" << rtcm_devname.c_str() << std::endl;
                    return false;
                }
        }
    return true;
}


std::string Rtcm_Printer::print_MT1005_test()
{
    std::string test = rtcm->print_MT1005_test();
    return test;
}
