/*!
 * \file an_packet_printer.cc
 * \brief Implementation of a class that prints PVT solutions in a serial device
 * following a custom version of the Advanced Navigation Packet Protocol
 * \author Carles Fernandez-Prades, 2021. cfernandez(at)cttc.es
 * \author Miguel Angel Gomez Lopez, 2021. gomezlma(at)inta.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "an_packet_printer.h"
#include "rtklib_solver.h"  // for Rtklib_Solver
#include <boost/date_time/posix_time/posix_time.hpp>
#include <glog/logging.h>  // for DLOG
#include <cmath>           // for std::sqrt, M_PI
#include <fcntl.h>         // for fcntl
#include <iostream>        // for std::cerr
#include <termios.h>       // values for termios
#include <unistd.h>        // for write(), read(), close()


An_Packet_Printer::An_Packet_Printer(const std::string& an_dump_devname)
{
    d_an_devname = an_dump_devname;

    d_an_dev_descriptor = init_serial(d_an_devname);
    if (d_an_dev_descriptor != -1)
        {
            DLOG(INFO) << "AN Printer writing on " << d_an_devname;
        }
}


An_Packet_Printer::~An_Packet_Printer()
{
    try
        {
            close_serial();
        }
    catch (const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
}


bool An_Packet_Printer::print_packet(const Rtklib_Solver* const pvt_data)
{
    an_packet_t an_packet{};
    raw_gnss_packet_t raw_packet{};

    update_raw_gnss_packet(&raw_packet, pvt_data);
    encode_raw_gnss_packet(&raw_packet, &an_packet);

    if (d_an_dev_descriptor != -1)
        {
            if (write(d_an_dev_descriptor, &an_packet, sizeof(an_packet)) == -1)
                {
                    LOG(ERROR) << "Advanced Navigation printer cannot write on serial device " << d_an_devname;
                    return false;
                }
        }
    return true;
}


void An_Packet_Printer::close_serial() const
{
    if (d_an_dev_descriptor != -1)
        {
            close(d_an_dev_descriptor);
        }
}

/*
 * @brief update_raw_gnss_packet
 * @param  raw_gnss_packet_t* Pointer to a structure that contains
 *         the output information.
 * @param  NavData_t* pointer to input packet with all the information
 * @reval  None
 */
void An_Packet_Printer::update_raw_gnss_packet(raw_gnss_packet_t* _packet, const Rtklib_Solver* const pvt) const
{
    const boost::posix_time::ptime time_origin(boost::gregorian::date(1970, 1, 1));
    boost::date_time::time_duration unix_t = pvt->get_position_UTC_time() - time_origin;

    _packet->unix_time_stamp = unix_t.total_seconds();
    _packet->microseconds = unix_t.total_microseconds() - unix_t.total_seconds() * 1e6;
    _packet->latitude = static_cast<double>(pvt->get_latitude()) / 1.0e-7 * (M_PI / 180.0);
    _packet->longitude = static_cast<double>(pvt->get_latitude()) / 1.0e-7 * (M_PI / 180.0);
    _packet->height = static_cast<double>(pvt->get_height()) / 100.0;
    _packet->velocity[0] = static_cast<float>(pvt->pvt_sol.rr[3]) / 100.0;
    _packet->velocity[1] = static_cast<float>(pvt->pvt_sol.rr[4]) / 100.0;
    _packet->velocity[2] = static_cast<float>(pvt->pvt_sol.rr[5]) / 100.0;
    _packet->position_standard_deviation[0] = std::sqrt(static_cast<float>(pvt->pvt_sol.qr[0]));
    _packet->position_standard_deviation[1] = std::sqrt(static_cast<float>(pvt->pvt_sol.qr[1]));
    _packet->position_standard_deviation[2] = std::sqrt(static_cast<float>(pvt->pvt_sol.qr[2]));

    _packet->status.b.fix_type = 2;  //!< 2: 3D fix
    _packet->status.b.velocity_valid = 0;
    _packet->status.b.time_valid = 0;
    _packet->status.b.external_gnss = 0;
}


/*
 * @brief encode_raw_gnss_packet
 * @param  raw_gnss_packet_t* Pointer to a  structure that contains
 *         the information.
 * @param  an_packet_t* pointer to output packet
 * @reval  None
 */
void An_Packet_Printer::encode_raw_gnss_packet(raw_gnss_packet_t* raw_packet, an_packet_t* _packet) const
{
    _packet->id = 201;
    _packet->length = RAW_GNSS_PACKET_LENGTH;
    if (_packet != NULL)
        {
            uint8_t offset = 0;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&raw_packet->unix_time_stamp), offset, _packet->data, sizeof(raw_packet->unix_time_stamp));
            offset += 4;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&raw_packet->microseconds), offset, _packet->data, sizeof(raw_packet->microseconds));
            offset += 4;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&raw_packet->latitude), offset, _packet->data, sizeof(raw_packet->latitude));
            offset += 8;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&raw_packet->longitude), offset, _packet->data, sizeof(raw_packet->longitude));
            offset += 8;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&raw_packet->height), offset, _packet->data, sizeof(raw_packet->height));
            offset += 8;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&raw_packet->velocity[0]), offset, _packet->data, sizeof(raw_packet->velocity[0]));
            offset += 4;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&raw_packet->velocity[1]), offset, _packet->data, sizeof(raw_packet->velocity[1]));
            offset += 4;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&raw_packet->velocity[2]), offset, _packet->data, sizeof(raw_packet->velocity[2]));
            offset += 4;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&raw_packet->position_standard_deviation[0]), offset, _packet->data, sizeof(raw_packet->position_standard_deviation[0]));
            offset += 4;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&raw_packet->position_standard_deviation[1]), offset, _packet->data, sizeof(raw_packet->position_standard_deviation[1]));
            offset += 4;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&raw_packet->position_standard_deviation[2]), offset, _packet->data, sizeof(raw_packet->position_standard_deviation[2]));
            offset += 4;
            // This could be optimized to add just zeros
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&raw_packet->reserved[0]), offset, _packet->data, sizeof(raw_packet->reserved[0]));
            offset += 4;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&raw_packet->reserved[1]), offset, _packet->data, sizeof(raw_packet->reserved[1]));
            offset += 4;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&raw_packet->reserved[2]), offset, _packet->data, sizeof(raw_packet->reserved[2]));
            offset += 4;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&raw_packet->reserved[3]), offset, _packet->data, sizeof(raw_packet->reserved[3]));
            offset += 4;

            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&raw_packet->status.r), offset, _packet->data, sizeof(raw_packet->status));
            offset += 2;
        }
    an_packet_encode(_packet);
}


/*
 * Function to encode an an_packet
 */
void An_Packet_Printer::an_packet_encode(an_packet_t* an_packet) const
{
    uint16_t crc;
    an_packet->header[1] = an_packet->id;
    an_packet->header[2] = an_packet->length;
    crc = calculate_crc16(an_packet->data, an_packet->length);
    memcpy(&an_packet->header[3], &crc, sizeof(uint16_t));
    an_packet->header[0] = calculate_header_lrc(&an_packet->header[1]);
}


/*
 * Function to calculate a 4 byte LRC
 */
uint8_t An_Packet_Printer::calculate_header_lrc(const uint8_t* data) const
{
    return ((data[0] + data[1] + data[2] + data[3]) ^ 0xFF) + 1;
}


void An_Packet_Printer::LSB_bytes_to_array(void* _in, int offset, uint8_t* _out, uint8_t var_size) const
{
    switch (var_size)
        {
        case 1:
            {
                uint8_t* tmp = reinterpret_cast<uint8_t*>(_in);
                for (int i = 0; i < var_size; i++)
                    {
                        _out[offset + i] = (*tmp >> 8 * i) & 0xFF;
                    }
                break;
            }
        case 2:
            {
                uint16_t* tmp = reinterpret_cast<uint16_t*>(_in);
                for (int i = 0; i < var_size; i++)
                    {
                        _out[offset + i] = (*tmp >> 8 * i) & 0xFF;
                    }
                break;
            }
        case 4:
            {
                uint32_t* tmp = reinterpret_cast<uint32_t*>(_in);
                for (int i = 0; i < var_size; i++)
                    {
                        _out[offset + i] = (*tmp >> 8 * i) & 0xFF;
                    }
                break;
            }
        case 8:
            {
                uint64_t* tmp = reinterpret_cast<uint64_t*>(_in);
                for (int i = 0; i < var_size; i++)
                    {
                        _out[offset + i] = (*tmp >> 8 * i) & 0xFF;
                    }
                break;
            }
        default:
            break;
        }
}


/*
 * Function to calculate the CRC16 of data
 * CRC16-CCITT
 * Initial value = 0xFFFF
 * Polynomial = x^16 + x^12 + x^5 + x^0
 */
uint16_t An_Packet_Printer::calculate_crc16(const void* data, uint16_t length) const
{
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(data);
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++)
        {
            crc = static_cast<uint16_t>((crc << 8) ^ d_crc16_table[(crc >> 8) ^ bytes[i]]);
        }
    return crc;
}


/*
 * Opens the serial device and sets the default baud rate for a NMEA transmission (9600,8,N,1)
 */
int An_Packet_Printer::init_serial(const std::string& serial_device)
{
    int fd = 0;
    // clang-format off
    struct termios options{};
    // clang-format on
    const int64_t BAUD = B9600;  // BAUD  =  B38400;
    const int64_t DATABITS = CS8;
    const int64_t STOPBITS = 0;
    const int64_t PARITYON = 0;
    const int64_t PARITY = 0;

    fd = open(serial_device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_CLOEXEC);
    if (fd == -1)
        {
            return fd;  // failed to open TTY port
        }

    if (fcntl(fd, F_SETFL, 0) == -1)
        {
            LOG(INFO) << "Error enabling direct I/O";  // clear all flags on descriptor, enable direct I/O
        }
    tcgetattr(fd, &options);  // read serial port options

    options.c_cflag = BAUD | DATABITS | STOPBITS | PARITYON | PARITY | CLOCAL | CREAD;
    // enable receiver, set 8 bit data, ignore control lines
    // options.c_cflag |= (CLOCAL | CREAD | CS8);
    options.c_iflag = IGNPAR;

    // set the new port options
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}
