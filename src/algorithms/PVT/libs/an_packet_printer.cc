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
#include <glog/logging.h>   // for DLOG
#include <cmath>            // for M_PI
#include <cstring>          // for memcpy
#include <fcntl.h>          // for fcntl
#include <iostream>         // for std::cerr
#include <limits>           // std::numeric_limits
#include <termios.h>        // values for termios
#include <unistd.h>         // for write(), read(), close()


An_Packet_Printer::An_Packet_Printer(const std::string& an_dump_devname)
    : d_start(std::chrono::system_clock::now()),
      d_an_devname(an_dump_devname),
      d_an_dev_descriptor(init_serial(d_an_devname))
{
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


bool An_Packet_Printer::print_packet(const Rtklib_Solver* const pvt_data, const std::map<int, Gnss_Synchro>& gnss_observables_map)
{
    an_packet_t an_packet{};
    sdr_gnss_packet_t sdr_gnss_packet{};

    update_sdr_gnss_packet(&sdr_gnss_packet, pvt_data, gnss_observables_map);
    encode_sdr_gnss_packet(&sdr_gnss_packet, &an_packet);

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
 * @brief update_sdr_gnss_packet
 * @param  sdr_gnss_packet_t* Pointer to a structure that contains
 *         the output information.
 * @param  NavData_t* pointer to input packet with all the information
 * @reval  None
 */
void An_Packet_Printer::update_sdr_gnss_packet(sdr_gnss_packet_t* _packet, const Rtklib_Solver* const pvt, const std::map<int, Gnss_Synchro>& gnss_observables_map) const
{
    std::chrono::time_point<std::chrono::system_clock> this_epoch;
    std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
    uint8_t num_gps_sats = 0;
    uint8_t num_gal_sats = 0;
    int index = 0;
    bool fix_3d = pvt->is_valid_position();
    const int max_reported_sats = *(&_packet->sats + 1) - _packet->sats;

    for (gnss_observables_iter = gnss_observables_map.cbegin();
         gnss_observables_iter != gnss_observables_map.cend();
         ++gnss_observables_iter)
        {
            if (gnss_observables_iter->second.Flag_valid_pseudorange)
                {
                    switch (gnss_observables_iter->second.System)
                        {
                        case 'G':
                            num_gps_sats++;
                            if (index < max_reported_sats)
                                {
                                    _packet->sats[index].prn = static_cast<uint8_t>(gnss_observables_iter->second.PRN);
                                    _packet->sats[index].snr = static_cast<uint8_t>(gnss_observables_iter->second.CN0_dB_hz);
                                    int16_t doppler = 0;
                                    double Carrier_Doppler_hz = gnss_observables_iter->second.Carrier_Doppler_hz;
                                    if (Carrier_Doppler_hz > static_cast<double>(std::numeric_limits<int16_t>::max()))
                                        {
                                            doppler = std::numeric_limits<int16_t>::max();
                                        }
                                    else if (Carrier_Doppler_hz < static_cast<double>(std::numeric_limits<int16_t>::min()))
                                        {
                                            doppler = std::numeric_limits<int16_t>::min();
                                        }
                                    else
                                        {
                                            doppler = static_cast<int16_t>(Carrier_Doppler_hz);
                                        }

                                    _packet->sats[index].doppler = doppler;
                                    index++;
                                }
                            break;
                        case 'E':
                            num_gal_sats++;
                            if (index < max_reported_sats)
                                {
                                    _packet->sats[index].prn = static_cast<uint8_t>(gnss_observables_iter->second.PRN) + 100;
                                    _packet->sats[index].snr = static_cast<uint8_t>(gnss_observables_iter->second.CN0_dB_hz);
                                    int16_t doppler = 0;
                                    double Carrier_Doppler_hz = gnss_observables_iter->second.Carrier_Doppler_hz;
                                    if (Carrier_Doppler_hz > static_cast<double>(std::numeric_limits<int16_t>::max()))
                                        {
                                            doppler = std::numeric_limits<int16_t>::max();
                                        }
                                    else if (Carrier_Doppler_hz < static_cast<double>(std::numeric_limits<int16_t>::min()))
                                        {
                                            doppler = std::numeric_limits<int16_t>::min();
                                        }
                                    else
                                        {
                                            doppler = static_cast<int16_t>(Carrier_Doppler_hz);
                                        }

                                    _packet->sats[index].doppler = doppler;
                                    index++;
                                }
                            break;
                        default:
                            break;
                        }
                }
        }

    this_epoch = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = this_epoch - d_start;
    _packet->nsvfix = static_cast<uint8_t>(pvt->get_num_valid_observations());
    _packet->gps_satellites = num_gps_sats;
    _packet->galileo_satellites = num_gal_sats;
    _packet->microseconds = static_cast<uint32_t>(elapsed_seconds.count() * 1.0e6);
    _packet->latitude = static_cast<double>(pvt->get_latitude()) * (M_PI / 180.0);
    _packet->longitude = static_cast<double>(pvt->get_longitude()) * (M_PI / 180.0);
    _packet->height = static_cast<double>(pvt->get_height());
    _packet->velocity[0] = static_cast<float>(pvt->get_rx_vel()[1]);
    _packet->velocity[1] = static_cast<float>(pvt->get_rx_vel()[0]);
    _packet->velocity[2] = static_cast<float>(-pvt->get_rx_vel()[2]);

    uint16_t status = 0;
    if (fix_3d)
        {
            status = 15;  // Set 3D fix (bit 0 and 1) / Set Doppler velocity valid (bit 2) / Set Time valid (bit 3)
        }
    _packet->status = status;
}


/*
 * @brief encode_sdr_gnss_packet
 * @param  sdr_gnss_packet_t* Pointer to a structure that contains the data.
 * @param  an_packet_t* pointer to output packet
 * @reval  None
 */
void An_Packet_Printer::encode_sdr_gnss_packet(sdr_gnss_packet_t* sdr_gnss_packet, an_packet_t* _packet) const
{
    uint8_t offset = 0;
    LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->nsvfix), offset, _packet->data, sizeof(sdr_gnss_packet->nsvfix));
    offset += sizeof(sdr_gnss_packet->nsvfix);
    LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->gps_satellites), offset, _packet->data, sizeof(sdr_gnss_packet->gps_satellites));
    offset += sizeof(sdr_gnss_packet->gps_satellites);
    LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->galileo_satellites), offset, _packet->data, sizeof(sdr_gnss_packet->galileo_satellites));
    offset += sizeof(sdr_gnss_packet->galileo_satellites);
    LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->microseconds), offset, _packet->data, sizeof(sdr_gnss_packet->microseconds));
    offset += sizeof(sdr_gnss_packet->microseconds);
    LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->latitude), offset, _packet->data, sizeof(sdr_gnss_packet->latitude));
    offset += sizeof(sdr_gnss_packet->latitude);
    LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->longitude), offset, _packet->data, sizeof(sdr_gnss_packet->longitude));
    offset += sizeof(sdr_gnss_packet->longitude);
    LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->height), offset, _packet->data, sizeof(sdr_gnss_packet->height));
    offset += sizeof(sdr_gnss_packet->height);
    LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->velocity[0]), offset, _packet->data, sizeof(sdr_gnss_packet->velocity[0]));
    offset += sizeof(sdr_gnss_packet->velocity[0]);
    LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->velocity[1]), offset, _packet->data, sizeof(sdr_gnss_packet->velocity[1]));
    offset += sizeof(sdr_gnss_packet->velocity[1]);
    LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->velocity[2]), offset, _packet->data, sizeof(sdr_gnss_packet->velocity[2]));
    offset += sizeof(sdr_gnss_packet->velocity[2]);
    for (auto& sat : sdr_gnss_packet->sats)
        {
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sat.prn), offset, _packet->data, sizeof(sat.prn));
            offset += sizeof(sat.prn);
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sat.snr), offset, _packet->data, sizeof(sat.snr));
            offset += sizeof(sat.snr);
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sat.doppler), offset, _packet->data, sizeof(sat.doppler));
            offset += sizeof(sat.doppler);
        }

    offset = static_cast<uint8_t>(SDR_GNSS_PACKET_LENGTH - sizeof(sdr_gnss_packet->status));
    LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->status), offset, _packet->data, sizeof(sdr_gnss_packet->status));

    an_packet_encode(_packet);
}


/*
 * Function to encode an an_packet
 */
void An_Packet_Printer::an_packet_encode(an_packet_t* an_packet) const
{
    uint16_t crc;
    an_packet->header[1] = SDR_GNSS_PACKET_ID;
    crc = calculate_crc16(an_packet->data, SDR_GNSS_PACKET_LENGTH);
    memcpy(&an_packet->header[2], &crc, sizeof(uint16_t));
    an_packet->header[0] = calculate_header_lrc(&an_packet->header[1]);
}


/*
 * Function to calculate a 4 byte LRC
 */
uint8_t An_Packet_Printer::calculate_header_lrc(const uint8_t* data) const
{
    return ((data[0] + data[1] + data[2]) ^ 0xFF) + 1;
}


void An_Packet_Printer::LSB_bytes_to_array(void* _in, int offset, uint8_t* _out, uint8_t var_size) const
{
    switch (var_size)
        {
        case 1:
            {
                auto* tmp = reinterpret_cast<uint8_t*>(_in);
                for (int i = 0; i < var_size; i++)
                    {
                        _out[offset + i] = (*tmp >> 8 * i) & 0xFF;
                    }
                break;
            }
        case 2:
            {
                auto* tmp = reinterpret_cast<uint16_t*>(_in);
                for (int i = 0; i < var_size; i++)
                    {
                        _out[offset + i] = (*tmp >> 8 * i) & 0xFF;
                    }
                break;
            }
        case 4:
            {
                auto* tmp = reinterpret_cast<uint32_t*>(_in);
                for (int i = 0; i < var_size; i++)
                    {
                        _out[offset + i] = (*tmp >> 8 * i) & 0xFF;
                    }
                break;
            }
        case 8:
            {
                auto* tmp = reinterpret_cast<uint64_t*>(_in);
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
    const auto* bytes = reinterpret_cast<const uint8_t*>(data);
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++)
        {
            crc = static_cast<uint16_t>((crc << 8) ^ d_crc16_table[(crc >> 8) ^ bytes[i]]);
        }
    return crc;
}


/*
 * Opens the serial device and sets the default baud rate for a transmission (115200,8,N,1)
 */
int An_Packet_Printer::init_serial(const std::string& serial_device)
{
    int fd = 0;
    // clang-format off
    struct termios options{};
    // clang-format on
    const int64_t BAUD = B115200;
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
