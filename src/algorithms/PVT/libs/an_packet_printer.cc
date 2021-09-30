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
#include <cmath>            // for std::sqrt, M_PI
#include <fcntl.h>          // for fcntl
#include <iostream>         // for std::cerr
#include <limits>           // std::numeric_limits
#include <termios.h>        // values for termios
#include <unistd.h>         // for write(), read(), close()


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
    std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
    uint8_t num_gps_sats = 0;
    uint8_t num_gal_sats = 0;
    uint32_t microseconds = 0;
    int index = 0;

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
                            if (index < 6)
                                {
                                    _packet->sats[index].prn = static_cast<uint8_t>(gnss_observables_iter->second.PRN);
                                    _packet->sats[index].snr = static_cast<uint8_t>(gnss_observables_iter->second.CN0_dB_hz);
                                    int16_t doppler = 0;
                                    double Carrier_Doppler_hz = gnss_observables_iter->second.CN0_dB_hz;
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
                                    microseconds = static_cast<uint32_t>(static_cast<double>(gnss_observables_iter->second.Tracking_sample_counter) / static_cast<double>(gnss_observables_iter->second.fs)) * 1e6;
                                    index++;
                                }
                            break;
                        case 'E':
                            num_gal_sats++;
                            if (index < 6)
                                {
                                    _packet->sats[index].prn = static_cast<uint8_t>(gnss_observables_iter->second.PRN) + 100;
                                    _packet->sats[index].snr = static_cast<uint8_t>(gnss_observables_iter->second.CN0_dB_hz);
                                    int16_t doppler = 0;
                                    double Carrier_Doppler_hz = gnss_observables_iter->second.CN0_dB_hz;
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
                                    microseconds = static_cast<uint32_t>(static_cast<double>(gnss_observables_iter->second.Tracking_sample_counter) / static_cast<double>(gnss_observables_iter->second.fs)) * 1e6;
                                    index++;
                                }
                            break;
                        default:
                            break;
                        }
                }
        }

    _packet->nsvfix = static_cast<uint8_t>(pvt->get_num_valid_observations());
    _packet->gps_satellites = num_gps_sats;
    _packet->galileo_satellites = num_gal_sats;
    _packet->microseconds = microseconds;
    _packet->latitude = static_cast<double>(pvt->get_latitude()) * (M_PI / 180.0);
    _packet->longitude = static_cast<double>(pvt->get_longitude()) * (M_PI / 180.0);
    _packet->height = static_cast<double>(pvt->get_height());
    _packet->velocity[0] = static_cast<float>(pvt->get_rx_vel()[1]);
    _packet->velocity[1] = static_cast<float>(pvt->get_rx_vel()[0]);
    _packet->velocity[2] = static_cast<float>(-pvt->get_rx_vel()[2]);

    uint16_t status = 0;
    // Set 3D fix
    status = status & 0b00000011;  // set bit 0 and 1
    // Set Doppler velocity valid
    status = status & 0b00000100;  // set bit 2
    // Set Time valid
    status = status & 0b00001000;  // set bit 3
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
    _packet->id = SDR_GNSS_PACKET_ID;
    _packet->length = SDR_GNSS_PACKET_LENGTH;
    if (_packet != nullptr)
        {
            uint8_t offset = 0;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->nsvfix), offset, _packet->data, sizeof(sdr_gnss_packet->nsvfix));
            offset += 1;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->gps_satellites), offset, _packet->data, sizeof(sdr_gnss_packet->gps_satellites));
            offset += 1;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->galileo_satellites), offset, _packet->data, sizeof(sdr_gnss_packet->galileo_satellites));
            offset += 1;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->microseconds), offset, _packet->data, sizeof(sdr_gnss_packet->microseconds));
            offset += 4;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->latitude), offset, _packet->data, sizeof(sdr_gnss_packet->latitude));
            offset += 8;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->longitude), offset, _packet->data, sizeof(sdr_gnss_packet->longitude));
            offset += 8;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->height), offset, _packet->data, sizeof(sdr_gnss_packet->height));
            offset += 8;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->velocity[0]), offset, _packet->data, sizeof(sdr_gnss_packet->velocity[0]));
            offset += 4;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->velocity[1]), offset, _packet->data, sizeof(sdr_gnss_packet->velocity[1]));
            offset += 4;
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->velocity[2]), offset, _packet->data, sizeof(sdr_gnss_packet->velocity[2]));
            offset += 4;
            for (int i = 0; i < 6; i++)
                {
                    LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->sats[i].prn), offset, _packet->data, sizeof(sdr_gnss_packet->sats[i].prn));
                    offset += 1;
                    LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->sats[i].snr), offset, _packet->data, sizeof(sdr_gnss_packet->sats[i].snr));
                    offset += 1;
                    LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->sats[i].doppler), offset, _packet->data, sizeof(sdr_gnss_packet->sats[i].doppler));
                    offset += 2;
                }
            offset += 4;  // reserved
            LSB_bytes_to_array(reinterpret_cast<uint8_t*>(&sdr_gnss_packet->status), offset, _packet->data, sizeof(sdr_gnss_packet->status));
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
