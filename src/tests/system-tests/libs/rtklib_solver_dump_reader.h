/*!
 * \file rtklib_solver_dump_reader.h
 * \brief Helper file for unit testing
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

#ifndef GNSS_SDR_RTKLIB_SOLVER_DUMP_READER_H
#define GNSS_SDR_RTKLIB_SOLVER_DUMP_READER_H

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

class rtklib_solver_dump_reader
{
public:
    ~rtklib_solver_dump_reader();
    bool read_binary_obs();
    bool restart();
    int64_t num_epochs();
    bool open_obs_file(std::string out_file);

    // rtklib_solver dump variables
    // TOW
    uint32_t TOW_at_current_symbol_ms;
    // WEEK
    uint32_t week;
    // PVT GPS time
    double RX_time;
    // User clock offset [s]
    double clk_offset_s;
    // ECEF POS X,Y,X [m] + ECEF VEL X,Y,X [m/s] (6 x double)
    double rr[6];
    //  position variance/covariance (m^2) {c_xx,c_yy,c_zz,c_xy,c_yz,c_zx} (6 x double)
    double qr[6];

    // GEO user position Latitude [deg]
    double latitude;
    // GEO user position Longitude [deg]
    double longitude;
    // GEO user position Height [m]
    double height;

    // NUMBER OF VALID SATS
    uint8_t ns;
    // RTKLIB solution status
    uint8_t status;
    // RTKLIB solution type (0:xyz-ecef,1:enu-baseline)
    uint8_t type;
    // AR ratio factor for validation
    float AR_ratio;
    // AR ratio threshold for validation
    float AR_thres;

    // GDOP / PDOP / HDOP / VDOP
    double dop[4];

private:
    std::string d_dump_filename;
    std::ifstream d_dump_file;
};

#endif  //GNSS_SDR_RTKLIB_SOLVER_DUMP_READER_H
