"""
 gps_l1_ca_read_pvt_dump.py
   gps_l1_ca_read_pvt_dump (filename)

 Open and read GNSS-SDR PVT binary log file (.dat) into Python, and
 return the contents.

 Irene Pérez Riega, 2023. iperrie@inta.es

      Args:
        filename: path to file PVT.dat with the raw data

      Return:
        nav_solutions: A dictionary with the processed data in lists

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

import math
import struct
import numpy as np


def gps_l1_ca_read_pvt_dump(filename):

    uint8_size_bytes = 1
    uint32_size_bytes = 4
    double_size_bytes = 8
    float_size_bytes = 4
    bytes_shift = 0

    TOW = []
    WEEK =[]
    PVT_GPS_time = []
    Clock_Offset = []
    ECEF_X_POS = []
    ECEF_Y_POS = []
    ECEF_Z_POS = []
    ECEF_X_VEL = []
    ECEF_Y_VEL = []
    ECEF_Z_VEL = []
    C_XX = []
    C_YY = []
    C_ZZ = []
    C_XY = []
    C_YZ = []
    C_ZX = []
    Lat = []
    Long = []
    Height = []
    num_valid_sats = []
    RTKLIB_status = []
    RTKLIB_type = []
    AR_factor = []
    AR_threshold = []
    GDOP = []
    PDOP = []
    HDOP = []
    VDOP = []

    f = open(filename, 'rb')
    if f is None:
        return None
    else:
        while True:
            f.seek(bytes_shift, 0)
            # TOW -> (Time Of Week) [usually sec] uint32
            TOW.append(struct.unpack('I',
                                     f.read(uint32_size_bytes))[0])
            bytes_shift += uint32_size_bytes
            f.seek(bytes_shift, 0)
            # WEEK -> uint32
            WEEK.append(struct.unpack('I',
                                      f.read(uint32_size_bytes))[0])
            bytes_shift += uint32_size_bytes
            f.seek(bytes_shift, 0)
            # PVT_GPS_time -> double
            PVT_GPS_time.append(struct.unpack('d',
                                              f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            # User_clock_offset -> [s] double
            Clock_Offset.append(struct.unpack('d',
                                              f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            # ##### ECEF POS X,Y,X [m] + ECEF VEL X,Y,X [m/s] (6 x double) ######
            ECEF_X_POS.append(struct.unpack('d',
                                            f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            ECEF_Y_POS.append(struct.unpack('d',
                                            f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            ECEF_Z_POS.append(struct.unpack('d',
                                            f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            ECEF_X_VEL.append(struct.unpack('d',
                                            f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            ECEF_Y_VEL.append(struct.unpack('d',
                                            f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            ECEF_Z_VEL.append(struct.unpack('d',
                                            f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            # #### Position variance/covariance [m²]
            # {c_xx,c_yy,c_zz,c_xy,c_yz,c_zx} (6 x double) ######
            C_XX.append(struct.unpack('d',
                                      f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            C_YY.append(struct.unpack('d',
                                      f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            C_ZZ.append(struct.unpack('d',
                                      f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            C_XY.append(struct.unpack('d',
                                      f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            C_YZ.append(struct.unpack('d',
                                      f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            C_ZX.append(struct.unpack('d',
                                      f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            # GEO user position Latitude -> [deg] double
            Lat.append(struct.unpack('d',
                                     f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            # GEO user position Longitude -> [deg] double
            Long.append(struct.unpack('d',
                                      f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            # GEO user position Height -> [m] double
            Height.append(struct.unpack('d',
                                        f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            # NUMBER OF VALID SATS -> uint8
            num_valid_sats.append(struct.unpack('B',
                                                f.read(uint8_size_bytes))[0])
            bytes_shift += uint8_size_bytes
            f.seek(bytes_shift, 0)
            # RTKLIB solution status (Real-Time Kinematic) -> uint8
            RTKLIB_status.append(struct.unpack('B',
                                               f.read(uint8_size_bytes))[0])
            bytes_shift += uint8_size_bytes
            f.seek(bytes_shift, 0)
            # RTKLIB solution type (0:xyz-ecef,1:enu-baseline) -> uint8
            RTKLIB_type.append(struct.unpack('B',
                                             f.read(uint8_size_bytes))[0])
            bytes_shift += uint8_size_bytes
            f.seek(bytes_shift, 0)
            # AR ratio factor for validation -> float
            AR_factor.append(struct.unpack('f',
                                           f.read(float_size_bytes))[0])
            bytes_shift += float_size_bytes
            f.seek(bytes_shift, 0)
            # AR ratio threshold for validation -> float
            AR_threshold.append(struct.unpack('f',
                                              f.read(float_size_bytes))[0])
            bytes_shift += float_size_bytes
            f.seek(bytes_shift, 0)
            # ##### GDOP / PDOP / HDOP / VDOP (4 * double) #####
            GDOP.append(struct.unpack('d',
                                      f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            PDOP.append(struct.unpack('d',
                                      f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            HDOP.append(struct.unpack('d',
                                      f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            VDOP.append(struct.unpack('d',
                                      f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)

            # Check file
            linea = f.readline()
            if not linea:
                break

    f.close()

    # Creating a list with total velocities [m/s]
    vel = []
    for i in range(len(ECEF_X_VEL)):
        vel.append(math.sqrt(ECEF_X_VEL[i]**2 + ECEF_Y_VEL[i]**2 +
                             ECEF_Z_VEL[i]**2))

    navSolutions = {
        'TOW': TOW,
        'WEEK': WEEK,
        'TransmitTime': PVT_GPS_time,
        'dt': Clock_Offset,
        'X': ECEF_X_POS,
        'Y': ECEF_Y_POS,
        'Z': ECEF_Z_POS,
        'X_vel': ECEF_X_VEL,
        'Y_vel': ECEF_Y_VEL,
        'Z_vel': ECEF_Z_VEL,
        'Tot_Vel': vel,
        'C_XX': C_XX,
        'C_YY': C_YY,
        'C_ZZ': C_ZZ,
        'C_XY': C_XY,
        'C_YZ': C_YZ,
        'C_ZX': C_ZX,
        'latitude': Lat,
        'longitude': Long,
        'height': Height,
        'SATS': num_valid_sats,
        'RTK_status': RTKLIB_status,
        'RTK_type': RTKLIB_type,
        'AR_factor': AR_factor,
        'AR_threshold': AR_threshold,
        'GDOP': np.array(GDOP),
        'PDOP': np.array(PDOP),
        'HDOP': np.array(HDOP),
        'VDOP': np.array(VDOP)
    }

    return navSolutions
