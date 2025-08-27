"""
 vtl_read_dump.py
   vtl_read_dump (filename)

 Read GNSS-SDR Vector Tracking dump binary file into Python.
 Opens GNSS-SDR vector tracking binary log file .dat and returns the contents

 Pedro Pereira, 2025. pereirapedro@gmail.com

    Args:
        settings: receiver settings.

    Return:
        GNSS_vtl: A dictionary with the processed data in lists

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

import struct


def vtl_read_dump(settings):

    v1 = []
    v2 = []
    v3 = []
    v4 = []
    v5 = []
    v6 = []
    v7 = []
    v8 = []
    v9 = []
    v10= []
    v11 = []
    v12 = []
    v13 = []
    v14 = []
    v15 = []
    v16 = []
    v17 = []
    v18 = []
    v19 = []
    v20 = []
    v21 = []
    v22 = []
    v23 = []
    v24 = []
    v25 = []
    v26 = []
    v27 = []
    v28 = [] 
    v29 = []
    v30 = []
    v31 = []
    v32 = []
    v33 = []
    v34 = []
    v35 = []
    v36 = []
    v37 = []
    v38 = []
    v39 = []
    v40 = []
    v41 = []
    v42 = []
    GNSS_vtl = {}

    bytes_shift = 0

    uint32_size_bytes = 4
    double_size_bytes = 8

    f = open(settings["dump_path"], 'rb')
    if f is None:
        return None
    else:
        while True:
            f.seek(bytes_shift, 0)
            # epoch number
            v1.append(struct.unpack('I', f.read(uint32_size_bytes))[0])
            bytes_shift += uint32_size_bytes
            f.seek(bytes_shift, 0)

            # receiver time
            v2.append(struct.unpack('d', f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)

            # time difference between vtl epochs
            v3.append(struct.unpack('d', f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)

            # VTL receiver position x-axis
            v4.append(struct.unpack('d', f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            # VTL receiver position y-axis
            v5.append(struct.unpack('d', f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            # VTL receiver position z-axis
            v6.append(struct.unpack('d', f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)

            # VTL receiver velocity x-axis
            v7.append(struct.unpack('d', f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            # VTL receiver velocity y-axis
            v8.append(struct.unpack('d', f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            # VTL receiver velocity z-axis
            v9.append(struct.unpack('d', f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)

            # VTL receiver clock bias (GPS)
            v10.append(struct.unpack('d', f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            # VTL receiver clock bias (GAL)
            if settings["GAL_en"] == 1:
                v11.append(struct.unpack('d', f.read(double_size_bytes))[0])
                bytes_shift += double_size_bytes
                f.seek(bytes_shift, 0)

            # VTL receiver clock drift
            v12.append(struct.unpack('d', f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)

            # RTKL receiver position x-axis
            v13.append(struct.unpack('d', f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            # RTKL receiver position y-axis
            v14.append(struct.unpack('d', f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            # RTKL receiver position z-axis
            v15.append(struct.unpack('d', f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)

            # RTKL receiver velocity x-axis
            v16.append(struct.unpack('d', f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            # RTKL receiver velocity y-axis
            v17.append(struct.unpack('d', f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            # RTKL receiver velocity z-axis
            v18.append(struct.unpack('d', f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)

            # RTKL receiver clock bias (GPS)
            v19.append(struct.unpack('d', f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)

            # RTKL receiver clock bias (GAL)
            v20.append(struct.unpack('d', f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)

            # RTKL receiver clock drift
            v21.append(struct.unpack('d', f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)

            # active channels
            for _ in range(settings["numberOfChannels"]):
                v22.append(struct.unpack('d', f.read(double_size_bytes))[0])
                bytes_shift += double_size_bytes
                f.seek(bytes_shift, 0)

            # EKF pre-fit (pseudorange)
            for _ in range(settings["numberOfChannels"]):
                v23.append(struct.unpack('d', f.read(double_size_bytes))[0])
                bytes_shift += double_size_bytes
                f.seek(bytes_shift, 0)

            # EKF pre-fit (pseudorange rate)
            for _ in range(settings["numberOfChannels"]):
                v24.append(struct.unpack('d', f.read(double_size_bytes))[0])
                bytes_shift += double_size_bytes
                f.seek(bytes_shift, 0)

            # EKF post-fit (pseudorange)
            for _ in range(settings["numberOfChannels"]):
                v25.append(struct.unpack('d', f.read(double_size_bytes))[0])
                bytes_shift += double_size_bytes
                f.seek(bytes_shift, 0)

            # EKF post-fit (pseudorange rate)
            for _ in range(settings["numberOfChannels"]):
                v26.append(struct.unpack('d', f.read(double_size_bytes))[0])
                bytes_shift += double_size_bytes
                f.seek(bytes_shift, 0)

            # EKF measurement covariance (pseudorange)
            for _ in range(settings["numberOfChannels"]):
                v27.append(struct.unpack('d', f.read(double_size_bytes))[0])
                bytes_shift += double_size_bytes
                f.seek(bytes_shift, 0)

            # EKF measurement covariance (pseudorange rate)
            for _ in range(settings["numberOfChannels"]):
                v28.append(struct.unpack('d', f.read(double_size_bytes))[0])
                bytes_shift += double_size_bytes
                f.seek(bytes_shift, 0)
          
            # EKF process noise
            for i in range(settings["numberOfStates"]):
                v29.append(struct.unpack('d', f.read(double_size_bytes))[0])
                bytes_shift += double_size_bytes
                f.seek(bytes_shift, 0)

            # observed pseudorange
            for _ in range(settings["numberOfChannels"]):
                v30.append(struct.unpack('d', f.read(double_size_bytes))[0])
                bytes_shift += double_size_bytes
                f.seek(bytes_shift, 0)

            # observed pseudorange rate
            for _ in range(settings["numberOfChannels"]):
                v31.append(struct.unpack('d', f.read(double_size_bytes))[0])
                bytes_shift += double_size_bytes
                f.seek(bytes_shift, 0)

            # computed pseudorange
            for _ in range(settings["numberOfChannels"]):
                v32.append(struct.unpack('d', f.read(double_size_bytes))[0])
                bytes_shift += double_size_bytes
                f.seek(bytes_shift, 0)

            # computed pseudorange rate
            for _ in range(settings["numberOfChannels"]):
                v33.append(struct.unpack('d', f.read(double_size_bytes))[0])
                bytes_shift += double_size_bytes
                f.seek(bytes_shift, 0)

            # vector tracking code frequency feedback
            for _ in range(settings["numberOfChannels"]):
                v34.append(struct.unpack('d', f.read(double_size_bytes))[0])
                bytes_shift += double_size_bytes
                f.seek(bytes_shift, 0)

            # satellite position in x-axis
            for _ in range(settings["numberOfChannels"]):
                v35.append(struct.unpack('d', f.read(double_size_bytes))[0])
                bytes_shift += double_size_bytes
                f.seek(bytes_shift, 0)

            # satellite position in y-axis
            for _ in range(settings["numberOfChannels"]):
                v36.append(struct.unpack('d', f.read(double_size_bytes))[0])
                bytes_shift += double_size_bytes
                f.seek(bytes_shift, 0)

            # satellite position in z-axis
            for _ in range(settings["numberOfChannels"]):
                v37.append(struct.unpack('d', f.read(double_size_bytes))[0])
                bytes_shift += double_size_bytes
                f.seek(bytes_shift, 0)

            # satellite clock bias
            for _ in range(settings["numberOfChannels"]):
                v38.append(struct.unpack('d', f.read(double_size_bytes))[0])
                bytes_shift += double_size_bytes
                f.seek(bytes_shift, 0)

            # satellite clock drift
            for _ in range(settings["numberOfChannels"]):
                v39.append(struct.unpack('d', f.read(double_size_bytes))[0])
                bytes_shift += double_size_bytes
                f.seek(bytes_shift, 0)

            # troposphere bias
            for _ in range(settings["numberOfChannels"]):
                v40.append(struct.unpack('d', f.read(double_size_bytes))[0])
                bytes_shift += double_size_bytes
                f.seek(bytes_shift, 0)

            # ionosphere bias
            for _ in range(settings["numberOfChannels"]):
                v41.append(struct.unpack('d', f.read(double_size_bytes))[0])
                bytes_shift += double_size_bytes
                f.seek(bytes_shift, 0)

            # satellite code bias
            for _ in range(settings["numberOfChannels"]):
                v42.append(struct.unpack('d', f.read(double_size_bytes))[0])
                bytes_shift += double_size_bytes
                f.seek(bytes_shift, 0)

            # Check file
            linea = f.readline()
            if not linea:
                break

        f.close()

        GNSS_vtl['nepoch'] = v1
        GNSS_vtl['receiver_time_s'] = v2
        GNSS_vtl['vtl_dt_s'] = v3
        GNSS_vtl['VTL_RX_P_X_ECEF_m'] = v4
        GNSS_vtl['VTL_RX_P_Y_ECEF_m'] = v5
        GNSS_vtl['VTL_RX_P_Z_ECEF_m'] = v6
        GNSS_vtl['VTL_RX_V_X_ECEF_ms'] = v7
        GNSS_vtl['VTL_RX_V_Y_ECEF_ms'] = v8
        GNSS_vtl['VTL_RX_V_Z_ECEF_ms'] = v9
        GNSS_vtl['VTL_RX_CLK_B_GPS_m'] = v10
        GNSS_vtl['VTL_RX_CLK_B_GAL_m'] = v11
        GNSS_vtl['VTL_RX_CLK_D_ms'] = v12
        GNSS_vtl['RTKL_RX_P_X_ECEF_m'] = v13
        GNSS_vtl['RTKL_RX_P_Y_ECEF_m'] = v14
        GNSS_vtl['RTKL_RX_P_Z_ECEF_m'] = v15
        GNSS_vtl['RTKL_RX_V_X_ECEF_ms'] = v16
        GNSS_vtl['RTKL_RX_V_Y_ECEF_ms'] = v17
        GNSS_vtl['RTKL_RX_V_Z_ECEF_ms'] = v18
        GNSS_vtl['RTKL_RX_CLK_B_GPS_m'] = v19
        GNSS_vtl['RTKL_RX_CLK_B_GAL_m'] = v20
        GNSS_vtl['RTKL_RX_CLK_D_ms'] = v21
        GNSS_vtl['VTL_active_channels'] = v22
        GNSS_vtl['EKF_prefit_PR_m'] = v23
        GNSS_vtl['EKF_prefit_PRR_ms'] = v24
        GNSS_vtl['EKF_postfit_PR_m'] = v25
        GNSS_vtl['EKF_postfit_PRR_ms'] = v26
        GNSS_vtl['EKF_meascov_PR'] = v27
        GNSS_vtl['EKF_meascov_PRR'] = v28
        GNSS_vtl['EKF_process_noise'] = v29
        GNSS_vtl['RTKL_observed_PR_m'] = v30
        GNSS_vtl['RTKL_observed_PRR_ms'] = v31
        GNSS_vtl['VTL_computed_PR_m'] = v32
        GNSS_vtl['VTL_computed_PRR_ms'] = v33
        GNSS_vtl['VTL_code_freq_hz'] = v34
        GNSS_vtl['RTKL_SV_P_X_ECEF_m'] = v35
        GNSS_vtl['RTKL_SV_P_Y_ECEF_m'] = v36
        GNSS_vtl['RTKL_SV_P_Z_ECEF_m'] = v37
        GNSS_vtl['RTKL_SV_CLK_B_m'] = v38
        GNSS_vtl['RTKL_SV_CLK_D_ms'] = v39
        GNSS_vtl['RTKL_topo_delay_m'] = v40
        GNSS_vtl['RTKL_iono_delay_m'] = v41
        GNSS_vtl['RTKL_code_bias_m'] = v42

    return GNSS_vtl
