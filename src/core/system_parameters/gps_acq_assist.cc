/*!
 * \file gps_acq_assist.cc
 * \brief  Interface of a GPS RRLL ACQUISITION ASSISTACE storage
 *
 * See https://www.gps.gov/technical/icwg/IS-GPS-200K.pdf Appendix II
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
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

#include "gps_acq_assist.h"

Gps_Acq_Assist::Gps_Acq_Assist()
{
    i_satellite_PRN = 0U;
    d_TOW = 0.0;
    d_Doppler0 = 0.0;
    d_Doppler1 = 0.0;
    dopplerUncertainty = 0.0;
    Code_Phase = 0.0;
    Code_Phase_int = 0.0;
    GPS_Bit_Number = 0.0;
    Code_Phase_window = 0.0;
    Azimuth = 0.0;
    Elevation = 0.0;
}
