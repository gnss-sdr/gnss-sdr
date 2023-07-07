/*!
 * \file pps_samplestamp.h
 * \brief A simple container for the sample count associated to PPS rising edge
 * \author Javier Arribas, jarribas(at)cttc.es
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2023  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef IIOPPS_PPS_SAMPLESTAMP_H
#define IIOPPS_PPS_SAMPLESTAMP_H

#include <cstdint>

class PpsSamplestamp
{
public:
    uint64_t samplestamp = 0;   // PPS rising edge samples counter from the beginning of rx stream opperation. Notice that it is reseted to zero if sample buffer overflow is detected on the FPGA side
    uint32_t overflow_reg = 0;  // >0 indicates overflow situation in the FPGA RX buffer
};

#endif
