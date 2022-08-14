/* -------------------------------------------------------------------------
 *
 * Copyright (C) 2022 (see AUTHORS file for a list of contributors)
 *
 *
 */

#ifndef IIOPPS_PPS_SAMPLESTAMP_H
#define IIOPPS_PPS_SAMPLESTAMP_H

#include <cstdint>

class PpsSamplestamp
{
public:
    uint64_t samplestamp;   //PPS rising edge samples counter from the beginning of rx stream opperation. Notice that it is reseted to zero if sample buffer overflow is detected on the FPGA side
    uint32_t overflow_reg;  // >0 indicates overflow situation in the FPGA RX buffer
};

#endif
