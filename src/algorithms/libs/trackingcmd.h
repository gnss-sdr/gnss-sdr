/*
 * trackingcmd.h
 *
 *  Created on: 20 ago. 2020
 *      Author: javier
 */

#ifndef SRC_ALGORITHMS_LIBS_TRACKINGCMD_H_
#define SRC_ALGORITHMS_LIBS_TRACKINGCMD_H_

#include <cstdint>

class TrackingCmd
{
public:
    TrackingCmd();

    bool enable_carrier_nco_cmd;
    bool enable_code_nco_cmd;
    double code_freq_chips;
    double carrier_freq_hz;
    double carrier_freq_rate_hz_s;
    uint64_t sample_counter;
};

#endif /* SRC_ALGORITHMS_LIBS_TRACKINGCMD_H_ */
