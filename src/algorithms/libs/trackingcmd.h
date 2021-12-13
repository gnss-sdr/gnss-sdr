/*!
 * \file trackingcmd.h
 * \brief Class that stores information to update the GNSS signal tracking estimations
 * \author Javier Arribas, 2021. jarribas(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
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
