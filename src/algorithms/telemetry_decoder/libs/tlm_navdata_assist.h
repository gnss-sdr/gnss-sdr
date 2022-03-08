/*!
 * \file tlm_navdata_assist.h
 * \brief Class that provides telemetry data assistance
 * \author Marc Majoral, 2021. mmajoral(at)cttc.es
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

#ifndef GNSS_SDR_TLM_NAVDATA_ASSIST_H
#define GNSS_SDR_TLM_NAVDATA_ASSIST_H

#include "tlm_conf.h"  // for Tlm_Conf

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_libs telemetry_decoder_libs
 * \{ */

/*!
 * \brief Class that provides telemetry data assistance
 */
class Tlm_navdata_assist
{
public:
    Tlm_navdata_assist(const Tlm_Conf &conf);  // = default;

    ~Tlm_navdata_assist();

    /*!
     * \brief get TOW at current symbol
     */
    uint32_t get_TOW_at_current_symbol_ms(uint64_t Tracking_sample_counter, uint64_t fs);

private:
    bool navdata_assist_real_time;
    uint32_t navdata_assist_Tow_ms;
    uint64_t navdata_assist_samplestamp;
    uint32_t navdata_assist_GNSS_UTC_leap_s;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_TLM_NAVDATA_ASSIST_H
