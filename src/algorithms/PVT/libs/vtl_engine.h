/*!
 * \file vtl_engine.h
 * \brief Class that implements a Vector Tracking Loop (VTL) Kalman filter engine
 * \author Javier Arribas, 2022. jarribas(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_VTL_ENGINE_H
#define GNSS_SDR_VTL_ENGINE_H

#include "trackingcmd.h"
#include "vtl_conf.h"
#include "vtl_data.h"
#include <armadillo>
#include <cstdint>
#include <string>
#include <vector>

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */


class Vtl_Engine
{
public:
    Vtl_Engine();

    ~Vtl_Engine();

    void configure(Vtl_Conf config_);  //set config parameters

    //TODO: output functions here (output for tracking KF updates, VTL computed user PVT, etc...)
    bool vtl_loop(Vtl_Data new_data);
    void reset();        // reset all internal states
    void debug_print();  // print debug information

    std::vector<TrackingCmd> trk_cmd_outs;  // vector holding the Tracking command states updates to be sent to tracking KFs

private:
    Vtl_Conf config;
    //TODO: Internal VTL persistent variables here
};


/** \} */
/** \} */
#endif  // GNSS_SDR_VTL_ENGINE_H
