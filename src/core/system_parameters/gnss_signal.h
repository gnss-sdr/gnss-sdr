/*!
 * \file gnss_signal.h
 * \brief  Implementation of the Gnss_Signal class
 * \author
 *  Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *  Javier Arribas, 2012. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_GNSS_SIGNAL_H
#define GNSS_SDR_GNSS_SIGNAL_H

#include "gnss_satellite.h"
#include <ostream>
#include <string>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class represents a GNSS signal.
 *
 * It contains information about the space vehicle and the specific signal.
 */
class Gnss_Signal
{
public:
    Gnss_Signal() = default;
    explicit Gnss_Signal(const std::string& signal_);
    Gnss_Signal(const Gnss_Satellite& satellite_, const std::string& signal_);
    ~Gnss_Signal() = default;
    std::string get_signal_str() const;    //!< Get the satellite signal {"1C" for GPS L1 C/A, "2S" for GPS L2C (M), "L5" for GPS L5, "1G" for GLONASS L1 C/A, "1B" for Galileo E1B, "5X" for Galileo E5a.
    Gnss_Satellite get_satellite() const;  //!< Get the Gnss_Satellite associated to the signal

    friend bool operator==(const Gnss_Signal& /*sig1*/, const Gnss_Signal& /*sig2*/);    //!< operator== for comparison
    friend std::ostream& operator<<(std::ostream& /*out*/, const Gnss_Signal& /*sig*/);  //!< operator<< for pretty printing

private:
    Gnss_Satellite satellite{};
    std::string signal{};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_SIGNAL_H
