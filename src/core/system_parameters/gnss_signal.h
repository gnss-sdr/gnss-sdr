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
 * Encapsulates a specific GNSS signal (e.g., GPS L1 C/A, Galileo E1B) and its
 * associated satellite.
 */
class Gnss_Signal
{
public:
    Gnss_Signal() = default;
    explicit Gnss_Signal(const std::string& signal_);
    Gnss_Signal(const Gnss_Satellite& satellite_, const std::string& signal_);
    ~Gnss_Signal() = default;

    /*!
     * \brief Return the signal identifier string.
     *
     *  - GPS: "1C" (L1 C/A), "2S" (L2C), "L5"
     *  - GLONASS: "1G" (L1 C/A), "2G" (L2 C/A)
     *  - Galileo: "1B" (E1B), "5X" (E5a), "7X" (E5b), "6C" (E6C)
     *  - BeiDou: "B1" (B1I), "B3" (B3I)
     */
    std::string get_signal_str() const;

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
