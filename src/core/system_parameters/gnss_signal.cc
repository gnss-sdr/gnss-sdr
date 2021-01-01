/*!
 * \file gnss_satellite.cc
 * \brief  Implementation of the Gnss_Signal class
 * \author
 *  Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *  Javier Arribas, 2012. jarribas(at)cttc.es
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

#include "gnss_signal.h"


Gnss_Signal::Gnss_Signal(const std::string& signal_)
{
    this->signal = signal_;
}


Gnss_Signal::Gnss_Signal(const Gnss_Satellite& satellite_, const std::string& signal_)
{
    this->satellite = satellite_;
    this->signal = signal_;
}


std::string Gnss_Signal::get_signal_str() const
{
    return this->signal;
}


Gnss_Satellite Gnss_Signal::get_satellite() const
{
    return this->satellite;
}


std::ostream& operator<<(std::ostream& out, const Gnss_Signal& sig)  // output
{
    out << sig.get_satellite() << " Signal " << sig.get_signal_str();
    return out;
}


bool operator==(const Gnss_Signal& sig1, const Gnss_Signal& sig2)
{
    bool equal = false;

    if (sig1.get_satellite() == sig2.get_satellite())
        {
            if (sig1.get_signal_str() == sig1.get_signal_str())
                {
                    equal = true;
                }
        }
    return equal;
}
