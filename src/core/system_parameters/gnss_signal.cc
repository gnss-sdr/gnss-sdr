/*!
 * \file gnss_satellite.cc
 * \brief  Implementation of the Gnss_Signal class
 * \author
 *  Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *  Javier Arribas, 2012. jarribas(at)cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */
#include "gnss_signal.h"

Gnss_Signal::Gnss_Signal()
{
	this->signal="";
}
Gnss_Signal::Gnss_Signal(Gnss_Satellite satellite_,std::string signal_)
{
	this->satellite=satellite_;
	this->signal=signal_;
}
Gnss_Signal::~Gnss_Signal()
{
}

std::string Gnss_Signal::get_signal()
{
	return this->signal;
}
Gnss_Satellite Gnss_Signal::get_satellite()
{
	return this->satellite;
}

