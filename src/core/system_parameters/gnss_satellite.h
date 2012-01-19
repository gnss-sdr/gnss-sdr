/*!
 * \file gnss_satellite.h
 * \brief  Interface of the Gnss_Satellite class
 * \author Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
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


#ifndef GNSS_SDR_GNSS_SATELLITE_H_
#define GNSS_SDR_GNSS_SATELLITE_H_

#include <string>
#include <set>
#include <iostream>

/*
 * \brief This class represents a GNSS satellite.
 *
 * It contains information about the space vehicles currently operational
 * of GPS, Glonass, SBAS and Galileo constellations.
 */
class Gnss_Satellite
{
public:
    Gnss_Satellite();                    //!< Default Constructor.
    Gnss_Satellite(std::string system_, unsigned int PRN_); //!< Concrete GNSS satellite Constructor.
    ~Gnss_Satellite();                   //!< Default Destructor.
    unsigned int get_PRN() const;              //!< Gets satellite's PRN
    std::string get_system() const;            //!< Gets the satellite system {"GPS", "GLONASS", "SBAS", "Galileo", "Compass"}
    std::string get_block() const;             //!< Gets the satellite block. If GPS, returns {"IIA", "IIR", "IIR-M", "IIF"}
    friend bool operator== (const Gnss_Satellite &, const Gnss_Satellite &);  // operator== for comparison
    friend std::ostream& operator<<(std::ostream &, const Gnss_Satellite &); // operator<< for pretty printing
private:
    unsigned int PRN;
    std::string system;
    std::string block;
    signed int rf_link;
    void set_system(std::string system);  // Sets the satellite system {"GPS", "GLONASS", "SBAS", "Galileo", "Compass"}. Returns 1 if success.
    void set_PRN(unsigned int PRN);       // Sets satellite's PRN
    void set_block(std::string system_, unsigned int PRN_ );
    std::set<std::string> system_set;     // = {"GPS", "GLONASS", "SBAS", "Galileo", "Compass"};
    void reset();

};
#endif
