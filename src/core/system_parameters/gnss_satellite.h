/*!
 * \file gnss_satellite.h
 * \brief  Interface of the Gnss_Satellite class
 * \author Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GNSS_SATELLITE_H_
#define GNSS_SDR_GNSS_SATELLITE_H_

#include <cstdint>
#include <map>
#include <ostream>
#include <set>
#include <string>


/*!
 * \brief This class represents a GNSS satellite.
 *
 * It contains information about the space vehicles currently operational
 * of GPS, Glonass, SBAS and Galileo constellations.
 */
class Gnss_Satellite
{
public:
    Gnss_Satellite();                                                   //!< Default Constructor.
    Gnss_Satellite(const std::string& system_, uint32_t PRN_);          //!< Concrete GNSS satellite Constructor.
    ~Gnss_Satellite() = default;                                        //!< Default Destructor.
    void update_PRN(uint32_t PRN);                                      //!< Updates the PRN Number when information is decoded, only applies to GLONASS GNAV messages
    uint32_t get_PRN() const;                                           //!< Gets satellite's PRN
    int32_t get_rf_link() const;                                        //!< Gets the satellite's rf link
    std::string get_system() const;                                     //!< Gets the satellite system {"GPS", "GLONASS", "SBAS", "Galileo", "Beidou"}
    std::string get_system_short() const;                               //!< Gets the satellite system {"G", "R", "SBAS", "E", "C"}
    std::string get_block() const;                                      //!< Gets the satellite block. If GPS, returns {"IIA", "IIR", "IIR-M", "IIF"}
    std::string what_block(const std::string& system_, uint32_t PRN_);  //!< Gets the block of a given satellite

    friend bool operator==(const Gnss_Satellite& /*sat1*/, const Gnss_Satellite& /*sat2*/);  //!< operator== for comparison
    friend std::ostream& operator<<(std::ostream& /*out*/, const Gnss_Satellite& /*sat*/);   //!< operator<< for pretty printing

    Gnss_Satellite(Gnss_Satellite&& other) noexcept;             //!< Copy constructor
    Gnss_Satellite& operator=(const Gnss_Satellite&);            //!< Copy assignment operator
    Gnss_Satellite(const Gnss_Satellite& other) noexcept;        //!< Move constructor
    Gnss_Satellite& operator=(Gnss_Satellite&& other) noexcept;  //!< Move assignment operator

private:
    uint32_t PRN{};
    int32_t rf_link{};
    std::string system{};
    std::string block{};
    const std::set<std::string> system_set = {"GPS", "Glonass", "SBAS", "Galileo", "Beidou"};
    const std::map<std::string, std::string> satelliteSystem = {{"GPS", "G"}, {"Glonass", "R"}, {"SBAS", "S"}, {"Galileo", "E"}, {"Beidou", "C"}};
    void set_system(const std::string& system);  // Sets the satellite system {"GPS", "GLONASS", "SBAS", "Galileo", "Beidou"}.
    void set_PRN(uint32_t PRN);                  // Sets satellite's PRN
    void set_block(const std::string& system_, uint32_t PRN_);
    void reset();
    void set_rf_link(int32_t rf_link_);
};
#endif
