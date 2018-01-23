/*!
 * \file gnss_satellite.cc
 * \brief  Implementation of the Gnss_Satellite class
 * \author Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "gnss_satellite.h"
#include <glog/logging.h>



Gnss_Satellite::Gnss_Satellite()
{
    Gnss_Satellite::reset();
}




Gnss_Satellite::Gnss_Satellite(const std::string& system_, unsigned int PRN_)
{
    Gnss_Satellite::reset();
    Gnss_Satellite::set_system(system_);
    Gnss_Satellite::set_PRN(PRN_);
    Gnss_Satellite::set_block(system_, PRN_);
}




Gnss_Satellite::~Gnss_Satellite()
{}





void Gnss_Satellite::reset()
{
    system_set = {"GPS", "Glonass", "SBAS", "Galileo", "Beidou"};
    satelliteSystem["GPS"] = "G";
    satelliteSystem["Glonass"] = "R";
    satelliteSystem["SBAS"] = "S";
    satelliteSystem["Galileo"] = "E";
    satelliteSystem["Beidou"] = "C";
    PRN = 0;
    system = std::string("");
    block = std::string("");
    rf_link = 0;
}



std::ostream& operator<<(std::ostream &out, const Gnss_Satellite &sat) // output
{
    std::string tag("");
    std::string tag2("");
    if(sat.get_system().compare("Galileo") == 0) tag = "E";
    if(sat.get_PRN() < 10) tag2 = "0";
    out << sat.get_system() << " PRN " << tag << tag2 << sat.get_PRN() << " (Block " << sat.get_block() << ")";
    return out;
}


bool operator== (const Gnss_Satellite &sat1, const Gnss_Satellite &sat2)
{
    bool equal = false;
    if (sat1.get_system().compare(sat2.get_system()) == 0)
        {
            if (sat1.get_PRN() == (sat2.get_PRN()))
                {
                    equal = true;
                }
        }
    return equal;
}

/*
Gnss_Satellite& Gnss_Satellite::operator=(const Gnss_Satellite &rhs) {

    // Only do assignment if RHS is a different object from this.
    if (this != &rhs) {
            // Deallocate, allocate new space, copy values...
            const std::string system_ = rhs.get_system();
            const unsigned int PRN_ = rhs.get_PRN();
            const std::string block_ = rhs.get_block();
           // const signed int rf_link_ = 0;
            this->set_system(system_);
            this->set_PRN(PRN_);
            this->set_block(system_, PRN_);
            //this.rf_link = rf_link_;
    }
    return *this;
}*/


void Gnss_Satellite::set_system(const std::string& system_)
{
    // Set the satellite system {"GPS", "Glonass", "SBAS", "Galileo", "Compass"}
    std::set<std::string>::iterator it = system_set.find(system_);

    if(it != system_set.cend())
        {
            system = system_;
        }
    else
        {
            DLOG(INFO) << "System " << system_ << " is not defined {GPS, Glonass, SBAS, Galileo, Beidou}. Initialization?";
            system =  std::string("");
        }
}


void Gnss_Satellite::update_PRN(unsigned int PRN_)
{
    if (system.compare("Glonass") != 0)
				{
						DLOG(INFO) << "Trying to update PRN for not GLONASS system";
						PRN = 0;
				}
    else
    		{
				 if (PRN_ < 1 or PRN_ > 24)
						 {
								 DLOG(INFO) << "This PRN is not defined";
								 // Adjusting for PRN 26, now used in
								 PRN = PRN_;
						 }
				 else
						 {
								 PRN = PRN_;
						 }
    		}
}


void Gnss_Satellite::set_PRN(unsigned int PRN_)
{
    // Set satellite's PRN
    if (system.compare("") == 0)
        {
            DLOG(INFO) << "Trying to define PRN while system is not defined";
            PRN = 0;
        }
    if (system.compare("GPS") == 0)
        {
            if (PRN_ < 1 or PRN_ > 32)
                {
                    DLOG(INFO) << "This PRN is not defined";
                    PRN = 0;
                }
            else
                {
                    PRN = PRN_;
                }
        }
    else if (system.compare("Glonass") == 0)
           {
               if (PRN_ < 1 or PRN_ > 24)
                   {
                       DLOG(INFO) << "This PRN is not defined";
                       PRN = 0;
                   }
               else
                   {
                       PRN = PRN_;
                   }
           }
    else if (system.compare("SBAS") == 0)
        {
            if (PRN_ == 122){ PRN = PRN_; }        // WAAS Inmarsat 3F4 (AOR-W)
            else if (PRN_ == 134){ PRN = PRN_; }   // WAAS Inmarsat 3F3 (POR)
            else if (PRN_ == 120){ PRN = PRN_; }   // EGNOS AOR-E Broadcast satellite http://www.egnos-pro.esa.int/index.html
            else if (PRN_ == 124){ PRN = PRN_; }   // EGNOS ESA ARTEMIS used for EGNOS Operations
            else if (PRN_ == 126){ PRN = PRN_; }   // EGNOS IOR-W  currently used by Industry to perform various tests on the system.
            else
                {
                    DLOG(INFO) << "This PRN is not defined";
                    PRN = 0;
                }
        }
    else if (system.compare("Galileo") == 0)
        {
        if (PRN_ < 1 or PRN_ > 36)
            {
                DLOG(INFO) << "This PRN is not defined";
                PRN = 0;
            }
        else
            {
                PRN = PRN_;
            }
        }
    else
        {
            DLOG(INFO) << "System " << system << " is not defined";
            PRN = 0;
        }
}





unsigned int Gnss_Satellite::get_PRN() const
{
    // Get satellite's PRN
    unsigned int PRN_;
    PRN_ = PRN;
    return PRN_;
}






std::string Gnss_Satellite::get_system() const
{
    // Get the satellite system {"GPS", "Glonass", "SBAS", "Galileo", "Beidou"}
    std::string system_;
    system_ = system;
    return system_;
}

std::string Gnss_Satellite::get_system_short() const
{
    // Get the satellite system {"G", "R", "S", "E", "C"}
    return satelliteSystem.at(system);
}




std::string Gnss_Satellite::get_block() const
{
    // Get the satellite block
    std::string block_;
    block_ = block;
    return block_;
}



std::string Gnss_Satellite::what_block(const std::string& system_, unsigned int PRN_)
{
    std::string block_ = "Unknown";
    if (system_.compare("GPS") == 0)
        {
            switch ( PRN_ )
            {
            // info from https://www.navcen.uscg.gov/?Do=constellationStatus

            case 1 :
                block_ = std::string("IIF");   // Plane D
                break;
            case 2 :
                block_ = std::string("IIR");   // Plane D
                break;
            case 3 :
                block_ = std::string("IIF");   // Plane E
                break;
            case 4 :
                block_ = std::string("Unknown");
                break;
            case 5 :
                block_ = std::string("IIR-M"); // Plane E
                break;
            case 6 :
                block_ = std::string("IIF");   // Plane D
                break;
            case 7 :
                block_ = std::string("IIR-M"); // Plane A
                break;
            case 8 :
                block_ = std::string("IIF");   // Plane C
                break;
            case 9 :
                block_ = std::string("IIF");   // Plane F
                break;
            case 10 :
                block_ = std::string("IIF");   // Plane E
                break;
            case 11 :
                block_ = std::string("IIR");   // Plane D
                break;
            case 12 :
                block_ = std::string("IIR-M"); // Plane B
                break;
            case 13 :
                block_ = std::string("IIR");   // Plane F
                break;
            case 14 :
                block_ = std::string("IIR");   // Plane F
                break;
            case 15 :
                block_ = std::string("IIR-M"); // Plane F
                break;
            case 16 :
                block_ = std::string("IIR");   // Plane B
                break;
            case 17 :
                block_ = std::string("IIR-M"); // Plane C
                break;
            case 18 :
                block_ = std::string("IIR");   // Plane E
                break;
            case 19 :
                block_ = std::string("IIR");   // Plane D
                break;
            case 20 :
                block_ = std::string("IIR");   // Plane B
                break;
            case 21 :
                block_ = std::string("IIR");   // Plane D
                break;
            case 22 :
                block_ = std::string("IIR");   // Plane E
                break;
            case 23 :
                block_ = std::string("IIR");   // Plane F
                break;
            case 24 :
                block_ = std::string("IIF");   // Plane A
                break;
            case 25 :
                block_ = std::string("IIF");   // Plane B
                break;
            case 26 :
                block_ = std::string("IIF");   // Plane B
                break;
            case 27 :
                block_ = std::string("IIF");   // Plane C
                break;
            case 28 :
                block_ = std::string("IIR");   // Plane B
                break;
            case 29 :
                block_ = std::string("IIR-M"); // Plane C
                break;
            case 30 :
                block_ = std::string("IIF");   // Plane A
                break;
            case 31 :
                block_ = std::string("IIR-M"); // Plane A
                break;
            case 32 :
                block_ = std::string("IIF");   // Plane F
                break;
            default :
                block_ = std::string("Unknown");
            }
        }


    if (system_.compare("Glonass") == 0)
        {
            switch ( PRN_ )
            {
            // Info from http://www.sdcm.ru/smglo/grupglo?version=eng&site=extern
            // See also http://www.glonass-center.ru/en/GLONASS/

            case 1 :
                block_ = std::string("1");   // Plane 1
                rf_link = 1;
                break;
            case 2 :
                block_ = std::string("-4");  // Plane 1
                rf_link = -4;
                break;
            case 3 :
                block_ = std::string("5");   // Plane 1
                rf_link = 5;
                break;
            case 4 :
                block_ = std::string("6");   // Plane 1
                rf_link = 6;
                break;
            case 5 :
                block_ = std::string("1");   // Plane 1
                rf_link = 1;
                break;
            case 6 :
                block_ = std::string("-4");  // Plane 1
                rf_link = -4;
                break;
            case 7 :
                block_ = std::string("5");   // Plane 1
                rf_link = 5;
                break;
            case 8 :
                block_ = std::string("6");   // Plane 1
                rf_link = 6;
                break;
            case 9 :
                block_ = std::string("-2");  // Plane 2
                rf_link = -2;
                break;
            case 10 :
                block_ = std::string("-7");  // Plane 2
                rf_link = -7;
                break;
            case 11 :
                block_ = std::string("0");   // Plane 2
                rf_link = 0;
                break;
            case 12 :
                block_ = std::string("-1");  // Plane 2
                rf_link = -1;
                break;
            case 13 :
                block_ = std::string("-2");  // Plane 2
                rf_link = -2;
                break;
            case 14 :
                block_ = std::string("-7");  // Plane 2
                rf_link = -7;
                break;
            case 15 :
                block_ = std::string("0");   // Plane 2
                rf_link = 0;
                break;
            case 16 :
                block_ = std::string("-1");  // Plane 2
                rf_link = -1;
                break;
            case 17 :
                block_ = std::string("4");   // Plane 3
                rf_link = 4;
                break;
            case 18 :
                block_ = std::string("-3");  // Plane 3
                rf_link = -3;
                break;
            case 19 :
                block_ = std::string("3");   // Plane 3
                rf_link = 3;
                break;
            case 20 :
                block_ = std::string("2");   // Plane 3
                rf_link = 2;
                break;
            case 21 :
                block_ = std::string("4");   // Plane 3
                rf_link = 4;
                break;
            case 22 :
                block_ = std::string("-3");  // Plane 3
                rf_link = -3;
                break;
            case 23 :
                block_ = std::string("3");   // Plane 3
                rf_link = 3;
                break;
            case 24 :
                block_ = std::string("2");   // Plane 3
                rf_link = 2;
                break;
            default :
                block_ = std::string("Unknown");
            }
        }
    if (system_.compare("SBAS") == 0)
        {
            switch ( PRN_ )
            {
            case 122 :
                block_ = std::string("WAAS");  // WAAS Inmarsat 3F4 (AOR-W)
                break;
            case 134 :
                block_ = std::string("WAAS");  // WAAS Inmarsat 3F3 (POR)
                break;
            case 120 :
                block_ = std::string("EGNOS"); // EGNOS AOR-E Broadcast satellite http://www.egnos-pro.esa.int/index.html
                break;
            case 124 :
                block_ = std::string("EGNOS"); // EGNOS ESA ARTEMIS used for EGNOS Operations
                break;
            case 126 :
                block_ = std::string("EGNOS"); // EGNOS IOR-W  currently used by Industry to perform various tests on the system.
                break;
            default:
                block_ = std::string("Unknown");
            }
        }
    if (system_.compare("Galileo") == 0)
        {
            // Check http://en.wikipedia.org/wiki/List_of_Galileo_satellites and https://www.gsc-europa.eu/system-status/Constellation-Information
            switch ( PRN_ )
            {
            case 1:
                block_ = std::string("FOC-FM10");  // Galileo Full Operational Capability (FOC) satellite FM10 / GSAT-0210, launched on May 24, 2016.
                break;
            case 2:
                block_ = std::string("FOC-FM11");  // Galileo Full Operational Capability (FOC) satellite FM11 / GSAT-0211, launched on May 24, 2016.
                break;
            case 3:
                block_ = std::string("FOC-FM12");  // Galileo Full Operational Capability (FOC) satellite FM12 / GSAT-0212, launched on November 17, 2016.
                break;
            case 4:
                block_ = std::string("FOC-FM13");  // Galileo Full Operational Capability (FOC) satellite FM13 / GSAT-0213, launched on November 17, 2016.
                break;
            case 5:
                block_ = std::string("FOC-FM14");  // Galileo Full Operational Capability (FOC) satellite FM14 / GSAT-0214, launched on November 17, 2016.
                break;
            case 7:
                block_ = std::string("FOC-FM7");   // Galileo Full Operational Capability (FOC) satellite FM7 / GSAT-0207, launched on November 17, 2016.
                break;
            case 8:
                block_ = std::string("FOC-FM8");   // Galileo Full Operational Capability (FOC) satellite FM8 / GSAT0208, launched on December 17, 2015.
                break;
            case 9:
                block_ = std::string("FOC-FM9");   // Galileo Full Operational Capability (FOC) satellite FM9 / GSAT0209, launched on December 17, 2015.
                break;
            case 11 :
                block_ = std::string("IOV-PFM");   // PFM, the ProtoFlight Model / GSAT0101, launched from French Guiana at 10:30 GMT on October 21, 2011.
                break;
            case 12 :
                block_ = std::string("IOV-FM2");   // Galileo In-Orbit Validation (IOV) satellite FM2 (Flight Model 2) also known as GSAT0102, from French Guiana at 10:30 GMT on October 21, 2011.
                break;
            case 14 :
                block_ = std::string("FOC-FM2*");  // Galileo Full Operational Capability (FOC) satellite FM2 / GSAT0202, launched into incorrect orbit on August 22, 2014. Moved to usable orbit in March, 2015. UNDER TESTING.
                break;
            case 18 :
                block_ = std::string("FOC-FM1*");  // Galileo Full Operational Capability (FOC) satellite FM1 / GSAT0201, launched into incorrect orbit on August 22, 2014. Moved to usable orbit in December, 2014. UNDER TESTING.
                break;
            case 19 :
                block_ = std::string("IOV-FM3");   // Galileo In-Orbit Validation (IOV) satellite FM3 (Flight Model 3) / GSAT0103, launched on October 12, 2012.
                break;
            case 20 :
                block_ = std::string("IOV-FM4**"); // Galileo In-Orbit Validation (IOV) satellite FM4 (Flight Model 4) / GSAT0104, launched on October 12, 2012. Payload power problem beginning May 27, 2014 led to permanent loss of E5 and E6 transmissions, E1 transmission restored. UNAVAILABLE FROM 2014-05-27 UNTIL FURTHER NOTICE
                break;
            case 21 :
                block_ = std::string("FOC-FM15");  // Galileo Full Operational Capability (FOC) satellite FM15 / GSAT0215, launched on Dec. 12, 2017. UNDER COMMISIONING.
                break;
            case 22 :
                block_ = std::string("FOC-FM4**"); // Galileo Full Operational Capability (FOC) satellite FM4 / GSAT0204, launched on March 27, 2015. REMOVED FROM ACTIVE SERVICE ON 2017-12-08 UNTIL FURTHER NOTICE FOR CONSTELLATION MANAGEMENT PURPOSES.
                break;
            case 24 :
                block_ = std::string("FOC-FM5");   // Galileo Full Operational Capability (FOC) satellite FM5 / GSAT0205, launched on Sept. 11, 2015.
                break;
            case 25 :
                block_ = std::string("FOC-FM16");  // Galileo Full Operational Capability (FOC) satellite FM16 / GSAT0216, launched on Dec. 12, 2017. UNDER COMMISIONING.
                break;
            case 26 :
                block_ = std::string("FOC-FM3");   // Galileo Full Operational Capability (FOC) satellite FM3 / GSAT0203, launched on March 27, 2015.
                break;
            case 27 :
                block_ = std::string("FOC-FM17");  // Galileo Full Operational Capability (FOC) satellite FM17 / GSAT0217, launched on Dec. 12, 2017. UNDER COMMISIONING.
                break;
            case 30 :
                block_ = std::string("FOC-FM6");   // Galileo Full Operational Capability (FOC) satellite FM6 / GSAT0206, launched on Sept. 11, 2015.
                break;
            case 31 :
                block_ = std::string("FOC-FM18");  // Galileo Full Operational Capability (FOC) satellite FM18 / GSAT0218, launched on Dec. 12, 2017. UNDER COMMISIONING.
                break;
            default:
                block_ = std::string("Unknown(Simulated)");
            }
        }
    return block_;
}


void Gnss_Satellite::set_block(const std::string& system_, unsigned int PRN_)
{
    block = what_block(system_, PRN_);
}
