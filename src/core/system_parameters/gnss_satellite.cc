/*!
 * \file gnss_satellite.cc
 * \brief  Implementation of the Gnss_Satellite class
 * \author Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
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

#include "gnss_satellite.h"
#include <glog/logging.h>
#include <utility>


Gnss_Satellite::Gnss_Satellite(const std::string& system_, uint32_t PRN_)
{
    Gnss_Satellite::reset();
    Gnss_Satellite::set_system(system_);
    Gnss_Satellite::set_PRN(PRN_);
    Gnss_Satellite::set_block(system_, PRN_);
}


void Gnss_Satellite::reset()
{
    this->system.clear();
    this->block.clear();
    this->PRN = 0;
    this->rf_link = 0;
}


std::ostream& operator<<(std::ostream& out, const Gnss_Satellite& sat)  // output
{
    std::string tag;
    std::string tag2;
    if (sat.get_system() == "Galileo")
        {
            tag = std::string("E");
        }
    if (sat.get_PRN() < 10)
        {
            tag2 = std::string("0");
        }
    out << sat.get_system() << " PRN " << tag << tag2 << sat.get_PRN() << " (Block " << sat.get_block() << ")";
    return out;
}


bool operator==(const Gnss_Satellite& sat1, const Gnss_Satellite& sat2)
{
    bool equal = false;
    if (sat1.get_system() == sat2.get_system())
        {
            if (sat1.get_PRN() == sat2.get_PRN())
                {
                    if (sat1.get_rf_link() == sat2.get_rf_link())
                        {
                            equal = true;
                        }
                }
        }
    return equal;
}


// Copy constructor
Gnss_Satellite::Gnss_Satellite(const Gnss_Satellite& other) noexcept
    : system(other.system),
      block(other.block),
      PRN(other.PRN),
      rf_link(other.rf_link)
{
}


// Copy assignment operator
Gnss_Satellite& Gnss_Satellite::operator=(const Gnss_Satellite& rhs) noexcept
{
    // Only do assignment if RHS is a different object from this.
    if (this != &rhs)
        {
            this->system = rhs.system;
            this->block = rhs.block;
            this->PRN = rhs.PRN;
            this->rf_link = rhs.rf_link;
        }
    return *this;
}


// Move constructor
Gnss_Satellite::Gnss_Satellite(Gnss_Satellite&& other) noexcept
    : system(std::move(other.system)),
      block(std::move(other.block)),
      PRN(other.PRN),
      rf_link(other.rf_link)
{
    other.system.clear();
    other.block.clear();
    other.PRN = 0;
    other.rf_link = 0;
}


// Move assignment operator
Gnss_Satellite& Gnss_Satellite::operator=(Gnss_Satellite&& other) noexcept
{
    if (this != &other)
        {
            system = std::move(other.system);
            block = std::move(other.block);
            PRN = other.PRN;
            rf_link = other.rf_link;
            other.system.clear();
            other.block.clear();
            other.PRN = 0;
            other.rf_link = 0;
        }
    return *this;
}


void Gnss_Satellite::set_system(const std::string& system_)
{
    // Set the satellite system {"GPS", "Glonass", "SBAS", "Galileo", "Compass"}
    auto it = system_set.find(system_);

    if (it != system_set.cend())
        {
            system = system_;
        }
    else
        {
            DLOG(INFO) << "System " << system_ << " is not defined {GPS, Glonass, SBAS, Galileo, Beidou}. Initialization?";
            system = std::string("");
        }
}


void Gnss_Satellite::update_PRN(uint32_t PRN_)
{
    if (system != "Glonass")
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


void Gnss_Satellite::set_PRN(uint32_t PRN_)
{
    // Set satellite's PRN
    if (system.empty())
        {
            DLOG(INFO) << "Trying to define PRN while system is not defined";
            PRN = 0;
        }
    if (system == "GPS")
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
    else if (system == "Glonass")
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
    else if (system == "SBAS")
        {
            if ((PRN_ == 120)      // EGNOS Test Platform.Inmarsat 3-F2 (Atlantic Ocean Region-East)
                || (PRN_ == 123)   // EGNOS Operational Platform. Astra 5B
                || (PRN_ == 131)   // WAAS Eutelsat 117 West B
                || (PRN_ == 135)   // WAAS Galaxy 15
                || (PRN_ == 136)   // EGNOS Operational Platform. SES-5 (a.k.a. Sirius 5 or Astra 4B)
                || (PRN_ == 138))  // WAAS Anik F1R
                {
                    PRN = PRN_;
                }
            else
                {
                    DLOG(INFO) << "This PRN is not defined";
                    PRN = 0;
                }
        }
    else if (system == "Galileo")
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
    else if (system == "Beidou")
        {
            if (PRN_ < 1 or PRN_ > 63)
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


int32_t Gnss_Satellite::get_rf_link() const
{
    // Get satellite's rf link. Identifies the GLONASS Frequency Channel
    int32_t rf_link_ = rf_link;
    return rf_link_;
}


void Gnss_Satellite::set_rf_link(int32_t rf_link_)
{
    // Set satellite's rf link. Identifies the GLONASS Frequency Channel
    rf_link = rf_link_;
}


uint32_t Gnss_Satellite::get_PRN() const
{
    // Get satellite's PRN
    uint32_t PRN_ = PRN;
    return PRN_;
}


std::string Gnss_Satellite::get_system() const
{
    // Get the satellite system {"GPS", "Glonass", "SBAS", "Galileo", "Beidou"}
    std::string system_ = system;
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
    std::string block_ = block;
    return block_;
}


std::string Gnss_Satellite::what_block(const std::string& system_, uint32_t PRN_)
{
    std::string block_ = "Unknown";
    if (system_ == "GPS")
        {
            // info from https://www.navcen.uscg.gov/?Do=constellationStatus
            switch (PRN_)
                {
                case 1:
                    block_ = std::string("IIF");  // Plane D
                    break;
                case 2:
                    block_ = std::string("IIR");  // Plane D
                    break;
                case 3:
                    block_ = std::string("IIF");  // Plane E
                    break;
                case 4:
                    block_ = std::string("III");  // Plane A
                    break;
                case 5:
                    block_ = std::string("IIR-M");  // Plane E
                    break;
                case 6:
                    block_ = std::string("IIF");  // Plane D
                    break;
                case 7:
                    block_ = std::string("IIR-M");  // Plane A
                    break;
                case 8:                           // NOLINT(bugprone-branch-clone)
                    block_ = std::string("IIF");  // Plane C
                    break;
                case 9:
                    block_ = std::string("IIF");  // Plane F
                    break;
                case 10:
                    block_ = std::string("IIF");  // Plane E
                    break;
                case 11:
                    block_ = std::string("III");  // Plane D
                    break;
                case 12:
                    block_ = std::string("IIR-M");  // Plane B
                    break;
                case 13:
                    block_ = std::string("IIR");  // Plane F
                    break;
                case 14:
                    block_ = std::string("III");  // Plane B
                    break;
                case 15:
                    block_ = std::string("IIR-M");  // Plane F
                    break;
                case 16:
                    block_ = std::string("IIR");  // Plane B
                    break;
                case 17:
                    block_ = std::string("IIR-M");  // Plane C
                    break;
                case 18:
                    block_ = std::string("III");  // Plane D
                    break;
                case 19:                          // NOLINT(bugprone-branch-clone)
                    block_ = std::string("IIR");  // Plane D
                    break;
                case 20:
                    block_ = std::string("IIR");  // Plane B
                    break;
                case 21:
                    block_ = std::string("IIR");  // Plane D
                    break;
                case 22:
                    block_ = std::string("IIR");  // Plane E
                    break;
                case 23:
                    block_ = std::string("III");  // Plane E
                    break;
                case 24:                          // NOLINT(bugprone-branch-clone)
                    block_ = std::string("IIF");  // Plane A
                    break;
                case 25:
                    block_ = std::string("IIF");  // Plane B
                    break;
                case 26:
                    block_ = std::string("IIF");  // Plane B
                    break;
                case 27:
                    block_ = std::string("IIF");  // Plane C
                    break;
                case 28:
                    block_ = std::string("IIR");  // Plane B
                    break;
                case 29:
                    block_ = std::string("IIR-M");  // Plane C
                    break;
                case 30:
                    block_ = std::string("IIF");  // Plane A
                    break;
                case 31:
                    block_ = std::string("IIR-M");  // Plane A
                    break;
                case 32:
                    block_ = std::string("IIF");  // Plane F
                    break;
                default:
                    block_ = std::string("Unknown");
                }
        }

    if (system_ == "Glonass")
        {
            // Info from http://www.sdcm.ru/smglo/grupglo?version=eng&site=extern
            // See also https://www.glonass-iac.ru/en/GLONASS/
            switch (PRN_)
                {
                case 1:
                    block_ = std::string("1");  // Plane 1
                    rf_link = 1;
                    break;
                case 2:
                    block_ = std::string("-4");  // Plane 1
                    rf_link = -4;
                    break;
                case 3:
                    block_ = std::string("5");  // Plane 1
                    rf_link = 5;
                    break;
                case 4:
                    block_ = std::string("6");  // Plane 1
                    rf_link = 6;
                    break;
                case 5:
                    block_ = std::string("1");  // Plane 1
                    rf_link = 1;
                    break;
                case 6:
                    block_ = std::string("-4");  // Plane 1
                    rf_link = -4;
                    break;
                case 7:
                    block_ = std::string("5");  // Plane 1
                    rf_link = 5;
                    break;
                case 8:
                    block_ = std::string("6");  // Plane 1
                    rf_link = 6;
                    break;
                case 9:
                    block_ = std::string("-2");  // Plane 2
                    rf_link = -2;
                    break;
                case 10:
                    block_ = std::string("-7");  // Plane 2
                    rf_link = -7;
                    break;
                case 11:
                    block_ = std::string("0");  // Plane 2
                    rf_link = 0;
                    break;
                case 12:
                    block_ = std::string("-1");  // Plane 2
                    rf_link = -1;
                    break;
                case 13:
                    block_ = std::string("-2");  // Plane 2
                    rf_link = -2;
                    break;
                case 14:
                    block_ = std::string("-7");  // Plane 2
                    rf_link = -7;
                    break;
                case 15:
                    block_ = std::string("0");  // Plane 2
                    rf_link = 0;
                    break;
                case 16:
                    block_ = std::string("-1");  // Plane 2
                    rf_link = -1;
                    break;
                case 17:
                    block_ = std::string("4");  // Plane 3
                    rf_link = 4;
                    break;
                case 18:
                    block_ = std::string("-3");  // Plane 3
                    rf_link = -3;
                    break;
                case 19:
                    block_ = std::string("3");  // Plane 3
                    rf_link = 3;
                    break;
                case 20:
                    block_ = std::string("2");  // Plane 3
                    rf_link = 2;
                    break;
                case 21:
                    block_ = std::string("4");  // Plane 3
                    rf_link = 4;
                    break;
                case 22:
                    block_ = std::string("-3");  // Plane 3
                    rf_link = -3;
                    break;
                case 23:
                    block_ = std::string("3");  // Plane 3
                    rf_link = 3;
                    break;
                case 24:
                    block_ = std::string("2");  // Plane 3
                    rf_link = 2;
                    break;
                default:
                    block_ = std::string("Unknown");
                }
        }
    if (system_ == "SBAS")
        {
            switch (PRN_)
                {
                case 120:
                    block_ = std::string("EGNOS Test Platform");  // Inmarsat 3-F2 (Atlantic Ocean Region-East)
                    break;
                case 123:
                    block_ = std::string("EGNOS");  // EGNOS Operational Platform. Astra 5B
                    break;
                case 131:                          // NOLINT(bugprone-branch-clone)
                    block_ = std::string("WAAS");  // WAAS Eutelsat 117 West B
                    break;
                case 135:
                    block_ = std::string("WAAS");  // WAAS Galaxy 15
                    break;
                case 136:
                    block_ = std::string("EGNOS");  // EGNOS Operational Platform. SES-5 (a.k.a. Sirius 5 or Astra 4B)
                    break;
                case 138:
                    block_ = std::string("WAAS");  // WAAS Anik F1R
                    break;
                default:
                    block_ = std::string("Unknown");
                }
        }
    if (system_ == "Galileo")
        {
            // Check https://en.wikipedia.org/wiki/List_of_Galileo_satellites and https://www.gsc-europa.eu/system-service-status/constellation-information
            switch (PRN_)
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
                    block_ = std::string("FOC-FM7");  // Galileo Full Operational Capability (FOC) satellite FM7 / GSAT-0207, launched on November 17, 2016.
                    break;
                case 8:
                    block_ = std::string("FOC-FM8");  // Galileo Full Operational Capability (FOC) satellite FM8 / GSAT0208, launched on December 17, 2015.
                    break;
                case 9:
                    block_ = std::string("FOC-FM9");  // Galileo Full Operational Capability (FOC) satellite FM9 / GSAT0209, launched on December 17, 2015.
                    break;
                case 10:
                    block_ = std::string("FOC-FM24");  // Galileo Full Operational Capability (FOC) satellite FM24 / GSAT0224, launched on December 5, 2021.
                    break;
                case 11:
                    block_ = std::string("IOV-PFM");  // PFM, the ProtoFlight Model / GSAT0101, launched from French Guiana at 10:30 GMT on October 21, 2011.
                    break;
                case 12:
                    block_ = std::string("IOV-FM2");  // Galileo In-Orbit Validation (IOV) satellite FM2 (Flight Model 2) also known as GSAT0102, from French Guiana at 10:30 GMT on October 21, 2011.
                    break;
                case 13:
                    block_ = std::string("FOC-FM20");  // Galileo Full Operational Capability (FOC) satellite FM20 / GSAT0220, launched on Jul. 25, 2018. UNDER COMMISSIONING.
                    break;
                case 14:
                    block_ = std::string("FOC-FM2*");  // Galileo Full Operational Capability (FOC) satellite FM2 / GSAT0202, launched into incorrect orbit on August 22, 2014. Moved to usable orbit in March, 2015. UNDER TESTING.
                    break;
                case 15:
                    block_ = std::string("FOC-FM21");  // Galileo Full Operational Capability (FOC) satellite FM21 / GSAT0221, launched on Jul. 25, 2018. UNDER COMMISSIONING.
                    break;
                case 18:
                    block_ = std::string("FOC-FM1*");  // Galileo Full Operational Capability (FOC) satellite FM1 / GSAT0201, launched into incorrect orbit on August 22, 2014. Moved to usable orbit in December, 2014. UNDER TESTING.
                    break;
                case 19:
                    block_ = std::string("IOV-FM3");  // Galileo In-Orbit Validation (IOV) satellite FM3 (Flight Model 3) / GSAT0103, launched on October 12, 2012.
                    break;
                case 20:
                    block_ = std::string("IOV-FM4**");  // Galileo In-Orbit Validation (IOV) satellite FM4 (Flight Model 4) / GSAT0104, launched on October 12, 2012. Payload power problem beginning May 27, 2014 led to permanent loss of E5 and E6 transmissions, E1 transmission restored. UNAVAILABLE FROM 2014-05-27 UNTIL FURTHER NOTICE
                    break;
                case 21:
                    block_ = std::string("FOC-FM15");  // Galileo Full Operational Capability (FOC) satellite FM15 / GSAT0215, launched on Dec. 12, 2017. UNDER COMMISSIONING.
                    break;
                case 22:
                    block_ = std::string("FOC-FM4**");  // Galileo Full Operational Capability (FOC) satellite FM4 / GSAT0204, launched on March 27, 2015. REMOVED FROM ACTIVE SERVICE ON 2017-12-08 UNTIL FURTHER NOTICE FOR CONSTELLATION MANAGEMENT PURPOSES.
                    break;
                case 24:
                    block_ = std::string("FOC-FM5");  // Galileo Full Operational Capability (FOC) satellite FM5 / GSAT0205, launched on Sept. 11, 2015.
                    break;
                case 25:
                    block_ = std::string("FOC-FM16");  // Galileo Full Operational Capability (FOC) satellite FM16 / GSAT0216, launched on Dec. 12, 2017. UNDER COMMISSIONING.
                    break;
                case 26:
                    block_ = std::string("FOC-FM3");  // Galileo Full Operational Capability (FOC) satellite FM3 / GSAT0203, launched on March 27, 2015.
                    break;
                case 27:
                    block_ = std::string("FOC-FM17");  // Galileo Full Operational Capability (FOC) satellite FM17 / GSAT0217, launched on Dec. 12, 2017. UNDER COMMISSIONING.
                    break;
                case 30:
                    block_ = std::string("FOC-FM6");  // Galileo Full Operational Capability (FOC) satellite FM6 / GSAT0206, launched on Sept. 11, 2015.
                    break;
                case 31:
                    block_ = std::string("FOC-FM18");  // Galileo Full Operational Capability (FOC) satellite FM18 / GSAT0218, launched on Dec. 12, 2017. UNDER COMMISSIONING.
                    break;
                case 33:
                    block_ = std::string("FOC-FM22");  // Galileo Full Operational Capability (FOC) satellite FM22 / GSAT0222, launched on Jul. 25, 2018. UNDER COMMISSIONING.
                    break;
                case 34:
                    block_ = std::string("FOC-FM23");  // Galileo Full Operational Capability (FOC) satellite FM23 / GSAT0223, launched on December 5, 2021.
                    break;
                case 36:
                    block_ = std::string("FOC-FM19");  // Galileo Full Operational Capability (FOC) satellite FM19 / GSAT0219, launched on Jul. 25, 2018. UNDER COMMISSIONING.
                    break;
                default:
                    block_ = std::string("Unknown");
                }
        }
    if (system_ == "Beidou")
        {
            // Check https://en.wikipedia.org/wiki/List_of_BeiDou_satellites
            switch (PRN_)
                {
                case 1:
                    block_ = std::string("BeiDou-2 GEO01*");  // GEO 140.0°E; launched 2010/01/16 (Retired)
                    break;
                case 2:
                    block_ = std::string("BeiDou-2 GEO06");  // GEO 80°E; launched 2012/10/25
                    break;
                case 3:
                    block_ = std::string("BeiDou-2 GEO07");  // GEO 110.5°E; launched 2016/06/12
                    break;
                case 4:
                    block_ = std::string("BeiDou-2 GEO04");  // GEO 160.0°E; launched 2010/10/31
                    break;
                case 5:
                    block_ = std::string("BeiDou-2 GEO05");  // GEO 58.75°E; launched 2012/02/24
                    break;
                case 6:
                    block_ = std::string("BeiDou-2 IGSO01");  // 55° inclination IGSO 118°E; launched 2010/07/31
                    break;
                case 7:
                    block_ = std::string("BeiDou-2 IGSO02");  // 55° inclination IGSO 118°E; launched 2010/12/17
                    break;
                case 8:
                    block_ = std::string("BeiDou-2 IGSO03");  // 55° inclination IGSO 118°E; launched 2011/04/09
                    break;
                case 9:
                    block_ = std::string("BeiDou-2 IGSO04");  // 55° inclination IGSO 95°E; launched 2011/07/27
                    break;
                case 10:
                    block_ = std::string("BeiDou-2 IGSO05");  // 55° inclination IGSO 118°E; launched 2011/12/01
                    break;
                case 11:
                    block_ = std::string("BeiDou-2 MEO03*");  // Slot A07; launched 2012/04/29 (Retired)
                    break;
                case 12:
                    block_ = std::string("BeiDou-2 MEO04*");  // Slot A08; launched 2012/04/29 (Retired)
                    break;
                case 13:
                    block_ = std::string("BeiDou-2 IGSO06");  // launched 2016/03/30
                    break;
                case 14:
                    block_ = std::string("BeiDou-2 MEO06*");  // launched 2012/09/19 (Retired)
                    break;
                case 16:
                    block_ = std::string("BeiDou-2 IGSO07");  // launched 2018/07/10
                    break;
                case 18:
                    block_ = std::string("BeiDou-3 GEOG8");  // launched 2019/05/17
                    break;
                case 19:
                    block_ = std::string("BeiDou-3 MEO01");  // Slot B07; launched 2017/11/05
                    break;
                case 20:
                    block_ = std::string("BeiDou-3 MEO02");  // Slot B05; launched 2017/11/05
                    break;
                case 21:
                    block_ = std::string("BeiDou-3 MEO03");  // Slot B0?; launched 2018/02/12
                    break;
                case 22:
                    block_ = std::string("BeiDou-3 MEO04");  // Slot B06; launched 2018/02/12
                    break;
                case 23:
                    block_ = std::string("BeiDou-3 MEO05");  // Slot C07; launched 2018/07/29
                    break;
                case 24:
                    block_ = std::string("BeiDou-3 MEO06");  // Slot C01; launched 2018/07/29
                    break;
                case 25:
                    block_ = std::string("BeiDou-3 MEO11");  // Slot C08; launched 2018/08/24
                    break;
                case 26:
                    block_ = std::string("BeiDou-3 MEO12");  // Slot C02; launched 2018/08/24
                    break;
                case 27:
                    block_ = std::string("BeiDou-3 3M3");  // Slot A04; launched 2018/01/11
                    break;
                case 28:
                    block_ = std::string("BeiDou-3 3M4");  // Slot A05; launched 2018/01/11
                    break;
                case 29:
                    block_ = std::string("BeiDou-3 3M7");  // Slot A02; launched 2018/03/29
                    break;
                case 30:
                    block_ = std::string("BeiDou-3 3M8");  // Slot A03; launched 2018/03/29
                    break;
                case 32:
                    block_ = std::string("BeiDou-3 MEO13");  // Slot B01; launched 2018/09/19
                    break;
                case 33:
                    block_ = std::string("BeiDou-3 MEO14");  // Slot B03; launched 2018/09/19
                    break;
                case 34:
                    block_ = std::string("BeiDou-3 MEO15");  // Slot B03; launched 2018/10/15
                    break;
                case 35:
                    block_ = std::string("BeiDou-3 MEO16");  // Slot B03; launched 2018/10/15
                    break;
                case 36:
                    block_ = std::string("BeiDou-3 MEO17");  // Slot B03; launched 2018/11/18
                    break;
                case 37:
                    block_ = std::string("BeiDou-3 MEO18");  // Slot B03; launched 2018/11/18
                    break;
                case 38:
                    block_ = std::string("BeiDou-3 IGSOI1");  // launched 2019/04/20
                    break;
                case 39:
                    block_ = std::string("BeiDou-3 IGSOI2");  // launched 2019/04/20
                    break;
                case 40:
                    block_ = std::string("BeiDou-3 IGSOI3");  // launched 2019/11/04
                    break;
                case 41:
                    block_ = std::string("BeiDou-3 MEO19");  // Slot B02, launched 2019/12/16
                    break;
                case 42:
                    block_ = std::string("BeiDou-3 MEO20");  // Slot B04, launched 2019/12/16
                    break;
                case 43:
                    block_ = std::string("BeiDou-3 MEO21");  // Slot A06, launched 2019/11/23
                    break;
                case 44:
                    block_ = std::string("BeiDou-3 MEO22");  // Slot A08, launched 2019/11/23
                    break;
                case 45:
                    block_ = std::string("BeiDou-3 MEO23");  // Slot C03, launched 2019/09/22
                    break;
                case 46:
                    block_ = std::string("BeiDou-3 MEO24");  // Slot C05, launched 2019/09/22
                    break;
                case 59:
                    block_ = std::string("BeiDou-3 GEOG1");  // launched 2018/11/01
                    break;
                case 60:
                    block_ = std::string("BeiDou-3 GEOG2");  // launched 2020/03/20
                    break;
                case 61:
                    block_ = std::string("BeiDou-3 GEOG3");  // launched 2020/06/2023
                    break;
                default:
                    block_ = std::string("Unknown");
                }
        }
    return block_;
}


void Gnss_Satellite::set_block(const std::string& system_, uint32_t PRN_)
{
    block = what_block(system_, PRN_);
}
