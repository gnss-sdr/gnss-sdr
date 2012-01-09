/*!
 * \file rinex_2_1_printer.cc (temporal name)
 * \brief Implementation of a RINEX 3.01 printer
 * See http://igscb.jpl.nasa.gov/igscb/data/format/rinex301.pdf
 * \author Carles Fernandez Prades, 2011. cfernandez(at)cttc.es
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

#include "rinex_2_1_printer.h"
#include "gps_navigation_message.h"
#include <ostream>
#include <fstream>
#include <stdlib.h> // for getenv()
#include <iostream>
#include <string>
#include <math.h>  // for floor
#include <algorithm> // for min and max
#include "boost/date_time/time_zone_base.hpp"
#include "boost/date_time/gregorian/gregorian.hpp"
#include "boost/date_time/local_time/local_time.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"
#include <glog/log_severity.h>
#include <glog/logging.h>
#include <gflags/gflags.h>



using google::LogMessage;


rinex_printer::rinex_printer()
{

    rinex_printer::navFile.open(rinex_printer::createFilename("RINEX_FILE_TYPE_GPS_NAV"), std::ios::out | std::ios::app);
    rinex_printer::obsFile.open(rinex_printer::createFilename("RINEX_FILE_TYPE_OBS"), std::ios::out | std::ios::app);

    satelliteSystem["GPS"]="G";
    satelliteSystem["GLONASS"]="R";
    satelliteSystem["SBAS payload"]="S";
    satelliteSystem["Galileo"]="E";
    satelliteSystem["Compass"]="C";

    observationCode["GPS_L1_CA"] = "1C";             //!< "1C" GPS L1 C/A
    observationCode["GPS_L1_P"] = "1P";               //!< "1P" GPS L1 P
    observationCode["GPS_L1_Z_TRACKING"] = "1W";      //!< "1W" GPS L1 Z-tracking and similar (AS on)
    observationCode["GPS_L1_Y"] = "1Y";               //!< "1Y" GPS L1 Y
    observationCode["GPS_L1_M "]= "1M";               //!< "1M" GPS L1 M
    observationCode["GPS_L1_CODELESS"] = "1N";       //!< "1N" GPS L1 codeless
    observationCode["GPS_L2_CA"]= "2C";             //!< "2C" GPS L2 C/A
    observationCode["L2_SEMI_CODELESS"] = "2D";   //!< "2D" GPS L2 L1(C/A)+(P2-P1) semi-codeless
    observationCode["GPS_L2_L2CM"] = "2S";            //!< "2S" GPS L2 L2C (M)
    observationCode["GPS_L2_L2CL"] = "2L";            //!< "2L" GPS L2 L2C (L)
    observationCode["GPS_L2_L2CML"] = "2X";           //!< "2X" GPS L2 L2C (M+L)
    observationCode["GPS_L2_P"] = "2P";               //!< "2P" GPS L2 P
    observationCode["GPS_L2_Z_TRACKING"] = "2W";      //!< "2W" GPS L2 Z-tracking and similar (AS on)
    observationCode["GPS_L2_Y"] = "2Y";               //!< "2Y" GPS L2 Y
    observationCode["GPS_L2_M"] = "2M";               //!< "2M" GPS GPS L2 M
    observationCode["GPS_L2_codeless"] = "2N";        //!< "2N" GPS L2 codeless
    observationCode["GPS_L5_I"] = "5I";               //!< "5I" GPS L5 I
    observationCode["GPS_L5_Q"] = "5Q";               //!< "5Q" GPS L5 Q
    observationCode["GPS_L5_IQ"] = "5X";              //!< "5X" GPS L5 I+Q
    observationCode["GLONASS_G1_CA"] = "1C";          //!< "1C" GLONASS G1 C/A
    observationCode["GLONASS_G1_P"]= "1P";            //!< "1P" GLONASS G1 P
    observationCode["GLONASS_G2_CA"]= "2C";           //!< "2C" GLONASS G2 C/A  (Glonass M)
    observationCode["GLONASS_G2_P"]= "2P";            //!< "2P" GLONASS G2 P
    observationCode["GALILEO_E1_A"]= "1A";            //!< "1A" GALILEO E1 A (PRS)
    observationCode["GALILEO_E1_B"]= "1B";            //!< "1B" GALILEO E1 B (I/NAV OS/CS/SoL)
    observationCode["GALILEO_E1_C"]= "1C";            //!< "1C" GALILEO E1 C (no data)
    observationCode["GALILEO_E1_BC"]= "1X";           //!< "1X" GALILEO E1 B+C
    observationCode["GALILEO_E1_ABC"]= "1Z";          //!< "1Z" GALILEO E1 A+B+C
    observationCode["GALILEO_E5a_I"]= "5I";           //!< "5I" GALILEO E5a I (F/NAV OS)
    observationCode["GALILEO_E5a_Q"]= "5Q";           //!< "5Q" GALILEO E5a Q  (no data)
    observationCode["GALILEO_E5aIQ"]= "5X";           //!< "5X" GALILEO E5a I+Q
    observationCode["GALILEO_E5b_I"]= "7I";           //!< "7I" GALILEO E5b I
    observationCode["GALILEO_E5b_Q"]= "7Q";           //!< "7Q" GALILEO E5b Q
    observationCode["GALILEO_E5b_IQ"]= "7X";          //!< "7X" GALILEO E5b I+Q
    observationCode["GALILEO_E5_I"]= "8I";            //!< "8I" GALILEO E5 I
    observationCode["GALILEO_E5_Q"]= "8Q";            //!< "8Q" GALILEO E5 Q
    observationCode["GALILEO_E5_IQ"]= "8X";           //!< "8X" GALILEO E5 I+Q
    observationCode["GALILEO_E56_A"]= "6A";           //!< "6A" GALILEO E6 A
    observationCode["GALILEO_E56_B"] = "6B";          //!< "6B" GALILEO E6 B
    observationCode["GALILEO_E56_B"] = "6C";          //!< "6C" GALILEO E6 C
    observationCode["GALILEO_E56_BC"] = "6X";         //!< "6X" GALILEO E6 B+C
    observationCode["GALILEO_E56_ABC"] = "6Z";        //!< "6Z" GALILEO E6 A+B+C
    observationCode["SBAS_L1_CA"] = "1C";             //!< "1C" SBAS L1 C/A
    observationCode["SBAS_L5_I"] = "5I";              //!< "5I" SBAS L5 I
    observationCode["SBAS_L5_Q"] = "5Q";              //!< "5Q" SBAS L5 Q
    observationCode["SBAS_L5_IQ"] = "5X";            //!< "5X" SBAS L5 I+Q
    observationCode["COMPASS_E2_I"] = "2I";
    observationCode["COMPASS_E2_Q"] = "2Q";
    observationCode["COMPASS_E2_IQ"] = "2X";
    observationCode["COMPASS_E5b_I"] = "7I";
    observationCode["COMPASS_E5b_Q"] = "7Q";
    observationCode["COMPASS_E5b_IQ"] = "7X";
    observationCode["COMPASS_E6_I"] = "6I";
    observationCode["COMPASS_E6_Q"] = "6Q";
    observationCode["COMPASS_E6_IQ"] = "6X";

    observationType["PSEUDORANGE"]="C";
    observationType["CARRIER_PHASE"]="L";
    observationType["DOPPLER"]="D";
    observationType["SIGNAL_STRENGTH"]="S";

}



rinex_printer::~rinex_printer()
{
    // close RINEX files
    rinex_printer::navFile.close();
    rinex_printer::obsFile.close();
}




void rinex_printer::lengthCheck(std::string line)
{
    if (line.length() != 80)
        {
            LOG_AT_LEVEL(ERROR) << "Bad defined RINEX line: "
                    << line.length() << " characters (must be 80)"<< std::endl
                    << line << std::endl
                    << "----|---1|0---|---2|0---|---3|0---|---4|0---|---5|0---|---6|0---|---7|0---|---8|"<< std::endl;
        }
}


std::string rinex_printer::createFilename(std::string type){
    const std::string stationName = "GSDR"; // 4-character station name designator
    boost::gregorian::date today = boost::gregorian::day_clock::local_day();
    const int dayOfTheYear = today.day_of_year();
    std::stringstream strm0;
    if (dayOfTheYear<100) strm0 << "0"; // three digits for day of the year
    if (dayOfTheYear<10) strm0 << "0"; // three digits for day of the year
    strm0 << dayOfTheYear;
    std::string dayOfTheYearTag=strm0.str();


    std::map<std::string, std::string> fileType;
    fileType.insert(std::pair<std::string, std::string>("RINEX_FILE_TYPE_OBS","O"));        // O - Observation file.
    fileType.insert(std::pair<std::string, std::string>("RINEX_FILE_TYPE_GPS_NAV","N"));    // N - GPS navigation message file.
    fileType.insert(std::pair<std::string, std::string>("RINEX_FILE_TYPE_MET","M"));        // M - Meteorological data file.
    fileType.insert(std::pair<std::string, std::string>("RINEX_FILE_TYPE_GLO_NAV","G"));    // G - GLONASS navigation file.
    fileType.insert(std::pair<std::string, std::string>("RINEX_FILE_TYPE_GAL_NAV","L"));    // L - Galileo navigation message file.
    fileType.insert(std::pair<std::string, std::string>("RINEX_FILE_TYPE_MIXED_NAV","P"));  // P - Mixed GNSS navigation message file.
    fileType.insert(std::pair<std::string, std::string>("RINEX_FILE_TYPE_GEO_NAV","H"));    // H - SBAS Payload navigation message file.
    fileType.insert(std::pair<std::string, std::string>("RINEX_FILE_TYPE_SBAS","B"));       // B - SBAS broadcast data file.
    fileType.insert(std::pair<std::string, std::string>("RINEX_FILE_TYPE_CLK","C"));        // C - Clock file.
    fileType.insert(std::pair<std::string, std::string>("RINEX_FILE_TYPE_SUMMARY","S"));    // S - Summary file (used e.g., by IGS, not a standard!).

    boost::posix_time::ptime pt=boost::posix_time::second_clock::local_time();
    tm pt_tm=boost::posix_time::to_tm(pt);
    int local_hour=pt_tm.tm_hour;
    std::stringstream strm;
    strm << local_hour;

    std::map<std::string, std::string> Hmap;

    Hmap.insert(std::pair<std::string, std::string>("0","a"));
    Hmap.insert(std::pair<std::string, std::string>("1","b"));
    Hmap.insert(std::pair<std::string, std::string>("2","c"));
    Hmap.insert(std::pair<std::string, std::string>("3","d"));
    Hmap.insert(std::pair<std::string, std::string>("4","e"));
    Hmap.insert(std::pair<std::string, std::string>("5","f"));
    Hmap.insert(std::pair<std::string, std::string>("6","g"));
    Hmap.insert(std::pair<std::string, std::string>("7","h"));
    Hmap.insert(std::pair<std::string, std::string>("8","i"));
    Hmap.insert(std::pair<std::string, std::string>("9","j"));
    Hmap.insert(std::pair<std::string, std::string>("10","k"));
    Hmap.insert(std::pair<std::string, std::string>("11","l"));
    Hmap.insert(std::pair<std::string, std::string>("12","m"));
    Hmap.insert(std::pair<std::string, std::string>("13","n"));
    Hmap.insert(std::pair<std::string, std::string>("14","o"));
    Hmap.insert(std::pair<std::string, std::string>("15","p"));
    Hmap.insert(std::pair<std::string, std::string>("16","q"));
    Hmap.insert(std::pair<std::string, std::string>("17","r"));
    Hmap.insert(std::pair<std::string, std::string>("8","s"));
    Hmap.insert(std::pair<std::string, std::string>("19","t"));
    Hmap.insert(std::pair<std::string, std::string>("20","u"));
    Hmap.insert(std::pair<std::string, std::string>("21","v"));
    Hmap.insert(std::pair<std::string, std::string>("22","w"));
    Hmap.insert(std::pair<std::string, std::string>("23","x"));

    std::string hourTag = Hmap[strm.str()];

    int local_minute=pt_tm.tm_min;
    std::stringstream strm2;
    if (local_minute<10) strm2 << "0"; // at least two digits for minutes
    strm2 << local_minute;

    std::string minTag = strm2.str();

    int local_year=pt_tm.tm_year-100; // 2012 is 112
    std::stringstream strm3;
    strm3 << local_year;
    std::string yearTag = strm3.str();

    std::string typeOfFile=fileType[type];

    std::string filename(stationName +dayOfTheYearTag+ hourTag + minTag + "." + yearTag + typeOfFile);
    return filename;
}


std::string rinex_printer::getLocalTime()
{
    std::string line;
    line +=std::string("GNSS-SDR");
    line +=std::string(12,' ');
    line += rinex_printer::leftJustify("CTTC", 20);//put a flag to let the user change this
    boost::gregorian::date today = boost::gregorian::day_clock::local_day();
    line +=boost::gregorian::to_iso_string(today);
    line +=std::string(1,' ');


    boost::local_time::time_zone_ptr zone(new boost::local_time::posix_time_zone("UTC"));
    boost::local_time::local_date_time pt = boost::local_time::local_sec_clock::local_time(zone);
    tm pt_tm=boost::local_time::to_tm(pt);

    std::stringstream strm0;
    int utc_hour=pt_tm.tm_hour;
    if (utc_hour<10) strm0 << "0"; //  two digits for hours
    strm0 << utc_hour;
    line += strm0.str();

    std::stringstream strm1;
    int utc_minute=pt_tm.tm_min;
    if (utc_minute<10) strm1 << "0"; //  two digits for minutes
    strm1 << utc_minute;
    line += strm1.str();

    std::stringstream strm2;
    int utc_seconds=pt_tm.tm_sec;
    if (utc_seconds<10) strm2 << "0"; //  two digits for seconds
    strm2 << utc_seconds;
    line += strm2.str();
    line +=std::string(1,' ');
    line +=std::string("UTC");
    line +=std::string(1,' ');
    return line;
}


void rinex_printer::Rinex2NavHeader(std::ofstream& out, gps_navigation_message nav_msg)
{

    std::string line;

    // -------- Line 1
    std::string version="3.01";
    line = std::string(5,' ');
    line += version;
    line += std::string(11,' ');
    line += std::string("N: GNSS NAV DATA");
    line += std::string(4,' ');
    //! \todo Add here other systems...
    line += std::string("G: GPS");
    line += std::string(14,' ');
    // ...
    line += std::string("RINEX VERSION / TYPE");
    rinex_printer::lengthCheck(line);

    out << line << std::endl;
    // -------- Line 2
    line.clear();
    line += rinex_printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1,' ');
    rinex_printer::lengthCheck(line);
    out << line << std::endl;
    // -------- Line 3
    line.clear();
    line += rinex_printer::leftJustify("GPS NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR",60);
    line += rinex_printer::leftJustify("COMMENT",20);
    rinex_printer::lengthCheck(line);
    out << line << std::endl;
    // -------- Line 4 ionospheric info
    line.clear();
    line += std::string("GPSA");
    line += std::string(1,' ');
    line += rinex_printer::rightJustify(rinex_printer::doub2for(nav_msg.d_alpha0, 12, 2),12);
    line += rinex_printer::rightJustify(rinex_printer::doub2for(nav_msg.d_alpha1, 12, 2),12);
    line += rinex_printer::rightJustify(rinex_printer::doub2for(nav_msg.d_alpha2, 12, 2),12);
    line += rinex_printer::rightJustify(rinex_printer::doub2for(nav_msg.d_alpha3, 12, 2),12);
    line += std::string(7,' ');
    line += rinex_printer::leftJustify("IONOSPHERIC CORR",20);
    rinex_printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 5 ionospheric info
    line.clear();
    line += std::string("GPSB");
    line += std::string(1,' ');
    line += rinex_printer::rightJustify(rinex_printer::doub2for(nav_msg.d_beta0, 12, 2),12);
    line += rinex_printer::rightJustify(rinex_printer::doub2for(nav_msg.d_beta1, 12, 2),12);
    line += rinex_printer::rightJustify(rinex_printer::doub2for(nav_msg.d_beta2, 12, 2),12);
    line += rinex_printer::rightJustify(rinex_printer::doub2for(nav_msg.d_beta3, 12, 2),12);
    line += std::string(7,' ');
    line += rinex_printer::leftJustify("IONOSPHERIC CORR",20);
    rinex_printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 5 system time correction
    line.clear();
    line += std::string("GPUT");
    line += std::string(1,' ');
    line += rinex_printer::doub2for(nav_msg.d_A0, 17, 2);
    line += rinex_printer::doub2for(nav_msg.d_A1, 16, 2);
    line += rinex_printer::rightJustify(boost::lexical_cast<std::string>(nav_msg.d_t_OT),7);
    line += rinex_printer::rightJustify(boost::lexical_cast<std::string>(nav_msg.i_WN_T),5);
    /*  if ( SBAS )
        {
          line += string(1, ' ');
          line += leftJustify(asString(d_t_OT_SBAS),5);
          line += string(1, ' ');
          line += leftJustify(asString(d_WN_T_SBAS),2);
          line += string(1, ' ');
               }
          else
     */
    line += std::string(10, ' ');
    line += rinex_printer::leftJustify("TIME SYSTEM CORR",20);
    rinex_printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 6 leap seconds

    // For leap second information, see http://www.endruntechnologies.com/leap.htm
    line.clear();
    line += rinex_printer::rightJustify(boost::lexical_cast<std::string>(nav_msg.d_DeltaT_LS),6);
    line += rinex_printer::rightJustify(boost::lexical_cast<std::string>(nav_msg.d_DeltaT_LSF),6);
    line += rinex_printer::rightJustify(boost::lexical_cast<std::string>(nav_msg.i_WN_LSF),6);
    line += rinex_printer::rightJustify(boost::lexical_cast<std::string>(nav_msg.i_DN),6);
    line += std::string(36, ' ');
    line += rinex_printer::leftJustify("LEAP SECONDS",20);
    rinex_printer::lengthCheck(line);
    out << line << std::endl;


    // -------- End of Header
    line.clear();
    line +=std::string(60,' ');
    line += rinex_printer::leftJustify("END OF HEADER",20);
    rinex_printer::lengthCheck(line);
    out << line << std::endl;
}










void rinex_printer::LogRinex2Nav(std::ofstream& out, gps_navigation_message nav_msg){

    /*
	if(fp_rin2 != NULL)
	{

		//preparacio lines de efemerides per imprimir!!!
		char linia0[256],linia1[256],linia2[256],linia3[256],linia4[256],linia5[256],linia6[256],linia7[256];
		char idef[256];
		sprintf(idef,"%2.0d",nav_msg.d_satellite_PRN);

		sprintf(linia0,"%19.12E%19.12E%19.12E",nav_msg.d_A_f0,nav_msg.d_A_f1,nav_msg.d_A_f2);

		sprintf(linia1,"%19.12E%19.12E%19.12E%19.12E",nav_msg.d_IODE_SF2,nav_msg.d_Crs,nav_msg.d_Delta_n,nav_msg.d_M_0);
		sprintf(linia2,"%19.12E%19.12E%19.12E%19.12E",nav_msg.d_Cuc,nav_msg.d_e_eccentricity,nav_msg.d_Cus,nav_msg.d_sqrt_A);
		sprintf(linia3,"%19.12E%19.12E%19.12E%19.12E",nav_msg.d_Toe,nav_msg.d_Cic,nav_msg.d_OMEGA0,nav_msg.d_Cis);
		sprintf(linia4,"%19.12E%19.12E%19.12E%19.12E",nav_msg.d_i_0,nav_msg.d_Crc,nav_msg.d_OMEGA,nav_msg.d_OMEGA_DOT);

		sprintf(linia5,"%19.12E%19.12E%19.12E%19.12E",nav_msg.d_IDOT,0.0,nav_msg.d_GPS_week+1024.0,0.0);//CodeL2, L2pData
		sprintf(linia6,"%19.12E%19.12E%19.12E%19.12E",nav_msg.d_SV_accuracy,nav_msg.d_SV_health,nav_msg.d_TGD,nav_msg.d_IODC);
		sprintf(linia7,"%19.12E%19.12E",nav_msg.d_TOW,0.0); //fit interval is set to 0

		fseek(fp_rin2, fp_rin_end2, SEEK_SET);

		fprintf(fp_rin2,"%s %s %s%s\n",idef,cad1,cad2,linia0);

		fprintf(fp_rin2,"   %s\n",linia1);

		fprintf(fp_rin2,"   %s\n",linia2);

		fprintf(fp_rin2,"   %s\n",linia3);

		fprintf(fp_rin2,"   %s\n",linia4);

		fprintf(fp_rin2,"   %s\n",linia5);

		fprintf(fp_rin2,"   %s\n",linia6);

		fprintf(fp_rin2,"   %s\n",linia7);
		fp_rin_end2 = ftell(fp_rin2);
	}*/

}

void rinex_printer::Rinex2ObsHeader(std::ofstream& out, gps_navigation_message nav_msg)
{

    std::string line;

    // -------- Line 1
    std::string version="3.01";
    line = std::string(5,' ');
    line += version;
    line +=std::string(11,' ');
    line += rinex_printer::leftJustify("OBSERVATION DATA",20);
    line +=satelliteSystem["GPS"];
    line +=std::string(19,' ');
    line +=std::string("RINEX VERSION / TYPE");
    rinex_printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 2
    line.clear();
    line += rinex_printer::leftJustify("G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED",60);
    line += rinex_printer::leftJustify("COMMENT",20);
    rinex_printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 3
    line.clear();
    line += rinex_printer::getLocalTime();
    line +=std::string("PGM / RUN BY / DATE");
    line +=std::string(1,' ');
    rinex_printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 4
    line.clear();
    line += rinex_printer::leftJustify("GPS OBSERVATION DATA FILE GENERATED BY GNSS-SDR",60);
    line += rinex_printer::leftJustify("COMMENT",20);
    rinex_printer::lengthCheck(line);
    out << line << std::endl;
    // -------- Line 5
    line.clear();
    line += rinex_printer::leftJustify("DEFAULT MARKER NAME",60); // put a flag or a property,
    line += rinex_printer::leftJustify("MARKER NAME",20);
    rinex_printer::lengthCheck(line);
    out << line << std::endl;

    line.clear();
    line += rinex_printer::leftJustify("GROUND_CRAFT",20); // put a flag or a property
    line +=std::string(40,' ');
    line += rinex_printer::leftJustify("MARKER TYPE",20);
    rinex_printer::lengthCheck(line);
    out << line << std::endl;


    // -------- Line OBSERVER / AGENCY
    line.clear();
    std::string username=getenv("USER");
    line += leftJustify(username,20);
    line += rinex_printer::leftJustify("CTTC",40); // add flag and property
    line += rinex_printer::leftJustify("OBSERVER / AGENCY",20);
    rinex_printer::lengthCheck(line);
    out << line << std::endl;



    // -------- Line  REC / TYPE VERS
    line.clear();
    line += rinex_printer::leftJustify("GNSS-SDR",20); // add flag and property
    line += rinex_printer::leftJustify("Software Receiver",20); // add flag and property
    //line += rinex_printer::leftJustify(google::VersionString(),20); // add flag and property
    line += rinex_printer::leftJustify("0.1",20);
    line += rinex_printer::leftJustify("REC # / TYPE / VERS",20);
    lengthCheck(line);
    out << line << std::endl;

    // -------- ANTENNA TYPE
    line.clear();
    line += rinex_printer::leftJustify("Antenna number",20);  // add flag and property
    line += rinex_printer::leftJustify("Antenna type",20);  // add flag and property
    line +=std::string(20,' ');
    line += rinex_printer::leftJustify("ANT # / TYPE",20);
    rinex_printer::lengthCheck(line);
    out << line << std::endl;

    // -------- APPROX POSITION  (optional for moving platforms)


    // -------- ANTENNA: DELTA H/E/N
    // put here real data!
    double antena_h=0.0;
    double antena_e=0.0;
    double antena_n=0.0;
    line.clear();
    line  = rinex_printer::rightJustify(rinex_printer::asString(antena_h, 4), 14);
    line += rinex_printer::rightJustify(rinex_printer::asString(antena_e, 4), 14);
    line += rinex_printer::rightJustify(rinex_printer::asString(antena_n, 4), 14);
    line += std::string(18, ' ');
    line += rinex_printer::leftJustify("ANTENNA: DELTA H/E/N",20);
    rinex_printer::lengthCheck(line);
    out << line << std::endl;


    // -------- SYS / OBS TYPES

    // one line per available system
    line.clear();
    line += satelliteSystem["GPS"];
    line +=std::string(2,' ');
    int numberObservations=2; // Count the number of available types of observable in the system
    std::stringstream strm;
    strm << numberObservations;
    line += rinex_printer::rightJustify(strm.str(),3);
    // per type of observation
    line += std::string(1,' ');
    line += observationType["PSEUDORANGE"];
    line += observationCode["GPS_L1_CA"];
    line += std::string(1,' ');
    line += observationType["SIGNAL_STRENGTH"];
    line += observationCode["GPS_L1_CA"];

    line +=std::string(60-line.size(),' ');
    line += rinex_printer::leftJustify("SYS / # / OBS TYPES",20);
    rinex_printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Signal Strength units
    line.clear();
    line += rinex_printer::leftJustify("DBHZ",20);
    line +=std::string(40,' ');
    line += rinex_printer::leftJustify("SIGNAL STRENGTH UNIT",20);
    rinex_printer::lengthCheck(line);
    out << line << std::endl;

    // -------- TIME OF FIRST OBS
    line.clear();
    boost::posix_time::ptime p_utc_time = rinex_printer::computeTime(nav_msg);
    std::string timestring=boost::posix_time::to_iso_string(p_utc_time);
    std::string year (timestring,0,4);
    std::string month (timestring,4,2);
    std::string day (timestring,6,2);
    std::string hour (timestring,9,2);
    std::string minutes (timestring,11,2);
    double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(nav_msg.d_TOW));
    double seconds = fmod(utc_t,60);
    line += rightJustify(year,  6);
    line += rightJustify(month,  6);
    line += rightJustify(day,  6);
    line += rightJustify(hour,  6);
    line += rightJustify(minutes,  6);
    line += rightJustify(asString(seconds,7), 13);
    line += rightJustify(std::string("GPS"),  8);
    line +=std::string(9,' ');
    line += rinex_printer::leftJustify("TIME OF FIRST OBS",20);
    rinex_printer::lengthCheck(line);
    out << line << std::endl;

    // -------- SYS /PHASE SHIFTS

    // -------- end of header
    line.clear();
    line +=std::string(60,' ');
    line += rinex_printer::leftJustify("END OF HEADER",20);
    rinex_printer::lengthCheck(line);
    out << line << std::endl;
}




void rinex_printer::LogRinex2Obs(gps_navigation_message nav_msg,double pseudoranges_clock, std::map<int,float> pseudoranges)
{
    /* int ss;
	char sat_vis[36];
	for(int i=0;i<36;i++) sat_vis[i]=' ';
	char packet[80];
	int index=0;
	char cad[2];
	double setmanes;
	std::map<int,float>::iterator pseudoranges_iter;

	//necessito
	//1-Data i hora<--- de struct Clock_S
	//2-#sat visibles, identificador dat visibles-----> de  Com fa a LogNav()  // (Chan_Packet_S *) &tChan[lcv]->sv
	//3-pseudodistancia de cada satèl·lit ----> Com fa a LogPseudo(), tb es pot treure la carrier_phase. Serveix per algo??
	//4- El punt 1 i 2 s'han d'escriure a la mateixa línia. El punt 3 una línia per a cada satèl·lit.
	if(fp_rin != NULL)
	{
		setmanes=nav_msg.d_GPS_week + 1024;
		//1-Calcul data i hora gps
		//Calculo el any,mes i dia a partir de l'hora UTC
		//calculate UTC_TIME
		time_t temps;
		char cad1[80];
		char cad2[80];
		char cad3[80];
		char cad4[80];
		struct tm *tmPtr;
		//Calculo hora, minut, segons a partir de pClocK.time =hora GPS
		double decimalday,daydecimalhour,decimalhour,decimalmin,decimalsec;
		double day,hour,minutes,seconds,enterseconds,a;
		double gpstime;
		gpstime=pseudoranges_clock; //[s]
		//calculate date of gps time:
		//Days & weeks between 00h 1 Jan 1970 and 00h 6 Jan 1980
		//520 weeks and 12 days.

		temps=(520+setmanes)*7*24*3600+gpstime+17*24*3600;
		tmPtr = gmtime(&temps);
		strftime( cad1, 20, " %y %m %d", tmPtr );
		strftime( cad2, 20, "  %Y    %m    %d", tmPtr );
		decimalday=(gpstime/(24*3600));//Dies dins de la semana
		daydecimalhour=modf(decimalday,&day);//day=#dies sencers, daydecimalhour=porcio de dia
		daydecimalhour=daydecimalhour*24;//porcio de dia en hores
		decimalhour=modf(daydecimalhour,&hour);//hour=hora del dia; decimalhour=porcio d'hora
		decimalmin=decimalhour*60;//decimalmin=minuts del dia amb decimal
		decimalsec=modf(decimalmin,&minutes);//minutes=minuts del dia enters,decimalsec=porcio de minuts
		seconds=decimalsec*60;//seconds=segons del dia en decimal

		a=modf(seconds,&enterseconds);
		sprintf(cad4,"%6.0f%6.0f%13.7f",hour,minutes,seconds);
		sprintf(cad3," %2.0f %2.0f%11.7f",hour,minutes,seconds);

		//TODO: Include receiver clock offset
		if(temps_primera_obs==1){
			//Escriure Hora Primera Observació
			fseek(fp_rin, fp_rin_end, SEEK_SET);
			fprintf(fp_rin,"%s%s     GPS         TIME OF FIRST OBS\n",cad2,cad4);
			fprintf(fp_rin,"00000CTTC                                                   MARKER NUMBER\n");
			//fprintf(fp_rin,"Edited by ....								                  COMMENT\n");
			fprintf(fp_rin,"                                                            END OF HEADER\n");
			fp_rin_end = ftell(fp_rin);
			temps_primera_obs=0;
		}

		//2-Num sat visibles i identificador
		signed long int nsvs = 0;

		//3-Escriure pseudodistancia
		for(pseudoranges_iter = pseudoranges.begin();
				pseudoranges_iter != pseudoranges.end();
				pseudoranges_iter++)
		{
			//PER FORMAT RINEX2
			nsvs++;
			sprintf(cad,"%2.0f",(double)pseudoranges_iter->first); //satellite PRN ID
			int k=3*index;
			sat_vis[k]='G';
			sat_vis[k+1]=cad[0];
			sat_vis[k+2]=cad[1];
			index++;
		}
		//sat_vis tinc vector de identif de sat visibles
		//Per format RINEX2
		//sprintf(packet,"%s%s  0%3d%s%12.9f",cad1,cad3,nsvs,sat_vis,offset);
		sprintf(packet,"%s%s  0%3d%s",cad1,cad3,nsvs,sat_vis);
		packet[69]=packet[68];
		packet[68]=' ';
		fseek(fp_rin, fp_rin_end, SEEK_SET);
		fprintf(fp_rin,"%s\n",packet);
		fp_rin_end = ftell(fp_rin);

		//3-Escriure pseudodistancia
		for(pseudoranges_iter = pseudoranges.begin();
				pseudoranges_iter != pseudoranges.end();
				pseudoranges_iter++)
		{
			ss=signalstrength(54.00); // TODO: include estimated signal strength
			fseek(fp_rin, fp_rin_end, SEEK_SET);
			fprintf(fp_rin,"%14.3f  %14.3f %d\n",pseudoranges_iter->second,0.0,ss); //TODO: include the carrier phase
			fp_rin_end = ftell(fp_rin);
		}
	}*/

}





int rinex_printer::signalStrength(double snr)
{

    int ss;
    ss= int (std::min(std::max(int (floor(snr/6)) ,1),9));
    return (ss);
}


boost::posix_time::ptime rinex_printer::computeTime(gps_navigation_message nav_msg)
{
    // if we are processing a file -> wait to leap second to resolve the ambiguity else take the week from the local system time
    //: idea resolve the ambiguity with the leap second  http://www.colorado.edu/geography/gcraft/notes/gps/gpseow.htm
    double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(nav_msg.d_TOW));
    boost::posix_time::time_duration t = boost::posix_time::millisec((utc_t+ 604800*(double)(nav_msg.i_GPS_week))*1000);
    boost::posix_time::ptime p_time(boost::gregorian::date(1999,8,22),t);
    return p_time;
}

/*

enum RINEX_enumObservationType
{
    RINEX_OBS_TYPE_PSEUDORANGE     = 'C', //!< 'C' Pseudorange observation
    RINEX_OBS_TYPE_CARRIER_PHASE   = 'L', //!< 'L' Carrier Phase observation
    RINEX_OBS_TYPE_DOPPLER         = 'D', //!< 'L' Doppler observation
    RINEX_OBS_TYPE_SIGNAL_STRENGTH = 'S' //!< 'S' Signal strength observation
} ;





enum RINEX_enumMarkerType {
    GEODETIC,      //!< GEODETIC Earth-fixed, high-precision monumentation
    NON_GEODETIC,  //!< NON_GEODETIC Earth-fixed, low-precision monumentation
    SPACEBORNE,    //!< SPACEBORNE Orbiting space vehicle
    AIRBORNE ,     //!< AIRBORNE Aircraft, balloon, etc.
    WATER_CRAFT,   //!< WATER_CRAFT Mobile water craft
    GROUND_CRAFT,  //!< GROUND_CRAFT Mobile terrestrial vehicle
    FIXED_BUOY,    //!< FIXED_BUOY "Fixed" on water surface
    FLOATING_BUOY, //!< FLOATING_BUOY Floating on water surface
    FLOATING_ICE,  //!< FLOATING_ICE Floating ice sheet, etc.
    GLACIER,       //!< GLACIER "Fixed" on a glacier
    BALLISTIC,     //!< BALLISTIC Rockets, shells, etc
    ANIMAL,        //!< ANIMAL Animal carrying a receiver
    HUMAN          //!< HUMAN Human being
};

 */


