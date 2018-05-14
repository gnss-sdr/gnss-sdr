/*!
 * \file gnsspvt.cpp
 * \brief Adapted version of high level gpstk PVT solver to read RINEX files,
 *  compute the position, velocity and time solution, and write Google Earth KML output file.
 *  The input Observables and Navigation files can be RINEX 2.10 or RINEX 3.00.
 *  It is a modified version of the example5.cpp code provided in gpstk source code.
 *
 * \author Javier Arribas, 2012. jarribas(at)cttc.es
 *
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

// Modified Example program Nro 5 for GPSTk
// This program shows how to use some high-level GPSTk classes

   // Basic input/output C++ class
#include <iostream>

   // Classes for handling observations RINEX files (data)
#include "gpstk/Rinex3ObsData.hpp"
#include "gpstk/Rinex3ObsStream.hpp"

   // Class to easily extract data from Rinex3ObsData objects
#include "gpstk/ExtractData.hpp"

   // Classes for handling satellite navigation parameters RINEX files
   // (Broadcast ephemerides)
#include "gpstk/Rinex3NavHeader.hpp"
#include "gpstk/Rinex3NavData.hpp"
#include "gpstk/Rinex3NavStream.hpp"

   // Class to store satellite broadcast navigation data
#include "gpstk/GPSEphemerisStore.hpp"

   // Class to model GPS data for a mobile receiver
#include "gpstk/ModeledPR.hpp"

#include "gpstk/GNSSconstants.hpp"
#include "gpstk/CommonTime.hpp"
#include "gpstk/SatID.hpp"
#include "gpstk/Matrix.hpp"
#include "gpstk/XvtStore.hpp"
#include "gpstk/TropModel.hpp"

   // Class to model the tropospheric delays
#include "gpstk/TropModel.hpp"

   // Classes to model ans store ionospheric delays
#include "gpstk/IonoModel.hpp"
#include "gpstk/IonoModelStore.hpp"

   // Class to solve the equations system using a Weighted Least Mean Square method
#include "gpstk/SolverWMS.hpp"

   // Class to compute the weights to be used for each satellite
#include "gpstk/MOPSWeight.hpp"

   // Basic framework for programs in the GPSTk. The 'process()' method MUST
   // be implemented
#include "gpstk/BasicFramework.hpp"

#include "gpstk/geometry.hpp"                   // DEG_TO_RAD

   // Time-class year-day-second
#include "gpstk/YDSTime.hpp"

#include "kml_printer_gpstk.h"

using namespace std;
using namespace gpstk;


   // A new class is declared that will handle program behaviour
   // This class inherits from BasicFramework
class gpstk_solver : public BasicFramework
{
public:

      // Constructor declaration
   gpstk_solver(char* arg0);
   ~gpstk_solver();

protected:

      // Method that will take care of processing
   virtual void process();

      // Method that hold code to be run BEFORE processing
   virtual void spinUp();

   virtual int Prepare( const CommonTime& Tr,
                           std::vector<SatID>& Satellite,
                           std::vector<double>& Pseudorange,
                           const XvtStore<SatID>& Eph );
   virtual int Prepare2( const CommonTime& Tr,
                           const Vector<SatID>& Satellite,
                           const Vector<double>& Pseudorange,
                           const XvtStore<SatID>& Eph );

private:

      // These field represent options at command line interface (CLI)
   CommandOptionWithArg dataFile;
   CommandOptionWithArg navFile;
   CommandOptionWithArg kmlFile;

   Kml_Printer_gpstk kml_printer;

      // If you want to share objects and variables among methods, you'd
      // better declare them here
   Rinex3ObsStream rObsFile;     // Object to read Rinex observation data files
   Rinex3ObsData rData;          // Object to store Rinex observation data
   Rinex3NavStream rNavFile;     // Object to read Rinex navigation data files
   Rinex3NavData rNavData;       // Object to store Rinex navigation data
   Rinex3NavHeader rNavHeader;   // Object to read the header of Rinex
                                 // navigation data files
   IonoModelStore ionoStore;     // Object to store ionospheric models
   GPSEphemerisStore bceStore;   // Object to store ephemeris
   ModeledPR modelPR;            // Declare a ModeledReferencePR object
   MOPSTropModel mopsTM;         // Declare a MOPSTropModel object
   ExtractData obsC1;            // Declare an ExtractData object
   int indexC1;                  // Index to "C1" observation
   bool useFormerPos;            // Flag indicating if we have an a priori
                                 // position
   Position formerPosition;      // Object to store the former position
   IonoModel ioModel;            // Declare a Ionospheric Model object
   SolverWMS solver;             // Declare an object to apply WMS method
   MOPSWeight mopsWeights;       // Object to compute satellites' weights

};


   // Let's implement constructor details
gpstk_solver::gpstk_solver(char* arg0)
   : BasicFramework(arg0, "\nProgram to print the position solution in ECEF "
                           "and longitude, latitude, height, based in C1 and "
                           "given a RINEX observations file and a RINEX "
                           "broadcast navigation file.\n\n"
                           "The output is: \n"
                           "  Time(sec)  X(m)  Y(m) Z(m)  Lon(deg)  "
                           "  Lat(deg)  Height(m)\n"),
      // Option initialization. "true" means a mandatory option
   dataFile(CommandOption::stdType, 'i', "datainput",
              " [-i|--datainput]      Name of RINEX observations file.", true),
   navFile(CommandOption::stdType, 'n', "navinput",
        " [-n|--navinput]      Name of RINEX broadcast navigation file.", true),
   kmlFile(CommandOption::stdType, 'k', "kmloutput",
             " [-n|--navinput]      Name of KML output file.", true)
{
      // These options may appear just once at CLI
   dataFile.setMaxCount(1);
   navFile.setMaxCount(1);
   kmlFile.setMaxCount(1);
}  // End of constructor details



      /* Method to set an a priori position of receiver using
       * Bancroft's method.
       *
       * @param Tr            Time of observation
       * @param Satellite     std::vector of satellites in view
       * @param Pseudorange   std::vector of pseudoranges measured from
       *                      rover station to satellites
       * @param Eph           Satellites Ephemeris
       *
       * @return
       *  0 if OK
       *  -1 if problems arose
       */
   int gpstk_solver::Prepare( const CommonTime& Tr,
                           std::vector<SatID>& Satellite,
                           std::vector<double>& Pseudorange,
                           const XvtStore<SatID>& Eph )
   {

      Matrix<double> SVP;
      Bancroft Ban;
      Vector<double> vPos;
      PRSolution2 raimObj;

      try
      {
	 cerr << "Tr=" <<Tr<<std::endl;
            for (std::vector<SatID>::iterator it = Satellite.begin() ; it != Satellite.end(); ++it)
            {
              cerr << "SatID=" << *it<<endl;
	      cerr << "EphemerisStore Sat XYZ= " << bceStore.getXvt(*it,rData.time).x<<std::endl;
            }

            for (std::vector<double>::iterator it = Pseudorange.begin() ; it != Pseudorange.end(); ++it)
            {
              cerr << "PseudoRanges=" << *it<<endl;
            }

	 //cerr << "Eph=" << Eph <<std::endl;
	 //cerr << "SVP=" << SVP <<std::endl;
         raimObj.PrepareAutonomousSolution( Tr,
                                            Satellite,
                                            Pseudorange,
                                            Eph,
                                            SVP );

         if( Ban.Compute(SVP, vPos) < 0 )
         {
	    cerr << "Error 1" <<std::endl;
            cerr << "SVP="<<SVP<<std::endl;
	    cerr << "vPos=" << vPos <<std::endl;
            return -1;
         }
      }
      catch(Exception &error)
      {
         cerr << "Excepción 1: "<<error;
         return -1;
      }
      cerr << "OK prepare"<<endl;
      return 1;

   }  // End of method 'ModeledPR::Prepare()'

      /* Method to set an a priori position of receiver using
       * Bancroft's method.
       *
       * @param Tr            Time of observation
       * @param Satellite     Vector of satellites in view
       * @param Pseudorange   Pseudoranges measured from rover station to
       *                      satellites
       * @param Eph           Satellites Ephemeris
       *
       * @return
       *  0 if OK
       *  -1 if problems arose
       */
   int gpstk_solver::Prepare2( const CommonTime& Tr,
                           const Vector<SatID>& Satellite,
                           const Vector<double>& Pseudorange,
                           const XvtStore<SatID>& Eph )
   {

      int i;
      std::vector<SatID> vSat;
      std::vector<double> vPR;

         // Convert from gpstk::Vector to std::vector
      for (i = 0; i < (int)Satellite.size(); i++)
      {
         vSat.push_back(Satellite[i]);
      }

      for (i = 0; i < (int)Pseudorange.size(); i++)
      {
         vPR.push_back(Pseudorange[i]);
      }

      return Prepare(Tr, vSat, vPR, Eph);

   }  // End of method 'ModeledPR::Prepare()'


   // Method that will be executed AFTER initialization but BEFORE processing
void gpstk_solver::spinUp()
{


   //open KML output file
   if (kml_printer.set_headers(kmlFile.getValue()[0].c_str())!=true)
   {
	   cerr << "Problem creating the kml file "<<kmlFile.getValue()[0].c_str();
	   exit (-1);
   }
      // From now on, some parts may look similar to 'example3.cpp' and
      // 'example4.cpp'
      // Activate failbit to enable exceptions
   rObsFile.exceptions(ios::failbit);

      // First, data RINEX reading object
   try
   {
      rObsFile.open(dataFile.getValue()[0].c_str(), std::ios::in);
   }
   catch(...)
   {
      cerr << "Problem opening file " << dataFile.getValue()[0].c_str()
           << endl;
      cerr << "Maybe it doesn't exist or you don't have proper read "
           << "permissions." << endl;

      exit (-1);
   }

      // We need to read the header of the observations file
   Rinex3ObsHeader roh;
   rObsFile >> roh;

      // We need the index pointing to C1-type observations
   try
   {
      indexC1 = roh.getObsIndex( "C1" );
   }
   catch(...)
   {
      cerr << "The observation file doesn't have C1 pseudoranges." << endl;
      exit(1);
   }


      // Activate failbit to enable exceptions
   rNavFile.exceptions(ios::failbit);

      // Read nav file and store unique list of ephemerides
   try
   {
      rNavFile.open(navFile.getValue()[0].c_str(), std::ios::in);
   }
   catch(...)
   {
      cerr << "Problem opening file " << navFile.getValue()[0].c_str() << endl;
      cerr << "Maybe it doesn't exist or you don't have proper read "
           << "permissions." << endl;

      exit (-1);
   }

      // We will need to read ionospheric parameters (Klobuchar model) from
      // the file header
   rNavFile >> rNavHeader;

      // Let's feed the ionospheric model (Klobuchar type) from data in the
      // navigation (ephemeris) file header. First, we must check if there are
      // valid ionospheric correction parameters in the header
   if(rNavHeader.valid & Rinex3NavHeader::validIonoCorrGPS)
   {
         // Extract the Alpha and Beta parameters from the header
      double* ionAlpha = rNavHeader.mapIonoCorr["GPSA"].param;
      double* ionBeta  = rNavHeader.mapIonoCorr["GPSB"].param;

         // Feed the ionospheric model with the parameters
      ioModel.setModel(ionAlpha, ionBeta);
   }
   else
   {
      cerr << "WARNING: Navigation file " << navFile.getValue()[0].c_str()
           << " doesn't have valid ionospheric correction parameters." << endl;
   }

      // WARNING-WARNING-WARNING: In this case, the same model will be used
      // for the full data span
   ionoStore.addIonoModel(CommonTime::BEGINNING_OF_TIME, ioModel);

      // Storing the ephemeris in "bceStore"
   while (rNavFile >> rNavData)
	   {
	   	   bceStore.addEphemeris(rNavData);
	   	   rNavData.dump(cerr);
	   	   cerr<<endl;
	   }

      // Setting the criteria for looking up ephemeris
   bceStore.SearchPast();  // This is the default

      // This is set to true if the former computed positon will be used as
      // a priori position
   useFormerPos = false;   // At first, we don't have an a priori position

      // Prepare for printing later on
   cout << fixed << setprecision(8);

}  // End of gpstk_solver::spinUp()


   // Method that will really process information
void gpstk_solver::process()
{

      // Let's read the observations RINEX, epoch by epoch
   while( rObsFile >> rData )
   {

         // Begin usable data with enough number of satellites
      if( (rData.epochFlag == 0 || rData.epochFlag == 1) &&
          (rData.numSVs > 3) )
      {

            // Number of satellites with valid data in this epoch
         int validSats = 0;
         int prepareResult;
         double rxAltitude;  // Receiver altitude for tropospheric model
         double rxLatitude;  // Receiver latitude for tropospheric model

            // We need to extract C1 data from this epoch. Skip epoch if not
            // enough data (4 SV at least) is available
         if( obsC1.getData(rData, indexC1) < 4 )
         {
               // The former position will not be valid next time
            useFormerPos = false;
            continue;
         }


            // If possible, use former position as a priori
         if( useFormerPos )
         {

            prepareResult = modelPR.Prepare(formerPosition);

               // We need to seed this kind of tropospheric model with
               // receiver altitude
            rxAltitude = formerPosition.getAltitude();
            rxLatitude = formerPosition.getGeodeticLatitude();

         }
         else
         {
               // Use Bancroft method is no a priori position is available
            cerr << "Bancroft method was used at epoch "
                 << static_cast<YDSTime>(rData.time).sod << endl;

            Prepare2( rData.time,obsC1.availableSV,obsC1.obsData,bceStore );
            prepareResult = modelPR.Prepare( rData.time,
                                             obsC1.availableSV,
                                             obsC1.obsData,
                                             bceStore );

               // We need to seed this kind of tropospheric model with
               // receiver altitude
            rxAltitude = modelPR.rxPos.getAltitude();
            rxLatitude = modelPR.rxPos.getGeodeticLatitude();
         }

            // If there were problems with Prepare(), skip this epoch
         if( prepareResult )
         {
               // The former position will not be valid next time
            useFormerPos = false;
            continue;
         }

            // If there were no problems, let's feed the tropospheric model
         mopsTM.setReceiverHeight(rxAltitude);
         mopsTM.setReceiverLatitude(rxLatitude);
         mopsTM.setDayOfYear(static_cast<YDSTime>(rData.time).doy);


            // Now, let's compute the GPS model for our observable (C1)
         validSats = modelPR.Compute( rData.time,
                                      obsC1.availableSV,
                                      obsC1.obsData,
                                      bceStore,
                                      &mopsTM,
                                      &ionoStore );

            // Only get into further computations if there are enough
            // satellites
         if( validSats >= 4 )
         {

               // Now let's solve the navigation equations using the WMS method
            try
            {
                  // First, compute the satellites' weights
               int goodSv = mopsWeights.getWeights( rData.time,
                                                    modelPR.availableSV,
                                                    bceStore,
                                                    modelPR.ionoCorrections,
                                                    modelPR.elevationSV,
                                                    modelPR.azimuthSV,
                                                    modelPR.rxPos );

                  // Some minimum checking is in order
               if ( goodSv != (int)modelPR.prefitResiduals.size() ) continue;

                  // Then, solve the system
               solver.Compute( modelPR.prefitResiduals,
                               modelPR.geoMatrix,
                               mopsWeights.weightsVector );

            }
            catch( InvalidSolver& e )
            {
               cerr << "Couldn't solve equations system at epoch "
                    << static_cast<YDSTime>(rData.time).sod << endl;
               cerr << e << endl;

                  // The former position will not be valid next time
               useFormerPos = false;
               continue;
            }

               // With "solver", we got the difference vector between the
               // a priori position and the computed, 'real' position. Then,
               // let's convert the solution to a Position object
            Position solPos( (modelPR.rxPos.X() + solver.solution[0]),
                             (modelPR.rxPos.Y() + solver.solution[1]),
                             (modelPR.rxPos.Z() + solver.solution[2]) );

               // Print results
            cout << static_cast<YDSTime>(rData.time).sod
                 << "   ";   // Output field #1
            cout << "X="<<solPos.X() << ";   ";                // Output field #2
            cout << "Y="<<solPos.Y() << ";   ";                // Output field #3
            cout << "Z="<<solPos.Z() << ";   ";                // Output field #4
            cout << solPos.longitude() << "   ";        // Output field #5
            cout << solPos.geodeticLatitude() << "   "; // Output field #6
            cout << solPos.height() << "   ";           // Output field #7
            cout << endl;

            cout << "Geocentric Latitude="<< solPos.geocentricLatitude() << "   ";        // Output field #5
            cout << endl;

            // write KML PVT line
            kml_printer.print_position(solPos);

            formerPosition = solPos;

               // Next time, former position will be used as a priori
            useFormerPos = true;

         }  // End of 'if( validSats >= 4 )'
         else
         {
               // The former position will not be valid next time
            useFormerPos = false;
         }

      }  // End of 'if( (rData.epochFlag == 0 || rData.epochFlag == 1) &&...'
      else
      {
            // The former position will not be valid next time
         useFormerPos = false;
      }

   }  // End of 'while( rObsFile >> rData )'

   return;

}  // End of 'gpstk_solver::process()'

gpstk_solver::~gpstk_solver()
{
	kml_printer.close_file();
}
   // Main function
int main(int argc, char* argv[])
{

   try
   {
      gpstk_solver program(argv[0]);
      if (!program.initialize(argc, argv))
         return 0;
      if (!program.run())
         return 1;

      return 0;
   }
   catch(Exception& e)
   {
      cout << "Problem: " << e << endl;
      return 1;
   }
   catch(...)
   {
      cout << "Unknown error." << endl;
      return 1;
   }

   return 0;

}  // End of 'main()'
