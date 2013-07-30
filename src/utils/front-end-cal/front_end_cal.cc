
#include <exception>
#include <boost/filesystem.hpp>
#include <gflags/gflags.h>
#include <glog/log_severity.h>
#include <glog/logging.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/lexical_cast.hpp>

#include "file_configuration.h"

#include "gps_navigation_message.h"
#include "gps_ephemeris.h"
#include "gps_almanac.h"
#include "gps_iono.h"
#include "gps_utc_model.h"
#include "gnss_sdr_supl_client.h"
#include <sys/time.h>
#include <ctime>
#include <memory>
#include <math.h>

#include "front_end_cal.h"


extern	concurrent_map<Gps_Ephemeris> global_gps_ephemeris_map;
extern	concurrent_map<Gps_Iono> global_gps_iono_map;
extern	concurrent_map<Gps_Utc_Model> global_gps_utc_model_map;
extern	concurrent_map<Gps_Almanac> global_gps_almanac_map;
extern	concurrent_map<Gps_Acq_Assist> global_gps_acq_assist_map;

FrontEndCal::FrontEndCal()
{

}
FrontEndCal::~FrontEndCal()
{

}

bool FrontEndCal::read_assistance_from_XML()
{
	gnss_sdr_supl_client supl_client_ephemeris_;
	std::string eph_xml_filename="gps_ephemeris.xml";
	std::cout<< "SUPL: Try read GPS ephemeris from XML file "<<eph_xml_filename<<std::endl;
	if (supl_client_ephemeris_.load_ephemeris_xml(eph_xml_filename)==true)
	{
		std::map<int,Gps_Ephemeris>::iterator gps_eph_iter;
		for(gps_eph_iter = supl_client_ephemeris_.gps_ephemeris_map.begin();
				gps_eph_iter != supl_client_ephemeris_.gps_ephemeris_map.end();
				gps_eph_iter++)
		{
			std::cout<<"SUPL: Read XML Ephemeris for GPS SV "<<gps_eph_iter->first<<std::endl;
			std::cout << "New Ephemeris record inserted with Toe="<<gps_eph_iter->second.d_Toe<<" and GPS Week="<<gps_eph_iter->second.i_GPS_week<<std::endl;
			global_gps_ephemeris_map.write(gps_eph_iter->second.i_satellite_PRN,gps_eph_iter->second);
		}
		return true;
	}else{
		std::cout<< "ERROR: SUPL client error reading XML"<<std::endl;
		return false;
	}
}

int FrontEndCal::Get_SUPL_Assist()
{
	//#########GNSS Asistence #################################
	gnss_sdr_supl_client supl_client_acquisition_;
	gnss_sdr_supl_client supl_client_ephemeris_;
	int supl_mcc; // Current network MCC (Mobile country code), 3 digits.
	int supl_mns; //Current network MNC (Mobile Network code), 2 or 3 digits.
	int supl_lac; // Current network LAC (Location area code),16 bits, 1-65520 are valid values.
	int supl_ci; // Cell Identity (16 bits, 0-65535 are valid values).
	// GNSS Assistance configuration
	int error=0;
	bool enable_gps_supl_assistance=configuration_->property("GNSS-SDR.SUPL_gps_enabled",false);
	if (enable_gps_supl_assistance==true)
		//SUPL SERVER TEST. Not operational yet!
	{
		DLOG(INFO) << "SUPL RRLP GPS assistance enabled!"<<std::endl;
		std::string default_acq_server="supl.nokia.com";
		std::string default_eph_server="supl.google.com";
		supl_client_ephemeris_.server_name=configuration_->property("GNSS-SDR.SUPL_gps_ephemeris_server",default_acq_server);
		supl_client_acquisition_.server_name=configuration_->property("GNSS-SDR.SUPL_gps_acquisition_server",default_eph_server);
		supl_client_ephemeris_.server_port=configuration_->property("GNSS-SDR.SUPL_gps_ephemeris_port",7275);
		supl_client_acquisition_.server_port=configuration_->property("GNSS-SDR.SUPL_gps_acquisition_port",7275);
		supl_mcc=configuration_->property("GNSS-SDR.SUPL_MCC",244);
		supl_mns=configuration_->property("GNSS-SDR.SUPL_MNS",5);

		std::string default_lac="0x59e2";
		std::string default_ci="0x31b0";
		try {
			supl_lac = boost::lexical_cast<int>(configuration_->property("GNSS-SDR.SUPL_LAC",default_lac));
		} catch(boost::bad_lexical_cast &) {
			supl_lac=0x59e2;
		}
		try {
			supl_ci = boost::lexical_cast<int>(configuration_->property("GNSS-SDR.SUPL_CI",default_ci));
		} catch(boost::bad_lexical_cast &) {
			supl_ci=0x31b0;
		}

		bool SUPL_read_gps_assistance_xml=configuration_->property("GNSS-SDR.SUPL_read_gps_assistance_xml",false);
		if (SUPL_read_gps_assistance_xml==true)
		{
			// read assistance from file
			read_assistance_from_XML();
		}else{

			// Request ephemeris from SUPL server
			supl_client_ephemeris_.request=1;
			DLOG(INFO) << "SUPL: Try read GPS ephemeris from SUPL server.."<<std::endl;
			error=supl_client_ephemeris_.get_assistance(supl_mcc,supl_mns,supl_lac,supl_ci);
			if (error==0)
			{
				std::map<int,Gps_Ephemeris>::iterator gps_eph_iter;
				for(gps_eph_iter = supl_client_ephemeris_.gps_ephemeris_map.begin();
						gps_eph_iter != supl_client_ephemeris_.gps_ephemeris_map.end();
						gps_eph_iter++)
				{
					DLOG(INFO) <<"SUPL: Received Ephemeris for GPS SV "<<gps_eph_iter->first<<std::endl;
					DLOG(INFO)  << "New Ephemeris record inserted with Toe="<<gps_eph_iter->second.d_Toe<<" and GPS Week="<<gps_eph_iter->second.i_GPS_week<<std::endl;
					global_gps_ephemeris_map.write(gps_eph_iter->second.i_satellite_PRN,gps_eph_iter->second);
				}
				//Save ephemeris to XML file
				std::string eph_xml_filename="gps_ephemeris.xml";
				if (supl_client_ephemeris_.save_ephemeris_xml(eph_xml_filename)==true)
				{
					DLOG(INFO) <<"SUPL: XML Ephemeris file created"<<std::endl;
				}
			}else{
				DLOG(INFO) << "ERROR: SUPL client for Ephemeris returned "<<error<<std::endl;
				DLOG(INFO) << "Please check internet connection and SUPL server configuration"<<error<<std::endl;
			}

			// Request almanac , IONO and UTC Model
			supl_client_ephemeris_.request=0;
			DLOG(INFO) << "SUPL: Try read Almanac, Iono, Utc Model, Ref Time and Ref Location from SUPL server.."<<std::endl;
			error=supl_client_ephemeris_.get_assistance(supl_mcc,supl_mns,supl_lac,supl_ci);
			if (error==0)
			{
				std::map<int,Gps_Almanac>::iterator gps_alm_iter;
				for(gps_alm_iter = supl_client_ephemeris_.gps_almanac_map.begin();
						gps_alm_iter != supl_client_ephemeris_.gps_almanac_map.end();
						gps_alm_iter++)
				{
					DLOG(INFO) <<"SUPL: Received Almanac for GPS SV "<<gps_alm_iter->first<<std::endl;
					global_gps_almanac_map.write(gps_alm_iter->first,gps_alm_iter->second);
				}
				if (supl_client_ephemeris_.gps_iono.valid==true)
				{
					DLOG(INFO) <<"SUPL: Received GPS Iono"<<std::endl;
					global_gps_iono_map.write(0,supl_client_ephemeris_.gps_iono);
				}
				if (supl_client_ephemeris_.gps_utc.valid==true)
				{
					DLOG(INFO) <<"SUPL: Received GPS UTC Model"<<std::endl;
					global_gps_utc_model_map.write(0,supl_client_ephemeris_.gps_utc);
				}
			}else{
				DLOG(INFO) << "ERROR: SUPL client for Almanac returned "<<error<<std::endl;
				DLOG(INFO) << "Please check internet connection and SUPL server configuration"<<error<<std::endl;
			}

			// Request acquisition assistance
			supl_client_acquisition_.request=2;
			DLOG(INFO) << "SUPL: Try read Acquisition assistance from SUPL server.."<<std::endl;

			error=supl_client_acquisition_.get_assistance(supl_mcc,supl_mns,supl_lac,supl_ci);
			if (error==0)
			{
				std::map<int,Gps_Acq_Assist>::iterator gps_acq_iter;
				for(gps_acq_iter = supl_client_acquisition_.gps_acq_map.begin();
						gps_acq_iter != supl_client_acquisition_.gps_acq_map.end();
						gps_acq_iter++)
				{
					DLOG(INFO) <<"SUPL: Received Acquisition assistance for GPS SV "<<gps_acq_iter->first<<std::endl;
					DLOG(INFO)  << "New acq assist record inserted"<<std::endl;
					global_gps_acq_assist_map.write(gps_acq_iter->second.i_satellite_PRN,gps_acq_iter->second);
				}
			}else{
				DLOG(INFO) << "ERROR: SUPL client for Acquisition assistance returned "<<error<<std::endl;
				DLOG(INFO) << "Please check internet connection and SUPL server configuration"<<error<<std::endl;
			}
		}
	}
	return error;
}


void FrontEndCal::set_configuration(ConfigurationInterface *configuration)
{
    configuration_= configuration;
}

bool FrontEndCal::get_ephemeris()
{
	bool read_ephemeris_from_xml=configuration_->property("GNSS-SDR.read_eph_from_xml",false);

	if (read_ephemeris_from_xml==true)
	{
		std::cout<< "Trying to read ephemeris from XML file..."<<std::endl;
		if (read_assistance_from_XML()==false)
		{
			std::cout<< "ERROR: Could not read Ephemeris file: Trying to get ephemeris from SUPL server.."<<std::endl;
			if (Get_SUPL_Assist()==1)
			{
				return true;
			}else{
				return false;
			}
		}else{
			return true;
		}
	}else{
		std::cout<< "Trying to read ephemeris from SUPL server..."<<std::endl;
		if (Get_SUPL_Assist()==0)
		{
			return true;
		}else{
			return false;
		}
	}
}

arma::vec FrontEndCal::lla2ecef(arma::vec lla)
{
//	%  LLA2ECEF Convert geodetic coordinates to Earth-centered Earth-fixed
//	%  (ECEF)  coordinates.
//	%   P = LLA2ECEF( LLA ) converts an M-by-3 array of geodetic coordinates
//	%   (latitude, longitude and altitude), LLA, to an M-by-3 array of ECEF
//	%   coordinates, P.  LLA is in [degrees degrees meters].  P is in meters.
//	%   The default ellipsoid planet is WGS84.


	// WGS84 flattening
	double f;
	f  = 1/298.257223563;

	// WGS84 equatorial radius
	double R;
	R =  6378137;

	double phi,lambda;
	arma::vec ellipsoid="0.0 0.0";
	phi=(lla(0)/360.0)*GPS_TWO_PI;
	lambda=(lla(1)/360.0)*GPS_TWO_PI;

	ellipsoid(0)=R;
	ellipsoid(1)=sqrt(1-(1-f)*(1-f));

	arma::vec ecef="0.0 0.0 0.0 0.0";
	ecef = geodetic2ecef(phi, lambda,lla(3),ellipsoid);

	return ecef;
}

arma::vec FrontEndCal::geodetic2ecef(double phi, double lambda, double h, arma::vec ellipsoid)
{
//%GEODETIC2ECEF Convert geodetic to geocentric (ECEF) coordinates
//%
//%   [X, Y, Z] = GEODETIC2ECEF(PHI, LAMBDA, H, ELLIPSOID) converts geodetic
//%   point locations specified by the coordinate arrays PHI (geodetic
//%   latitude in radians), LAMBDA (longitude in radians), and H (ellipsoidal
//%   height) to geocentric Cartesian coordinates X, Y, and Z.  The geodetic
//%   coordinates refer to the reference ellipsoid specified by ELLIPSOID (a
//%   row vector with the form [semimajor axis, eccentricity]).  H must use
//%   the same units as the semimajor axis;  X, Y, and Z will be expressed in
//%   these units also.
//%
//%   The geocentric Cartesian coordinate system is fixed with respect to the
//%   Earth, with its origin at the center of the ellipsoid and its X-, Y-,
//%   and Z-axes intersecting the surface at the following points:
//%
//%                PHI  LAMBDA
//%      X-axis:    0     0      (Equator at the Prime Meridian)
//%      Y-axis:    0   pi/2     (Equator at 90-degrees East)
//%      Z-axis:  pi/2    0      (North Pole)
//%
//%   A common synonym is Earth-Centered, Earth-Fixed coordinates, or ECEF.
//%
//%   See also ECEF2GEODETIC, ECEF2LV, GEODETIC2GEOCENTRICLAT, LV2ECEF.
//% Copyright 2004-2009 The MathWorks, Inc.
//% $Revision: 1.1.6.4 $  $Date: 2009/04/15 23:34:46 $

//% Reference
//% ---------
//% Paul R. Wolf and Bon A. Dewitt, "Elements of Photogrammetry with
//% Applications in GIS," 3rd Ed., McGraw-Hill, 2000 (Appendix F-3).

	double a;
	a  = ellipsoid(0);
	double e2;
	e2 = ellipsoid(1)*ellipsoid(1);

	double sinphi,cosphi;

	sinphi = sin(phi);
	cosphi = cos(phi);
	double N;
	N  = a / sqrt(1 - e2 * sinphi*sinphi);

	arma::vec ecef="0.0 0.0 0.0 0.0";

	ecef(0) = (N + h) * cosphi * cos(lambda);
	ecef(1) = (N + h) * cosphi * sin(lambda);
	ecef(2)= (N*(1 - e2) + h) * sinphi;

	return ecef;
}

double FrontEndCal::estimate_doppler_from_eph(unsigned int PRN, double TOW, double lat, double lon, double height)
{

	int num_secs=10;
	double step_secs=0.5;


	//Observer position ECEF
	arma::vec obs_ecef= "0.0 0.0 0.0 0.0";
	arma::vec lla = "0.0 0.0 0.0 0.0";
	lla(0)=lat;
	lla(1)=lon;
	lla(2)=height;
	obs_ecef=lla2ecef(lla);


	//Satellite positions ECEF
	std::map<int,Gps_Ephemeris> eph_map;
	eph_map=global_gps_ephemeris_map.get_map_copy();

	std::map<int,Gps_Ephemeris>::iterator eph_it;
	eph_it=eph_map.find(PRN);

	if (eph_it!=eph_map.end())
	{

		arma::vec SV_pos_ecef = "0.0 0.0 0.0 0.0";
		double obs_time_start,obs_time_stop;
		obs_time_start=TOW-num_secs/2;
		obs_time_stop=TOW+num_secs/2;
		int n_points=round((obs_time_stop-obs_time_start)/step_secs);
		arma::vec ranges=arma::zeros(n_points,1);
		double obs_time=obs_time_start;
		for (int i=0;i<n_points;i++)
		{
			eph_it->second.satellitePosition(obs_time);
			SV_pos_ecef(0)=eph_it->second.d_satpos_X;
			SV_pos_ecef(1)=eph_it->second.d_satpos_Y;
			SV_pos_ecef(2)=eph_it->second.d_satpos_Z;
			// SV distances to observer (true range)
			ranges(i)=arma::norm(SV_pos_ecef-obs_ecef,2);

			obs_time+=step_secs;
		}
		//Observer to satellite radial velocity
		//Numeric derivative: Positive slope means that the distance from obs to
		//satellite is increasing
	    arma::vec obs_to_sat_velocity;
		obs_to_sat_velocity=(ranges.subvec(1,(n_points-1))-ranges.subvec(0,(n_points-2)))/step_secs;
		//Doppler equations are formulated accounting for positive velocities if the
		//tx and rx are aproaching to each other. So, the satellite velocity must
		//be redefined as:

		obs_to_sat_velocity=-obs_to_sat_velocity;

		//Doppler estimation
		arma::vec Doppler_Hz;
		Doppler_Hz=(obs_to_sat_velocity/GPS_C_m_s)*GPS_L1_FREQ_HZ;
		double mean_Doppler_Hz;
		mean_Doppler_Hz=arma::mean(Doppler_Hz);
		return mean_Doppler_Hz;
	return 0;
	}else{
		throw(1);
	}

}

void FrontEndCal::GPS_L1_front_end_model_E4000(double f_bb_true_Hz,double f_bb_meas_Hz,double fs_nominal_hz, double *estimated_fs_Hz, double *estimated_f_if_Hz, double *f_osc_err_ppm )
{
//	%   Inputs:
//	%       f_bb_true_Hz  - Ideal output frequency in baseband [Hz]
//	%       f_in_bb_meas_Hz       - measured output frequency in baseband [Hz]
//	%
//	%   Outputs:
//	%       estimated_fs_Hz  - Sampling frequency estimation based on the
//	%       measurements and the front-end model
//	%       estimated_f_if_bb_Hz    - Equivalent bb if frequency estimation based on the
//	%       measurements and the front-end model

	// Front-end TUNNER Elonics E4000 + RTL2832 sampler
	//For GPS L1 1575.42 MHz

	double f_osc_n=28.8e6;
	//PLL registers settings (according to E4000 datasheet)

	double N=109;
	double Y=65536;
	double X=26487;
	double R=2;
	// Obtained RF center frequency
	double f_rf_pll;
	f_rf_pll=(f_osc_n*(N+X/Y))/R;
	// RF frequency error caused by fractional PLL roundings
	double f_bb_err_pll;
	f_bb_err_pll=GPS_L1_FREQ_HZ-f_rf_pll;

	// Measured F_rf error
	double f_rf_err;
	f_rf_err=(f_bb_meas_Hz-f_bb_true_Hz)-f_bb_err_pll;

	double f_osc_err_hz;
	f_osc_err_hz=(f_rf_err*R)/(N+X/Y);

	// OJO,segun los datos gnss, la IF positiva hace disminuir la fs!!
	f_osc_err_hz=-f_osc_err_hz;

	*f_osc_err_ppm=f_osc_err_hz/(f_osc_n/1e6);
	double frac;
	frac=fs_nominal_hz/f_osc_n;

	*estimated_fs_Hz=frac*(f_osc_n+f_osc_err_hz);
	*estimated_f_if_Hz=f_rf_err;

}


