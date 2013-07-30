#include "concurrent_map.h"
#include "armadillo"

class FrontEndCal
{
private:
	ConfigurationInterface *configuration_;
	arma::vec lla2ecef(arma::vec lla);
	arma::vec geodetic2ecef(double phi, double lambda, double h, arma::vec ellipsoid);
public:

	bool read_assistance_from_XML();
	int Get_SUPL_Assist();
	void set_configuration(ConfigurationInterface *configuration);
	bool get_ephemeris();
	double estimate_doppler_from_eph(unsigned int PRN, double TOW, double lat, double lon, double height);
	void GPS_L1_front_end_model_E4000(double f_bb_true_Hz,double f_bb_meas_Hz,double fs_nominal_hz, double *estimated_fs_Hz, double *estimated_f_if_Hz, double *f_osc_err_ppm );
	FrontEndCal();
	~FrontEndCal();
};
