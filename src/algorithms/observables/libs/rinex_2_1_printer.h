/**
 * Copyright notice
 */

/**
 * Author: Javier Arribas, 2011. jarribas(at)cttc.es
 * 		   Luis Esteve, 2011. luis(at)epsilon-formacion.com
 */

#ifndef RINEX_PRINTER_H_
#define	RINEX_PRINTER_H_

#include <string>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <map>
#include <algorithm>

#include "gps_navigation_message.h"

class rinex_printer
{
private:
	int temps_primera_obs;  //per saber si he escrit el temps de la primera observació.
	int correccio_primera_obs;  //per saber si he escrit l correccio de temps de la primera observació.
	int primeravegada;//per evitar problemes primera pseudodistancia
	int PERMIS_RINEX; //PER DEIXAR ESCRIURE PER PRIMERA VEGADA AL RINEX

	FILE *fp_rin;		//!< RINEX OBS FILE Output
	FILE *fp_rin2;		//!< RINEX NAV FILE Output
	unsigned long int fp_rin_end;	//!< Hold place of last RINEX OBS FILE pointer, minus the header
	unsigned long int fp_rin_end2;	//!< Hold place of last RINEX NAV FILE pointer, minus the header

	int signalstrength( double snr);
	void Rinex2ObsHeader();
	void Rinex2NavHeader();

public:

	void set_headers(std::string file_name);
	void LogRinex2Nav(gps_navigation_message nav_msg);
	void LogRinex2Obs(gps_navigation_message nav_msg,double interframe_seconds, std::map<int,float> pseudoranges);
	rinex_printer();
	~rinex_printer();
};

#endif
