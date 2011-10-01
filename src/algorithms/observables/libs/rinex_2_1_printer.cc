#include "rinex_2_1_printer.h"


void rinex_printer::set_headers(std::string file_name)
{

        correccio_primera_obs=1;

        fp_rin = fopen((file_name+".09o").c_str(),"wt");
        fp_rin_end = ftell(fp_rin);

        fp_rin2 = fopen((file_name+".09n").c_str(),"wt");
        fp_rin_end2 = ftell(fp_rin2);

        // write RINEX headers
        Rinex2ObsHeader();
        Rinex2NavHeader();

}

rinex_printer::rinex_printer ()
{
}

rinex_printer::~rinex_printer ()
{
        fclose(fp_rin);
        fclose(fp_rin2);
}

void rinex_printer::Rinex2NavHeader()
{

	if(fp_rin2 != NULL)
	{
		//calculate UTC_TIME
		time_t tiempo;
		char cad[80];
		struct tm *tmPtr;
		tiempo = time(NULL);
		tmPtr = gmtime(&tiempo);
		strftime( cad, 20, "%d-%b-%y %H:%M", tmPtr );

		fseek(fp_rin2, fp_rin_end2, SEEK_SET);
		//fprintf(fp_rin2,"----|---1|0---|---2|0---|---3|0---|---4|0---|---5|0---|---6|0---|---7|0---|---8|\n\n");
		fprintf(fp_rin2,"     2.10           NAVIGATION DATA                         RINEX VERSION / TYPE\n");
		fprintf(fp_rin2,"GNSS-SDR-mercurio         CTTC           %s    PGM / RUN BY / DATE\n",cad);
		//fprintf(fp_rin2,"CTTC                                                         MARKER NAME\n");
		//fprintf(fp_rin2,"0000                                                         MARKERNUMBER\n");
		fp_rin_end2 = ftell(fp_rin2);
		correccio_primera_obs=1;
	}
}

void rinex_printer::LogRinex2Nav(gps_navigation_message nav_msg){


	if(fp_rin2 != NULL)
	{
		//double decimalday,daydecimalhour,decimalhour,decimalmin,decimalsec;
		//double day,hour,minutes,seconds,enterseconds,a;
		double gpstime;
		struct tm *tmPtr;
		time_t temps;
		//1-Calcul data i hora gps
		//Calculo el any,mes i dia a partir de l'hora UTC
		//calculate UTC_TIME
		char cad1[80];
		char cad2[80];
		//calculate date of gps time:
		double setmanes=nav_msg.d_GPS_week+1024;
		//Days & weeks between 00h 1 Jan 1970 and 00h 6 Jan 1980
		//520 weeks and 12 days.
		gpstime=nav_msg.d_master_clock;
		temps=(520+setmanes)*7*24*3600+gpstime+17*24*3600;

		tmPtr = gmtime(&temps);
		strftime( cad1, 20, "%y %m %d", tmPtr );
		//std::cout<<"gps_time="<<cad1<<std::endl;

		sprintf(cad2,"%2.0f %2.0f %3.1f",(double)tmPtr->tm_hour,(double)tmPtr->tm_min,(double)tmPtr->tm_sec);

		if(correccio_primera_obs==1){
			//Escriure CORRECCIO DE TEMPS GPS a0 i a1;
			fseek(fp_rin2, fp_rin_end2, SEEK_SET);
			char correction[256],correction2[256];
			double A0,A1;
			/* 8.3819D-09 -7.4506D-09 -5.9605D-08  5.9605D-08           ION ALPHA
	   8.8                             064D+04 -3.2768D+04 -1.9661D+05  1.9661D+05           ION BETA
	    5.                             587935447693D-09 8.881784197001D-15   233472     1518 DELTA-UTC: A0,A1,T,W*/
			A0=587.935447693E-09;
			A1=8.881784197001E-15;
			sprintf(correction,"%19.12E",A0);
			sprintf(correction2,"%19.12E",A1);
			double alpha0,alpha1,alpha2,alpha3,beta0,beta1,beta2,beta3;
			alpha0=8.3819E-09;
			alpha1=-7.4506E-09;
			alpha2=-5.9605E-08;
			alpha3=5.9605E-08;
			beta0=064E+04;
			beta1=-3.2768E+04;
			beta2=-1.9661E+05;
			beta3= 1.9661E+05;
			//Format(&correction[0],1);
			//Format(&correction2[0],1);
			//fp_rin_correction_time = ftell(fp_rin2);
			// fprintf(fp_rin2,"  %12.4E%12.4E%12.4E%12.4E          ION ALPHA\n",alpha0,alpha1,alpha2,alpha3);
			// fprintf(fp_rin2,"  %12.4E%12.4E%12.4E%12.4E          ION BETA\n",beta0,beta1,beta2,beta3);
			fprintf(fp_rin2,"   %s%s%9.0f%9d DELTA-UTC: A0,A1,T,W\n",correction,correction2,gpstime,(int)nav_msg.d_GPS_week+1024);
			fprintf(fp_rin2,"    15                                                      LEAP SECONDS\n");

			fprintf(fp_rin2,"                                                            END OF HEADER\n");
			fp_rin_end2 = ftell(fp_rin2);
			correccio_primera_obs=0;

		}

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
	}

}

void rinex_printer::Rinex2ObsHeader()
{
	//Clock_S				*pClock		= &tNav.master_clock;			/* Clock solution */
	if(fp_rin != NULL)
	{
		//calculate UTC_TIME
		time_t tiempo;
		char cad[80];
		struct tm *tmPtr;
		tiempo = time(NULL);
		tmPtr = gmtime(&tiempo);
		strftime( cad, 24, "%d/%m/%Y %H:%M:%S", tmPtr );

		fseek(fp_rin, fp_rin_end, SEEK_SET);
		//fprintf(fp_rin,"----|---1|0---|---2|0---|---3|0---|---4|0---|---5|0---|---6|0---|---7|0---|---8|\n\n");
		fprintf(fp_rin,"     2.10           Observation         G (GPS)             RINEX VERSION / TYPE\n");
		fprintf(fp_rin,"GNSS-SDR-mercurio        CTTC           %s PGM / RUN BY / DATE\n",cad);
		fprintf(fp_rin,"CTTC                                                        MARKER NAME\n");
		//fprintf(fp_rin,"0000                                                        MARKERNUMBER\n");

		fprintf(fp_rin,"JAVIER ARRIBAS           CTTC                               OBSERVER / AGENCY\n");
		fprintf(fp_rin,"GNSS-SDR-mercurio         SDR            1.0                REC # / TYPE / VERS\n");
		fprintf(fp_rin,"0000000000          CTTC00000.00                            ANT # / TYPE\n");
		fprintf(fp_rin,"  4797642.2790   166436.1500  4185504.6370                  APPROX POSITION XYZ\n");
		fprintf(fp_rin,"        0.0000        0.0000        0.0000                  ANTENNA: DELTA H/E/N\n");
		fprintf(fp_rin,"     1     0                                                WAVELENGTH FACT L1/2\n");
		fprintf(fp_rin,"     2    C1    L1                                          # / TYPES OF OBSERV\n");

		fprintf(fp_rin,"     0.1000                                                 INTERVAL\n");
		//INTRODUIR HORA PRIMERA OBSERVACIO
		//ho escriure un cop hagi adquirit dades
		//fprintf(fp_rin,"  2001     3    24    13    10   36.0000000                TIME OF FIRST OBS\n");
		//fprintf(fp_rin,"                                                           END OF HEADER\n");
		fp_rin_end = ftell(fp_rin);
		temps_primera_obs=1;
	}

}

void rinex_printer::LogRinex2Obs(gps_navigation_message nav_msg,double interframe_seconds, std::map<int,float> pseudoranges)
{
	int ss;
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
		gpstime=nav_msg.d_master_clock;
		//calculate date of gps time:
		//Days & weeks between 00h 1 Jan 1970 and 00h 6 Jan 1980
		//520 weeks and 12 days.

		temps=(520+setmanes)*7*24*3600+gpstime+17*24*3600;
		tmPtr = gmtime(&temps);
		strftime( cad1, 20, " %y %m %d", tmPtr );
		strftime( cad2, 20, "  %Y    %m    %d", tmPtr );
		decimalday=((gpstime+interframe_seconds)/(24*3600));//Dies dins de la semana
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
	}

}

int rinex_printer::signalstrength( double snr)
{
	int ss;
	if(snr<=12.00) ss=1;
	else if(snr>12.00 && snr<=18.00) ss=2;
	else if(snr>18.00 && snr<=24.00) ss=3;
	else if(snr>24.00 && snr<=30.00) ss=4;
	else if(snr>30.00 && snr<=36.00) ss=5;
	else if(snr>36.00 && snr<=42.00) ss=6;
	else if(snr>42.00 && snr<=48.00) ss=7;
	else if(snr>48.00 && snr<=54) ss=8;
	else if(snr>=54.00)   ss=9;
	return (ss);
}
