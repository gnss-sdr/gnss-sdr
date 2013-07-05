/*!
 * \file main.cc
 * \brief Main file of the Front-end calibration program.
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2013  (see AUTHORS file for a list of contributors)
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
#ifndef FRONT_END_CAL_VERSION
#define FRONT_END_CAL_VERSION "0.0.1"
#endif

#include <exception>
#include <boost/filesystem.hpp>
#include <gflags/gflags.h>
#include <glog/log_severity.h>
#include <glog/logging.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/skiphead.h>
#include <gnuradio/blocks/head.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/file_sink.h>

#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <vector>
#include "concurrent_map.h"
#include "file_configuration.h"
#include "gps_l1_ca_pcps_acquisition_fine_doppler.h"

#include "gnss_signal.h"
#include "gnss_synchro.h"
#include "gnss_block_factory.h"

#include "gps_navigation_message.h"
#include "gps_ephemeris.h"
#include "gps_almanac.h"
#include "gps_iono.h"
#include "gps_utc_model.h"
#include "gnss_sdr_supl_client.h"
#include <sys/time.h>
#include <ctime>
#include <memory>

#include "front_end_cal.h"

using google::LogMessage;

DECLARE_string(log_dir);

DEFINE_string(config_file, "../conf/gnss-sdr.conf",
		"Path to the file containing the configuration parameters");

concurrent_queue<Gps_Ephemeris> global_gps_ephemeris_queue;
concurrent_queue<Gps_Iono> global_gps_iono_queue;
concurrent_queue<Gps_Utc_Model> global_gps_utc_model_queue;
concurrent_queue<Gps_Almanac> global_gps_almanac_queue;
concurrent_queue<Gps_Acq_Assist> global_gps_acq_assist_queue;

concurrent_map<Gps_Ephemeris> global_gps_ephemeris_map;
concurrent_map<Gps_Iono> global_gps_iono_map;
concurrent_map<Gps_Utc_Model> global_gps_utc_model_map;
concurrent_map<Gps_Almanac> global_gps_almanac_map;
concurrent_map<Gps_Acq_Assist> global_gps_acq_assist_map;


bool stop;
concurrent_queue<int> channel_internal_queue;
GpsL1CaPcpsAcquisitionFineDoppler *acquisition;
Gnss_Synchro *gnss_synchro;
std::vector<Gnss_Synchro> gnss_sync_vector;

void wait_message()
{
    while (!stop)
        {
    		int message;
    		channel_internal_queue.wait_and_pop(message);
    		//std::cout<<"Acq mesage rx="<<message<<std::endl;
    		switch (message)
    		{
    		case 1: // Positive acq
    		    gnss_sync_vector.push_back(*gnss_synchro);
    			acquisition->reset();
    			break;
    		case 2: // negative acq
    			acquisition->reset();
    			break;
    		case 3:
        		stop=true;
        		break;
    		default:
    			break;
    		}
        }
}

bool front_end_capture(ConfigurationInterface *configuration)
{
    gr::top_block_sptr top_block;
    GNSSBlockFactory block_factory;
    boost::shared_ptr<gr::msg_queue> queue;

    queue =  gr::msg_queue::make(0);
    top_block = gr::make_top_block("Acquisition test");


    GNSSBlockInterface *source;
    source=block_factory.GetSignalSource(configuration, queue);

    GNSSBlockInterface *conditioner;
    conditioner=block_factory.GetSignalConditioner(configuration,queue);

    gr::block_sptr sink;
    sink = gr::blocks::file_sink::make(sizeof(gr_complex), "tmp_capture.dat");

    //--- Find number of samples per spreading code ---
    long fs_in_ = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);
    int samples_per_code = round(fs_in_
            / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS));
    int nsamples=samples_per_code*50;

    int skip_samples=fs_in_; // skip 5 seconds

    gr::block_sptr head = gr::blocks::head::make(sizeof(gr_complex), nsamples);

    gr::block_sptr skiphead = gr::blocks::skiphead::make(sizeof(gr_complex), skip_samples);

    try{

    	source->connect(top_block);
    	conditioner->connect(top_block);
        top_block->connect(source->get_right_block(), 0, conditioner->get_left_block(), 0);
        top_block->connect(conditioner->get_right_block(), 0, skiphead, 0);
        top_block->connect(skiphead, 0,head, 0);
        top_block->connect(head, 0, sink, 0);
        top_block->run();
    }catch(std::exception& e)
    {
    	std::cout<<"Failure connecting the GNU Radio blocks "<<e.what()<<std::endl;
    	return false;
    }

    delete conditioner;
    delete source;

    return true;

}

int main(int argc, char** argv)
{
    const std::string intro_help(
            std::string("\n RTL-SDR E4000 RF fornt-end center frequency and sampling rate calibration tool that uses GPS signals\n")
    +
    "Copyright (C) 2010-2013 (see AUTHORS file for a list of contributors)\n"
    +
    "This program comes with ABSOLUTELY NO WARRANTY;\n"
    +
    "See COPYING file to see a copy of the General Public License\n \n");


    google::SetUsageMessage(intro_help);
    google::SetVersionString(FRONT_END_CAL_VERSION);
    google::ParseCommandLineFlags(&argc, &argv, true);

    std::cout << "Initializing... Please wait." << std::endl;

    google::InitGoogleLogging(argv[0]);
    if (FLAGS_log_dir.empty())
        {
             std::cout << "Logging will be done at "

                 << "/tmp"
                 << std::endl
                 << "Use gnss-sdr --log_dir=/path/to/log to change that."
                 << std::endl;
        }
    else
        {
            const boost::filesystem::path p (FLAGS_log_dir);
            if (!boost::filesystem::exists(p))
                {
                    std::cout << "The path "
                        << FLAGS_log_dir
                        << " does not exist, attempting to create it"
                        << std::endl;
                    boost::filesystem::create_directory(p);
                }
            std::cout << "Logging with be done at "
                << FLAGS_log_dir << std::endl;
        }


    // 0. Instantiate the FrontEnd Calibration class
    FrontEndCal front_end_cal;

    // 1. Load configuration parameters from config file

    ConfigurationInterface *configuration;
    configuration= new FileConfiguration(FLAGS_config_file);
    front_end_cal.set_configuration(configuration);

    // Capture file

    if (front_end_capture(configuration))
    {
    	std::cout<<"Front-end RAW samples captured"<<std::endl;
    }else{
    	std::cout<<"Failure capturing front-end samples"<<std::endl;
    }

    // 3. Setup GNU Radio flowgraph (RTL-SDR -> Acquisition_10m)

    gr::top_block_sptr top_block;
    boost::shared_ptr<gr::msg_queue> queue;
    queue = gr::msg_queue::make(0);
    top_block = gr::make_top_block("Acquisition test");

    // Satellite signal definition
    gnss_synchro=new Gnss_Synchro();
    gnss_synchro->Channel_ID=0;
    gnss_synchro->System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro->Signal,2,0);
    gnss_synchro->PRN=1;

    long fs_in_ = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);

    GNSSBlockFactory block_factory;
    acquisition = new GpsL1CaPcpsAcquisitionFineDoppler(configuration, "Acquisition", 1, 1, queue);

    acquisition->set_channel(1);
    acquisition->set_gnss_synchro(gnss_synchro);
    acquisition->set_channel_queue(&channel_internal_queue);
    acquisition->set_threshold(configuration->property("Acquisition.threshold", 0.0));
    acquisition->set_doppler_max(configuration->property("Acquisition.doppler_max", 10000));
    acquisition->set_doppler_step(configuration->property("Acquisition.doppler_step", 250));


    gr::block_sptr source;
    source = gr::blocks::file_source::make(sizeof(gr_complex), "tmp_capture.dat");

    //gr_basic_block_sptr head = gr_make_head(sizeof(gr_complex), nsamples);
    //gr_head_sptr head_sptr = boost::dynamic_pointer_cast<gr_head>(head);
    //head_sptr->set_length(nsamples);
    //head_sptr->reset();

    try{

    	acquisition->connect(top_block);
        top_block->connect(source, 0, acquisition->get_left_block(), 0);
    }catch(std::exception& e)
    {
    	std::cout<<"Failure connecting the GNU Radio blocks "<<std::endl;
    }

    // 4. Run the flowgraph

    std::map<int,double> doppler_measurements_map;
    std::map<int,double> cn0_measurements_map;

    boost::thread ch_thread;
    // record startup time
    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    bool start_msg=true;

    for (unsigned int PRN=1;PRN<33;PRN++)
    {
    	gnss_synchro->PRN=PRN;
        acquisition->set_gnss_synchro(gnss_synchro);
        acquisition->init();
        acquisition->reset();
        stop=false;
        ch_thread = boost::thread(wait_message);
        top_block->run();
        if (start_msg==true)
        {
        	std::cout<<"Searchig for GPS Satellites..."<<std::endl;
        	std::cout<<"[";
        	start_msg=false;
        }
        if (gnss_sync_vector.size()>0)
        {
        	std::cout<<" "<<PRN<<" ";
        	double doppler_measurement_hz=0;
            for (std::vector<Gnss_Synchro>::iterator it = gnss_sync_vector.begin() ; it != gnss_sync_vector.end(); ++it)
              {
            	doppler_measurement_hz+=(*it).Acq_doppler_hz;
            	//std::cout << "Doppler (SV=" << (*it).PRN<<")="<<(*it).Acq_doppler_hz<<"[Hz]"<<std::endl;
              }
            doppler_measurement_hz=doppler_measurement_hz/gnss_sync_vector.size();
            doppler_measurements_map.insert(std::pair<int,double>(PRN,doppler_measurement_hz));
        }else{
        	std::cout<<" . ";
        }
        channel_internal_queue.push(3);
        ch_thread.join();
        gnss_sync_vector.clear();
        boost::dynamic_pointer_cast<gr::blocks::file_source>(source)->seek(0,0);
        std::cout.flush();
    }
    std::cout<<"]"<<std::endl;

    // report the elapsed time
    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    std::cout << "Total signal acquisition run time "
        << ((double)(end - begin))/1000000.0
        << " [seconds]" << std::endl;

    // 5. Get visible GPS satellites (positive acquisitions with Doppler measurements)

    // 2. Get SUPL information from server: Ephemeris record, assistance info and TOW
    front_end_cal.get_ephemeris();

    // 6. Compute Doppler estimations

    //find TOW from SUPL assistance

    double current_TOW=0;
        if (global_gps_ephemeris_map.size()>0)
        {
        	std::map<int,Gps_Ephemeris> Eph_map;
        	Eph_map=global_gps_ephemeris_map.get_map_copy();
        	current_TOW=Eph_map.begin()->second.d_TOW;
        	std::cout<<"Current TOW obtained from SUPL assistance = "<<current_TOW<<std::endl;
        }else{
        	std::cout<<"Unable to get Ephemeris SUPL assistance. TOW is unknown!"<<std::endl;
        }
    //Get user position from config file (or from SUPL using GSM Cell ID)
    double lat_deg = configuration->property("GNSS-SDR.init_latitude_deg", 41.0);
    double lon_deg = configuration->property("GNSS-SDR.init_longitude_deg", 2.0);
    double altitude_m = configuration->property("GNSS-SDR.init_altitude_m", 100);

    std::map<int,double> f_if_estimation_Hz_map;
    std::map<int,double> f_fs_estimation_Hz_map;
    std::map<int,double> f_ppm_estimation_Hz_map;

    for (std::map<int,double>::iterator it = doppler_measurements_map.begin() ; it != doppler_measurements_map.end(); ++it)
      {
    	std::cout << "Doppler measured for (SV=" << it->first<<")="<<it->second<<" [Hz]"<<std::endl;
    	try{
    		double doppler_estimated_hz;
    		doppler_estimated_hz=front_end_cal.estimate_doppler_from_eph(it->first,current_TOW,lat_deg,lon_deg,altitude_m);
    		std::cout << "Doppler estimated for (SV=" << it->first<<")="<<doppler_estimated_hz<<" [Hz]"<<std::endl;
    		// 7. Compute front-end IF and sampling frequency estimation
    		// Compare with the measurements and compute clock drift using FE model
    		double estimated_fs_Hz, estimated_f_if_Hz,f_osc_err_ppm;
    		front_end_cal.GPS_L1_front_end_model_E4000(doppler_estimated_hz,it->second,fs_in_, &estimated_fs_Hz, &estimated_f_if_Hz, &f_osc_err_ppm );

    		f_if_estimation_Hz_map.insert(std::pair<int,double>(it->first,estimated_f_if_Hz));
    		f_fs_estimation_Hz_map.insert(std::pair<int,double>(it->first,estimated_fs_Hz));
    		f_ppm_estimation_Hz_map.insert(std::pair<int,double>(it->first,f_osc_err_ppm));

    	}catch(int ex)
    	{
    		std::cout<<"Eph not found for SV "<<it->first<<std::endl;
    	}
      }

    // FINAL FE estimations
    double mean_f_if_Hz=0;
    double mean_fs_Hz=0;
    double mean_osc_err_ppm=0;
    int n_elements=f_if_estimation_Hz_map.size();

    for (std::map<int,double>::iterator it = f_if_estimation_Hz_map.begin() ; it != f_if_estimation_Hz_map.end(); ++it)
    {
    	mean_f_if_Hz+=(*it).second;
    	mean_fs_Hz+=f_fs_estimation_Hz_map.find((*it).first)->second;
    	mean_osc_err_ppm+=f_ppm_estimation_Hz_map.find((*it).first)->second;
    }

    mean_f_if_Hz/=n_elements;
    mean_fs_Hz/=n_elements;
    mean_osc_err_ppm/=n_elements;




	std::cout <<std::setiosflags(std::ios::fixed)<<std::setprecision(2)<<"FE parameters estimation for Elonics E4000 Front-End:"<<std::endl;

	std::cout<<"Sampling frequency ="<<mean_fs_Hz<<" [Hz]"<<std::endl;
	std::cout<<"IF bias present in baseband="<<mean_f_if_Hz<<" [Hz]"<<std::endl;
	std::cout<<"Reference oscillator error ="<<mean_osc_err_ppm<<" [ppm]"<<std::endl;

    // 8. Generate GNSS-SDR config file.

    delete configuration;
    delete acquisition;
    delete gnss_synchro;

    google::ShutDownCommandLineFlags();
    std::cout << "GNSS-SDR program ended." << std::endl;

//    if (global_gps_acq_assist_map.size()>0)
//    {
//    	std::map<int,Gps_Acq_Assist> Acq_Assist_map;
//    	Acq_Assist_map=global_gps_acq_assist_map.get_map_copy();
//    	current_TOW=Acq_Assist_map.begin()->second.d_TOW;
//    	std::cout<<"Current TOW obtained from acquisition assistance = "<<current_TOW<<std::endl;
//    }else{
//    	std::cout<<"Unable to get acquisition assistance information. TOW is unknown!"<<std::endl;
//    }
}
