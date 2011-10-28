
/**
 * Copyright notice
 */

/**
 * Author: Javier Arribas, 2011. jarribas(at)cttc.es
 */


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>
# include <bitset>

#include <cmath>
#include "math.h"

#include "gps_l1_ca_observables_cc.h"

#include "control_message_factory.h"

#include <gnuradio/gr_io_signature.h>

#include <glog/log_severity.h>
#include <glog/logging.h>

using google::LogMessage;

using namespace std;


gps_l1_ca_observables_cc_sptr
gps_l1_ca_make_observables_cc(unsigned int nchannels, gr_msg_queue_sptr queue, bool dump) {

  return gps_l1_ca_observables_cc_sptr(new gps_l1_ca_observables_cc(nchannels, queue, dump));
}


gps_l1_ca_observables_cc::gps_l1_ca_observables_cc(unsigned int nchannels, gr_msg_queue_sptr queue, bool dump) :
		        gr_block ("gps_l1_ca_observables_cc", gr_make_io_signature (nchannels, nchannels,  sizeof(gnss_synchro)),
		            gr_make_io_signature(1, 1, sizeof(gr_complex))) {
  //TODO: change output channels to have Pseudorange GNURadio signature: nchannels input (float), nchannels output (float)
  // initialize internal vars
  d_queue = queue;
  d_dump = dump;
  d_nchannels = nchannels;
  d_rinex_printer.set_headers("GNSS-SDR"); //TODO: read filename from config
  d_ls_pvt=new gps_l1_ca_ls_pvt(nchannels);
  d_ephemeris_clock_s=0.0;
  if (d_dump==true)
  {
      std::stringstream d_dump_filename_str;//create a stringstream to form the dump filename
      d_dump_filename_str<<"./data/obs.dat"; //TODO: get filename and path from config in the adapter
      d_dump_filename=d_dump_filename_str.str();
      d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
  }

}

gps_l1_ca_observables_cc::~gps_l1_ca_observables_cc() {

    delete d_ls_pvt;
}

bool pairCompare_min( pair<int,gnss_synchro> a, pair<int,gnss_synchro> b)
{
  return (a.second.preamble_delay_ms+a.second.prn_delay_ms) < (b.second.preamble_delay_ms+b.second.prn_delay_ms);
}

bool pairCompare_max( pair<int,gnss_synchro> a, pair<int,gnss_synchro> b)
{
    return (a.second.preamble_delay_ms+a.second.prn_delay_ms) > (b.second.preamble_delay_ms+b.second.prn_delay_ms);
}


int gps_l1_ca_observables_cc::general_work (int noutput_items, gr_vector_int &ninput_items,
    gr_vector_const_void_star &input_items,	gr_vector_void_star &output_items) {

  gnss_synchro **in = (gnss_synchro **)  &input_items[0]; //Get the input samples pointer

  // CONSTANTS TODO: place in a file
  const float code_length_s=0.001; //1 ms
  const float C_m_ms= GPS_C_m_s/1000.0;    // The speed of light, [m/ms]

  const float nav_sol_period_ms=1000;
  //--- Find number of samples per spreading code ----------------------------
  const signed int codeFreqBasis=1023000; //Hz
  const signed int codeLength=1023;
  const unsigned int samples_per_code = round(d_fs_in/ (codeFreqBasis / codeLength));

  map<int,gnss_synchro> gps_words;
  map<int,gnss_synchro>::iterator gps_words_iter;

  map<int,float> pseudoranges;
  map<int,float>::iterator pseudoranges_iter;
  map<int,float> pseudoranges_dump;

  float min_preamble_delay_ms;
  float max_preamble_delay_ms;
  bool  flag_valid_pseudoranges=false;

  float pseudoranges_timestamp_ms;
  float traveltime_ms;
  float pseudorange_m;

  int pseudoranges_reference_sat_ID=0;

  for (unsigned int i=0; i<d_nchannels ; i++)
    {
    if (in[i][0].valid_word) //if this channel have valid word
      {
      gps_words.insert(pair<int,gnss_synchro>(in[i][0].channel_ID,in[i][0])); //record the word structure in a map for pseudoranges
      }
    }

  if(gps_words.size()>0)
    {

      // find the minimum index (nearest satellite, will be the reference)
      gps_words_iter=min_element(gps_words.begin(),gps_words.end(),pairCompare_min);
      min_preamble_delay_ms=gps_words_iter->second.preamble_delay_ms+gps_words_iter->second.prn_delay_ms; //[ms]
      pseudoranges_timestamp_ms=min_preamble_delay_ms; //save the shortest pseudorange timestamp to compute the RINEX timestamp
      pseudoranges_reference_sat_ID=gps_words_iter->second.satellite_PRN;
      gps_words_iter=max_element(gps_words.begin(),gps_words.end(),pairCompare_max);
      max_preamble_delay_ms=gps_words_iter->second.preamble_delay_ms; //[ms]

      if ((max_preamble_delay_ms-min_preamble_delay_ms)<70) flag_valid_pseudoranges=true;


      //compute the pseudoranges
      for(gps_words_iter = gps_words.begin(); gps_words_iter != gps_words.end(); gps_words_iter++)
        {
        std::cout<<"prn_delay="<<gps_words_iter->second.prn_delay_ms<<std::endl;
        traveltime_ms=gps_words_iter->second.preamble_delay_ms+gps_words_iter->second.prn_delay_ms-min_preamble_delay_ms+GPS_STARTOFFSET_ms; //[ms]
        pseudorange_m=traveltime_ms*C_m_ms; // [m]
        pseudoranges.insert(pair<int,float>(gps_words_iter->second.satellite_PRN,pseudorange_m)); //record the preamble index in a map
        if (d_dump==true)
        {
            pseudoranges_dump.insert(pair<int,float>(gps_words_iter->second.channel_ID,pseudorange_m));
        }
        }
    // write the pseudoranges to RINEX OBS file
    // 1- need a valid clock
    if (flag_valid_pseudoranges==true and d_last_nav_msg.d_satellite_PRN>0)
      {
        //d_rinex_printer.LogRinex2Obs(d_last_nav_msg,d_ephemeris_clock_s+((double)pseudoranges_timestamp_ms-d_ephemeris_timestamp_ms)/1000.0,pseudoranges);
        // compute on the fly PVT solution
        d_ls_pvt->get_PVT(pseudoranges,d_ephemeris_clock_s+((double)pseudoranges_timestamp_ms-d_ephemeris_timestamp_ms)/1000.0);
      }
    }

  //debug
  cout << setprecision(16);
  for(pseudoranges_iter = pseudoranges.begin();
      pseudoranges_iter != pseudoranges.end();
      pseudoranges_iter++)
    {
    cout<<"Pseudoranges =("<<pseudoranges_iter->first<<","<<pseudoranges_iter->second<<")"<<endl;
    }
  gps_navigation_message nav_msg;
  if (d_nav_queue->try_pop(nav_msg)==true)
    {
    cout<<"New ephemeris record has arrived from SAT ID "<<nav_msg.d_satellite_PRN<<endl;
    d_last_nav_msg=nav_msg;
    d_ls_pvt->d_ephemeris[nav_msg.d_channel_ID]=nav_msg;
    // **** update pseudoranges clock ****
    if (nav_msg.d_satellite_PRN==pseudoranges_reference_sat_ID)
    {
        d_ephemeris_clock_s=d_last_nav_msg.d_TOW;
        d_ephemeris_timestamp_ms=d_last_nav_msg.d_subframe1_timestamp_ms;
    }
    // **** write ephemeris to RINES NAV file
    //d_rinex_printer.LogRinex2Nav(nav_msg);
    }

  if (d_dump==true)
  {
      float tmp_float=0.0;
      for (int i=0;i<d_nchannels;i++)
      {
        pseudoranges_iter=pseudoranges_dump.find(i);
        if (pseudoranges_iter!=pseudoranges_dump.end())
        {
            d_dump_file.write((char*)&pseudoranges_iter->second, sizeof(float));
        }else{
            d_dump_file.write((char*)&tmp_float, sizeof(float));
        }
      }
  }
  consume_each(1); //one by one
  return 0;
}


