
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

}

gps_l1_ca_observables_cc::~gps_l1_ca_observables_cc() {

}

bool pairCompare( pair<int,gnss_synchro> a, pair<int,gnss_synchro> b)
{
  return a.second.last_preamble_index < b.second.last_preamble_index;
}

int gps_l1_ca_observables_cc::general_work (int noutput_items, gr_vector_int &ninput_items,
    gr_vector_const_void_star &input_items,	gr_vector_void_star &output_items) {

  gnss_synchro **in = (gnss_synchro **)  &input_items[0]; //Get the input samples pointer

  // CONSTANTS TODO: place in a file
  const float code_length_s=0.001; //1 ms
  const float C_m_ms= GPS_C_m_s/1000;    // The speed of light, [m/ms]

  const float nav_sol_period_ms=1000;
  //--- Find number of samples per spreading code ----------------------------
  const signed int codeFreqBasis=1023000; //Hz
  const signed int codeLength=1023;
  const unsigned int samples_per_code = round(d_fs_in/ (codeFreqBasis / codeLength));

  map<int,gnss_synchro> gps_words;
  map<int,gnss_synchro>::iterator gps_words_iter;

  map<int,float> pseudoranges;
  map<int,float>::iterator pseudoranges_iter;


  unsigned long int min_preamble_index;

  float prn_delay_ms;
  float traveltime_ms;
  float pseudorange_m;

  for (unsigned int i=0; i<d_nchannels ; i++)
    {
    if (in[i][0].valid_word) //if this channel have valid word
      {
      gps_words.insert(pair<int,gnss_synchro>(in[i][0].satellite_PRN,in[i][0])); //record the word structure in a map for pseudoranges
      }
    }

  if(gps_words.size()>0)
    {
    // find the minimum index (nearest satellite, will be the reference)
    gps_words_iter=min_element(gps_words.begin(),gps_words.end(),pairCompare);
    min_preamble_index=gps_words_iter->second.last_preamble_index;

    //compute the pseudoranges
    for(gps_words_iter = gps_words.begin(); gps_words_iter != gps_words.end(); gps_words_iter++)
      {
      prn_delay_ms=gps_words_iter->second.prn_delay/(float)samples_per_code;
      traveltime_ms=(float)(1000*(gps_words_iter->second.last_preamble_index-min_preamble_index))/(float)samples_per_code+GPS_STARTOFFSET_ms+prn_delay_ms;
      //cout<<"traveltime ms"<<gps_words_iter->first<<" ="<<traveltime_ms<<endl;
      pseudorange_m=traveltime_ms*C_m_ms;
      pseudoranges.insert(pair<int,float>(gps_words_iter->first,pseudorange_m)); //record the preamble index in a map
      }

    // write the pseudoranges to RINEX OBS file
    // 1- need a valid clock
    if (d_last_nav_msg.d_satellite_PRN>0)
      {
        std::cout<<"d_inter_frame_sec_counter="<<d_inter_frame_sec_counter<<std::endl;
        d_rinex_printer.LogRinex2Obs(d_last_nav_msg,d_inter_frame_sec_counter,pseudoranges);
      }
    d_inter_frame_sec_counter+=((float)NAVIGATION_OUTPUT_RATE_MS)/1000.0; //TODO: synchronize the gps time of the ephemeris with the obs
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

    cout<<"New ephemeris record has arrived!"<<endl;
    cout<<"d_channel_ID="<<nav_msg.d_channel_ID<<endl;
    cout<<"d_satellite_PRN="<<nav_msg.d_satellite_PRN<<endl;
    cout<<"d_satpos_X="<<nav_msg.d_satpos_X<<endl;
    cout<<"d_satpos_Y="<<nav_msg.d_satpos_Y<<endl;
    cout<<"d_satpos_Z="<<nav_msg.d_satpos_Z<<endl;
    cout<<"d_master_clock="<<nav_msg.d_master_clock<<endl;
    cout<<"d_satClkCorr="<<nav_msg.d_satClkCorr<<endl;
    cout<<"d_dtr="<<nav_msg.d_dtr<<endl;

    // write ephemeris to RINES NAV file
    // TODO: check operation ok
    d_last_nav_msg=nav_msg;
    d_inter_frame_sec_counter=0; //reset the interframe seconds counter
    d_rinex_printer.LogRinex2Nav(nav_msg);
    }

  consume_each(1); //one by one
  return 0;
}


