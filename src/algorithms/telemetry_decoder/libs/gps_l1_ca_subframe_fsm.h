/**
 * Copyright notice
 */

/**
 * Author: Javier Arribas, 2011. jarribas(at)cttc.es
 */
//************ GPS WORD TO SUBFRAME DECODER STATE MACHINE **********
#ifndef GPS_L1_CA_SUBFRAME_FSM_H
#define GPS_L1_CA_SUBFRAME_FSM_H

#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/mpl/list.hpp>

#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include "concurrent_queue.h"

#include <iostream>
#include <cstring>

#include "GPS_L1_CA.h"
#include "gps_telemetry.h"
#include "gps_navigation_message.h"

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

struct gps_subframe_fsm_S0;
struct gps_subframe_fsm_S1;
struct gps_subframe_fsm_S2;
struct gps_subframe_fsm_S3;
struct gps_subframe_fsm_S4;
struct gps_subframe_fsm_S5;
struct gps_subframe_fsm_S6;
struct gps_subframe_fsm_S7;
struct gps_subframe_fsm_S8;
struct gps_subframe_fsm_S9;
struct gps_subframe_fsm_S10;
struct gps_subframe_fsm_S11;

class GpsL1CaSubframeFsm : public sc::state_machine< GpsL1CaSubframeFsm, gps_subframe_fsm_S0 >{
private:

public:

  // channel and satellite info
  int d_channel_ID;
  int d_satellite_PRN;

  // ephemeris queue
  concurrent_queue<gps_navigation_message>      *d_nav_queue;
  // navigation message class
  gps_navigation_message d_nav;

  char d_subframe[GPS_SUBFRAME_LENGTH];
  char d_GPS_frame_4bytes[GPS_WORD_LENGTH];

  double d_preamble_time_ms;

  void gps_word_to_subframe(int position);
  void gps_subframe_to_nav_msg();

  //FSM EVENTS
  void Event_gps_word_valid();
  void Event_gps_word_invalid();
  void Event_gps_word_preamble();

  GpsL1CaSubframeFsm();

};

#endif
