/*!
 * \file gps_l1_ca_subframe_fsm.h
 * \brief  Interface of a Galileo NAV message word-to-subframe decoder state machine
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_GALILEO_INAV_FSM_H_
#define GNSS_SDR_GALILEO_INAV_FSM_H_

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

#include "galileo_navigation_message.h"
#include "galileo_ephemeris.h"
#include "galileo_almanac.h"
#include "galileo_iono.h"
#include "galileo_utc_model.h"

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

class GalileoINAVFsm : public sc::state_machine< GalileoINAVFsm, gps_subframe_fsm_S0 >
{
public:
  // channel and satellite info
  int i_channel_ID;
  unsigned int i_satellite_PRN;

  // ephemeris queue
  concurrent_queue<Galileo_Ephemeris> *d_ephemeris_queue;
  // ionospheric parameters queue
  concurrent_queue<Galileo_Iono> *d_iono_queue;
  // UTC model parameters queue
  concurrent_queue<Galileo_Utc_Model> *d_utc_model_queue;
  // Almanac queue
  concurrent_queue<Galileo_Almanac> *d_almanac_queue;

  // navigation message class
  Gps_Navigation_Message d_nav;

  // GPS SV and System parameters
  Galileo_Ephemeris ephemeris;
  Galileo_Almanac almanac;
  Galileo_Utc_Model utc_model;
  Galileo_Iono iono;


  char d_subframe[GPS_SUBFRAME_LENGTH];
  char d_GPS_frame_4bytes[GPS_WORD_LENGTH];

  double d_preamble_time_ms;

  void gps_word_to_subframe(int position);
  void gps_subframe_to_nav_msg();

  //FSM EVENTS
  void Event_gps_word_valid();
  void Event_gps_word_invalid();
  void Event_gps_word_preamble();

  GalileoINAVFsm();
};

#endif
