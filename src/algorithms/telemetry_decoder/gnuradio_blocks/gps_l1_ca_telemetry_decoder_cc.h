/*!
 * \file gps_l1_ca_telemetry_decoder_cc.h
 * \brief Interface of a NAV message demodulator block based on
 * Kay Borre book MATLAB-based GPS receiver
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_GPS_L1_CA_TELEMETRY_DECODER_CC_H
#define	GNSS_SDR_GPS_L1_CA_TELEMETRY_DECODER_CC_H


#include "GPS_L1_CA.h"
#include "gps_l1_ca_subframe_fsm.h"
#include "concurrent_queue.h"
#include <fstream>
#include <bitset>
#include <gnuradio/gr_block.h>
#include <gnuradio/gr_msg_queue.h>
//#include <gnuradio/gr_sync_block.h>



class gps_l1_ca_telemetry_decoder_cc;

typedef boost::shared_ptr<gps_l1_ca_telemetry_decoder_cc> gps_l1_ca_telemetry_decoder_cc_sptr;

gps_l1_ca_telemetry_decoder_cc_sptr
gps_l1_ca_make_telemetry_decoder_cc(unsigned int satellite, long if_freq, long fs_in, unsigned
    int vector_length, gr_msg_queue_sptr queue, bool dump);

/*!
 * \brief This class implements a block that decodes the NAV data defined in IS-GPS-200E
 *
 */
class gps_l1_ca_telemetry_decoder_cc : public gr_block {

public:

  ~gps_l1_ca_telemetry_decoder_cc();

  void set_satellite(int satellite);  //!< Set satellite PRN
  void set_channel(int channel);      //!< Set receiver's channel

  /*!
   * \brief Set the navigation queue
   */
  void set_navigation_queue(concurrent_queue<Gps_Navigation_Message> *nav_queue){d_GPS_FSM.d_nav_queue=nav_queue;}

  int general_work (int noutput_items, gr_vector_int &ninput_items,
      gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

  void forecast (int noutput_items, gr_vector_int &ninput_items_required);


private:

  friend gps_l1_ca_telemetry_decoder_cc_sptr
  gps_l1_ca_make_telemetry_decoder_cc(unsigned int satellite, long if_freq, long fs_in,unsigned
      int vector_length, gr_msg_queue_sptr queue, bool dump);

  gps_l1_ca_telemetry_decoder_cc(unsigned int satellite, long if_freq, long fs_in,unsigned
      int vector_length, gr_msg_queue_sptr queue, bool dump);

  bool gps_word_parityCheck(unsigned int gpsword);

  // constants
  unsigned short int d_preambles_bits[8];

  // class private vars

  signed int *d_preambles_symbols;
  unsigned int d_samples_per_bit;
  long unsigned int d_sample_counter;
  long unsigned int d_preamble_index;
  unsigned int d_stat;
  bool d_flag_frame_sync;

  // symbols
  int d_symbol_accumulator;
  short int d_symbol_accumulator_counter;

  //bits and frame
  unsigned short int d_frame_bit_index;
  unsigned int d_GPS_frame_4bytes;
  unsigned int d_prev_GPS_frame_4bytes;
  bool d_flag_parity;
  bool d_flag_preamble;
  int d_word_number;

  long d_fs_in;
  double d_preamble_duration_seconds;
  // navigation message vars
  Gps_Navigation_Message d_nav;
  GpsL1CaSubframeFsm d_GPS_FSM;


  gr_msg_queue_sptr d_queue;
  unsigned int d_vector_length;
  bool d_dump;
  int d_satellite;
  int d_channel;

  //std::deque<double> d_prn_start_sample_history;

  double d_preamble_time_seconds;
  double d_preamble_code_phase_seconds;

  std::string d_dump_filename;
  std::ofstream d_dump_file;
};

#endif
