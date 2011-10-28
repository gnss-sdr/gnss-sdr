
/**
 * Copyright notice
 */

/**
 * Author: Javier Arribas, 2011. jarribas(at)cttc.es
 */

#ifndef GPS_L1_CA_TELEMETRY_DECODER_CC_H
#define	GPS_L1_CA_TELEMETRY_DECODER_CC_H

#include <fstream>

#include <gnuradio/gr_block.h>
//#include <gnuradio/gr_sync_block.h>
#include <gnuradio/gr_msg_queue.h>

#include <bitset>

#include "GPS_L1_CA.h"
#include "gps_telemetry.h"
#include "gps_l1_ca_subframe_fsm.h"

#include "concurrent_queue.h"

class gps_l1_ca_telemetry_decoder_cc;
typedef boost::shared_ptr<gps_l1_ca_telemetry_decoder_cc> gps_l1_ca_telemetry_decoder_cc_sptr;
gps_l1_ca_telemetry_decoder_cc_sptr
gps_l1_ca_make_telemetry_decoder_cc(unsigned int satellite, long if_freq, long fs_in, unsigned
    int vector_length, gr_msg_queue_sptr queue, bool dump);


class gps_l1_ca_telemetry_decoder_cc : public gr_block {

private:

  friend gps_l1_ca_telemetry_decoder_cc_sptr
  gps_l1_ca_make_telemetry_decoder_cc(unsigned int satellite, long if_freq, long fs_in,unsigned
      int vector_length, gr_msg_queue_sptr queue, bool dump);

  gps_l1_ca_telemetry_decoder_cc(unsigned int satellite, long if_freq, long fs_in,unsigned
      int vector_length, gr_msg_queue_sptr queue, bool dump);

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
  int d_word_number;

  // navigation message vars
  gps_navigation_message d_nav;
  GpsL1CaSubframeFsm d_GPS_FSM;


  gr_msg_queue_sptr d_queue;
  unsigned int d_vector_length;
  bool d_dump;
  int d_satellite;
  int d_channel;

  float d_preamble_phase;

  std::string d_dump_filename;
  std::ofstream d_dump_file;

public:

  ~gps_l1_ca_telemetry_decoder_cc();

  void set_satellite(int satellite);
  void set_channel(int channel);

  void set_navigation_queue(concurrent_queue<gps_navigation_message> *nav_queue){d_GPS_FSM.d_nav_queue=nav_queue;}

  int general_work (int noutput_items, gr_vector_int &ninput_items,
      gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);
  void forecast (int noutput_items, gr_vector_int &ninput_items_required);

};

#endif
