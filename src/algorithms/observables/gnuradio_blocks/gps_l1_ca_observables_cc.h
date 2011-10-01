
/**
 * Copyright notice
 */

/**
 * Author: Javier Arribas, 2011. jarribas(at)cttc.es
 */

#ifndef GPS_L1_CA_OBSERVABLES_CC_H
#define	GPS_L1_CA_OBSERVABLES_CC_H

#include <fstream>

#include <gnuradio/gr_block.h>
#include <gnuradio/gr_msg_queue.h>

#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include "concurrent_queue.h"

#include "gps_navigation_message.h"

#include "rinex_2_1_printer.h"

#include "GPS_L1_CA.h"

class gps_l1_ca_observables_cc;
typedef boost::shared_ptr<gps_l1_ca_observables_cc> gps_l1_ca_observables_cc_sptr;
gps_l1_ca_observables_cc_sptr
gps_l1_ca_make_observables_cc(unsigned int n_channels, gr_msg_queue_sptr queue, bool dump);


class gps_l1_ca_observables_cc : public gr_block {

private:

  friend gps_l1_ca_observables_cc_sptr
  gps_l1_ca_make_observables_cc(unsigned int nchannels, gr_msg_queue_sptr queue, bool dump);

  gps_l1_ca_observables_cc(unsigned int nchannels, gr_msg_queue_sptr queue, bool dump);

  // class private vars
  gr_msg_queue_sptr d_queue;
  bool d_dump;

  unsigned int d_nchannels;
  unsigned long int d_fs_in;

  std::string d_dump_filename;
  std::ofstream d_dump_file;

  concurrent_queue<gps_navigation_message> *d_nav_queue; // Navigation ephemeris queue

  rinex_printer d_rinex_printer; // RINEX printer class

  double d_inter_frame_sec_counter; // counter for seconds between GPS frames

  gps_navigation_message d_last_nav_msg; //last navigation message

public:

  ~gps_l1_ca_observables_cc ();

  void set_navigation_queue(concurrent_queue<gps_navigation_message> *nav_queue){d_nav_queue=nav_queue;}

  void set_fs_in(unsigned long int fs_in) {d_fs_in=fs_in;};

  int general_work (int noutput_items, gr_vector_int &ninput_items,
      gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

};

#endif
