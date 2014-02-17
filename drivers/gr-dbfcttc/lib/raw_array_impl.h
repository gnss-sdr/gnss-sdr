/*!
 * \file raw_array_impl.h
 * \brief GNU Radio source block to acces to experimental GNSS Array platform.
 * \author Javier Arribas, 2014. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2014  (see AUTHORS file for a list of contributors)
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


#ifndef INCLUDED_DBFCTTC_RAW_ARRAY_IMPL_H
#define INCLUDED_DBFCTTC_RAW_ARRAY_IMPL_H

#include <dbfcttc/raw_array.h>
//#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <pcap.h>

namespace gr {
namespace dbfcttc {

class raw_array_impl : public raw_array
{
private:
	//omni_mutex	d_mutex; // no longer available in GNU Radio
	boost::mutex d_mutex;

	pcap_t*       descr; //ethernet pcap device descriptor
	int fifo_pipe[2];

	gr_complex **fifo_buff_ch;

	int    fifo_read_ptr;
	int    fifo_write_ptr;
	int    fifo_items;

	const char *d_src_device;
	short d_number_of_channels;
	int d_snapshots_per_frame;
	int d_inter_frame_delay;
	int d_sampling_freq;

	bool d_flag_start_frame;
	bool d_fifo_full;

	int d_last_frame_counter;
	int  d_num_rx_errors;

	boost::thread *d_pcap_thread;
	/*!
	 * \brief
	 * Opens the ethernet device using libpcap raw capture mode
	 * If any of these fail, the fuction retuns the error and exits.
	 */
	bool open();
	/*!
	 * \brief
	 * Configure the Array hardware platform with the selected parameters. Uses the same ethernet connection
	 */
	bool configure_array();
	/*!
	 * \brief
	 * Start the array operation. Uses the same ethernet connection
	 */
	bool start_array();
	/*!
	 * \brief
	 * Stop the array operation. Uses the same ethernet connection
	 */
	bool stop_array();

    void my_pcap_loop_thread(pcap_t *pcap_handle);

    void pcap_callback(u_char *args, const struct pcap_pkthdr* pkthdr, const u_char* packet);

    static void static_pcap_callback(u_char *args, const struct pcap_pkthdr* pkthdr, const u_char* packet);


public:
	raw_array_impl(const char *src_device,short number_of_channels, int snapshots_per_frame, int inter_frame_delay, int sampling_freq);
	~raw_array_impl();

	// Where all the action really happens
	int work(int noutput_items,
			gr_vector_const_void_star &input_items,
			gr_vector_void_star &output_items);
};

} // namespace dbfcttc
} // namespace gr

#endif /* INCLUDED_DBFCTTC_RAW_ARRAY_IMPL_H */

