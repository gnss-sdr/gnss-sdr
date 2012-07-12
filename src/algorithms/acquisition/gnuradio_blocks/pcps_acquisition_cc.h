/*!
 * \file pcps_acquisition_cc.h
 * \brief This class implements a Parallell Code Phase Search Acquisition
 *
 *  Acquisition strategy (Kay Borre book + CFAR threshold):
 *  1. Compute the input signal power estimation
 *  2. Doppler serial search loop
 *  3. Perform the FFT-based circular convolution (parallel time search)
 *  4. Record the maximum peak and the associated synchronization parameters
 *  5. Compute the test statistics and compare to the threshold
 *  6. Declare positive or negative acquisition using a message queue
 *
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *         Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *
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

#ifndef PCPS_ACQUISITION_CC_H_
#define PCPS_ACQUISITION_CC_H_

#include <fstream>
#include <gnuradio/gr_block.h>
#include <gnuradio/gr_msg_queue.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/gri_fft.h>
#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include "concurrent_queue.h"
#include "gnss_synchro.h"

class pcps_acquisition_cc;
typedef boost::shared_ptr<pcps_acquisition_cc>
		pcps_acquisition_cc_sptr;
pcps_acquisition_cc_sptr
pcps_make_acquisition_cc(unsigned int sampled_ms,
		unsigned int doppler_max, long freq, long fs_in, int samples_per_ms,
		gr_msg_queue_sptr queue, bool dump, std::string dump_filename);

class pcps_acquisition_cc: public gr_block {

private:

	friend pcps_acquisition_cc_sptr
	pcps_make_acquisition_cc(unsigned int sampled_ms,
			unsigned int doppler_max, long freq, long fs_in,
			int samples_per_ms, gr_msg_queue_sptr queue, bool dump,
			std::string dump_filename);

	pcps_acquisition_cc(unsigned int sampled_ms,
			unsigned int doppler_max, long freq, long fs_in,
			int samples_per_ms, gr_msg_queue_sptr queue, bool dump,
			std::string dump_filename);

	void calculate_magnitudes(gr_complex* fft_begin, int doppler_shift,
			int doppler_offset);

	long d_fs_in;
	long d_freq;
	int d_samples_per_ms;
	unsigned int d_doppler_resolution;
	float d_threshold;
	std::string d_satellite_str;
	unsigned int d_doppler_max;
	unsigned int d_doppler_step;
	unsigned int d_sampled_ms;
	unsigned int d_fft_size;
	unsigned long int d_sample_counter;
	gr_complex* d_sine_if;

	gr_complex* d_fft_codes;

	gri_fft_complex* d_fft_if;

	gri_fft_complex* d_ifft;

	Gnss_Synchro *d_gnss_synchro;
	unsigned int d_code_phase;
	float d_doppler_freq;
	float d_mag;
	float d_input_power;
	float d_test_statistics;

	gr_msg_queue_sptr d_queue;
	concurrent_queue<int> *d_channel_internal_queue;
	std::ofstream d_dump_file;

	bool d_active;
	bool d_dump;
	unsigned int d_channel;
	std::string d_dump_filename;

public:

	~pcps_acquisition_cc();

	void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
	{
		d_gnss_synchro = p_gnss_synchro;
	}

	unsigned int mag()
	{
		return d_mag;
	}

	void init();

	void set_local_code(std::complex<float> * code);

	void set_active(bool active)
	{
		d_active = active;
	}

	void set_channel(unsigned int channel)
	{
		d_channel = channel;
	}

	void set_threshold(float threshold)
	{
		d_threshold = threshold;
	}

	void set_doppler_max(unsigned int doppler_max)
	{
		d_doppler_max = doppler_max;
	}

	void set_doppler_step(unsigned int doppler_step)
	{
		d_doppler_step = doppler_step;
	}

	void set_channel_queue(concurrent_queue<int> *channel_internal_queue)
	{
		d_channel_internal_queue = channel_internal_queue;
	}

	int general_work(int noutput_items, gr_vector_int &ninput_items,
			gr_vector_const_void_star &input_items,
			gr_vector_void_star &output_items);
};

#endif /* PCPS_ACQUISITION_CC_H_*/
