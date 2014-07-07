/*!
 * \file galileo_e5a_3ms_noncoherent_iq_acquisition_cc.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E5a data and pilot Signals
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
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

#include "galileo_e5a_3ms_noncoherent_iq_acquisition_cc.h"
#include <sys/time.h>
#include <sstream>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include <volk/volk.h>
#include "gnss_signal_processing.h"
#include "control_message_factory.h"

using google::LogMessage;

galileo_e5a_3ms_noncoherentIQ_acquisition_cc_sptr galileo_e5a_3ms_noncoherentIQ_make_acquisition_cc(
				 unsigned int sampled_ms,
                                 unsigned int max_dwells,
                                 unsigned int doppler_max, long freq, long fs_in,
                                 int samples_per_ms, int samples_per_code,
                                 bool bit_transition_flag,
                                 gr::msg_queue::sptr queue, bool dump,
                                 std::string dump_filename)
{

    return galileo_e5a_3ms_noncoherentIQ_acquisition_cc_sptr(
            new galileo_e5a_3ms_noncoherentIQ_acquisition_cc(sampled_ms, max_dwells, doppler_max, freq, fs_in, samples_per_ms,
                                     samples_per_code, bit_transition_flag, queue, dump, dump_filename));
}

galileo_e5a_3ms_noncoherentIQ_acquisition_cc::galileo_e5a_3ms_noncoherentIQ_acquisition_cc(
			 unsigned int sampled_ms,
			 unsigned int max_dwells,
                         unsigned int doppler_max, long freq, long fs_in,
                         int samples_per_ms, int samples_per_code,
                         bool bit_transition_flag,
                         gr::msg_queue::sptr queue, bool dump,
                         std::string dump_filename) :
    gr::block("galileo_e5a_3ms_noncoherentIQ_acquisition_cc",
		gr::io_signature::make(1, 1, sizeof(gr_complex)),
		gr::io_signature::make(0, 0, sizeof(gr_complex)))
    //gr::io_signature::make(1, 1, sizeof(gr_complex) * 3 * samples_per_ms),
    //gr::io_signature::make(0, 0, sizeof(gr_complex) * 3 * samples_per_ms))
{
    //this->set_relative_rate(1.0/1*samples_per_ms);
    d_sample_counter = 0;    // SAMPLE COUNTER
    d_active = false;
    d_state = 0;
    d_queue = queue;
    d_freq = freq;
    d_fs_in = fs_in;
    d_samples_per_ms = samples_per_ms;
    d_samples_per_code = samples_per_code;
    d_max_dwells = max_dwells;
    d_well_count = 0;
    d_doppler_max = doppler_max;
    d_fft_size = sampled_ms * d_samples_per_ms;
    d_mag = 0;
    d_input_power = 0.0;
    d_num_doppler_bins = 0;
    d_bit_transition_flag = bit_transition_flag;
    d_buffer_count=0;
    d_gr_stream_buffer = 7000; // number of samples entering each general work, arbitrary number. Works with all numbers below gnu radio maximum buffer

    //todo: do something if posix_memalign fails
    if (posix_memalign((void**)&d_inbuffer, 16, ceil((double)d_fft_size/(double)d_gr_stream_buffer)*d_gr_stream_buffer * sizeof(gr_complex)) == 0){};
    if (posix_memalign((void**)&d_fft_code_I_A, 16, d_fft_size * sizeof(gr_complex)) == 0){};
    if (posix_memalign((void**)&d_fft_code_I_B, 16, d_fft_size * sizeof(gr_complex)) == 0){};
    if (posix_memalign((void**)&d_fft_code_Q_A, 16, d_fft_size * sizeof(gr_complex)) == 0){};
    if (posix_memalign((void**)&d_fft_code_Q_B, 16, d_fft_size * sizeof(gr_complex)) == 0){};
    if (posix_memalign((void**)&d_magnitudeIA, 16, d_fft_size * sizeof(float)) == 0){};
    if (posix_memalign((void**)&d_magnitudeIB, 16, d_fft_size * sizeof(float)) == 0){};
    if (posix_memalign((void**)&d_magnitudeQA, 16, d_fft_size * sizeof(float)) == 0){};
    if (posix_memalign((void**)&d_magnitudeQB, 16, d_fft_size * sizeof(float)) == 0){};

    // Direct FFT
    d_fft_if = new gr::fft::fft_complex(d_fft_size, true);

    // Inverse FFT
    d_ifft = new gr::fft::fft_complex(d_fft_size, false);
//    d_ifft = new gr::fft::fft_complex(d_fft_size, true);

    // For dumping samples into a file
    d_dump = dump;
    d_dump_filename = dump_filename;
}

galileo_e5a_3ms_noncoherentIQ_acquisition_cc::~galileo_e5a_3ms_noncoherentIQ_acquisition_cc()
{
    if (d_num_doppler_bins > 0)
        {
            for (unsigned int i = 0; i < d_num_doppler_bins; i++)
                {
                    free(d_grid_doppler_wipeoffs[i]);
                }
            delete[] d_grid_doppler_wipeoffs;
        }

    free(d_fft_code_I_A);
    free(d_fft_code_I_B);
    free(d_fft_code_Q_A);
    free(d_fft_code_Q_B);
    free(d_magnitudeIA);
    free(d_magnitudeIB);
    free(d_magnitudeQA);
    free(d_magnitudeQB);


    delete d_fft_if;
    delete d_ifft;


    if (d_dump)
        {
            d_dump_file.close();
        }
}

void galileo_e5a_3ms_noncoherentIQ_acquisition_cc::forecast (int noutput_items,
		gr_vector_int &ninput_items_required)
{
    ninput_items_required[0] = d_gr_stream_buffer ; //set the required available samples in each call
}

void galileo_e5a_3ms_noncoherentIQ_acquisition_cc::set_local_code(std::complex<float> * codeI, std::complex<float> * codeQ )
{
    // DATA SIGNAL
    // Three replicas of data primary code. CODE A: (1,1,1)
    memcpy(d_fft_if->get_inbuf(), codeI, sizeof(gr_complex)*d_fft_size);

    d_fft_if->execute(); // We need the FFT of local code

    //Conjugate the local code
    if (is_unaligned())
        {
            volk_32fc_conjugate_32fc_u(d_fft_code_I_A,d_fft_if->get_outbuf(),d_fft_size);
        }
    else
        {
            volk_32fc_conjugate_32fc_a(d_fft_code_I_A,d_fft_if->get_outbuf(),d_fft_size);
        }

    // CODE B: First replica is inverted (0,1,1)
    volk_32fc_s32fc_multiply_32fc_a(&(d_fft_if->get_inbuf())[0],
                                    &codeI[0], gr_complex(-1,0),
                                    d_samples_per_code);
    d_fft_if->execute(); // We need the FFT of local code

    //Conjugate the local code
    if (is_unaligned())
        {
            volk_32fc_conjugate_32fc_u(d_fft_code_I_B,d_fft_if->get_outbuf(),d_fft_size);
        }
    else
        {
            volk_32fc_conjugate_32fc_a(d_fft_code_I_B,d_fft_if->get_outbuf(),d_fft_size);
        }

    // SAME FOR PILOT SIGNAL
    // Three replicas of pilot primary code. CODE A: (1,1,1)
    memcpy(d_fft_if->get_inbuf(), codeQ, sizeof(gr_complex)*d_fft_size);

    d_fft_if->execute(); // We need the FFT of local code

    //Conjugate the local code
    if (is_unaligned())
        {
            volk_32fc_conjugate_32fc_u(d_fft_code_Q_A,d_fft_if->get_outbuf(),d_fft_size);
        }
    else
        {
            volk_32fc_conjugate_32fc_a(d_fft_code_Q_A,d_fft_if->get_outbuf(),d_fft_size);
        }

    // CODE B: First replica is inverted (0,1,1)
    volk_32fc_s32fc_multiply_32fc_a(&(d_fft_if->get_inbuf())[0],
                                    &codeQ[0], gr_complex(-1,0),
                                    d_samples_per_code);
    d_fft_if->execute(); // We need the FFT of local code

    //Conjugate the local code
    if (is_unaligned())
        {
            volk_32fc_conjugate_32fc_u(d_fft_code_Q_B,d_fft_if->get_outbuf(),d_fft_size);
        }
    else
        {
            volk_32fc_conjugate_32fc_a(d_fft_code_Q_B,d_fft_if->get_outbuf(),d_fft_size);
        }


}

void galileo_e5a_3ms_noncoherentIQ_acquisition_cc::init()
{
    d_gnss_synchro->Acq_delay_samples = 0.0;
    d_gnss_synchro->Acq_doppler_hz = 0.0;
    d_gnss_synchro->Acq_samplestamp_samples = 0;
    d_mag = 0.0;
    d_input_power = 0.0;

    // Count the number of bins
    d_num_doppler_bins = 0;
    for (int doppler = (int)(-d_doppler_max);
         doppler <= (int)d_doppler_max;
         doppler += d_doppler_step)
    {
        d_num_doppler_bins++;
    }

    // Create the carrier Doppler wipeoff signals
    d_grid_doppler_wipeoffs = new gr_complex*[d_num_doppler_bins];
    for (unsigned int doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            if (posix_memalign((void**)&(d_grid_doppler_wipeoffs[doppler_index]), 16,
                               d_fft_size * sizeof(gr_complex)) == 0){};

            int doppler = -(int)d_doppler_max + d_doppler_step*doppler_index;
            complex_exp_gen_conj(d_grid_doppler_wipeoffs[doppler_index],
                                 d_freq + doppler, d_fs_in, d_fft_size);
        }
}


int galileo_e5a_3ms_noncoherentIQ_acquisition_cc::general_work(int noutput_items,
        gr_vector_int &ninput_items, gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
{
    /*
     * By J.Arribas, L.Esteve, M.Molina and M.Sales
     * Acquisition strategy (Kay Borre book + CFAR threshold):
     * 1. Compute the input signal power estimation
     * 2. Doppler serial search loop
     * 3. Perform the FFT-based circular convolution (parallel time search)
     * 4. Record the maximum peak and the associated synchronization parameters
     * 5. Compute the test statistics and compare to the threshold
     * 6. Declare positive or negative acquisition using a message queue
     */

    int acquisition_message = -1; //0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL
    /* States: 	0 Reset and load first stream
     * 		1 Load the buffer until it reaches fft_size
     * 		2 Acquisition algorithm
     * 		3 Positive acquisition
     * 		4 Negative acquisition
     */

    d_sample_counter += d_gr_stream_buffer;
    std::cout << d_state <<" "<< d_sample_counter << std::endl;
    switch (d_state)
    {
	case 0:
	    {
		if (d_active)
		    {
			//restart acquisition variables
			d_gnss_synchro->Acq_delay_samples = 0.0;
			d_gnss_synchro->Acq_doppler_hz = 0.0;
			d_gnss_synchro->Acq_samplestamp_samples = 0;
			d_well_count = 0;
			d_mag = 0.0;
			d_input_power = 0.0;
			d_test_statistics = 0.0;
			d_state = 1;
		    }
		const gr_complex *in = (const gr_complex *)input_items[0]; //Get the input samples pointer
		memcpy(&d_inbuffer[d_buffer_count*d_gr_stream_buffer], in, sizeof(gr_complex)*d_gr_stream_buffer);
		d_buffer_count++;
	        //d_sample_counter += ninput_items[0]; // sample counter
	        //consume_each(ninput_items[0]);
		break;
	    }
	case 1:
	    {
		const gr_complex *in = (const gr_complex *)input_items[0]; //Get the input samples pointer
		memcpy(&d_inbuffer[d_buffer_count*d_gr_stream_buffer], in, sizeof(gr_complex)*d_gr_stream_buffer);
		d_buffer_count++;
		if (d_buffer_count*d_gr_stream_buffer >= d_fft_size-d_gr_stream_buffer)
		    {
			d_state=2;
		    }
//		volk_32fc_x2_multiply_32fc_a(d_fft_if->get_inbuf(), in,
//                            d_grid_doppler_wipeoffs[0], d_fft_size);
		//consume_each(7000);
		break;
	    }
	case 2:
	    {
		// Fill last part of the buffer and reset counter
		const gr_complex *in = (const gr_complex *)input_items[0]; //Get the input samples pointer
		memcpy(&d_inbuffer[d_buffer_count*d_gr_stream_buffer], in, sizeof(gr_complex)*d_gr_stream_buffer);
		d_buffer_count = 0;
		// initialize acquisition algorithm
		int doppler;
		unsigned int indext = 0;
		unsigned int indext_IA = 0;
		unsigned int indext_IB = 0;
		unsigned int indext_QA = 0;
		unsigned int indext_QB = 0;
		float magt = 0.0;
		float magt_IA = 0.0;
		float magt_IB = 0.0;
		float magt_QA = 0.0;
		float magt_QB = 0.0;
		//const gr_complex *in = (const gr_complex *)input_items[0]; //Get the input samples pointer
		float fft_normalization_factor = (float)d_fft_size * (float)d_fft_size;
		d_input_power = 0.0;
		d_mag = 0.0;
		int comb = 0;

		//d_sample_counter += d_fft_size; // sample counter

		d_well_count++;

		DLOG(INFO) << "Channel: " << d_channel
			<< " , doing acquisition of satellite: " << d_gnss_synchro->System << " "<< d_gnss_synchro->PRN
			<< " ,sample stamp: " << d_sample_counter << ", threshold: "
			<< d_threshold << ", doppler_max: " << d_doppler_max
			<< ", doppler_step: " << d_doppler_step;

		// 1- Compute the input signal power estimation
		volk_32fc_magnitude_squared_32f_a(d_magnitudeIA, d_inbuffer, d_fft_size);
		volk_32f_accumulator_s32f_a(&d_input_power, d_magnitudeIA, d_fft_size);
		d_input_power /= (float)d_fft_size;

		// 2- Doppler frequency search loop
		for (unsigned int doppler_index=0;doppler_index<d_num_doppler_bins;doppler_index++)
		    {
			// doppler search steps

			doppler=-(int)d_doppler_max+d_doppler_step*doppler_index;

			volk_32fc_x2_multiply_32fc_a(d_fft_if->get_inbuf(), d_inbuffer,
			                             d_grid_doppler_wipeoffs[doppler_index], d_fft_size);

			// 3- Perform the FFT-based convolution  (parallel time search)
			// Compute the FFT of the carrier wiped--off incoming signal
			d_fft_if->execute();

			// CODE IA
			// Multiply carrier wiped--off, Fourier transformed incoming signal
			// with the local FFT'd code reference using SIMD operations with VOLK library
			volk_32fc_x2_multiply_32fc_a(d_ifft->get_inbuf(),
			                             d_fft_if->get_outbuf(), d_fft_code_I_A, d_fft_size);

			// compute the inverse FFT
			d_ifft->execute();

			// Search maximum
			volk_32fc_magnitude_squared_32f_a(d_magnitudeIA, d_ifft->get_outbuf(), d_fft_size);
			volk_32f_index_max_16u_a(&indext_IA, d_magnitudeIA, d_fft_size);

			// Normalize the maximum value to correct the scale factor introduced by FFTW
			magt_IA = d_magnitudeIA[indext_IA] / (fft_normalization_factor * fft_normalization_factor);

			// only 1 ms
			//magt=magt_A;
			//indext=indext_A;


			// REPEAT FOR ALL CODES. CODE_IB
			volk_32fc_x2_multiply_32fc_a(d_ifft->get_inbuf(),
			                             d_fft_if->get_outbuf(), d_fft_code_I_B, d_fft_size);
			d_ifft->execute();
			volk_32fc_magnitude_squared_32f_a(d_magnitudeIB, d_ifft->get_outbuf(), d_fft_size);
			volk_32f_index_max_16u_a(&indext_IB, d_magnitudeIB, d_fft_size);
			magt_IB = d_magnitudeIB[indext_IB] / (fft_normalization_factor * fft_normalization_factor);

			// REPEAT FOR ALL CODES. CODE_QA
			volk_32fc_x2_multiply_32fc_a(d_ifft->get_inbuf(),
			                             d_fft_if->get_outbuf(), d_fft_code_Q_A, d_fft_size);
			d_ifft->execute();
			volk_32fc_magnitude_squared_32f_a(d_magnitudeQA, d_ifft->get_outbuf(), d_fft_size);
			volk_32f_index_max_16u_a(&indext_QA, d_magnitudeQA, d_fft_size);
			magt_QA = d_magnitudeQA[indext_QA] / (fft_normalization_factor * fft_normalization_factor);

			// REPEAT FOR ALL CODES. CODE_QB
			volk_32fc_x2_multiply_32fc_a(d_ifft->get_inbuf(),
			                             d_fft_if->get_outbuf(), d_fft_code_Q_B, d_fft_size);
			d_ifft->execute();
			volk_32fc_magnitude_squared_32f_a(d_magnitudeQB, d_ifft->get_outbuf(), d_fft_size);
			volk_32f_index_max_16u_a(&indext_QB, d_magnitudeQB, d_fft_size);
			magt_QB = d_magnitudeIB[indext_QB] / (fft_normalization_factor * fft_normalization_factor);

			// Integrate noncoherently the two best combinations (I² + Q²)
			// and store the result in the I channel.
			if (magt_IA >= magt_IB)
			    {
				if (magt_QA >= magt_QB)
				    {
					for (unsigned int i=0; i<d_fft_size; i++)
					    {
						d_magnitudeIA[i] += d_magnitudeQA[i];
					    }
				    }
				else
				    {
					for (unsigned int i=0; i<d_fft_size; i++)
					    {
						d_magnitudeIA[i] += d_magnitudeQB[i];
					    }
				    }
				volk_32f_index_max_16u_a(&indext, d_magnitudeIA, d_fft_size);
				magt = d_magnitudeIA[indext] / (fft_normalization_factor * fft_normalization_factor);
			    }
			else
			    {
				if (magt_QA >= magt_QB)
				    {
					for (unsigned int i=0; i<d_fft_size; i++)
					    {
						d_magnitudeIB[i] += d_magnitudeQA[i];
					    }
				    }
				else
				    {
					for (unsigned int i=0; i<d_fft_size; i++)
					    {
						d_magnitudeIB[i] += d_magnitudeQB[i];
					    }
				    }
				volk_32f_index_max_16u_a(&indext, d_magnitudeIB, d_fft_size);
				magt = d_magnitudeIB[indext] / (fft_normalization_factor * fft_normalization_factor);
			    }

			// 4- record the maximum peak and the associated synchronization parameters
			if (d_mag < magt)
			    {
				d_mag = magt;
				//std::cout << "ACQ_block_e5a_3ms secondary combination " << sec_comb << std::endl;

				// In case that d_bit_transition_flag = true, we compare the potentially
				// new maximum test statistics (d_mag/d_input_power) with the value in
				// d_test_statistics. When the second dwell is being processed, the value
				// of d_mag/d_input_power could be lower than d_test_statistics (i.e,
				// the maximum test statistics in the previous dwell is greater than
				// current d_mag/d_input_power). Note that d_test_statistics is not
				// restarted between consecutive dwells in multidwell operation.
				if (d_test_statistics < (d_mag / d_input_power) || !d_bit_transition_flag)
				    {
					d_gnss_synchro->Acq_delay_samples = (double)(indext % d_samples_per_code);
					d_gnss_synchro->Acq_doppler_hz = (double)doppler;
					d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter;

					// 5- Compute the test statistics and compare to the threshold
					//d_test_statistics = 2 * d_fft_size * d_mag / d_input_power;
					d_test_statistics = d_mag / d_input_power;
				    }
			    }

			// Record results to file if required
			if (d_dump)
			    {
				std::stringstream filename;
				std::streamsize n = sizeof(float) * (d_fft_size); // noncomplex file write
				filename.str("");
				filename << "../data/test_statistics_" << d_gnss_synchro->System
					<<"_" << d_gnss_synchro->Signal << "_sat_"
					<< d_gnss_synchro->PRN << "_doppler_" <<  doppler << ".dat";
				d_dump_file.open(filename.str().c_str(), std::ios::out | std::ios::binary);
				if (magt_IA >= magt_IB)
				    {
					d_dump_file.write((char*)d_magnitudeIA, n);
				    }
				else
				    {
					d_dump_file.write((char*)d_magnitudeIB, n);
				    }
				//d_dump_file.write((char*)d_magnitudeIA, n);
				d_dump_file.close();
			    }
		    }

		if (!d_bit_transition_flag)
		    {
			if (d_test_statistics > d_threshold)
			    {
				d_state = 3; // Positive acquisition
			    }
			else if (d_well_count == d_max_dwells)
			    {
				d_state = 4; // Negative acquisition
			    }
			else
			    {
				d_state = 2;
			    }
		    }
		else
		    {
			if (d_well_count == d_max_dwells) // d_max_dwells = 2
			    {
				if (d_test_statistics > d_threshold)
				    {
					d_state = 3; // Positive acquisition
				    }
				else
				    {
					d_state = 4; // Negative acquisition
				    }
			    }
			else
			    {
				d_state = 2;
			    }
		    }

		//consume_each(1);
		//consume_each(d_fft_size);
		break;
	    }
	case 3:
	    {
		// 6.1- Declare positive acquisition using a message queue
		DLOG(INFO) << "positive acquisition";
		DLOG(INFO) << "satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;
		DLOG(INFO) << "sample_stamp " << d_sample_counter;
		DLOG(INFO) << "test statistics value " << d_test_statistics;
		DLOG(INFO) << "test statistics threshold " << d_threshold;
		DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
		DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
		DLOG(INFO) << "magnitude " << d_mag;
		DLOG(INFO) << "input signal power " << d_input_power;

		d_active = false;
		d_state = 0;

		//d_sample_counter += 7000;
		//d_sample_counter += d_fft_size * ninput_items[0]; // sample counter
		//consume_each(ninput_items[0]);
		//consume_each(d_fft_size);
		acquisition_message = 1;
		d_channel_internal_queue->push(acquisition_message);
		break;
	    }
	case 4:
	    {
		// 6.2- Declare negative acquisition using a message queue
		DLOG(INFO) << "negative acquisition";
		DLOG(INFO) << "satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;
		DLOG(INFO) << "sample_stamp " << d_sample_counter;
		DLOG(INFO) << "test statistics value " << d_test_statistics;
		DLOG(INFO) << "test statistics threshold " << d_threshold;
		DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
		DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
		DLOG(INFO) << "magnitude " << d_mag;
		DLOG(INFO) << "input signal power " << d_input_power;

		d_active = false;
		d_state = 0;

		//d_sample_counter += 7000;
		//d_sample_counter += d_fft_size * ninput_items[0]; // sample counter
		//consume_each(ninput_items[0]);
		//consume_each(d_fft_size);
		acquisition_message = 2;
		d_channel_internal_queue->push(acquisition_message);
		break;
	    }
    }

    consume_each(d_gr_stream_buffer);
    return 0;
}

