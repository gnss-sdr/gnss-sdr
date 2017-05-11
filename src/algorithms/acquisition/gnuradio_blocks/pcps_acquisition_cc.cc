/*!
 * \file pcps_acquisition_cc.cc
 * \brief This class implements a Parallel Code Phase Search Acquisition
 * \authors <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          <li> Marc Molina, 2013. marc.molina.pena@gmail.com
 *          </ul>
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
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

#include "pcps_acquisition_cc.h"
#include <sstream>
#include <boost/filesystem.hpp>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include "control_message_factory.h"
#include "GPS_L1_CA.h" //GPS_TWO_PI


using google::LogMessage;

pcps_acquisition_cc_sptr pcps_make_acquisition_cc(
                                 unsigned int sampled_ms, unsigned int max_dwells,
                                 unsigned int doppler_max, long freq, long fs_in,
                                 int samples_per_ms, int samples_per_code,
                                 bool bit_transition_flag, bool use_CFAR_algorithm_flag,
                                 bool dump,
                                 std::string dump_filename)
{
    return pcps_acquisition_cc_sptr(
            new pcps_acquisition_cc(sampled_ms, max_dwells, doppler_max, freq, fs_in, samples_per_ms,
                    samples_per_code, bit_transition_flag, use_CFAR_algorithm_flag, dump, dump_filename));
}


pcps_acquisition_cc::pcps_acquisition_cc(
                         unsigned int sampled_ms, unsigned int max_dwells,
                         unsigned int doppler_max, long freq, long fs_in,
                         int samples_per_ms, int samples_per_code,
                         bool bit_transition_flag, bool use_CFAR_algorithm_flag,
                         bool dump,
                         std::string dump_filename) :
    gr::block("pcps_acquisition_cc",
    gr::io_signature::make(1, 1, sizeof(gr_complex) * sampled_ms * samples_per_ms * ( bit_transition_flag ? 2 : 1 )),
    gr::io_signature::make(0, 0, sizeof(gr_complex) * sampled_ms * samples_per_ms * ( bit_transition_flag ? 2 : 1 )) )
{
    this->message_port_register_out(pmt::mp("events"));

    d_sample_counter = 0;    // SAMPLE COUNTER
    d_active = false;
    d_state = 0;
    d_freq = freq;
    d_fs_in = fs_in;
    d_samples_per_ms = samples_per_ms;
    d_samples_per_code = samples_per_code;
    d_sampled_ms = sampled_ms;
    d_max_dwells = max_dwells;
    d_well_count = 0;
    d_doppler_max = doppler_max;
    d_fft_size = d_sampled_ms * d_samples_per_ms;
    d_mag = 0;
    d_input_power = 0.0;
    d_num_doppler_bins = 0;
    d_bit_transition_flag = bit_transition_flag;
    d_use_CFAR_algorithm_flag = use_CFAR_algorithm_flag;
    d_threshold = 0.0;
    d_doppler_step = 0;
    d_code_phase = 0;
    d_test_statistics = 0.0;
    d_channel = 0;
    d_doppler_freq = 0.0;

    //set_relative_rate( 1.0/d_fft_size );

    // COD:
    // Experimenting with the overlap/save technique for handling bit trannsitions
    // The problem: Circular correlation is asynchronous with the received code.
    // In effect the first code phase used in the correlation is the current
    // estimate of the code phase at the start of the input buffer. If this is 1/2
    // of the code period a bit transition would move all the signal energy into
    // adjacent frequency bands at +/- 1/T where T is the integration time.
    //
    // We can avoid this by doing linear correlation, effectively doubling the
    // size of the input buffer and padding the code with zeros.
    if( d_bit_transition_flag )
        {
            d_fft_size *= 2;
            d_max_dwells = 1; //Activation of d_bit_transition_flag invalidates the value of d_max_dwells
        }

    d_fft_codes = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_magnitude = static_cast<float*>(volk_gnsssdr_malloc(d_fft_size * sizeof(float), volk_gnsssdr_get_alignment()));

    // Direct FFT
    d_fft_if = new gr::fft::fft_complex(d_fft_size, true);

    // Inverse FFT
    d_ifft = new gr::fft::fft_complex(d_fft_size, false);

    // For dumping samples into a file
    d_dump = dump;
    d_dump_filename = dump_filename;

    d_gnss_synchro = 0;
    d_grid_doppler_wipeoffs = 0;
}


pcps_acquisition_cc::~pcps_acquisition_cc()
{
    if (d_num_doppler_bins > 0)
        {
            for (unsigned int i = 0; i < d_num_doppler_bins; i++)
                {
                    volk_gnsssdr_free(d_grid_doppler_wipeoffs[i]);
                }
            delete[] d_grid_doppler_wipeoffs;
        }

    volk_gnsssdr_free(d_fft_codes);
    volk_gnsssdr_free(d_magnitude);

    delete d_ifft;
    delete d_fft_if;

    if (d_dump)
        {
            d_dump_file.close();
        }
}


void pcps_acquisition_cc::set_local_code(std::complex<float> * code)
{
    // COD
    // Here we want to create a buffer that looks like this:
    // [ 0 0 0 ... 0 c_0 c_1 ... c_L]
    // where c_i is the local code and there are L zeros and L chips
    gr::thread::scoped_lock lock(d_setlock); // require mutex with work function called by the scheduler
    if( d_bit_transition_flag )
        {
            int offset = d_fft_size/2;
            std::fill_n( d_fft_if->get_inbuf(), offset, gr_complex( 0.0, 0.0 ) );
            memcpy(d_fft_if->get_inbuf() + offset, code, sizeof(gr_complex) * offset);
        } 
    else 
        {
            memcpy(d_fft_if->get_inbuf(), code, sizeof(gr_complex) * d_fft_size);
        }
    
    d_fft_if->execute(); // We need the FFT of local code
    volk_32fc_conjugate_32fc(d_fft_codes, d_fft_if->get_outbuf(), d_fft_size);
}


void pcps_acquisition_cc::update_local_carrier(gr_complex* carrier_vector, int correlator_length_samples, float freq)
{
    float phase_step_rad = GPS_TWO_PI * freq / static_cast<float>(d_fs_in);
    float _phase[1];
    _phase[0] = 0;
    volk_gnsssdr_s32f_sincos_32fc(carrier_vector, - phase_step_rad, _phase, correlator_length_samples);
}


void pcps_acquisition_cc::init()
{
    d_gnss_synchro->Flag_valid_acquisition = false;
    d_gnss_synchro->Flag_valid_symbol_output = false;
    d_gnss_synchro->Flag_valid_pseudorange = false;
    d_gnss_synchro->Flag_valid_word = false;

    d_gnss_synchro->Acq_delay_samples = 0.0;
    d_gnss_synchro->Acq_doppler_hz = 0.0;
    d_gnss_synchro->Acq_samplestamp_samples = 0;
    d_mag = 0.0;
    d_input_power = 0.0;

    d_num_doppler_bins = ceil( static_cast<double>(static_cast<int>(d_doppler_max) - static_cast<int>(-d_doppler_max)) / static_cast<double>(d_doppler_step));

    // Create the carrier Doppler wipeoff signals
    d_grid_doppler_wipeoffs = new gr_complex*[d_num_doppler_bins];

    for (unsigned int doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            d_grid_doppler_wipeoffs[doppler_index] = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
            int doppler = -static_cast<int>(d_doppler_max) + d_doppler_step * doppler_index;
            update_local_carrier(d_grid_doppler_wipeoffs[doppler_index], d_fft_size, d_freq + doppler);
        }
}


void pcps_acquisition_cc::set_state(int state)
{
    gr::thread::scoped_lock lock(d_setlock); // require mutex with work function called by the scheduler
    d_state = state;
    if (d_state == 1)
        {
            d_gnss_synchro->Acq_delay_samples = 0.0;
            d_gnss_synchro->Acq_doppler_hz = 0.0;
            d_gnss_synchro->Acq_samplestamp_samples = 0;
            d_well_count = 0;
            d_mag = 0.0;
            d_input_power = 0.0;
            d_test_statistics = 0.0;
        }
    else if (d_state == 0)
        {}
    else
        {
            LOG(ERROR) << "State can only be set to 0 or 1";
        }
}


void pcps_acquisition_cc::send_positive_acquisition()
{
    // 6.1- Declare positive acquisition using a message port
    //0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL
    DLOG(INFO) << "positive acquisition"
    << "satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
    << "sample_stamp " << d_sample_counter
    << "test statistics value " << d_test_statistics
    << "test statistics threshold " << d_threshold
    << "code phase " << d_gnss_synchro->Acq_delay_samples
    << "doppler " << d_gnss_synchro->Acq_doppler_hz
    << "magnitude " << d_mag
    << "input signal power " << d_input_power;

    this->message_port_pub(pmt::mp("events"), pmt::from_long(1));

}

void pcps_acquisition_cc::send_negative_acquisition()
{
    // 6.2- Declare negative acquisition using a message port
    //0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL
    DLOG(INFO) << "negative acquisition"
    << "satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
    << "sample_stamp " << d_sample_counter
    << "test statistics value " << d_test_statistics
    << "test statistics threshold " << d_threshold
    << "code phase " << d_gnss_synchro->Acq_delay_samples
    << "doppler " << d_gnss_synchro->Acq_doppler_hz
    << "magnitude " << d_mag
    << "input signal power " << d_input_power;

    this->message_port_pub(pmt::mp("events"), pmt::from_long(2));

}

int pcps_acquisition_cc::general_work(int noutput_items,
        gr_vector_int &ninput_items, gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items __attribute__((unused)))
{
    /*
     * By J.Arribas, L.Esteve and M.Molina
     * Acquisition strategy (Kay Borre book + CFAR threshold):
     * 1. Compute the input signal power estimation
     * 2. Doppler serial search loop
     * 3. Perform the FFT-based circular convolution (parallel time search)
     * 4. Record the maximum peak and the associated synchronization parameters
     * 5. Compute the test statistics and compare to the threshold
     * 6. Declare positive or negative acquisition using a message port
     */

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

            d_sample_counter += d_fft_size * ninput_items[0]; // sample counter
            consume_each(ninput_items[0]);

            break;
        }

    case 1:
        {
            // initialize acquisition algorithm
            int doppler;
            uint32_t indext = 0;
            float magt = 0.0;
            const gr_complex *in = (const gr_complex *)input_items[0]; //Get the input samples pointer

            int effective_fft_size = ( d_bit_transition_flag ? d_fft_size/2 : d_fft_size );

            float fft_normalization_factor = static_cast<float>(d_fft_size) * static_cast<float>(d_fft_size);

            d_input_power = 0.0;
            d_mag = 0.0;
            d_sample_counter += d_fft_size; // sample counter
            d_well_count++;

            DLOG(INFO)<< "Channel: " << d_channel
                    << " , doing acquisition of satellite: " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
                    << " ,sample stamp: " << d_sample_counter << ", threshold: "
                    << d_threshold << ", doppler_max: " << d_doppler_max
                    << ", doppler_step: " << d_doppler_step<<std::endl;

            if (d_use_CFAR_algorithm_flag == true)
                {
                    // 1- (optional) Compute the input signal power estimation
                    volk_32fc_magnitude_squared_32f(d_magnitude, in, d_fft_size);
                    volk_32f_accumulator_s32f(&d_input_power, d_magnitude, d_fft_size);
                    d_input_power /= static_cast<float>(d_fft_size);
                }
            // 2- Doppler frequency search loop
            for (unsigned int doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
                {
                    // doppler search steps
                    doppler = -static_cast<int>(d_doppler_max) + d_doppler_step * doppler_index;

                    volk_32fc_x2_multiply_32fc(d_fft_if->get_inbuf(), in,
                            d_grid_doppler_wipeoffs[doppler_index], d_fft_size);

                    // 3- Perform the FFT-based convolution  (parallel time search)
                    // Compute the FFT of the carrier wiped--off incoming signal
                    d_fft_if->execute();

                    // Multiply carrier wiped--off, Fourier transformed incoming signal
                    // with the local FFT'd code reference using SIMD operations with VOLK library
                    volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(),
                            d_fft_if->get_outbuf(), d_fft_codes, d_fft_size);

                    // compute the inverse FFT
                    d_ifft->execute();

                    // Search maximum
                    size_t offset = ( d_bit_transition_flag ? effective_fft_size : 0 );
                    volk_32fc_magnitude_squared_32f(d_magnitude, d_ifft->get_outbuf() + offset, effective_fft_size);
                    volk_gnsssdr_32f_index_max_32u(&indext, d_magnitude, effective_fft_size);
                    magt = d_magnitude[indext];

                    if (d_use_CFAR_algorithm_flag == true)
                        {
                            // Normalize the maximum value to correct the scale factor introduced by FFTW
                            magt = d_magnitude[indext] / (fft_normalization_factor * fft_normalization_factor);
                        }
                    // 4- record the maximum peak and the associated synchronization parameters
                    if (d_mag < magt)
                        {
                            d_mag = magt;

                            if (d_use_CFAR_algorithm_flag == false)
                                {
                                    // Search grid noise floor approximation for this doppler line
                                    volk_32f_accumulator_s32f(&d_input_power, d_magnitude, effective_fft_size);
                                    d_input_power = (d_input_power - d_mag) / (effective_fft_size - 1);
                                }

                            // In case that d_bit_transition_flag = true, we compare the potentially
                            // new maximum test statistics (d_mag/d_input_power) with the value in
                            // d_test_statistics. When the second dwell is being processed, the value
                            // of d_mag/d_input_power could be lower than d_test_statistics (i.e,
                            // the maximum test statistics in the previous dwell is greater than
                            // current d_mag/d_input_power). Note that d_test_statistics is not
                            // restarted between consecutive dwells in multidwell operation.

                            if (d_test_statistics < (d_mag / d_input_power) || !d_bit_transition_flag)
                                {
                                    d_gnss_synchro->Acq_delay_samples = static_cast<double>(indext % d_samples_per_code);
                                    d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
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
                            std::streamsize n = 2 * sizeof(float) * (d_fft_size); // complex file write
                            filename.str("");

                            boost::filesystem::path p = d_dump_filename;
                            filename << p.parent_path().string()
                                     << boost::filesystem::path::preferred_separator
                                     << p.stem().string()
                                     << "_" << d_gnss_synchro->System
                                     <<"_" << d_gnss_synchro->Signal << "_sat_"
                                     << d_gnss_synchro->PRN << "_doppler_"
                                     <<  doppler
                                     << p.extension().string();

                            DLOG(INFO) << "Writing ACQ out to " << filename.str();

                            d_dump_file.open(filename.str().c_str(), std::ios::out | std::ios::binary);
                            d_dump_file.write((char*)d_ifft->get_outbuf(), n); //write directly |abs(x)|^2 in this Doppler bin?
                            d_dump_file.close();
                        }
                }

            if (!d_bit_transition_flag)
                {
                    if (d_test_statistics > d_threshold)
                        {
                            d_state = 0; // Positive acquisition
                            d_active = false;
                            send_positive_acquisition();
                        }
                    else if (d_well_count == d_max_dwells)
                        {
                            d_state = 0;
                            d_active = false;
                            send_negative_acquisition();
                        }
                }
            else
                {
                    if (d_well_count == d_max_dwells) // d_max_dwells = 2
                        {
                            if (d_test_statistics > d_threshold)
                                {
                                    d_state = 0; // Positive acquisition
                                    d_active = false;
                                    send_positive_acquisition();
                                }
                            else
                                {
                                    d_state = 0; // Negative acquisition
                                    d_active = false;
                                    send_negative_acquisition();
                                }
                        }
                }

            consume_each(1);
            break;
        }

    }

    return noutput_items;
}


//void pcps_acquisition_cc::forecast (int noutput_items, gr_vector_int &ninput_items_required)
//{
    //// COD:
    //// For zero-padded case we need one extra code period
    //if( d_bit_transition_flag )
    //{
        //ninput_items_required[0] = noutput_items*(d_samples_per_code * d_max_dwells + d_samples_per_code);
    //}
    //else
    //{
        //ninput_items_required[0] = noutput_items*d_fft_size*d_max_dwells;
    //}
//}
