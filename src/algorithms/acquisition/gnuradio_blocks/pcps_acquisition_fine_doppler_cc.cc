/*!
 * \file pcps_acquisition_fine_doppler_acquisition_cc.cc
 * \brief This class implements a Parallel Code Phase Search Acquisition with multi-dwells and fine Doppler estimation
 * \authors <ul>
 *          <li> Javier Arribas, 2013. jarribas(at)cttc.es
 *          </ul>
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "pcps_acquisition_fine_doppler_cc.h"
#include "gps_sdr_signal_processing.h"
#include "control_message_factory.h"
#include "GPS_L1_CA.h"
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <algorithm>  // std::rotate, std::fill_n
#include <sstream>


using google::LogMessage;

pcps_acquisition_fine_doppler_cc_sptr pcps_make_acquisition_fine_doppler_cc(
    int max_dwells, unsigned int sampled_ms, int doppler_max, int doppler_min, long freq,
    long fs_in, int samples_per_ms, bool dump,
    std::string dump_filename)
{
    return pcps_acquisition_fine_doppler_cc_sptr(
        new pcps_acquisition_fine_doppler_cc(max_dwells, sampled_ms, doppler_max, doppler_min, freq,
            fs_in, samples_per_ms, dump, dump_filename));
}


pcps_acquisition_fine_doppler_cc::pcps_acquisition_fine_doppler_cc(
    int max_dwells, unsigned int sampled_ms, int doppler_max, int doppler_min, long freq,
    long fs_in, int samples_per_ms, bool dump,
    std::string dump_filename) : gr::block("pcps_acquisition_fine_doppler_cc",
                                     gr::io_signature::make(1, 1, sizeof(gr_complex)),
                                     gr::io_signature::make(0, 0, sizeof(gr_complex)))
{
    this->message_port_register_out(pmt::mp("events"));
    d_sample_counter = 0;  // SAMPLE COUNTER
    d_active = false;
    d_freq = freq;
    d_fs_in = fs_in;
    d_samples_per_ms = samples_per_ms;
    d_sampled_ms = sampled_ms;
    d_config_doppler_max = doppler_max;
    d_config_doppler_min = doppler_min;
    d_fft_size = d_sampled_ms * d_samples_per_ms;
    // HS Acquisition
    d_max_dwells = max_dwells;
    d_gnuradio_forecast_samples = d_fft_size;
    d_input_power = 0.0;
    d_state = 0;
    d_carrier = static_cast<gr_complex *>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_fft_codes = static_cast<gr_complex *>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_magnitude = static_cast<float *>(volk_gnsssdr_malloc(d_fft_size * sizeof(float), volk_gnsssdr_get_alignment()));

    // Direct FFT
    d_fft_if = new gr::fft::fft_complex(d_fft_size, true);

    // Inverse FFT
    d_ifft = new gr::fft::fft_complex(d_fft_size, false);

    // For dumping samples into a file
    d_dump = dump;
    d_dump_filename = dump_filename;

    d_doppler_resolution = 0;
    d_threshold = 0;
    d_num_doppler_points = 0;
    d_doppler_step = 0;
    d_grid_data = 0;
    d_grid_doppler_wipeoffs = 0;
    d_gnss_synchro = 0;
    d_code_phase = 0;
    d_doppler_freq = 0;
    d_test_statistics = 0;
    d_well_count = 0;
    d_channel = 0;
}


void pcps_acquisition_fine_doppler_cc::set_doppler_step(unsigned int doppler_step)
{
    d_doppler_step = doppler_step;
    // Create the search grid array

    d_num_doppler_points = floor(std::abs(d_config_doppler_max - d_config_doppler_min) / d_doppler_step);

    d_grid_data = new float *[d_num_doppler_points];
    for (int i = 0; i < d_num_doppler_points; i++)
        {
            d_grid_data[i] = static_cast<float *>(volk_gnsssdr_malloc(d_fft_size * sizeof(float), volk_gnsssdr_get_alignment()));
        }
    update_carrier_wipeoff();
}


void pcps_acquisition_fine_doppler_cc::free_grid_memory()
{
    for (int i = 0; i < d_num_doppler_points; i++)
        {
            volk_gnsssdr_free(d_grid_data[i]);
            delete[] d_grid_doppler_wipeoffs[i];
        }
    delete d_grid_data;
    delete d_grid_doppler_wipeoffs;
}


pcps_acquisition_fine_doppler_cc::~pcps_acquisition_fine_doppler_cc()
{
    volk_gnsssdr_free(d_carrier);
    volk_gnsssdr_free(d_fft_codes);
    volk_gnsssdr_free(d_magnitude);
    delete d_ifft;
    delete d_fft_if;
    if (d_dump)
        {
            d_dump_file.close();
        }
    free_grid_memory();
}


void pcps_acquisition_fine_doppler_cc::set_local_code(std::complex<float> *code)
{
    memcpy(d_fft_if->get_inbuf(), code, sizeof(gr_complex) * d_fft_size);
    d_fft_if->execute();  // We need the FFT of local code
    //Conjugate the local code
    volk_32fc_conjugate_32fc(d_fft_codes, d_fft_if->get_outbuf(), d_fft_size);
}


void pcps_acquisition_fine_doppler_cc::init()
{
    d_gnss_synchro->Flag_valid_acquisition = false;
    d_gnss_synchro->Flag_valid_symbol_output = false;
    d_gnss_synchro->Flag_valid_pseudorange = false;
    d_gnss_synchro->Flag_valid_word = false;

    d_gnss_synchro->Acq_delay_samples = 0.0;
    d_gnss_synchro->Acq_doppler_hz = 0.0;
    d_gnss_synchro->Acq_samplestamp_samples = 0;
    d_input_power = 0.0;
    d_state = 0;
}


void pcps_acquisition_fine_doppler_cc::forecast(int noutput_items,
    gr_vector_int &ninput_items_required)
{
    if (noutput_items != 0)
        {
            ninput_items_required[0] = d_gnuradio_forecast_samples;  //set the required available samples in each call
        }
}


void pcps_acquisition_fine_doppler_cc::reset_grid()
{
    d_well_count = 0;
    for (int i = 0; i < d_num_doppler_points; i++)
        {
            for (unsigned int j = 0; j < d_fft_size; j++)
                {
                    d_grid_data[i][j] = 0.0;
                }
        }
}


void pcps_acquisition_fine_doppler_cc::update_carrier_wipeoff()
{
    // create the carrier Doppler wipeoff signals
    int doppler_hz;
    float phase_step_rad;
    d_grid_doppler_wipeoffs = new gr_complex *[d_num_doppler_points];
    for (int doppler_index = 0; doppler_index < d_num_doppler_points; doppler_index++)
        {
            doppler_hz = d_config_doppler_min + d_doppler_step * doppler_index;
            // doppler search steps
            // compute the carrier doppler wipe-off signal and store it
            phase_step_rad = static_cast<float>(GPS_TWO_PI) * (d_freq + doppler_hz) / static_cast<float>(d_fs_in);
            d_grid_doppler_wipeoffs[doppler_index] = new gr_complex[d_fft_size];
            float _phase[1];
            _phase[0] = 0;
            volk_gnsssdr_s32f_sincos_32fc(d_grid_doppler_wipeoffs[doppler_index], -phase_step_rad, _phase, d_fft_size);
        }
}


double pcps_acquisition_fine_doppler_cc::search_maximum()
{
    float magt = 0.0;
    float fft_normalization_factor;
    int index_doppler = 0;
    uint32_t tmp_intex_t = 0;
    uint32_t index_time = 0;

    for (int i = 0; i < d_num_doppler_points; i++)
        {
            volk_gnsssdr_32f_index_max_32u(&tmp_intex_t, d_grid_data[i], d_fft_size);
            if (d_grid_data[i][tmp_intex_t] > magt)
                {
                    magt = d_grid_data[i][tmp_intex_t];
                    //std::cout<<magt<<std::endl;
                    index_doppler = i;
                    index_time = tmp_intex_t;
                }
        }

    // Normalize the maximum value to correct the scale factor introduced by FFTW
    fft_normalization_factor = static_cast<float>(d_fft_size) * static_cast<float>(d_fft_size);
    magt = magt / (fft_normalization_factor * fft_normalization_factor);

    // 5- Compute the test statistics and compare to the threshold
    d_test_statistics = magt / (d_input_power * std::sqrt(d_well_count));

    // 4- record the maximum peak and the associated synchronization parameters
    d_gnss_synchro->Acq_delay_samples = static_cast<double>(index_time);
    d_gnss_synchro->Acq_doppler_hz = static_cast<double>(index_doppler * d_doppler_step + d_config_doppler_min);
    d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter;

    // Record results to file if required
    if (d_dump)
        {
            std::stringstream filename;
            std::streamsize n = 2 * sizeof(float) * (d_fft_size);  // complex file write
            filename.str("");
            filename << "../data/test_statistics_" << d_gnss_synchro->System
                     << "_" << d_gnss_synchro->Signal << "_sat_"
                     << d_gnss_synchro->PRN << "_doppler_" << d_gnss_synchro->Acq_doppler_hz << ".dat";
            d_dump_file.open(filename.str().c_str(), std::ios::out | std::ios::binary);
            d_dump_file.write(reinterpret_cast<char *>(d_grid_data[index_doppler]), n);  //write directly |abs(x)|^2 in this Doppler bin?
            d_dump_file.close();
        }

    return d_test_statistics;
}


float pcps_acquisition_fine_doppler_cc::estimate_input_power(gr_vector_const_void_star &input_items)
{
    const gr_complex *in = reinterpret_cast<const gr_complex *>(input_items[0]);  //Get the input samples pointer
    // Compute the input signal power estimation
    float power = 0;
    volk_32fc_magnitude_squared_32f(d_magnitude, in, d_fft_size);
    volk_32f_accumulator_s32f(&power, d_magnitude, d_fft_size);
    power /= static_cast<float>(d_fft_size);
    return power;
}


int pcps_acquisition_fine_doppler_cc::compute_and_accumulate_grid(gr_vector_const_void_star &input_items)
{
    // initialize acquisition algorithm
    const gr_complex *in = reinterpret_cast<const gr_complex *>(input_items[0]);  //Get the input samples pointer

    DLOG(INFO) << "Channel: " << d_channel
               << " , doing acquisition of satellite: " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
               << " ,sample stamp: " << d_sample_counter << ", threshold: "
               << d_threshold << ", doppler_max: " << d_config_doppler_max
               << ", doppler_step: " << d_doppler_step;

    // 2- Doppler frequency search loop
    float *p_tmp_vector = static_cast<float *>(volk_gnsssdr_malloc(d_fft_size * sizeof(float), volk_gnsssdr_get_alignment()));

    for (int doppler_index = 0; doppler_index < d_num_doppler_points; doppler_index++)
        {
            // doppler search steps
            // Perform the carrier wipe-off
            volk_32fc_x2_multiply_32fc(d_fft_if->get_inbuf(), in, d_grid_doppler_wipeoffs[doppler_index], d_fft_size);
            // 3- Perform the FFT-based convolution  (parallel time search)
            // Compute the FFT of the carrier wiped--off incoming signal
            d_fft_if->execute();

            // Multiply carrier wiped--off, Fourier transformed incoming signal
            // with the local FFT'd code reference using SIMD operations with VOLK library
            volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(), d_fft_if->get_outbuf(), d_fft_codes, d_fft_size);

            // compute the inverse FFT
            d_ifft->execute();

            // save the grid matrix delay file

            volk_32fc_magnitude_squared_32f(p_tmp_vector, d_ifft->get_outbuf(), d_fft_size);
            const float *old_vector = d_grid_data[doppler_index];
            volk_32f_x2_add_32f(d_grid_data[doppler_index], old_vector, p_tmp_vector, d_fft_size);
        }

    volk_gnsssdr_free(p_tmp_vector);
    return d_fft_size;
}


int pcps_acquisition_fine_doppler_cc::estimate_Doppler(gr_vector_const_void_star &input_items)
{
    // Direct FFT
    int zero_padding_factor = 2;
    int fft_size_extended = d_fft_size * zero_padding_factor;
    gr::fft::fft_complex *fft_operator = new gr::fft::fft_complex(fft_size_extended, true);

    //zero padding the entire vector
    std::fill_n(fft_operator->get_inbuf(), fft_size_extended, gr_complex(0.0, 0.0));

    //1. generate local code aligned with the acquisition code phase estimation
    gr_complex *code_replica = static_cast<gr_complex *>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));

    gps_l1_ca_code_gen_complex_sampled(code_replica, d_gnss_synchro->PRN, d_fs_in, 0);

    int shift_index = static_cast<int>(d_gnss_synchro->Acq_delay_samples);

    // Rotate to align the local code replica using acquisition time delay estimation
    if (shift_index != 0)
        {
            std::rotate(code_replica, code_replica + (d_fft_size - shift_index), code_replica + d_fft_size - 1);
        }

    //2. Perform code wipe-off
    const gr_complex *in = reinterpret_cast<const gr_complex *>(input_items[0]);  //Get the input samples pointer

    volk_32fc_x2_multiply_32fc(fft_operator->get_inbuf(), in, code_replica, d_fft_size);

    // 3. Perform the FFT (zero padded!)
    fft_operator->execute();

    // 4. Compute the magnitude and find the maximum
    float *p_tmp_vector = static_cast<float *>(volk_gnsssdr_malloc(fft_size_extended * sizeof(float), volk_gnsssdr_get_alignment()));

    volk_32fc_magnitude_squared_32f(p_tmp_vector, fft_operator->get_outbuf(), fft_size_extended);

    uint32_t tmp_index_freq = 0;
    volk_gnsssdr_32f_index_max_32u(&tmp_index_freq, p_tmp_vector, fft_size_extended);

    //case even
    int counter = 0;

    float fftFreqBins[fft_size_extended];
    std::fill_n(fftFreqBins, fft_size_extended, 0.0);

    for (int k = 0; k < (fft_size_extended / 2); k++)
        {
            fftFreqBins[counter] = ((static_cast<float>(d_fs_in) / 2.0) * static_cast<float>(k)) / (static_cast<float>(fft_size_extended) / 2.0);
            counter++;
        }

    for (int k = fft_size_extended / 2; k > 0; k--)
        {
            fftFreqBins[counter] = ((-static_cast<float>(d_fs_in) / 2) * static_cast<float>(k)) / (static_cast<float>(fft_size_extended) / 2.0);
            counter++;
        }

    // 5. Update the Doppler estimation in Hz
    if (std::abs(fftFreqBins[tmp_index_freq] - d_gnss_synchro->Acq_doppler_hz) < 1000)
        {
            d_gnss_synchro->Acq_doppler_hz = static_cast<double>(fftFreqBins[tmp_index_freq]);
            //std::cout<<"FFT maximum present at "<<fftFreqBins[tmp_index_freq]<<" [Hz]"<<std::endl;
        }
    else
        {
            DLOG(INFO) << "Abs(Grid Doppler - FFT Doppler)=" << std::abs(fftFreqBins[tmp_index_freq] - d_gnss_synchro->Acq_doppler_hz);
            DLOG(INFO) << "Error estimating fine frequency Doppler";
            //debug log
            //
            //        std::cout<<"FFT maximum present at "<<fftFreqBins[tmp_index_freq]<<" [Hz]"<<std::endl;
            //        std::stringstream filename;
            //        std::streamsize n = sizeof(gr_complex) * (d_fft_size);
            //
            //        filename.str("");
            //        filename << "../data/code_prn_" << d_gnss_synchro->PRN << ".dat";
            //        d_dump_file.open(filename.str().c_str(), std::ios::out
            //                | std::ios::binary);
            //        d_dump_file.write(reinterpret_cast<char*>(code_replica), n); //write directly |abs(x)|^2 in this Doppler bin?
            //        d_dump_file.close();
            //
            //        filename.str("");
            //        filename << "../data/signal_prn_" << d_gnss_synchro->PRN << ".dat";
            //        d_dump_file.open(filename.str().c_str(), std::ios::out
            //                | std::ios::binary);
            //        d_dump_file.write(reinterpret_cast<char*>(in), n); //write directly |abs(x)|^2 in this Doppler bin?
            //        d_dump_file.close();
            //
            //
            //        n = sizeof(float) * (fft_size_extended);
            //        filename.str("");
            //        filename << "../data/fft_prn_" << d_gnss_synchro->PRN << ".dat";
            //        d_dump_file.open(filename.str().c_str(), std::ios::out
            //                | std::ios::binary);
            //        d_dump_file.write(reinterpret_cast<char*>(p_tmp_vector), n); //write directly |abs(x)|^2 in this Doppler bin?
            //        d_dump_file.close();
        }

    // free memory!!
    delete fft_operator;
    volk_gnsssdr_free(code_replica);
    volk_gnsssdr_free(p_tmp_vector);
    return d_fft_size;
}


int pcps_acquisition_fine_doppler_cc::general_work(int noutput_items,
    gr_vector_int &ninput_items __attribute__((unused)), gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items __attribute__((unused)))
{
    /*!
     * TODO:     High sensitivity acquisition algorithm:
     *             State Mechine:
     *             S0. StandBy. If d_active==1 -> S1
     *             S1. ComputeGrid. Perform the FFT acqusition doppler and delay grid.
     *                 Accumulate the search grid matrix (#doppler_bins x #fft_size)
     *                 Compare maximum to threshold and decide positive or negative
     *                 If T>=gamma -> S4 else
     *                 If d_well_count<max_dwells -> S2
     *                 else -> S5.
     *             S4. Positive_Acq: Send message and stop acq -> S0
     *             S5. Negative_Acq: Send message and stop acq -> S0
     */

    switch (d_state)
        {
        case 0:  // S0. StandBy
            //DLOG(INFO) <<"S0"<<std::endl;
            if (d_active == true)
                {
                    reset_grid();
                    d_state = 1;
                }
            break;
        case 1:  // S1. ComputeGrid
            //DLOG(INFO) <<"S1"<<std::endl;
            compute_and_accumulate_grid(input_items);
            d_well_count++;
            if (d_well_count >= d_max_dwells)
                {
                    d_state = 2;
                }
            break;
        case 2:  // Compute test statistics and decide
            //DLOG(INFO) <<"S2"<<std::endl;
            d_input_power = estimate_input_power(input_items);
            d_test_statistics = search_maximum();
            if (d_test_statistics > d_threshold)
                {
                    d_state = 3;  //perform fine doppler estimation
                }
            else
                {
                    d_state = 5;  //negative acquisition
                }
            break;
        case 3:  // Fine doppler estimation
            //DLOG(INFO) <<"S3"<<std::endl;
            DLOG(INFO) << "Performing fine Doppler estimation";
            estimate_Doppler(input_items);  //disabled in repo
            d_state = 4;
            break;
        case 4:  // Positive_Acq
            //DLOG(INFO) <<"S4"<<std::endl;
            DLOG(INFO) << "positive acquisition";
            DLOG(INFO) << "satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;
            DLOG(INFO) << "sample_stamp " << d_sample_counter;
            DLOG(INFO) << "test statistics value " << d_test_statistics;
            DLOG(INFO) << "test statistics threshold " << d_threshold;
            DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
            DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
            DLOG(INFO) << "input signal power " << d_input_power;

            d_active = false;
            // Send message to channel port //0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL
            this->message_port_pub(pmt::mp("events"), pmt::from_long(1));
            d_state = 0;
            break;
        case 5:  // Negative_Acq
            //DLOG(INFO) <<"S5"<<std::endl;
            DLOG(INFO) << "negative acquisition";
            DLOG(INFO) << "satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;
            DLOG(INFO) << "sample_stamp " << d_sample_counter;
            DLOG(INFO) << "test statistics value " << d_test_statistics;
            DLOG(INFO) << "test statistics threshold " << d_threshold;
            DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
            DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
            DLOG(INFO) << "input signal power " << d_input_power;

            d_active = false;
            // Send message to channel port //0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL
            this->message_port_pub(pmt::mp("events"), pmt::from_long(2));
            d_state = 0;
            break;
        default:
            d_state = 0;
            break;
        }

    //DLOG(INFO)<<"d_sample_counter="<<d_sample_counter<<std::endl;
    d_sample_counter += d_fft_size;  // sample counter
    consume_each(d_fft_size);
    return noutput_items;
}
