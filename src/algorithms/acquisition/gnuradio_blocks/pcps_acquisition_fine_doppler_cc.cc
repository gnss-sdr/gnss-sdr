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
#include <matio.h>


using google::LogMessage;

pcps_acquisition_fine_doppler_cc_sptr pcps_make_acquisition_fine_doppler_cc(const Acq_Conf &conf_)
{
    return pcps_acquisition_fine_doppler_cc_sptr(
        new pcps_acquisition_fine_doppler_cc(conf_));
}


pcps_acquisition_fine_doppler_cc::pcps_acquisition_fine_doppler_cc(const Acq_Conf &conf_)
    : gr::block("pcps_acquisition_fine_doppler_cc",
          gr::io_signature::make(1, 1, sizeof(gr_complex)),
          gr::io_signature::make(0, 0, sizeof(gr_complex)))
{
    this->message_port_register_out(pmt::mp("events"));
    acq_parameters = conf_;
    d_sample_counter = 0;  // SAMPLE COUNTER
    d_active = false;
    d_fs_in = conf_.fs_in;
    d_samples_per_ms = conf_.samples_per_ms;
    d_sampled_ms = conf_.sampled_ms;
    d_config_doppler_max = conf_.doppler_max;
    d_config_doppler_min = -conf_.doppler_max;
    d_fft_size = d_sampled_ms * d_samples_per_ms;
    // HS Acquisition
    d_max_dwells = conf_.max_dwells;
    d_gnuradio_forecast_samples = d_fft_size;
    d_state = 0;
    d_carrier = static_cast<gr_complex *>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_fft_codes = static_cast<gr_complex *>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_magnitude = static_cast<float *>(volk_gnsssdr_malloc(d_fft_size * sizeof(float), volk_gnsssdr_get_alignment()));

    d_10_ms_buffer = static_cast<gr_complex *>(volk_gnsssdr_malloc(10 * d_samples_per_ms * sizeof(gr_complex), volk_gnsssdr_get_alignment()));

    // Direct FFT
    d_fft_if = new gr::fft::fft_complex(d_fft_size, true);

    // Inverse FFT
    d_ifft = new gr::fft::fft_complex(d_fft_size, false);

    // For dumping samples into a file
    d_dump = conf_.dump;
    d_dump_filename = conf_.dump_filename;

    d_n_samples_in_buffer = 0;
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
    d_positive_acq = 0;
    d_dump_number = 0;
    d_dump_channel = 0;  // this implementation can only produce dumps in channel 0
    //todo: migrate config parameters to the unified acquisition config class
}


// Finds next power of two
// for n. If n itself is a
// power of two then returns n
unsigned int pcps_acquisition_fine_doppler_cc::nextPowerOf2(unsigned int n)
{
    n--;
    n |= n >> 1;
    n |= n >> 2;
    n |= n >> 4;
    n |= n >> 8;
    n |= n >> 16;
    n++;
    return n;
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
    volk_gnsssdr_free(d_10_ms_buffer);
    delete d_ifft;
    delete d_fft_if;
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
    d_state = 0;

    if (d_dump)
        {
            grid_ = arma::fmat(d_fft_size, d_num_doppler_points, arma::fill::zeros);
        }
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
            //todo: use memset here
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
            phase_step_rad = static_cast<float>(GPS_TWO_PI) * doppler_hz / static_cast<float>(d_fs_in);
            d_grid_doppler_wipeoffs[doppler_index] = new gr_complex[d_fft_size];
            float _phase[1];
            _phase[0] = 0;
            volk_gnsssdr_s32f_sincos_32fc(d_grid_doppler_wipeoffs[doppler_index], -phase_step_rad, _phase, d_fft_size);
        }
}


double pcps_acquisition_fine_doppler_cc::compute_CAF()
{
    float firstPeak = 0.0;
    int index_doppler = 0;
    uint32_t tmp_intex_t = 0;
    uint32_t index_time = 0;

    // Look for correlation peaks in the results ==============================
    // Find the highest peak and compare it to the second highest peak
    // The second peak is chosen not closer than 1 chip to the highest peak
    //--- Find the correlation peak and the carrier frequency --------------
    for (int i = 0; i < d_num_doppler_points; i++)
        {
            volk_gnsssdr_32f_index_max_32u(&tmp_intex_t, d_grid_data[i], d_fft_size);
            if (d_grid_data[i][tmp_intex_t] > firstPeak)
                {
                    firstPeak = d_grid_data[i][tmp_intex_t];
                    index_doppler = i;
                    index_time = tmp_intex_t;
                }

            // Record results to file if required
            if (d_dump and d_channel == d_dump_channel)
                {
                    memcpy(grid_.colptr(i), d_grid_data[i], sizeof(float) * d_fft_size);
                }
        }

    // -- - Find 1 chip wide code phase exclude range around the peak
    uint32_t samplesPerChip = ceil(GPS_L1_CA_CHIP_PERIOD * static_cast<float>(this->d_fs_in));
    int32_t excludeRangeIndex1 = index_time - samplesPerChip;
    int32_t excludeRangeIndex2 = index_time + samplesPerChip;

    // -- - Correct code phase exclude range if the range includes array boundaries
    if (excludeRangeIndex1 < 0)
        {
            excludeRangeIndex1 = d_fft_size + excludeRangeIndex1;
        }
    else if (excludeRangeIndex2 >= static_cast<int>(d_fft_size))
        {
            excludeRangeIndex2 = excludeRangeIndex2 - d_fft_size;
        }

    int32_t idx = excludeRangeIndex1;
    do
        {
            d_grid_data[index_doppler][idx] = 0.0;
            idx++;
            if (idx == static_cast<int>(d_fft_size)) idx = 0;
        }
    while (idx != excludeRangeIndex2);

    //--- Find the second highest correlation peak in the same freq. bin ---
    volk_gnsssdr_32f_index_max_32u(&tmp_intex_t, d_grid_data[index_doppler], d_fft_size);
    float secondPeak = d_grid_data[index_doppler][tmp_intex_t];

    // 5- Compute the test statistics and compare to the threshold
    d_test_statistics = firstPeak / secondPeak;

    // 4- record the maximum peak and the associated synchronization parameters
    d_gnss_synchro->Acq_delay_samples = static_cast<double>(index_time);
    d_gnss_synchro->Acq_doppler_hz = static_cast<double>(index_doppler * d_doppler_step + d_config_doppler_min);
    d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter;

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
            //accumulate grid values
            volk_32f_x2_add_32f(d_grid_data[doppler_index], d_grid_data[doppler_index], p_tmp_vector, d_fft_size);
        }

    volk_gnsssdr_free(p_tmp_vector);
    return d_fft_size;
}


int pcps_acquisition_fine_doppler_cc::estimate_Doppler()
{
    // Direct FFT
    int zero_padding_factor = 8;
    int prn_replicas = 10;
    int signal_samples = prn_replicas * d_fft_size;
    //int fft_size_extended = nextPowerOf2(signal_samples * zero_padding_factor);
    int fft_size_extended = signal_samples * zero_padding_factor;
    gr::fft::fft_complex *fft_operator = new gr::fft::fft_complex(fft_size_extended, true);

    //zero padding the entire vector
    std::fill_n(fft_operator->get_inbuf(), fft_size_extended, gr_complex(0.0, 0.0));

    //1. generate local code aligned with the acquisition code phase estimation
    gr_complex *code_replica = static_cast<gr_complex *>(volk_gnsssdr_malloc(signal_samples * sizeof(gr_complex), volk_gnsssdr_get_alignment()));

    gps_l1_ca_code_gen_complex_sampled(code_replica, d_gnss_synchro->PRN, d_fs_in, 0);

    int shift_index = static_cast<int>(d_gnss_synchro->Acq_delay_samples);

    // Rotate to align the local code replica using acquisition time delay estimation
    if (shift_index != 0)
        {
            std::rotate(code_replica, code_replica + (d_fft_size - shift_index), code_replica + d_fft_size - 1);
        }

    for (int n = 0; n < prn_replicas - 1; n++)
        {
            memcpy(&code_replica[(n + 1) * d_fft_size], code_replica, d_fft_size * sizeof(gr_complex));
        }
    //2. Perform code wipe-off

    volk_32fc_x2_multiply_32fc(fft_operator->get_inbuf(), d_10_ms_buffer, code_replica, signal_samples);

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
            //std::cout << "FFT maximum present at " << fftFreqBins[tmp_index_freq] << " [Hz]" << std::endl;
        }
    else
        {
            DLOG(INFO) << "Abs(Grid Doppler - FFT Doppler)=" << std::abs(fftFreqBins[tmp_index_freq] - d_gnss_synchro->Acq_doppler_hz);
            DLOG(INFO) << "Error estimating fine frequency Doppler";
        }

    // free memory!!
    delete fft_operator;
    volk_gnsssdr_free(code_replica);
    volk_gnsssdr_free(p_tmp_vector);
    return d_fft_size;
}


// Called by gnuradio to enable drivers, etc for i/o devices.
bool pcps_acquisition_fine_doppler_cc::start()
{
    d_sample_counter = 0;
    return true;
}


void pcps_acquisition_fine_doppler_cc::set_state(int state)
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with work function called by the scheduler
    d_state = state;
    if (d_state == 1)
        {
            d_gnss_synchro->Acq_delay_samples = 0.0;
            d_gnss_synchro->Acq_doppler_hz = 0.0;
            d_gnss_synchro->Acq_samplestamp_samples = 0;
            d_well_count = 0;
            d_test_statistics = 0.0;
            d_active = true;
        }
    else if (d_state == 0)
        {
        }
    else
        {
            LOG(ERROR) << "State can only be set to 0 or 1";
        }
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

    int samples_remaining;
    switch (d_state)
        {
        case 0:  // S0. StandBy
            if (d_active == true)
                {
                    reset_grid();
                    d_state = 1;
                }
            if (!acq_parameters.blocking_on_standby)
                {
                    d_sample_counter += d_fft_size;  // sample counter
                    consume_each(d_fft_size);
                }
            break;
        case 1:  // S1. ComputeGrid
            compute_and_accumulate_grid(input_items);
            d_well_count++;
            if (d_well_count >= d_max_dwells)
                {
                    d_state = 2;
                }
            d_sample_counter += d_fft_size;  // sample counter
            consume_each(d_fft_size);
            break;
        case 2:  // Compute test statistics and decide
            d_test_statistics = compute_CAF();
            if (d_test_statistics > d_threshold)
                {
                    d_state = 3;  //perform fine doppler estimation
                }
            else
                {
                    d_state = 5;  //negative acquisition
                }
            d_n_samples_in_buffer = 0;
            // Record results to file if required
            if (d_dump and d_channel == d_dump_channel)
                {
                    dump_results(d_fft_size);
                }
            d_sample_counter += d_fft_size;  // sample counter
            consume_each(d_fft_size);
            break;
        case 3:  // Fine doppler estimation
            samples_remaining = 10 * d_samples_per_ms - d_n_samples_in_buffer;

            if (samples_remaining > noutput_items)
                {
                    memcpy(&d_10_ms_buffer[d_n_samples_in_buffer], reinterpret_cast<const gr_complex *>(input_items[0]), noutput_items * sizeof(gr_complex));
                    d_n_samples_in_buffer += noutput_items;
                    d_sample_counter += noutput_items;  // sample counter
                    consume_each(noutput_items);
                }
            else
                {
                    memcpy(&d_10_ms_buffer[d_n_samples_in_buffer], reinterpret_cast<const gr_complex *>(input_items[0]), samples_remaining * sizeof(gr_complex));
                    estimate_Doppler();                     //disabled in repo
                    d_sample_counter += samples_remaining;  // sample counter
                    consume_each(samples_remaining);
                    d_state = 4;
                }
            break;
        case 4:  // Positive_Acq
            DLOG(INFO) << "positive acquisition";
            DLOG(INFO) << "satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;
            DLOG(INFO) << "sample_stamp " << d_sample_counter;
            DLOG(INFO) << "test statistics value " << d_test_statistics;
            DLOG(INFO) << "test statistics threshold " << d_threshold;
            DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
            DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
            d_positive_acq = 1;
            d_active = false;
            // Send message to channel port //0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL
            this->message_port_pub(pmt::mp("events"), pmt::from_long(1));
            d_state = 0;
            if (!acq_parameters.blocking_on_standby)
                {
                    d_sample_counter += noutput_items;  // sample counter
                    consume_each(noutput_items);
                }
            break;
        case 5:  // Negative_Acq
            DLOG(INFO) << "negative acquisition";
            DLOG(INFO) << "satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;
            DLOG(INFO) << "sample_stamp " << d_sample_counter;
            DLOG(INFO) << "test statistics value " << d_test_statistics;
            DLOG(INFO) << "test statistics threshold " << d_threshold;
            DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
            DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
            d_positive_acq = 0;
            d_active = false;
            // Send message to channel port //0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL
            this->message_port_pub(pmt::mp("events"), pmt::from_long(2));
            d_state = 0;
            if (!acq_parameters.blocking_on_standby)
                {
                    d_sample_counter += noutput_items;  // sample counter
                    consume_each(noutput_items);
                }
            break;
        default:
            d_state = 0;
            if (!acq_parameters.blocking_on_standby)
                {
                    d_sample_counter += noutput_items;  // sample counter
                    consume_each(noutput_items);
                }
            break;
        }
    return 0;
}

void pcps_acquisition_fine_doppler_cc::dump_results(int effective_fft_size)
{
    d_dump_number++;
    std::string filename = d_dump_filename;
    filename.append("_");
    filename.append(1, d_gnss_synchro->System);
    filename.append("_");
    filename.append(1, d_gnss_synchro->Signal[0]);
    filename.append(1, d_gnss_synchro->Signal[1]);
    filename.append("_ch_");
    filename.append(std::to_string(d_channel));
    filename.append("_");
    filename.append(std::to_string(d_dump_number));
    filename.append("_sat_");
    filename.append(std::to_string(d_gnss_synchro->PRN));
    filename.append(".mat");

    mat_t *matfp = Mat_CreateVer(filename.c_str(), NULL, MAT_FT_MAT73);
    if (matfp == NULL)
        {
            std::cout << "Unable to create or open Acquisition dump file" << std::endl;
            d_dump = false;
        }
    else
        {
            size_t dims[2] = {static_cast<size_t>(effective_fft_size), static_cast<size_t>(d_num_doppler_points)};
            matvar_t *matvar = Mat_VarCreate("acq_grid", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, grid_.memptr(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            dims[0] = static_cast<size_t>(1);
            dims[1] = static_cast<size_t>(1);
            matvar = Mat_VarCreate("doppler_max", MAT_C_UINT32, MAT_T_UINT32, 1, dims, &d_config_doppler_max, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("doppler_step", MAT_C_UINT32, MAT_T_UINT32, 1, dims, &d_doppler_step, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("d_positive_acq", MAT_C_INT32, MAT_T_INT32, 1, dims, &d_positive_acq, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            float aux = static_cast<float>(d_gnss_synchro->Acq_doppler_hz);
            matvar = Mat_VarCreate("acq_doppler_hz", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims, &aux, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            aux = static_cast<float>(d_gnss_synchro->Acq_delay_samples);
            matvar = Mat_VarCreate("acq_delay_samples", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims, &aux, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("test_statistic", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims, &d_test_statistics, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("threshold", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims, &d_threshold, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);
            aux = 0.0;
            matvar = Mat_VarCreate("input_power", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims, &aux, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("sample_counter", MAT_C_UINT64, MAT_T_UINT64, 1, dims, &d_sample_counter, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("PRN", MAT_C_UINT32, MAT_T_UINT32, 1, dims, &d_gnss_synchro->PRN, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            Mat_Close(matfp);
        }
}
