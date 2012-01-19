/*!
 * \file gps_l1_ca_gps_sdr_acquisition_cc.cc
 * \brief Brief description of the file here
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *
 * Detailed description of the file here if needed.
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


#include "gps_l1_ca_gps_sdr_acquisition_cc.h"
#include "control_message_factory.h"
#include "gps_sdr_x86.h"
#include <gnuradio/gr_io_signature.h>
#include <sstream>
#include <glog/log_severity.h>
#include <glog/logging.h>
#include <iostream>

using google::LogMessage;

gps_l1_ca_gps_sdr_acquisition_cc_sptr gps_l1_ca_gps_sdr_make_acquisition_cc(
        unsigned int sampled_ms, long freq, long fs_in, int samples_per_ms,
        gr_msg_queue_sptr queue, bool dump, std::string dump_filename)
{

    return gps_l1_ca_gps_sdr_acquisition_cc_sptr(
            new gps_l1_ca_gps_sdr_acquisition_cc(sampled_ms, freq, fs_in,
                    samples_per_ms, queue, dump, dump_filename));
}

gps_l1_ca_gps_sdr_acquisition_cc::gps_l1_ca_gps_sdr_acquisition_cc(
        unsigned int sampled_ms, long freq, long fs_in, int samples_per_ms,
        gr_msg_queue_sptr queue, bool dump, std::string dump_filename) :
    gr_block("gps_l1_ca_gps_sdr_acquisition_cc", gr_make_io_signature(1, 1,
            sizeof(gr_complex) * samples_per_ms), gr_make_io_signature(0, 0,
            sizeof(gr_complex) * samples_per_ms))
{

    // SAMPLE COUNTER
    d_sample_counter = 0;

    d_active = false;
    d_dump = dump;
    d_queue = queue;
    d_dump_filename = dump_filename;

    d_freq = freq;
    d_fs_in = fs_in;

    d_samples_per_ms = samples_per_ms;
    d_sampled_ms = sampled_ms;

    d_doppler_max = 0;

    d_satellite = 0;

    d_fft_size = d_sampled_ms * d_samples_per_ms;

    d_doppler_freq_phase = 0.0;
    d_prn_code_phase = 0;
    d_mag = 0;
    d_mean = 0.0;

    d_count = 0;

    d_sine_if = new gr_complex[d_fft_size];
    d_sine_250 = new gr_complex[d_fft_size];
    d_sine_500 = new gr_complex[d_fft_size];
    d_sine_750 = new gr_complex[d_fft_size];

    d_fft_codes = (gr_complex*)malloc(sizeof(gr_complex) * d_samples_per_ms);

    // Direct FFT
    d_fft_if = new gri_fft_complex(d_fft_size, true);
    d_fft_250 = new gri_fft_complex(d_fft_size, true);
    d_fft_500 = new gri_fft_complex(d_fft_size, true);
    d_fft_750 = new gri_fft_complex(d_fft_size, true);

    // Inverse FFT
    d_ifft = new gri_fft_complex(d_fft_size, false);

    d_best_magnitudes = new float[d_fft_size];

    sine_gen_complex(d_sine_if, -d_freq, d_fs_in, d_fft_size);
    sine_gen_complex(d_sine_250, -d_freq - 250, d_fs_in, d_fft_size);
    sine_gen_complex(d_sine_500, -d_freq - 500, d_fs_in, d_fft_size);
    sine_gen_complex(d_sine_750, -d_freq - 750, d_fs_in, d_fft_size);

    DLOG(INFO) << "fs in " << d_fs_in;
    DLOG(INFO) << "Doppler max " << d_doppler_max;
    DLOG(INFO) << "IF " << d_freq;
    DLOG(INFO) << "Satellite " << d_satellite;
    DLOG(INFO) << "Sampled_ms " << d_sampled_ms;
    DLOG(INFO) << "FFT_size " << d_fft_size;
    DLOG(INFO) << "dump filename " << d_dump_filename;
    DLOG(INFO) << "dump " << d_dump;
}

gps_l1_ca_gps_sdr_acquisition_cc::~gps_l1_ca_gps_sdr_acquisition_cc()
{
    delete[] d_sine_if;
    delete[] d_sine_250;
    delete[] d_sine_500;
    delete[] d_sine_750;
    delete[] d_fft_codes;
    delete[] d_fft_if;
    delete[] d_fft_250;
    delete[] d_fft_500;
    delete[] d_fft_750;

    if (d_dump)
    {
        d_dump_file.close();
    }
}

void gps_l1_ca_gps_sdr_acquisition_cc::set_satellite(unsigned int satellite)
{
    d_satellite = satellite;
    d_prn_code_phase = 0;
    d_doppler_freq_phase = 0;
    d_mag = 0;
    // Now the GPS codes are generated on the fly using a custom version of the GPS code generator
    code_gen_complex_sampled(d_fft_if->get_inbuf(), satellite, d_fs_in, 0);
    d_fft_if->execute(); // We need the FFT of GPS C/A code
    memcpy(d_fft_codes, d_fft_if->get_outbuf(), sizeof(gr_complex)
            * d_samples_per_ms);
}

signed int gps_l1_ca_gps_sdr_acquisition_cc::prn_code_phase()
{
    return d_prn_code_phase;
}

int gps_l1_ca_gps_sdr_acquisition_cc::general_work(int noutput_items,
        gr_vector_int &ninput_items, gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
{
    if (!d_active)
    {
        d_sample_counter += d_fft_size * noutput_items; // sample counter
        consume_each(noutput_items);
        return 0;
    }

    d_sample_counter += d_fft_size; // sample counter
    bool positive_acquisition = false;
    int acquisition_message = -1; //0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL

    LOG_AT_LEVEL(INFO) << "Channel: " << d_channel
            << " , doing acquisition of satellite: " << d_satellite
            << " ,at sample stamp: " << d_sample_counter;

    const gr_complex *in = (const gr_complex *)input_items[0];

    //Perform the carrier wipe-off
    for (unsigned int j = 0; j < d_fft_size; j++)
    {
        d_fft_if->get_inbuf()[j] = in[j] * d_sine_if[j];
        d_fft_250->get_inbuf()[j] = in[j] * d_sine_250[j];
        d_fft_500->get_inbuf()[j] = in[j] * d_sine_500[j];
        d_fft_750->get_inbuf()[j] = in[j] * d_sine_750[j];
    }
    // Perform the FFT of the resulting signal
    d_fft_if->execute();
    d_fft_250->execute();
    d_fft_500->execute();
    d_fft_750->execute();

    //----------------------------------------------------------------------

    //---Calculate cross-correlation magnitudes using:
    //----> the circular convolution property of FFT:
    //----> the frequency-shift property of the FFT: y(t)=exp(jw0t)x(t) -> Y(w)=X(w-w0)
    for (int j = (int)-(d_doppler_max / 1000); j < (int)d_doppler_max / 1000; j++)
    { // doppler search steps
        calculate_magnitudes(d_fft_if->get_outbuf(), j, 0);
        calculate_magnitudes(d_fft_250->get_outbuf(), j, 1);
        calculate_magnitudes(d_fft_500->get_outbuf(), j, 2);
        calculate_magnitudes(d_fft_750->get_outbuf(), j, 3);
    }

    if (d_dump)
    {
        std::stringstream ss;
        ss << "./data/acquisition_" << d_channel << "_" << d_satellite << "_"
                << d_count << ".dat";
        d_dump_file.open(ss.str().c_str(), std::ios::out | std::ios::binary);
        std::streamsize n = sizeof(float) * (d_fft_size);
        d_dump_file.write((char*)d_best_magnitudes, n);
        d_dump_file.close();
    }

    if (d_test_statistics > d_threshold)
    { //TODO: Include in configuration parameters and check value!!

        positive_acquisition = true;
        d_acq_sample_stamp = d_sample_counter;

        LOG_AT_LEVEL(INFO) << "POSITIVE ACQUISITION of channel " << d_channel;
        LOG_AT_LEVEL(INFO) << "Satellite " << d_satellite;
        LOG_AT_LEVEL(INFO) << "Code Phase " << d_prn_code_phase;
        LOG_AT_LEVEL(INFO) << "Doppler " << d_doppler_freq_phase;
        LOG_AT_LEVEL(INFO) << "Magnitude " << d_mag;
        LOG_AT_LEVEL(INFO) << "Mean " << d_mean;
        LOG_AT_LEVEL(INFO) << "Test statistics value " << d_test_statistics;
        LOG_AT_LEVEL(INFO) << "Test statistics threshold " << d_threshold;
        LOG_AT_LEVEL(INFO) << "Acq sample stamp " << d_acq_sample_stamp;
    }
    else
    {
        LOG_AT_LEVEL(INFO) << "NEGATIVE ACQUISITION of channel " << d_channel;
        LOG_AT_LEVEL(INFO) << "Satellite " << d_satellite;
        LOG_AT_LEVEL(INFO) << "Test statistics value " << d_test_statistics;
        LOG_AT_LEVEL(INFO) << "Test statistics threshold " << d_threshold;
        LOG_AT_LEVEL(INFO) << "Acq sample stamp " << d_acq_sample_stamp;
    }
    d_active = false;

    LOG_AT_LEVEL(INFO) << "d_count " << d_count;

    if (positive_acquisition)
    {
        acquisition_message = 1;
    }
    else
    {
        acquisition_message = 2;
    }

    d_channel_internal_queue->push(acquisition_message);

    return 0;
}

void gps_l1_ca_gps_sdr_acquisition_cc::calculate_magnitudes(
        gr_complex* fft_signal, int doppler_shift, int doppler_offset)
{

    unsigned int indext = 0;
    float magt = 0.0;
    std::complex<float> tmp_cpx;

    // FFT frequency-shift property
    if (doppler_shift != 0)
    {
        for (unsigned int i = 0; i < d_fft_size; i++)
        {
            //complex conjugate (optimize me!)
            tmp_cpx = std::complex<float>(d_fft_codes[i].real(),
                    -d_fft_codes[i].imag());
            d_ifft->get_inbuf()[i] = fft_signal[(doppler_shift + i
                    + d_fft_size) % d_fft_size] * tmp_cpx;
        }
    }
    else
    {
        for (unsigned int i = 0; i < d_fft_size; i++)
        {
            //complex conjugate (optimize me!)
            tmp_cpx = std::complex<float>(d_fft_codes[i].real(),
                    -d_fft_codes[i].imag());
            d_ifft->get_inbuf()[i] = fft_signal[i] * tmp_cpx;
        }
    }

    d_ifft->execute(); // inverse FFT of the result = convolution in time

    x86_gr_complex_mag(d_ifft->get_outbuf(), d_fft_size); // d_ifft->get_outbuf()=|abs(Â·)|^2
    x86_float_max((float*)d_ifft->get_outbuf(), &indext, &magt, d_fft_size); // find max of |abs(á)|^2 -> index and magt

    if (magt > d_mag)
    { // if the magnitude is > threshold

        // save the synchronization parameters
        d_mag = magt;
        d_prn_code_phase = indext;
        d_doppler_freq_phase = -((doppler_shift * 1000.0) + (doppler_offset
                * 250.0));
        // save the circular correlation of this Doppler shift
        memcpy(d_best_magnitudes, d_ifft->get_outbuf(), sizeof(float)
                * d_fft_size);

        // Remove the maximum and its neighbors to calculate the mean
        ((float*)d_ifft->get_outbuf())[indext] = 0.0;
        if (indext != 0)
        {
            ((float*)d_ifft->get_outbuf())[indext - 1] = 0.0;
        }
        if (indext != d_fft_size - 1)
        {
            ((float*)d_ifft->get_outbuf())[indext + 1] = 0.0;
        }

        for (unsigned int i = 0; i < d_fft_size; i++)
        {
            d_mean += ((float*)d_ifft->get_outbuf())[i];
        }
        d_mean = d_mean / d_fft_size;
        d_test_statistics = d_mag / d_mean;
    }
}
