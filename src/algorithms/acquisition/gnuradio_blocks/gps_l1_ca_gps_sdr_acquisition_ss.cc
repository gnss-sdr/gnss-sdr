/*!
 * \file gps_l1_ca_gps_sdr_acquisition_ss.cc
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
#include "gps_l1_ca_gps_sdr_acquisition_ss.h"
#include "gps_sdr_fft.h"
#include "gps_sdr_prn_codes_short.h"
#include "control_message_factory.h"
#include "gps_sdr_x86.h"
#ifndef NO_SIMD
  #include "gps_sdr_simd.h"
#endif
#include <gnuradio/gr_io_signature.h>

#include <sstream>

#include <glog/log_severity.h>
#include <glog/logging.h>

using google::LogMessage;

gps_l1_ca_gps_sdr_acquisition_ss_sptr gps_l1_ca_gps_sdr_make_acquisition_ss(
        unsigned int sampled_ms, long freq, long fs_in, int samples_per_ms,
        gr_msg_queue_sptr queue, bool dump, std::string dump_filename)
{

    return gps_l1_ca_gps_sdr_acquisition_ss_sptr(
            new gps_l1_ca_gps_sdr_acquisition_ss(sampled_ms, freq, fs_in,
                    samples_per_ms, queue, dump, dump_filename));
}

gps_l1_ca_gps_sdr_acquisition_ss::gps_l1_ca_gps_sdr_acquisition_ss(
        unsigned int sampled_ms, long freq, long fs_in, int samples_per_ms,
        gr_msg_queue_sptr queue, bool dump, std::string dump_filename) :
    gr_block("gps_l1_ca_gps_sdr_acquisition_ss", gr_make_io_signature(1, 1,
            sizeof(short) * 2 * samples_per_ms), gr_make_io_signature(0, 0,
            sizeof(short) * 2 * samples_per_ms))
{

    // SAMPLE COUNTER
    d_sample_counter = 0;

    d_active = false;
    d_dump = dump;
    d_queue = queue;
    d_dump_filename = dump_filename;

    d_fs_in = fs_in;
    d_samples_per_ms = samples_per_ms;
    d_doppler_resolution = 4;
    d_freq = freq;
    d_satellite = Gnss_Satellite();
    d_doppler_max = 0;
    d_sampled_ms = sampled_ms;
    d_fft_size = d_sampled_ms * d_samples_per_ms;

    d_doppler_freq_shift = 0.0;
    d_prn_code_phase = 0;
    d_mag = 0;

    d_sine_if = new CPX[d_fft_size];
    d_sine_250 = new CPX[d_fft_size];
    d_sine_500 = new CPX[d_fft_size];
    d_sine_750 = new CPX[d_fft_size];

    d_baseband_signal = new CPX[d_doppler_resolution * d_fft_size];
    d_baseband_signal_shift = new CPX[d_doppler_resolution * (d_fft_size
            + 201)];

    signed int R1[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    d_pFFT = new FFT(d_fft_size, R1);

    signed int R2[16] = { 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1 };
    d_piFFT = new FFT(d_fft_size, R2);

    for (int i = 0; i < 32; i++)
    {
        d_fft_codes[i] = (CPX *)&PRN_Codes_Short[sizeof(short) * i
                * d_samples_per_ms];
    }

    sine_gen(d_sine_if, -d_freq, d_fs_in, d_fft_size);
    sine_gen(d_sine_250, -d_freq - 250, d_fs_in, d_fft_size);
    sine_gen(d_sine_500, -d_freq - 500, d_fs_in, d_fft_size);
    sine_gen(d_sine_750, -d_freq - 750, d_fs_in, d_fft_size);

    DLOG(INFO) << "fs in " << d_fs_in;
    DLOG(INFO) << "samples per ms " << d_samples_per_ms;
    DLOG(INFO) << "doppler resolution " << d_doppler_resolution;
    DLOG(INFO) << "freq " << d_freq;
    DLOG(INFO) << "satellite " << d_satellite;
    DLOG(INFO) << "sampled_ms " << d_sampled_ms;
    DLOG(INFO) << "fft_size " << d_fft_size;
    DLOG(INFO) << "dump filename " << d_dump_filename;
    DLOG(INFO) << "dump " << d_dump;
}

gps_l1_ca_gps_sdr_acquisition_ss::~gps_l1_ca_gps_sdr_acquisition_ss()
{
    delete[] d_baseband_signal;
    delete[] d_baseband_signal_shift;
    delete[] d_sine_if;
    delete[] d_sine_250;
    delete[] d_sine_500;
    delete[] d_sine_750;
    delete d_pFFT;
    delete d_piFFT;

    if (d_dump)
    {
        d_dump_file.close();
    }
}

void gps_l1_ca_gps_sdr_acquisition_ss::set_satellite(Gnss_Satellite satellite)
{
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    d_prn_code_phase = 0;
    d_doppler_freq_shift = 0;
    d_mag = 0;
    DLOG(INFO) << "satellite set to " << d_satellite;
}

signed int gps_l1_ca_gps_sdr_acquisition_ss::prn_code_phase()
{
    return d_prn_code_phase;
}

int gps_l1_ca_gps_sdr_acquisition_ss::general_work(int noutput_items,
        gr_vector_int &ninput_items, gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
{

    if (!d_active)
    {
        d_sample_counter += d_fft_size * noutput_items; // sample counter
        consume_each(noutput_items);
    }
    else
    {
        d_sample_counter += d_fft_size; // sample counter

        const CPX *in = (const CPX *)input_items[0];

        CPX* buffer = new CPX[d_fft_size];

        signed int index = 0;
        unsigned int indext = 0;
        unsigned int magt = 0;

        DLOG(INFO) << "copied " << (d_fft_size * sizeof(CPX))
                << " bytes into buffer (" << d_fft_size << " samples)";
        memcpy(d_baseband_signal, in, d_fft_size * sizeof(CPX));
        #ifdef NO_SIMD
	    x86_cmulsc(d_baseband_signal, d_sine_250,
                &d_baseband_signal[d_fft_size], d_fft_size, 14);
        x86_cmulsc(d_baseband_signal, d_sine_500, &d_baseband_signal[2
                * d_fft_size], d_fft_size, 14);
        x86_cmulsc(d_baseband_signal, d_sine_750, &d_baseband_signal[3
                * d_fft_size], d_fft_size, 14);
        x86_cmuls(d_baseband_signal, d_sine_if, d_fft_size, 14);
        #else 
        sse_cmulsc(d_baseband_signal, d_sine_250,
                &d_baseband_signal[d_fft_size], d_fft_size, 14);
        sse_cmulsc(d_baseband_signal, d_sine_500, &d_baseband_signal[2
                * d_fft_size], d_fft_size, 14);
        sse_cmulsc(d_baseband_signal, d_sine_750, &d_baseband_signal[3
                * d_fft_size], d_fft_size, 14);
        sse_cmuls(d_baseband_signal, d_sine_if, d_fft_size, 14);
        #endif
        for (unsigned int i = 0; i < d_doppler_resolution; i++)
        {
            d_pFFT->doFFT(&d_baseband_signal[i * d_fft_size], true);
            memcpy(&d_baseband_signal_shift[i * (d_fft_size + 201)],
                    &d_baseband_signal[(i + 1) * d_fft_size - 100], 100
                            * sizeof(CPX));
            memcpy(&d_baseband_signal_shift[(i * (d_fft_size + 201)) + 100],
                    &d_baseband_signal[i * d_fft_size], d_fft_size
                            * sizeof(CPX));
            memcpy(&d_baseband_signal_shift[(i * (d_fft_size + 201)) + 100
                    + d_fft_size], &d_baseband_signal[i * d_fft_size], 100
                    * sizeof(CPX));
        }

        // Here begins the actual acquisition process.

        for (int i = -d_doppler_max / 1000; i < (int)d_doppler_max / 1000; i++)
        {
            for (unsigned int j = 0; j < d_doppler_resolution; j++)
            {

                #ifdef NO_SIMD
                x86_cmulsc(&d_baseband_signal_shift[(j * (d_fft_size + 201))
                        + 100 + i], d_fft_codes[d_satellite], buffer,
                        d_fft_size, 10);
                #else
                sse_cmulsc(&d_baseband_signal_shift[(j * (d_fft_size + 201))
                        + 100 + i], d_fft_codes[d_satellite.get_PRN()], buffer,
                        d_fft_size, 10);        
                #endif
		        d_piFFT->doiFFT(buffer, true);
                x86_cmag(buffer, d_fft_size);
                x86_max((unsigned int *)buffer, &indext, &magt, d_fft_size);

                if (magt > d_mag)
                {
                    d_mag = magt;
                    index = indext;
                    d_prn_code_phase = ceil((index * d_samples_per_ms)
                            / d_fft_size);
                    d_doppler_freq_shift = (i * 1000.0) + (j * 250.0);

                    if (d_dump)
                    {
                        d_dump_file.open(d_dump_filename.c_str(),
                                std::ios::out | std::ios::binary);
                        std::streamsize n = sizeof(unsigned int) * d_fft_size;
                        d_dump_file.write((char*)buffer, n);
                        d_dump_file.close();
                    }
                }
            }
        }

        DLOG(INFO) << "satellite " << d_satellite;
        //result->code_phase = 2048 - index;
        DLOG(INFO) << "code phase " << d_prn_code_phase;
        //result->doppler = (lcv*1000) + (float)lcv2*250;
        DLOG(INFO) << "doppler " << d_doppler_freq_shift;
        //result->magnitude = mag;
        DLOG(INFO) << "magnitude " << d_mag;

        d_active = false;

        delete buffer;

        DLOG(INFO) << "Acquisition done";

        int acquisition_message = -1; //0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL

        if (d_mag > d_threshold)
        {
            d_acq_sample_stamp = d_sample_counter;
            acquisition_message = 1; //ACQ_SUCCES
        }
        else
        {
            acquisition_message = 2; //ACQ_FAIL

        }

        d_channel_internal_queue->push(acquisition_message);

        consume_each(1);
    }
    return 0;

}
