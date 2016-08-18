/*!
 * \file fft_length_test.cc
 * \brief  This file implements timing tests for the FFT.
 * \author Carles Fernandez-Prades, 2016. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2016  (see AUTHORS file for a list of contributors)
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

#include <ctime>
#include <complex>
#include <thread>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include "cpu_multicorrelator.h"
#include "gps_sdr_signal_processing.h"
#include "GPS_L1_CA.h"


DEFINE_int32(cpu_multicorrelator_iterations_test, 1000, "Number of averaged iterations in CPU multicorrelator test timing test");
DEFINE_int32(cpu_multicorrelator_max_threads_test, 12, "Number of maximum concurrent correlators in CPU multicorrelator test timing test");

void run_correlator_cpu(cpu_multicorrelator* correlator,
                    float d_rem_carrier_phase_rad,
                    float d_carrier_phase_step_rad,
                    float d_code_phase_step_chips,
                    float d_rem_code_phase_chips,
                    int correlation_size)
{
    for(int k = 0; k < FLAGS_cpu_multicorrelator_iterations_test; k++)
    {
        correlator->Carrier_wipeoff_multicorrelator_resampler(d_rem_carrier_phase_rad,
                                                                   d_carrier_phase_step_rad,
                                                                   d_code_phase_step_chips,
                                                                   d_rem_code_phase_chips,
                                                                   correlation_size);
    }
}

TEST(CPU_multicorrelator_test, MeasureExecutionTime)
{
    struct timeval tv;
    int max_threads=FLAGS_cpu_multicorrelator_max_threads_test;
    std::vector<std::thread> thread_pool;
    cpu_multicorrelator* correlator_pool[max_threads];
    unsigned int correlation_sizes [3] = { 2048, 4096, 8192};
    double execution_times [3];

    gr_complex* d_ca_code;
    gr_complex* in_cpu;
    gr_complex* d_correlator_outs;

    int d_n_correlator_taps=3;
    int d_vector_length=correlation_sizes[2]; //max correlation size to allocate all the necessary memory
    float* d_local_code_shift_chips;

    //allocate host memory
    // Get space for a vector with the C/A code replica sampled 1x/chip
    d_ca_code = static_cast<gr_complex*>(volk_gnsssdr_malloc(static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS) * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    in_cpu = static_cast<gr_complex*>(volk_gnsssdr_malloc(2 * d_vector_length * sizeof(gr_complex), volk_gnsssdr_get_alignment()));

    // correlator outputs (scalar)
    d_n_correlator_taps = 3; // Early, Prompt, and Late
    d_correlator_outs = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_n_correlator_taps*sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    for (int n = 0; n < d_n_correlator_taps; n++)
    {
        d_correlator_outs[n] = gr_complex(0,0);
    }
    d_local_code_shift_chips = static_cast<float*>(volk_gnsssdr_malloc(d_n_correlator_taps*sizeof(float), volk_gnsssdr_get_alignment()));
    // Set TAPs delay values [chips]
    float d_early_late_spc_chips=0.5;
    d_local_code_shift_chips[0] = - d_early_late_spc_chips;
    d_local_code_shift_chips[1] = 0.0;
    d_local_code_shift_chips[2] = d_early_late_spc_chips;

    //--- Perform initializations ------------------------------

    //local code resampler on GPU
    // generate local reference (1 sample per chip)
    gps_l1_ca_code_gen_complex(d_ca_code, 1, 0);
    // generate inut signal
    for (int n=0;n<2*d_vector_length;n++)
    {
        in_cpu[n]=std::complex<float>(static_cast <float> (rand())/static_cast<float>(RAND_MAX),static_cast <float> (rand())/static_cast<float>(RAND_MAX));
    }

    for (int n=0;n<max_threads;n++)
    {
        correlator_pool[n] = new cpu_multicorrelator();
        correlator_pool[n]->init(d_vector_length, d_n_correlator_taps);
        correlator_pool[n]->set_input_output_vectors(d_correlator_outs, in_cpu);
        correlator_pool[n]->set_local_code_and_taps(static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS), d_ca_code, d_local_code_shift_chips);
    }

    float d_rem_carrier_phase_rad=0.0;
    float d_carrier_phase_step_rad=0.1;
    float d_code_phase_step_chips=0.3;
    float d_rem_code_phase_chips=0.4;

    EXPECT_NO_THROW(
        for(int correlation_sizes_idx = 0; correlation_sizes_idx < 3; correlation_sizes_idx++)
        {
            for(int current_max_threads=1; current_max_threads<(max_threads+1); current_max_threads++)
            {
                std::cout<<"Running "<<current_max_threads<<" concurrent correlators"<<std::endl;
                gettimeofday(&tv, NULL);
                long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;
                //create the concurrent correlator threads
                for (int current_thread=0;current_thread<current_max_threads;current_thread++)
                {
                    thread_pool.push_back(std::thread(run_correlator_cpu,
                            correlator_pool[current_thread],
                            d_rem_carrier_phase_rad,
                            d_carrier_phase_step_rad,
                            d_code_phase_step_chips,
                            d_rem_code_phase_chips,
                            correlation_sizes[correlation_sizes_idx]));
                }
                //wait the threads to finish they work and destroy the thread objects
                for(auto &t : thread_pool){
                    t.join();
                }
                thread_pool.clear();
                gettimeofday(&tv, NULL);
                long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
                execution_times[correlation_sizes_idx] = static_cast<double>(end - begin) / (1000000.0 * static_cast<double>(FLAGS_cpu_multicorrelator_iterations_test));
                std::cout << "CPU Multicorrelator execution time for length=" << correlation_sizes[correlation_sizes_idx] << " : " << execution_times[correlation_sizes_idx] << " [s]" << std::endl;

            }
        }
    );


    volk_gnsssdr_free(d_local_code_shift_chips);
    volk_gnsssdr_free(d_correlator_outs);
    volk_gnsssdr_free(d_ca_code);
    volk_gnsssdr_free(in_cpu);

    for (int n=0;n<max_threads;n++)
    {
        correlator_pool[n]->free();
    }
}
