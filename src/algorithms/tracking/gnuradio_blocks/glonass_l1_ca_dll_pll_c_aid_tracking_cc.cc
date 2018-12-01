/*!
 * \file glonass_l1_ca_dll_pll_c_aid_tracking_cc.cc
 * \brief  Implementation of a code DLL + carrier PLL tracking block
 * \author Gabriel Araujo, 2017. gabriel.araujo.5000(at)gmail.com
 * \author Luis Esteve, 2017. luis(at)epsilon-formacion.com
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
 *
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkha user, 2007
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

#include "glonass_l1_ca_dll_pll_c_aid_tracking_cc.h"
#include "glonass_l1_signal_processing.h"
#include "GLONASS_L1_L2_CA.h"
#include "tracking_discriminators.h"
#include "lock_detectors.h"
#include "gnss_sdr_flags.h"
#include "control_message_factory.h"
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>
#include <gnuradio/io_signature.h>
#include <matio.h>
#include <pmt/pmt.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <glog/logging.h>
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>


#define CN0_ESTIMATION_SAMPLES 10

using google::LogMessage;

glonass_l1_ca_dll_pll_c_aid_tracking_cc_sptr
glonass_l1_ca_dll_pll_c_aid_make_tracking_cc(
    int64_t fs_in,
    uint32_t vector_length,
    bool dump,
    std::string dump_filename,
    float pll_bw_hz,
    float dll_bw_hz,
    float pll_bw_narrow_hz,
    float dll_bw_narrow_hz,
    int32_t extend_correlation_ms,
    float early_late_space_chips)
{
    return glonass_l1_ca_dll_pll_c_aid_tracking_cc_sptr(new glonass_l1_ca_dll_pll_c_aid_tracking_cc(
        fs_in, vector_length, dump, dump_filename, pll_bw_hz, dll_bw_hz, pll_bw_narrow_hz, dll_bw_narrow_hz, extend_correlation_ms, early_late_space_chips));
}


void glonass_l1_ca_dll_pll_c_aid_tracking_cc::forecast(int noutput_items,
    gr_vector_int &ninput_items_required)
{
    if (noutput_items != 0)
        {
            ninput_items_required[0] = static_cast<int32_t>(d_vector_length) * 2;  //set the required available samples in each call
        }
}


void glonass_l1_ca_dll_pll_c_aid_tracking_cc::msg_handler_preamble_index(pmt::pmt_t msg)
{
    //pmt::print(msg);
    DLOG(INFO) << "Extended correlation enabled for Tracking CH " << d_channel << ": Satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN);
    if (d_enable_extended_integration == false)  //avoid re-setting preamble indicator
        {
            d_preamble_timestamp_s = pmt::to_double(msg);
            d_enable_extended_integration = true;
            d_preamble_synchronized = false;
        }
}


glonass_l1_ca_dll_pll_c_aid_tracking_cc::glonass_l1_ca_dll_pll_c_aid_tracking_cc(
    int64_t fs_in,
    uint32_t vector_length,
    bool dump,
    std::string dump_filename,
    float pll_bw_hz,
    float dll_bw_hz,
    float pll_bw_narrow_hz,
    float dll_bw_narrow_hz,
    int32_t extend_correlation_ms,
    float early_late_space_chips) : gr::block("glonass_l1_ca_dll_pll_c_aid_tracking_cc", gr::io_signature::make(1, 1, sizeof(gr_complex)),
                                        gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    // Telemetry bit synchronization message port input
    this->message_port_register_in(pmt::mp("preamble_timestamp_s"));

    this->set_msg_handler(pmt::mp("preamble_timestamp_s"),
        boost::bind(&glonass_l1_ca_dll_pll_c_aid_tracking_cc::msg_handler_preamble_index, this, _1));

    this->message_port_register_out(pmt::mp("events"));
    // initialize internal vars
    d_dump = dump;
    d_fs_in = fs_in;
    d_vector_length = vector_length;
    d_dump_filename = dump_filename;
    d_correlation_length_samples = static_cast<int32_t>(d_vector_length);

    // Initialize tracking  ==========================================
    d_pll_bw_hz = pll_bw_hz;
    d_dll_bw_hz = dll_bw_hz;
    d_pll_bw_narrow_hz = pll_bw_narrow_hz;
    d_dll_bw_narrow_hz = dll_bw_narrow_hz;
    d_extend_correlation_ms = extend_correlation_ms;
    d_code_loop_filter.set_DLL_BW(d_dll_bw_hz);
    d_carrier_loop_filter.set_params(10.0, d_pll_bw_hz, 2);

    // --- DLL variables --------------------------------------------------------
    d_early_late_spc_chips = early_late_space_chips;  // Define early-late offset (in chips)

    // Initialization of local code replica
    // Get space for a vector with the C/A code replica sampled 1x/chip
    d_ca_code = static_cast<gr_complex *>(volk_gnsssdr_malloc(static_cast<int32_t>(GLONASS_L1_CA_CODE_LENGTH_CHIPS) * sizeof(gr_complex), volk_gnsssdr_get_alignment()));

    // correlator outputs (scalar)
    d_n_correlator_taps = 3;  // Early, Prompt, and Late
    d_correlator_outs = static_cast<gr_complex *>(volk_gnsssdr_malloc(d_n_correlator_taps * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    for (int32_t n = 0; n < d_n_correlator_taps; n++)
        {
            d_correlator_outs[n] = gr_complex(0, 0);
        }
    d_local_code_shift_chips = static_cast<float *>(volk_gnsssdr_malloc(d_n_correlator_taps * sizeof(float), volk_gnsssdr_get_alignment()));
    // Set TAPs delay values [chips]
    d_local_code_shift_chips[0] = -d_early_late_spc_chips;
    d_local_code_shift_chips[1] = 0.0;
    d_local_code_shift_chips[2] = d_early_late_spc_chips;

    multicorrelator_cpu.init(2 * d_correlation_length_samples, d_n_correlator_taps);

    //--- Perform initializations ------------------------------
    // define initial code frequency basis of NCO
    d_code_freq_chips = GLONASS_L1_CA_CODE_RATE_HZ;
    // define residual code phase (in chips)
    d_rem_code_phase_samples = 0.0;
    // define residual carrier phase
    d_rem_carrier_phase_rad = 0.0;

    // sample synchronization
    d_sample_counter = 0ULL;  //(from trk to tlm)
    d_acq_sample_stamp = 0;
    d_enable_tracking = false;
    d_pull_in = false;

    // CN0 estimation and lock detector buffers
    d_cn0_estimation_counter = 0;
    d_Prompt_buffer = new gr_complex[FLAGS_cn0_samples];
    d_carrier_lock_test = 1;
    d_CN0_SNV_dB_Hz = 0;
    d_carrier_lock_fail_counter = 0;
    d_carrier_lock_threshold = FLAGS_carrier_lock_th;

    systemName["R"] = std::string("Glonass");

    set_relative_rate(1.0 / static_cast<double>(d_vector_length));

    d_acquisition_gnss_synchro = 0;
    d_channel = 0;
    d_acq_code_phase_samples = 0.0;
    d_acq_carrier_doppler_hz = 0.0;
    d_carrier_doppler_hz = 0.0;
    d_code_error_filt_chips_Ti = 0.0;
    d_acc_carrier_phase_cycles = 0.0;
    d_code_phase_samples = 0.0;

    d_pll_to_dll_assist_secs_Ti = 0.0;
    d_rem_code_phase_chips = 0.0;
    d_code_phase_step_chips = 0.0;
    d_carrier_phase_step_rad = 0.0;
    d_enable_extended_integration = false;
    d_preamble_synchronized = false;
    d_rem_code_phase_integer_samples = 0;
    d_code_error_chips_Ti = 0.0;
    d_code_error_filt_chips_s = 0.0;
    d_carr_phase_error_secs_Ti = 0.0;
    d_preamble_timestamp_s = 0.0;

    d_carrier_frequency_hz = 0.0;
    d_carrier_doppler_old_hz = 0.0;

    d_glonass_freq_ch = 0;

    //set_min_output_buffer((int64_t)300);
}


void glonass_l1_ca_dll_pll_c_aid_tracking_cc::start_tracking()
{
    /*
     *  correct the code phase according to the delay between acq and trk
     */
    d_acq_code_phase_samples = d_acquisition_gnss_synchro->Acq_delay_samples;
    d_acq_carrier_doppler_hz = d_acquisition_gnss_synchro->Acq_doppler_hz;
    d_acq_sample_stamp = d_acquisition_gnss_synchro->Acq_samplestamp_samples;

    int64_t acq_trk_diff_samples;
    double acq_trk_diff_seconds;
    acq_trk_diff_samples = static_cast<int64_t>(d_sample_counter) - static_cast<int64_t>(d_acq_sample_stamp);  //-d_vector_length;
    DLOG(INFO) << "Number of samples between Acquisition and Tracking =" << acq_trk_diff_samples;
    acq_trk_diff_seconds = static_cast<double>(acq_trk_diff_samples) / static_cast<double>(d_fs_in);
    // Doppler effect
    // Fd=(C/(C+Vr))*F
    d_glonass_freq_ch = GLONASS_L1_CA_FREQ_HZ + (DFRQ1_GLO * static_cast<double>(GLONASS_PRN.at(d_acquisition_gnss_synchro->PRN)));
    double radial_velocity = (d_glonass_freq_ch + d_acq_carrier_doppler_hz) / d_glonass_freq_ch;
    // new chip and prn sequence periods based on acq Doppler
    double T_chip_mod_seconds;
    double T_prn_mod_seconds;
    double T_prn_mod_samples;
    d_code_freq_chips = radial_velocity * GLONASS_L1_CA_CODE_RATE_HZ;
    d_code_phase_step_chips = static_cast<double>(d_code_freq_chips) / static_cast<double>(d_fs_in);
    T_chip_mod_seconds = 1.0 / d_code_freq_chips;
    T_prn_mod_seconds = T_chip_mod_seconds * GLONASS_L1_CA_CODE_LENGTH_CHIPS;
    T_prn_mod_samples = T_prn_mod_seconds * static_cast<double>(d_fs_in);

    d_correlation_length_samples = round(T_prn_mod_samples);

    double T_prn_true_seconds = GLONASS_L1_CA_CODE_LENGTH_CHIPS / GLONASS_L1_CA_CODE_RATE_HZ;
    double T_prn_true_samples = T_prn_true_seconds * static_cast<double>(d_fs_in);
    double T_prn_diff_seconds = T_prn_true_seconds - T_prn_mod_seconds;
    double N_prn_diff = acq_trk_diff_seconds / T_prn_true_seconds;
    double corrected_acq_phase_samples, delay_correction_samples;
    corrected_acq_phase_samples = fmod((d_acq_code_phase_samples + T_prn_diff_seconds * N_prn_diff * static_cast<double>(d_fs_in)), T_prn_true_samples);
    if (corrected_acq_phase_samples < 0)
        {
            corrected_acq_phase_samples = T_prn_mod_samples + corrected_acq_phase_samples;
        }
    delay_correction_samples = d_acq_code_phase_samples - corrected_acq_phase_samples;

    d_acq_code_phase_samples = corrected_acq_phase_samples;

    // d_carrier_doppler_hz = d_acq_carrier_doppler_hz + (DFRQ1_GLO *  GLONASS_PRN.at(d_acquisition_gnss_synchro->PRN));
    // d_carrier_doppler_hz = d_acq_carrier_doppler_hz;
    // d_carrier_phase_step_rad = GLONASS_TWO_PI * d_carrier_doppler_hz / static_cast<double>(d_fs_in);
    d_carrier_frequency_hz = d_acq_carrier_doppler_hz + (DFRQ1_GLO * static_cast<double>(GLONASS_PRN.at(d_acquisition_gnss_synchro->PRN)));
    d_carrier_doppler_hz = d_acq_carrier_doppler_hz;
    d_carrier_phase_step_rad = GLONASS_TWO_PI * d_carrier_frequency_hz / static_cast<double>(d_fs_in);


    // DLL/PLL filter initialization
    d_carrier_loop_filter.initialize(d_carrier_frequency_hz);  // The carrier loop filter implements the Doppler accumulator
    d_code_loop_filter.initialize();                           // initialize the code filter

    // generate local reference ALWAYS starting at chip 1 (1 sample per chip)
    glonass_l1_ca_code_gen_complex(d_ca_code, 0);

    multicorrelator_cpu.set_local_code_and_taps(static_cast<int32_t>(GLONASS_L1_CA_CODE_LENGTH_CHIPS), d_ca_code, d_local_code_shift_chips);
    for (int32_t n = 0; n < d_n_correlator_taps; n++)
        {
            d_correlator_outs[n] = gr_complex(0, 0);
        }

    d_carrier_lock_fail_counter = 0;
    d_rem_code_phase_samples = 0.0;
    d_rem_carrier_phase_rad = 0.0;
    d_rem_code_phase_chips = 0.0;
    d_acc_carrier_phase_cycles = 0.0;
    d_pll_to_dll_assist_secs_Ti = 0.0;
    d_code_phase_samples = d_acq_code_phase_samples;

    std::string sys_ = &d_acquisition_gnss_synchro->System;
    sys = sys_.substr(0, 1);

    // DEBUG OUTPUT
    std::cout << "Tracking of GLONASS L1 C/A signal started on channel " << d_channel << " for satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << std::endl;
    LOG(INFO) << "Tracking of GLONASS L1 C/A signal for satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << " on channel " << d_channel;

    // enable tracking
    d_pull_in = true;
    d_enable_tracking = true;
    d_enable_extended_integration = false;
    d_preamble_synchronized = false;
    LOG(INFO) << "PULL-IN Doppler [Hz]=" << d_carrier_doppler_hz
              << " Code Phase correction [samples]=" << delay_correction_samples
              << " PULL-IN Code Phase [samples]=" << d_acq_code_phase_samples;
}


glonass_l1_ca_dll_pll_c_aid_tracking_cc::~glonass_l1_ca_dll_pll_c_aid_tracking_cc()
{
    if (d_dump_file.is_open())
        {
            try
                {
                    d_dump_file.close();
                }
            catch (const std::exception &ex)
                {
                    LOG(WARNING) << "Exception in destructor " << ex.what();
                }
        }

    if (d_dump)
        {
            if (d_channel == 0)
                {
                    std::cout << "Writing .mat files ...";
                }
            glonass_l1_ca_dll_pll_c_aid_tracking_cc::save_matfile();
            if (d_channel == 0)
                {
                    std::cout << " done." << std::endl;
                }
        }

    try
        {
            volk_gnsssdr_free(d_local_code_shift_chips);
            volk_gnsssdr_free(d_correlator_outs);
            volk_gnsssdr_free(d_ca_code);
            delete[] d_Prompt_buffer;
            multicorrelator_cpu.free();
        }
    catch (const std::exception &ex)
        {
            LOG(WARNING) << "Exception in destructor " << ex.what();
        }
}


int32_t glonass_l1_ca_dll_pll_c_aid_tracking_cc::save_matfile()
{
    // READ DUMP FILE
    std::ifstream::pos_type size;
    int32_t number_of_double_vars = 11;
    int32_t number_of_float_vars = 5;
    int32_t epoch_size_bytes = sizeof(uint64_t) + sizeof(double) * number_of_double_vars +
                               sizeof(float) * number_of_float_vars + sizeof(uint32_t);
    std::ifstream dump_file;
    dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    try
        {
            dump_file.open(d_dump_filename.c_str(), std::ios::binary | std::ios::ate);
        }
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Problem opening dump file:" << e.what() << std::endl;
            return 1;
        }
    // count number of epochs and rewind
    int64_t num_epoch = 0;
    if (dump_file.is_open())
        {
            size = dump_file.tellg();
            num_epoch = static_cast<int64_t>(size) / static_cast<int64_t>(epoch_size_bytes);
            dump_file.seekg(0, std::ios::beg);
        }
    else
        {
            return 1;
        }
    float *abs_E = new float[num_epoch];
    float *abs_P = new float[num_epoch];
    float *abs_L = new float[num_epoch];
    float *Prompt_I = new float[num_epoch];
    float *Prompt_Q = new float[num_epoch];
    uint64_t *PRN_start_sample_count = new uint64_t[num_epoch];
    double *acc_carrier_phase_rad = new double[num_epoch];
    double *carrier_doppler_hz = new double[num_epoch];
    double *code_freq_chips = new double[num_epoch];
    double *carr_error_hz = new double[num_epoch];
    double *carr_error_filt_hz = new double[num_epoch];
    double *code_error_chips = new double[num_epoch];
    double *code_error_filt_chips = new double[num_epoch];
    double *CN0_SNV_dB_Hz = new double[num_epoch];
    double *carrier_lock_test = new double[num_epoch];
    double *aux1 = new double[num_epoch];
    double *aux2 = new double[num_epoch];
    uint32_t *PRN = new uint32_t[num_epoch];

    try
        {
            if (dump_file.is_open())
                {
                    for (int64_t i = 0; i < num_epoch; i++)
                        {
                            dump_file.read(reinterpret_cast<char *>(&abs_E[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&abs_P[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&abs_L[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&Prompt_I[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&Prompt_Q[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&PRN_start_sample_count[i]), sizeof(uint64_t));
                            dump_file.read(reinterpret_cast<char *>(&acc_carrier_phase_rad[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&carrier_doppler_hz[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&code_freq_chips[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&carr_error_hz[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&carr_error_filt_hz[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&code_error_chips[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&code_error_filt_chips[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&CN0_SNV_dB_Hz[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&carrier_lock_test[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&aux1[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&aux2[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&PRN[i]), sizeof(uint32_t));
                        }
                }
            dump_file.close();
        }
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Problem reading dump file:" << e.what() << std::endl;
            delete[] abs_E;
            delete[] abs_P;
            delete[] abs_L;
            delete[] Prompt_I;
            delete[] Prompt_Q;
            delete[] PRN_start_sample_count;
            delete[] acc_carrier_phase_rad;
            delete[] carrier_doppler_hz;
            delete[] code_freq_chips;
            delete[] carr_error_hz;
            delete[] carr_error_filt_hz;
            delete[] code_error_chips;
            delete[] code_error_filt_chips;
            delete[] CN0_SNV_dB_Hz;
            delete[] carrier_lock_test;
            delete[] aux1;
            delete[] aux2;
            delete[] PRN;
            return 1;
        }

    // WRITE MAT FILE
    mat_t *matfp;
    matvar_t *matvar;
    std::string filename = d_dump_filename;
    filename.erase(filename.length() - 4, 4);
    filename.append(".mat");
    matfp = Mat_CreateVer(filename.c_str(), NULL, MAT_FT_MAT73);
    if (reinterpret_cast<int64_t *>(matfp) != NULL)
        {
            size_t dims[2] = {1, static_cast<size_t>(num_epoch)};
            matvar = Mat_VarCreate("abs_E", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, abs_E, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("abs_P", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, abs_P, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("abs_L", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, abs_L, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("Prompt_I", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, Prompt_I, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("Prompt_Q", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, Prompt_Q, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("PRN_start_sample_count", MAT_C_UINT64, MAT_T_UINT64, 2, dims, PRN_start_sample_count, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("acc_carrier_phase_rad", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, acc_carrier_phase_rad, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("carrier_doppler_hz", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, carrier_doppler_hz, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("code_freq_chips", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, code_freq_chips, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("carr_error_hz", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, carr_error_hz, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("carr_error_filt_hz", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, carr_error_filt_hz, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("code_error_chips", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, code_error_chips, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("code_error_filt_chips", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, code_error_filt_chips, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("CN0_SNV_dB_Hz", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, CN0_SNV_dB_Hz, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("carrier_lock_test", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, carrier_lock_test, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("aux1", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, aux1, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("aux2", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, aux2, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("PRN", MAT_C_UINT32, MAT_T_UINT32, 2, dims, PRN, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);
        }
    Mat_Close(matfp);
    delete[] abs_E;
    delete[] abs_P;
    delete[] abs_L;
    delete[] Prompt_I;
    delete[] Prompt_Q;
    delete[] PRN_start_sample_count;
    delete[] acc_carrier_phase_rad;
    delete[] carrier_doppler_hz;
    delete[] code_freq_chips;
    delete[] carr_error_hz;
    delete[] carr_error_filt_hz;
    delete[] code_error_chips;
    delete[] code_error_filt_chips;
    delete[] CN0_SNV_dB_Hz;
    delete[] carrier_lock_test;
    delete[] aux1;
    delete[] aux2;
    delete[] PRN;
    return 0;
}


void glonass_l1_ca_dll_pll_c_aid_tracking_cc::set_channel(uint32_t channel)
{
    d_channel = channel;
    LOG(INFO) << "Tracking Channel set to " << d_channel;
    // ############# ENABLE DATA FILE LOG #################
    if (d_dump == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                        {
                            d_dump_filename.append(boost::lexical_cast<std::string>(d_channel));
                            d_dump_filename.append(".dat");
                            d_dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "Tracking dump enabled on channel " << d_channel << " Log file: " << d_dump_filename.c_str() << std::endl;
                        }
                    catch (const std::ifstream::failure *e)
                        {
                            LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump file " << e->what() << std::endl;
                        }
                }
        }
}


void glonass_l1_ca_dll_pll_c_aid_tracking_cc::set_gnss_synchro(Gnss_Synchro *p_gnss_synchro)
{
    d_acquisition_gnss_synchro = p_gnss_synchro;
}


int glonass_l1_ca_dll_pll_c_aid_tracking_cc::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    // Block input data and block output stream pointers
    const gr_complex *in = reinterpret_cast<const gr_complex *>(input_items[0]);  // PRN start block alignment
    Gnss_Synchro **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);

    // GNSS_SYNCHRO OBJECT to interchange data between tracking->telemetry_decoder
    Gnss_Synchro current_synchro_data = Gnss_Synchro();

    // process vars
    double code_error_filt_secs_Ti = 0.0;
    double CURRENT_INTEGRATION_TIME_S = 0.0;
    double CORRECTED_INTEGRATION_TIME_S = 0.0;

    if (d_enable_tracking == true)
        {
            // Fill the acquisition data
            current_synchro_data = *d_acquisition_gnss_synchro;
            // Receiver signal alignment
            if (d_pull_in == true)
                {
                    int32_t samples_offset;
                    double acq_trk_shif_correction_samples;
                    int32_t acq_to_trk_delay_samples;
                    acq_to_trk_delay_samples = d_sample_counter - d_acq_sample_stamp;
                    acq_trk_shif_correction_samples = d_correlation_length_samples - fmod(static_cast<double>(acq_to_trk_delay_samples), static_cast<double>(d_correlation_length_samples));
                    samples_offset = round(d_acq_code_phase_samples + acq_trk_shif_correction_samples);
                    current_synchro_data.Tracking_sample_counter = d_sample_counter + static_cast<uint64_t>(samples_offset);
                    d_sample_counter += static_cast<uint64_t>(samples_offset);  // count for the processed samples
                    d_pull_in = false;
                    d_acc_carrier_phase_cycles -= d_carrier_phase_step_rad * samples_offset / GLONASS_TWO_PI;
                    current_synchro_data.Carrier_phase_rads = d_acc_carrier_phase_cycles * GLONASS_TWO_PI;
                    current_synchro_data.Carrier_Doppler_hz = d_carrier_doppler_hz;
                    current_synchro_data.fs = d_fs_in;
                    *out[0] = current_synchro_data;
                    consume_each(samples_offset);  // shift input to perform alignment with local replica
                    return 1;
                }

            // ################# CARRIER WIPEOFF AND CORRELATORS ##############################
            // perform carrier wipe-off and compute Early, Prompt and Late correlation
            multicorrelator_cpu.set_input_output_vectors(d_correlator_outs, in);
            multicorrelator_cpu.Carrier_wipeoff_multicorrelator_resampler(d_rem_carrier_phase_rad,
                d_carrier_phase_step_rad,
                d_rem_code_phase_chips,
                d_code_phase_step_chips,
                d_correlation_length_samples);

            // ####### coherent integration extension
            // keep the last symbols
            d_E_history.push_back(d_correlator_outs[0]);  // save early output
            d_P_history.push_back(d_correlator_outs[1]);  // save prompt output
            d_L_history.push_back(d_correlator_outs[2]);  // save late output

            if (static_cast<int32_t>(d_P_history.size()) > d_extend_correlation_ms)
                {
                    d_E_history.pop_front();
                    d_P_history.pop_front();
                    d_L_history.pop_front();
                }

            bool enable_dll_pll;
            if (d_enable_extended_integration == true)
                {
                    int64_t symbol_diff = round(1000.0 * ((static_cast<double>(d_sample_counter) + d_rem_code_phase_samples) / static_cast<double>(d_fs_in) - d_preamble_timestamp_s));
                    if (symbol_diff > 0 and symbol_diff % d_extend_correlation_ms == 0)
                        {
                            // compute coherent integration and enable tracking loop
                            // perform coherent integration using correlator output history
                            // std::cout<<"##### RESET COHERENT INTEGRATION ####"<<std::endl;
                            d_correlator_outs[0] = gr_complex(0.0, 0.0);
                            d_correlator_outs[1] = gr_complex(0.0, 0.0);
                            d_correlator_outs[2] = gr_complex(0.0, 0.0);
                            for (int32_t n = 0; n < d_extend_correlation_ms; n++)
                                {
                                    d_correlator_outs[0] += d_E_history.at(n);
                                    d_correlator_outs[1] += d_P_history.at(n);
                                    d_correlator_outs[2] += d_L_history.at(n);
                                }

                            if (d_preamble_synchronized == false)
                                {
                                    d_code_loop_filter.set_DLL_BW(d_dll_bw_narrow_hz);
                                    d_carrier_loop_filter.set_params(10.0, d_pll_bw_narrow_hz, 2);
                                    d_preamble_synchronized = true;
                                    std::cout << "Enabled " << d_extend_correlation_ms << " [ms] extended correlator for CH " << d_channel << " : Satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN)
                                              << " pll_bw = " << d_pll_bw_hz << " [Hz], pll_narrow_bw = " << d_pll_bw_narrow_hz << " [Hz]" << std::endl
                                              << " dll_bw = " << d_dll_bw_hz << " [Hz], dll_narrow_bw = " << d_dll_bw_narrow_hz << " [Hz]" << std::endl;
                                }
                            // UPDATE INTEGRATION TIME
                            CURRENT_INTEGRATION_TIME_S = static_cast<double>(d_extend_correlation_ms) * GLONASS_L1_CA_CODE_PERIOD;
                            d_code_loop_filter.set_pdi(CURRENT_INTEGRATION_TIME_S);
                            enable_dll_pll = true;
                        }
                    else
                        {
                            if (d_preamble_synchronized == true)
                                {
                                    // continue extended coherent correlation
                                    // Compute the next buffer length based on the period of the PRN sequence and the code phase error estimation
                                    double T_chip_seconds = 1.0 / d_code_freq_chips;
                                    double T_prn_seconds = T_chip_seconds * GLONASS_L1_CA_CODE_LENGTH_CHIPS;
                                    double T_prn_samples = T_prn_seconds * static_cast<double>(d_fs_in);
                                    int32_t K_prn_samples = round(T_prn_samples);
                                    double K_T_prn_error_samples = K_prn_samples - T_prn_samples;

                                    d_rem_code_phase_samples = d_rem_code_phase_samples - K_T_prn_error_samples;
                                    d_rem_code_phase_integer_samples = round(d_rem_code_phase_samples);  // round to a discrete number of samples
                                    d_correlation_length_samples = K_prn_samples + d_rem_code_phase_integer_samples;
                                    d_rem_code_phase_samples = d_rem_code_phase_samples - d_rem_code_phase_integer_samples;
                                    // code phase step (Code resampler phase increment per sample) [chips/sample]
                                    d_code_phase_step_chips = d_code_freq_chips / static_cast<double>(d_fs_in);
                                    // remnant code phase [chips]
                                    d_rem_code_phase_chips = d_rem_code_phase_samples * (d_code_freq_chips / static_cast<double>(d_fs_in));
                                    d_rem_carrier_phase_rad = fmod(d_rem_carrier_phase_rad + d_carrier_phase_step_rad * static_cast<double>(d_correlation_length_samples), GLONASS_TWO_PI);

                                    // UPDATE ACCUMULATED CARRIER PHASE
                                    CORRECTED_INTEGRATION_TIME_S = (static_cast<double>(d_correlation_length_samples) / static_cast<double>(d_fs_in));
                                    d_acc_carrier_phase_cycles -= d_carrier_phase_step_rad * d_correlation_length_samples / GLONASS_TWO_PI;

                                    // disable tracking loop and inform telemetry decoder
                                    enable_dll_pll = false;
                                }
                            else
                                {
                                    //  perform basic (1ms) correlation
                                    // UPDATE INTEGRATION TIME
                                    CURRENT_INTEGRATION_TIME_S = static_cast<double>(d_correlation_length_samples) / static_cast<double>(d_fs_in);
                                    d_code_loop_filter.set_pdi(CURRENT_INTEGRATION_TIME_S);
                                    enable_dll_pll = true;
                                }
                        }
                }
            else
                {
                    // UPDATE INTEGRATION TIME
                    CURRENT_INTEGRATION_TIME_S = static_cast<double>(d_correlation_length_samples) / static_cast<double>(d_fs_in);
                    enable_dll_pll = true;
                }

            if (enable_dll_pll == true)
                {
                    // ################## PLL ##########################################################
                    // Update PLL discriminator [rads/Ti -> Secs/Ti]
                    d_carr_phase_error_secs_Ti = pll_cloop_two_quadrant_atan(d_correlator_outs[1]) / GLONASS_TWO_PI;  // prompt output
                    d_carrier_doppler_old_hz = d_carrier_doppler_hz;
                    // Carrier discriminator filter
                    // NOTICE: The carrier loop filter includes the Carrier Doppler accumulator, as described in Kaplan
                    // Input [s/Ti] -> output [Hz]
                    d_carrier_doppler_hz = d_carrier_loop_filter.get_carrier_error(0.0, d_carr_phase_error_secs_Ti, CURRENT_INTEGRATION_TIME_S);
                    // PLL to DLL assistance [Secs/Ti]
                    d_pll_to_dll_assist_secs_Ti = (d_carrier_doppler_hz * CURRENT_INTEGRATION_TIME_S) / d_glonass_freq_ch;
                    // code Doppler frequency update
                    d_code_freq_chips = GLONASS_L1_CA_CODE_RATE_HZ + (((d_carrier_doppler_hz - d_carrier_doppler_old_hz) * GLONASS_L1_CA_CODE_RATE_HZ) / d_glonass_freq_ch);

                    // ################## DLL ##########################################################
                    // DLL discriminator
                    d_code_error_chips_Ti = dll_nc_e_minus_l_normalized(d_correlator_outs[0], d_correlator_outs[2]);  // [chips/Ti] //early and late
                    // Code discriminator filter
                    d_code_error_filt_chips_s = d_code_loop_filter.get_code_nco(d_code_error_chips_Ti);  // input [chips/Ti] -> output [chips/second]
                    d_code_error_filt_chips_Ti = d_code_error_filt_chips_s * CURRENT_INTEGRATION_TIME_S;
                    code_error_filt_secs_Ti = d_code_error_filt_chips_Ti / d_code_freq_chips;  // [s/Ti]

                    // ################## CARRIER AND CODE NCO BUFFER ALIGNMENT #######################
                    // keep alignment parameters for the next input buffer
                    // Compute the next buffer length based in the new period of the PRN sequence and the code phase error estimation
                    double T_chip_seconds = 1.0 / d_code_freq_chips;
                    double T_prn_seconds = T_chip_seconds * GLONASS_L1_CA_CODE_LENGTH_CHIPS;
                    double T_prn_samples = T_prn_seconds * static_cast<double>(d_fs_in);
                    double K_prn_samples = round(T_prn_samples);
                    double K_T_prn_error_samples = K_prn_samples - T_prn_samples;

                    d_rem_code_phase_samples = d_rem_code_phase_samples - K_T_prn_error_samples + code_error_filt_secs_Ti * static_cast<double>(d_fs_in);  //(code_error_filt_secs_Ti + d_pll_to_dll_assist_secs_Ti) * static_cast<double>(d_fs_in);
                    d_rem_code_phase_integer_samples = round(d_rem_code_phase_samples);                                                                    // round to a discrete number of samples
                    d_correlation_length_samples = K_prn_samples + d_rem_code_phase_integer_samples;
                    d_rem_code_phase_samples = d_rem_code_phase_samples - d_rem_code_phase_integer_samples;

                    //################### PLL COMMANDS #################################################
                    //carrier phase step (NCO phase increment per sample) [rads/sample]
                    d_carrier_phase_step_rad = GLONASS_TWO_PI * d_carrier_doppler_hz / static_cast<double>(d_fs_in);
                    d_acc_carrier_phase_cycles -= d_carrier_phase_step_rad * d_correlation_length_samples / GLONASS_TWO_PI;
                    // UPDATE ACCUMULATED CARRIER PHASE
                    CORRECTED_INTEGRATION_TIME_S = (static_cast<double>(d_correlation_length_samples) / static_cast<double>(d_fs_in));
                    //remnant carrier phase [rad]
                    d_rem_carrier_phase_rad = fmod(d_rem_carrier_phase_rad + GLONASS_TWO_PI * d_carrier_doppler_hz * CORRECTED_INTEGRATION_TIME_S, GLONASS_TWO_PI);

                    //################### DLL COMMANDS #################################################
                    //code phase step (Code resampler phase increment per sample) [chips/sample]
                    d_code_phase_step_chips = d_code_freq_chips / static_cast<double>(d_fs_in);
                    //remnant code phase [chips]
                    d_rem_code_phase_chips = d_rem_code_phase_samples * (d_code_freq_chips / static_cast<double>(d_fs_in));

                    // ####### CN0 ESTIMATION AND LOCK DETECTORS #######################################
                    if (d_cn0_estimation_counter < CN0_ESTIMATION_SAMPLES)
                        {
                            // fill buffer with prompt correlator output values
                            d_Prompt_buffer[d_cn0_estimation_counter] = d_correlator_outs[1];  // prompt
                            d_cn0_estimation_counter++;
                        }
                    else
                        {
                            d_cn0_estimation_counter = 0;
                            // Code lock indicator
                            d_CN0_SNV_dB_Hz = cn0_svn_estimator(d_Prompt_buffer, CN0_ESTIMATION_SAMPLES, GLONASS_L1_CA_CODE_PERIOD);
                            // Carrier lock indicator
                            d_carrier_lock_test = carrier_lock_detector(d_Prompt_buffer, CN0_ESTIMATION_SAMPLES);
                            // Loss of lock detection
                            if (d_carrier_lock_test < d_carrier_lock_threshold or d_CN0_SNV_dB_Hz < FLAGS_cn0_min)
                                {
                                    d_carrier_lock_fail_counter++;
                                }
                            else
                                {
                                    if (d_carrier_lock_fail_counter > 0) d_carrier_lock_fail_counter--;
                                }
                            if (d_carrier_lock_fail_counter > FLAGS_max_lock_fail)
                                {
                                    std::cout << "Loss of lock in channel " << d_channel << "!" << std::endl;
                                    LOG(INFO) << "Loss of lock in channel " << d_channel << "!";
                                    this->message_port_pub(pmt::mp("events"), pmt::from_long(3));  //3 -> loss of lock
                                    d_carrier_lock_fail_counter = 0;
                                    d_enable_tracking = false;  // TODO: check if disabling tracking is consistent with the channel state machine
                                }
                        }
                    // ########### Output the tracking data to navigation and PVT ##########
                    current_synchro_data.Prompt_I = static_cast<double>((d_correlator_outs[1]).real());
                    current_synchro_data.Prompt_Q = static_cast<double>((d_correlator_outs[1]).imag());
                    current_synchro_data.Tracking_sample_counter = d_sample_counter + static_cast<uint64_t>(d_correlation_length_samples);
                    current_synchro_data.Code_phase_samples = d_rem_code_phase_samples;
                    current_synchro_data.Carrier_phase_rads = GLONASS_TWO_PI * d_acc_carrier_phase_cycles;
                    current_synchro_data.Carrier_Doppler_hz = d_carrier_doppler_hz;
                    current_synchro_data.CN0_dB_hz = d_CN0_SNV_dB_Hz;
                    current_synchro_data.Flag_valid_symbol_output = true;
                    if (d_preamble_synchronized == true)
                        {
                            current_synchro_data.correlation_length_ms = d_extend_correlation_ms;
                        }
                    else
                        {
                            current_synchro_data.correlation_length_ms = 1;
                        }
                }
            else
                {
                    current_synchro_data.Prompt_I = static_cast<double>((d_correlator_outs[1]).real());
                    current_synchro_data.Prompt_Q = static_cast<double>((d_correlator_outs[1]).imag());
                    current_synchro_data.Tracking_sample_counter = d_sample_counter + static_cast<uint64_t>(d_correlation_length_samples);
                    current_synchro_data.Code_phase_samples = d_rem_code_phase_samples;
                    current_synchro_data.Carrier_phase_rads = GLONASS_TWO_PI * d_acc_carrier_phase_cycles;
                    current_synchro_data.Carrier_Doppler_hz = d_carrier_doppler_hz;  // todo: project the carrier doppler
                    current_synchro_data.CN0_dB_hz = d_CN0_SNV_dB_Hz;
                }
        }
    else
        {
            for (int32_t n = 0; n < d_n_correlator_taps; n++)
                {
                    d_correlator_outs[n] = gr_complex(0, 0);
                }

            current_synchro_data.System = {'R'};
            current_synchro_data.Tracking_sample_counter = d_sample_counter + static_cast<uint64_t>(d_correlation_length_samples);
        }
    //assign the GNURadio block output data
    current_synchro_data.fs = d_fs_in;
    *out[0] = current_synchro_data;
    if (d_dump)
        {
            // MULTIPLEXED FILE RECORDING - Record results to file
            float prompt_I;
            float prompt_Q;
            float tmp_E, tmp_P, tmp_L;
            float tmp_VE = 0.0;
            float tmp_VL = 0.0;
            float tmp_float;
            prompt_I = d_correlator_outs[1].real();
            prompt_Q = d_correlator_outs[1].imag();
            tmp_E = std::abs<float>(d_correlator_outs[0]);
            tmp_P = std::abs<float>(d_correlator_outs[1]);
            tmp_L = std::abs<float>(d_correlator_outs[2]);
            try
                {
                    // Dump correlators output
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_VE), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_E), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_P), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_L), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_VL), sizeof(float));
                    // PROMPT I and Q (to analyze navigation symbols)
                    d_dump_file.write(reinterpret_cast<char *>(&prompt_I), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char *>(&prompt_Q), sizeof(float));
                    // PRN start sample stamp
                    d_dump_file.write(reinterpret_cast<char *>(&d_sample_counter), sizeof(uint64_t));
                    // accumulated carrier phase
                    tmp_float = d_acc_carrier_phase_cycles * GLONASS_TWO_PI;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    // carrier and code frequency
                    tmp_float = d_carrier_doppler_hz;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    tmp_float = d_code_freq_chips;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    // PLL commands
                    tmp_float = 1.0 / (d_carr_phase_error_secs_Ti * CURRENT_INTEGRATION_TIME_S);
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    tmp_float = 1.0 / (d_code_error_filt_chips_Ti * CURRENT_INTEGRATION_TIME_S);
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    // DLL commands
                    tmp_float = d_code_error_chips_Ti * CURRENT_INTEGRATION_TIME_S;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    tmp_float = d_code_error_filt_chips_Ti;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    // CN0 and carrier lock test
                    tmp_float = d_CN0_SNV_dB_Hz;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    tmp_float = d_carrier_lock_test;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    // AUX vars (for debug purposes)
                    tmp_float = d_code_error_chips_Ti * CURRENT_INTEGRATION_TIME_S;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    double tmp_double = static_cast<double>(d_sample_counter + d_correlation_length_samples);
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                    // PRN
                    uint32_t prn_ = d_acquisition_gnss_synchro->PRN;
                    d_dump_file.write(reinterpret_cast<char *>(&prn_), sizeof(uint32_t));
                }
            catch (const std::ifstream::failure *e)
                {
                    LOG(WARNING) << "Exception writing trk dump file " << e->what();
                }
        }

    consume_each(d_correlation_length_samples);        // this is necessary in gr::block derivates
    d_sample_counter += d_correlation_length_samples;  //count for the processed samples

    return 1;  //output tracking result ALWAYS even in the case of d_enable_tracking==false
}
