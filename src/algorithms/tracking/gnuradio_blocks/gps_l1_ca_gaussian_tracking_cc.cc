/*!
 * \file gps_l1_ca_gaussian_tracking_cc.cc
 * \brief Implementation of a processing block of a DLL + Kalman carrier
 * tracking loop for GPS L1 C/A signals
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
 * \author Jordi Vila-Valls 2018. jvila(at)cttc.es
 * \author Carles Fernandez-Prades 2018. cfernandez(at)cttc.es
 *
 * Reference:
 * J. Vila-Valls, P. Closas, M. Navarro and C. Fernandez-Prades,
 * "Are PLLs Dead? A Tutorial on Kalman Filter-based Techniques for Digital
 * Carrier Synchronization", IEEE Aerospace and Electronic Systems Magazine,
 * Vol. 32, No. 7, pp. 28–45, July 2017. DOI: 10.1109/MAES.2017.150260
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "gps_l1_ca_gaussian_tracking_cc.h"
#include "GPS_L1_CA.h"
#include "gnss_satellite.h"
#include "gnss_sdr_flags.h"
#include "gps_sdr_signal_replica.h"
#include "lock_detectors.h"
#include "tracking_discriminators.h"
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <matio.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <iostream>
#include <memory>
#include <sstream>
#include <utility>
#include <vector>


gps_l1_ca_gaussian_tracking_cc_sptr gps_l1_ca_gaussian_make_tracking_cc(
    uint32_t order,
    int64_t fs_in,
    uint32_t vector_length,
    bool dump,
    const std::string &dump_filename,
    float dll_bw_hz,
    float early_late_space_chips,
    bool bce_run,
    uint32_t bce_ptrans,
    uint32_t bce_strans,
    int32_t bce_nu,
    int32_t bce_kappa)
{
    return gps_l1_ca_gaussian_tracking_cc_sptr(new Gps_L1_Ca_Gaussian_Tracking_cc(order,
        fs_in, vector_length, dump, dump_filename, dll_bw_hz, early_late_space_chips,
        bce_run, bce_ptrans, bce_strans, bce_nu, bce_kappa));
}


void Gps_L1_Ca_Gaussian_Tracking_cc::forecast(int noutput_items,
    gr_vector_int &ninput_items_required)
{
    if (noutput_items != 0)
        {
            ninput_items_required[0] = static_cast<int>(d_vector_length) * 2;  // set the required available samples in each call
        }
}


Gps_L1_Ca_Gaussian_Tracking_cc::Gps_L1_Ca_Gaussian_Tracking_cc(
    uint32_t order,
    int64_t fs_in,
    uint32_t vector_length,
    bool dump,
    const std::string &dump_filename,
    float dll_bw_hz,
    float early_late_space_chips,
    bool bce_run,
    uint32_t bce_ptrans,
    uint32_t bce_strans,
    int32_t bce_nu,
    int32_t bce_kappa)
    : gr::block("Gps_L1_Ca_Gaussian_Tracking_cc", gr::io_signature::make(1, 1, sizeof(gr_complex)),
          gr::io_signature::make(1, 1, sizeof(Gnss_Synchro))),
      d_order(order),
      d_vector_length(vector_length),
      d_dump(dump),
      d_acquisition_gnss_synchro(nullptr),
      d_channel(0),
      d_fs_in(fs_in),
      d_early_late_spc_chips(early_late_space_chips),
      d_rem_code_phase_samples(0.0),
      d_rem_code_phase_chips(0.0),
      d_rem_carr_phase_rad(0.0),
      bayes_ptrans(bce_ptrans),
      bayes_strans(bce_strans),
      bayes_nu(bce_nu),
      bayes_kappa(bce_kappa),
      bayes_run(bce_run),
      kf_iter(0),
      d_acq_code_phase_samples(0.0),
      d_acq_carrier_doppler_hz(0.0),
      d_n_correlator_taps(3),
      d_code_freq_chips(GPS_L1_CA_CODE_RATE_CPS),
      d_code_phase_step_chips(0.0),
      d_code_phase_rate_step_chips(0.0),
      d_carrier_doppler_hz(0.0),
      d_carrier_dopplerrate_hz2(0.0),
      d_carrier_phase_step_rad(0.0),
      d_acc_carrier_phase_rad(0.0),
      d_carr_phase_sigma2(0.0),
      d_code_phase_samples(0.0),
      code_error_chips(0.0),
      code_error_filt_chips(0.0),
      d_current_prn_length_samples(static_cast<int>(d_vector_length)),
      d_sample_counter(0),
      d_acq_sample_stamp(0),
      d_cn0_estimation_counter(0),
      d_carrier_lock_test(1),
      d_CN0_SNV_dB_Hz(0),
      d_carrier_lock_threshold(FLAGS_carrier_lock_th),
      d_carrier_lock_fail_counter(0),
      d_enable_tracking(false),
      d_pull_in(false),
      d_dump_filename(dump_filename)

{
    // Telemetry bit synchronization message port input
    this->message_port_register_in(pmt::mp("preamble_timestamp_s"));
    this->message_port_register_out(pmt::mp("events"));
    this->message_port_register_in(pmt::mp("telemetry_to_trk"));

    d_code_loop_filter.set_DLL_BW(dll_bw_hz);

    // Initialization of local code replica
    // Get space for a vector with the C/A code replica sampled 1x/chip
    d_ca_code = volk_gnsssdr::vector<float>(static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS));

    d_correlator_outs = volk_gnsssdr::vector<gr_complex>(d_n_correlator_taps, gr_complex(0.0, 0.0));

    d_local_code_shift_chips = volk_gnsssdr::vector<float>(d_n_correlator_taps);
    // Set TAPs delay values [chips]
    d_local_code_shift_chips[0] = -d_early_late_spc_chips;
    d_local_code_shift_chips[1] = 0.0;
    d_local_code_shift_chips[2] = d_early_late_spc_chips;

    multicorrelator_cpu.init(2 * d_current_prn_length_samples, d_n_correlator_taps);

    d_Prompt_buffer = volk_gnsssdr::vector<gr_complex>(FLAGS_cn0_samples);

    systemName["G"] = std::string("GPS");
    systemName["S"] = std::string("SBAS");

#if GNURADIO_GREATER_THAN_38
    this->set_relative_rate(1, static_cast<uint64_t>(d_vector_length));
#else
    this->set_relative_rate(1.0 / static_cast<double>(d_vector_length));
#endif

    // Kalman filter initialization (receiver initialization)

    const double CN_dB_Hz = 30;
    const double CN_lin = pow(10, CN_dB_Hz / 10.0);

    const double sigma2_phase_detector_cycles2 = (1.0 / (2.0 * CN_lin * GPS_L1_CA_CODE_PERIOD_S)) * (1.0 + 1.0 / (2.0 * CN_lin * GPS_L1_CA_CODE_PERIOD_S));

    // covariances (static)
    const double sigma2_carrier_phase = TWO_PI / 4;
    const double sigma2_doppler = 450;
    const double sigma2_doppler_rate = pow(4.0 * TWO_PI, 2) / 12.0;

    kf_P_x_ini = arma::zeros(2, 2);
    kf_P_x_ini(0, 0) = sigma2_carrier_phase;
    kf_P_x_ini(1, 1) = sigma2_doppler;

    kf_R = arma::zeros(1, 1);
    kf_R(0, 0) = sigma2_phase_detector_cycles2;

    kf_Q = arma::zeros(2, 2);
    kf_Q(0, 0) = pow(GPS_L1_CA_CODE_PERIOD_S, 4);
    kf_Q(1, 1) = GPS_L1_CA_CODE_PERIOD_S;

    kf_F = arma::zeros(2, 2);
    kf_F(0, 0) = 1.0;
    kf_F(0, 1) = TWO_PI * GPS_L1_CA_CODE_PERIOD_S;
    kf_F(1, 0) = 0.0;
    kf_F(1, 1) = 1.0;

    kf_H = arma::zeros(1, 2);
    kf_H(0, 0) = 1.0;

    kf_x = arma::zeros(2, 1);
    kf_y = arma::zeros(1, 1);
    kf_P_y = arma::zeros(1, 1);

    // order three
    if (d_order == 3)
        {
            kf_P_x_ini = arma::resize(kf_P_x_ini, 3, 3);
            kf_P_x_ini(2, 2) = sigma2_doppler_rate;

            kf_Q = arma::zeros(3, 3);
            kf_Q(0, 0) = pow(GPS_L1_CA_CODE_PERIOD_S, 4);
            kf_Q(1, 1) = GPS_L1_CA_CODE_PERIOD_S;
            kf_Q(2, 2) = GPS_L1_CA_CODE_PERIOD_S;

            kf_F = arma::resize(kf_F, 3, 3);
            kf_F(0, 2) = 0.5 * TWO_PI * pow(GPS_L1_CA_CODE_PERIOD_S, 2);
            kf_F(1, 2) = GPS_L1_CA_CODE_PERIOD_S;
            kf_F(2, 0) = 0.0;
            kf_F(2, 1) = 0.0;
            kf_F(2, 2) = 1.0;

            kf_H = arma::resize(kf_H, 1, 3);
            kf_H(0, 2) = 0.0;

            kf_x = arma::resize(kf_x, 3, 1);
            kf_x(2, 0) = 0.0;
        }

    // Gaussian covariance estimator initialization
    kf_R_est = kf_R;

    bayes_estimator.init(arma::zeros(1, 1), bayes_kappa, bayes_nu, (kf_H * kf_P_x_ini * kf_H.t() + kf_R) * (bayes_nu + 2));
}


void Gps_L1_Ca_Gaussian_Tracking_cc::start_tracking()
{
    /*
     *  correct the code phase according to the delay between acq and trk
     */
    d_acq_code_phase_samples = d_acquisition_gnss_synchro->Acq_delay_samples;
    d_acq_carrier_doppler_hz = d_acquisition_gnss_synchro->Acq_doppler_hz;
    d_acq_sample_stamp = d_acquisition_gnss_synchro->Acq_samplestamp_samples;
    d_acq_carrier_doppler_step_hz = static_cast<double>(d_acquisition_gnss_synchro->Acq_doppler_step);

    // Correct Kalman filter covariance according to acq doppler step size (3 sigma)
    if (d_acquisition_gnss_synchro->Acq_doppler_step > 0)
        {
            kf_P_x_ini(1, 1) = pow(d_acq_carrier_doppler_step_hz / 3.0, 2);
            bayes_estimator.init(arma::zeros(1, 1), bayes_kappa, bayes_nu, (kf_H * kf_P_x_ini * kf_H.t() + kf_R) * (bayes_nu + 2));
        }

    int64_t acq_trk_diff_samples;
    double acq_trk_diff_seconds;
    acq_trk_diff_samples = static_cast<int64_t>(d_sample_counter) - static_cast<int64_t>(d_acq_sample_stamp);  // -d_vector_length;
    DLOG(INFO) << "Number of samples between Acquisition and Tracking = " << acq_trk_diff_samples;
    acq_trk_diff_seconds = static_cast<float>(acq_trk_diff_samples) / static_cast<float>(d_fs_in);
    // Doppler effect Fd = (C / (C + Vr)) * F
    double radial_velocity = (GPS_L1_FREQ_HZ + d_acq_carrier_doppler_hz) / GPS_L1_FREQ_HZ;
    // new chip and prn sequence periods based on acq Doppler
    double T_chip_mod_seconds;
    double T_prn_mod_seconds;
    double T_prn_mod_samples;
    d_code_freq_chips = radial_velocity * GPS_L1_CA_CODE_RATE_CPS;
    d_code_phase_step_chips = static_cast<double>(d_code_freq_chips) / static_cast<double>(d_fs_in);
    T_chip_mod_seconds = 1 / d_code_freq_chips;
    T_prn_mod_seconds = T_chip_mod_seconds * GPS_L1_CA_CODE_LENGTH_CHIPS;
    T_prn_mod_samples = T_prn_mod_seconds * static_cast<double>(d_fs_in);

    d_current_prn_length_samples = round(T_prn_mod_samples);

    double T_prn_true_seconds = GPS_L1_CA_CODE_LENGTH_CHIPS / GPS_L1_CA_CODE_RATE_CPS;
    double T_prn_true_samples = T_prn_true_seconds * static_cast<double>(d_fs_in);
    double T_prn_diff_seconds = T_prn_true_seconds - T_prn_mod_seconds;
    double N_prn_diff = acq_trk_diff_seconds / T_prn_true_seconds;
    double corrected_acq_phase_samples;
    double delay_correction_samples;
    corrected_acq_phase_samples = fmod((d_acq_code_phase_samples + T_prn_diff_seconds * N_prn_diff * static_cast<double>(d_fs_in)), T_prn_true_samples);
    if (corrected_acq_phase_samples < 0)
        {
            corrected_acq_phase_samples = T_prn_mod_samples + corrected_acq_phase_samples;
        }
    delay_correction_samples = d_acq_code_phase_samples - corrected_acq_phase_samples;

    d_acq_code_phase_samples = corrected_acq_phase_samples;

    d_carrier_doppler_hz = d_acq_carrier_doppler_hz;
    d_carrier_dopplerrate_hz2 = 0;
    d_carrier_phase_step_rad = TWO_PI * d_carrier_doppler_hz / static_cast<double>(d_fs_in);

    // DLL filter initialization
    d_code_loop_filter.initialize();  // initialize the code filter

    // generate local reference ALWAYS starting at chip 1 (1 sample per chip)
    gps_l1_ca_code_gen_float(d_ca_code, d_acquisition_gnss_synchro->PRN, 0);

    multicorrelator_cpu.set_local_code_and_taps(static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS), d_ca_code.data(), d_local_code_shift_chips.data());
    std::fill_n(d_correlator_outs.begin(), d_n_correlator_taps, gr_complex(0.0, 0.0));

    d_carrier_lock_fail_counter = 0;
    d_rem_code_phase_samples = 0;
    d_rem_carr_phase_rad = 0.0;
    d_rem_code_phase_chips = 0.0;
    d_acc_carrier_phase_rad = 0.0;
    d_carr_phase_sigma2 = 0.0;

    d_code_phase_samples = d_acq_code_phase_samples;

    sys = std::string(1, d_acquisition_gnss_synchro->System);

    // DEBUG OUTPUT
    std::cout << "Tracking of GPS L1 C/A signal started on channel " << d_channel << " for satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << '\n';
    LOG(INFO) << "Starting tracking of satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << " on channel " << d_channel;

    // enable tracking
    d_pull_in = true;
    d_enable_tracking = true;

    LOG(INFO) << "PULL-IN Doppler [Hz]=" << d_carrier_doppler_hz
              << " Code Phase correction [samples]=" << delay_correction_samples
              << " PULL-IN Code Phase [samples]=" << d_acq_code_phase_samples;
}


Gps_L1_Ca_Gaussian_Tracking_cc::~Gps_L1_Ca_Gaussian_Tracking_cc()
{
    if (d_dump_file.is_open())
        {
            try
                {
                    d_dump_file.close();
                }
            catch (const std::exception &ex)
                {
                    LOG(WARNING) << "Exception in Tracking block destructor: " << ex.what();
                }
        }
    if (d_dump)
        {
            if (d_channel == 0)
                {
                    std::cout << "Writing .mat files ...";
                }
            try
                {
                    Gps_L1_Ca_Gaussian_Tracking_cc::save_matfile();
                }
            catch (const std::exception &ex)
                {
                    LOG(WARNING) << "Error saving the .mat file: " << ex.what();
                }

            if (d_channel == 0)
                {
                    std::cout << " done.\n";
                }
        }
    try
        {
            multicorrelator_cpu.free();
        }
    catch (const std::exception &ex)
        {
            LOG(WARNING) << "Exception in Tracking block destructor: " << ex.what();
        }
}


int32_t Gps_L1_Ca_Gaussian_Tracking_cc::save_matfile()
{
    // READ DUMP FILE
    std::ifstream::pos_type size;
    int32_t number_of_double_vars = 1;
    int32_t number_of_float_vars = 19;
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
            std::cerr << "Problem opening dump file:" << e.what() << '\n';
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
    auto abs_VE = std::vector<float>(num_epoch);
    auto abs_E = std::vector<float>(num_epoch);
    auto abs_P = std::vector<float>(num_epoch);
    auto abs_L = std::vector<float>(num_epoch);
    auto abs_VL = std::vector<float>(num_epoch);
    auto Prompt_I = std::vector<float>(num_epoch);
    auto Prompt_Q = std::vector<float>(num_epoch);
    auto PRN_start_sample_count = std::vector<uint64_t>(num_epoch);
    auto acc_carrier_phase_rad = std::vector<float>(num_epoch);
    auto carrier_doppler_hz = std::vector<float>(num_epoch);
    auto carrier_dopplerrate_hz2 = std::vector<float>(num_epoch);
    auto code_freq_chips = std::vector<float>(num_epoch);
    auto carr_error_hz = std::vector<float>(num_epoch);
    auto carr_noise_sigma2 = std::vector<float>(num_epoch);
    auto carr_error_filt_hz = std::vector<float>(num_epoch);
    auto code_error_chips_aux = std::vector<float>(num_epoch);
    auto code_error_filt_chips_aux = std::vector<float>(num_epoch);
    auto CN0_SNV_dB_Hz = std::vector<float>(num_epoch);
    auto carrier_lock_test = std::vector<float>(num_epoch);
    auto aux1 = std::vector<float>(num_epoch);
    auto aux2 = std::vector<double>(num_epoch);
    auto PRN = std::vector<int32_t>(num_epoch);

    try
        {
            if (dump_file.is_open())
                {
                    for (int64_t i = 0; i < num_epoch; i++)
                        {
                            dump_file.read(reinterpret_cast<char *>(&abs_VE[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&abs_E[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&abs_P[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&abs_L[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&abs_VL[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&Prompt_I[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&Prompt_Q[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&PRN_start_sample_count[i]), sizeof(uint64_t));
                            dump_file.read(reinterpret_cast<char *>(&acc_carrier_phase_rad[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&carrier_doppler_hz[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&carrier_dopplerrate_hz2[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&code_freq_chips[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&carr_error_hz[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&carr_noise_sigma2[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&carr_error_filt_hz[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&code_error_chips_aux[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&code_error_filt_chips_aux[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&CN0_SNV_dB_Hz[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&carrier_lock_test[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&aux1[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&aux2[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&PRN[i]), sizeof(uint32_t));
                        }
                }
            dump_file.close();
        }
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Problem reading dump file:" << e.what() << '\n';
            return 1;
        }

    // WRITE MAT FILE
    mat_t *matfp;
    matvar_t *matvar;
    std::string filename = d_dump_filename;
    filename.erase(filename.length() - 4, 4);
    filename.append(".mat");
    matfp = Mat_CreateVer(filename.c_str(), nullptr, MAT_FT_MAT73);
    if (reinterpret_cast<int64_t *>(matfp) != nullptr)
        {
            std::array<size_t, 2> dims{1, static_cast<size_t>(num_epoch)};
            matvar = Mat_VarCreate("abs_VE", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), abs_VE.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("abs_E", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), abs_E.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("abs_P", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), abs_P.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("abs_L", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), abs_L.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("abs_VL", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), abs_VL.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("Prompt_I", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), Prompt_I.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("Prompt_Q", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), Prompt_Q.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("PRN_start_sample_count", MAT_C_UINT64, MAT_T_UINT64, 2, dims.data(), PRN_start_sample_count.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("acc_carrier_phase_rad", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), acc_carrier_phase_rad.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("carrier_doppler_hz", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), carrier_doppler_hz.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("carrier_dopplerrate_hz2", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), carrier_dopplerrate_hz2.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("code_freq_chips", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), code_freq_chips.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("carr_error_hz", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), carr_error_hz.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("carr_noise_sigma2", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), carr_noise_sigma2.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("carr_error_filt_hz", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), carr_error_filt_hz.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("code_error_chips", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), code_error_chips_aux.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("code_error_filt_chips", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), code_error_filt_chips_aux.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("CN0_SNV_dB_Hz", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), CN0_SNV_dB_Hz.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("carrier_lock_test", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), carrier_lock_test.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("aux1", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), aux1.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("aux2", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), aux2.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("PRN", MAT_C_UINT32, MAT_T_UINT32, 2, dims.data(), PRN.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);
        }
    Mat_Close(matfp);
    return 0;
}


void Gps_L1_Ca_Gaussian_Tracking_cc::set_channel(uint32_t channel)
{
    gr::thread::scoped_lock l(d_setlock);
    d_channel = channel;
    LOG(INFO) << "Tracking Channel set to " << d_channel;
    // ############# ENABLE DATA FILE LOG #################
    if (d_dump)
        {
            if (!d_dump_file.is_open())
                {
                    try
                        {
                            d_dump_filename.append(std::to_string(d_channel));
                            d_dump_filename.append(".dat");
                            d_dump_file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "Tracking dump enabled on channel " << d_channel << " Log file: " << d_dump_filename.c_str();
                        }
                    catch (const std::ofstream::failure &e)
                        {
                            LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump file " << e.what();
                        }
                }
        }
}


void Gps_L1_Ca_Gaussian_Tracking_cc::set_gnss_synchro(Gnss_Synchro *p_gnss_synchro)
{
    d_acquisition_gnss_synchro = p_gnss_synchro;
}


int Gps_L1_Ca_Gaussian_Tracking_cc::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    // process vars
    d_carr_phase_error_rad = 0.0;
    code_error_chips = 0.0;
    code_error_filt_chips = 0.0;
    bool loss_of_lock = false;

    // Block input data and block output stream pointers
    const auto *in = reinterpret_cast<const gr_complex *>(input_items[0]);
    auto **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);

    // GNSS_SYNCHRO OBJECT to interchange data between tracking->telemetry_decoder
    Gnss_Synchro current_synchro_data = Gnss_Synchro();

    if (d_enable_tracking == true)
        {
            // Fill the acquisition data
            current_synchro_data = *d_acquisition_gnss_synchro;
            // Receiver signal alignment
            if (d_pull_in == true)
                {
                    // Signal alignment (skip samples until the incoming signal is aligned with local replica)
                    uint64_t acq_to_trk_delay_samples = d_sample_counter - d_acq_sample_stamp;
                    double acq_trk_shif_correction_samples = static_cast<double>(d_current_prn_length_samples) - std::fmod(static_cast<double>(acq_to_trk_delay_samples), static_cast<double>(d_current_prn_length_samples));
                    int32_t samples_offset = std::round(d_acq_code_phase_samples + acq_trk_shif_correction_samples);
                    if (samples_offset < 0)
                        {
                            samples_offset = 0;
                        }
                    d_acc_carrier_phase_rad -= d_carrier_phase_step_rad * d_acq_code_phase_samples;

                    d_sample_counter += samples_offset;  // count for the processed samples
                    d_pull_in = false;

                    current_synchro_data.Carrier_phase_rads = d_acc_carrier_phase_rad;
                    current_synchro_data.Carrier_Doppler_hz = d_carrier_doppler_hz;
                    current_synchro_data.fs = d_fs_in;
                    current_synchro_data.correlation_length_ms = 1;
                    *out[0] = current_synchro_data;
                    // Kalman filter initialization reset
                    kf_P_x = kf_P_x_ini;
                    // Update Kalman states based on acquisition information
                    kf_x(0) = d_carrier_phase_step_rad * samples_offset;
                    kf_x(1) = d_carrier_doppler_hz;
                    if (kf_x.n_elem > 2)
                        {
                            kf_x(2) = d_carrier_dopplerrate_hz2;
                        }

                    // Covariance estimation initialization reset
                    kf_iter = 0;
                    bayes_estimator.init(arma::zeros(1, 1), bayes_kappa, bayes_nu, (kf_H * kf_P_x_ini * kf_H.t() + kf_R) * (bayes_nu + 2));

                    consume_each(samples_offset);  // shift input to perform alignment with local replica
                    return 1;
                }

            // ################# CARRIER WIPEOFF AND CORRELATORS ##############################
            // Perform carrier wipe-off and compute Early, Prompt and Late correlation
            multicorrelator_cpu.set_input_output_vectors(d_correlator_outs.data(), in);
            multicorrelator_cpu.Carrier_wipeoff_multicorrelator_resampler(d_rem_carr_phase_rad,
                static_cast<float>(d_carrier_phase_step_rad),
                static_cast<float>(d_rem_code_phase_chips),
                static_cast<float>(d_code_phase_step_chips),
                static_cast<float>(d_code_phase_rate_step_chips),
                d_current_prn_length_samples);

            // ################## Kalman Carrier Tracking ######################################

            // Kalman state prediction (time update)
            kf_x_pre = kf_F * kf_x;                        // state prediction
            kf_P_x_pre = kf_F * kf_P_x * kf_F.t() + kf_Q;  // state error covariance prediction

            // Update discriminator [rads/Ti]
            d_carr_phase_error_rad = pll_cloop_two_quadrant_atan(d_correlator_outs[1]);  // prompt output

            // Kalman estimation (measurement update)
            double sigma2_phase_detector_cycles2;
            const double CN_lin = pow(10, d_CN0_SNV_dB_Hz / 10.0);
            sigma2_phase_detector_cycles2 = (1.0 / (2.0 * CN_lin * GPS_L1_CA_CODE_PERIOD_S)) * (1.0 + 1.0 / (2.0 * CN_lin * GPS_L1_CA_CODE_PERIOD_S));

            kf_y(0) = d_carr_phase_error_rad;  // measurement vector
            kf_R(0, 0) = sigma2_phase_detector_cycles2;

            if (bayes_run && (kf_iter >= bayes_ptrans))
                {
                    bayes_estimator.update_sequential(kf_y);
                }
            if (bayes_run && (kf_iter >= (bayes_ptrans + bayes_strans)))
                {
                    // TODO: Resolve segmentation fault
                    kf_P_y = bayes_estimator.get_Psi_est();
                    kf_R_est = kf_P_y - kf_H * kf_P_x_pre * kf_H.t();
                }
            else
                {
                    kf_P_y = kf_H * kf_P_x_pre * kf_H.t() + kf_R;  // innovation covariance matrix
                    kf_R_est = kf_R;
                }

            // Kalman filter update step
            kf_K = (kf_P_x_pre * kf_H.t()) * arma::inv(kf_P_y);                 // Kalman gain
            kf_x = kf_x_pre + kf_K * kf_y;                                      // updated state estimation
            kf_P_x = (arma::eye(size(kf_P_x_pre)) - kf_K * kf_H) * kf_P_x_pre;  // update state estimation error covariance matrix

            // Store Kalman filter results
            d_rem_carr_phase_rad = kf_x(0);  // set a new carrier Phase estimation to the NCO
            d_carrier_doppler_hz = kf_x(1);  // set a new carrier Doppler estimation to the NCO
            if (kf_x.n_elem > 2)
                {
                    d_carrier_dopplerrate_hz2 = kf_x(2);
                }
            else
                {
                    d_carrier_dopplerrate_hz2 = 0;
                }
            d_carr_phase_sigma2 = kf_R_est(0, 0);

            // ################## DLL ##########################################################
            // New code Doppler frequency estimation based on carrier frequency estimation
            d_code_freq_chips = GPS_L1_CA_CODE_RATE_CPS + ((d_carrier_doppler_hz * GPS_L1_CA_CODE_RATE_CPS) / GPS_L1_FREQ_HZ);
            // DLL discriminator
            code_error_chips = dll_nc_e_minus_l_normalized(d_correlator_outs[0], d_correlator_outs[2], static_cast<float>(d_early_late_spc_chips), 1.0);  // [chips/Ti] early and late
            // Code discriminator filter
            code_error_filt_chips = d_code_loop_filter.get_code_nco(static_cast<float>(code_error_chips));  // [chips/second]
            double T_chip_seconds = 1.0 / static_cast<double>(d_code_freq_chips);
            double T_prn_seconds = T_chip_seconds * GPS_L1_CA_CODE_LENGTH_CHIPS;
            double code_error_filt_secs = (T_prn_seconds * code_error_filt_chips * T_chip_seconds);  // [seconds]

            // ################## CARRIER AND CODE NCO BUFFER ALIGNMENT #######################
            // keep alignment parameters for the next input buffer
            // Compute the next buffer length based in the new period of the PRN sequence and the code phase error estimation
            double T_prn_samples = T_prn_seconds * static_cast<double>(d_fs_in);
            double K_blk_samples = T_prn_samples + d_rem_code_phase_samples + code_error_filt_secs * static_cast<double>(d_fs_in);
            d_current_prn_length_samples = static_cast<int>(round(K_blk_samples));  // round to a discrete number of samples

            // ################### NCO COMMANDS #################################################
            // carrier phase step (NCO phase increment per sample) [rads/sample]
            d_carrier_phase_step_rad = TWO_PI * d_carrier_doppler_hz / static_cast<double>(d_fs_in);
            // carrier phase accumulator
            d_acc_carrier_phase_rad -= d_carrier_phase_step_rad * static_cast<double>(d_current_prn_length_samples);

            // ################### DLL COMMANDS #################################################
            // code phase step (Code resampler phase increment per sample) [chips/sample]
            d_code_phase_step_chips = d_code_freq_chips / static_cast<double>(d_fs_in);
            // remnant code phase [chips]
            d_rem_code_phase_samples = K_blk_samples - static_cast<double>(d_current_prn_length_samples);  // rounding error < 1 sample
            d_rem_code_phase_chips = d_code_freq_chips * (d_rem_code_phase_samples / static_cast<double>(d_fs_in));

            // ####### CN0 ESTIMATION AND LOCK DETECTORS ######
            if (d_cn0_estimation_counter < FLAGS_cn0_samples)
                {
                    // fill buffer with prompt correlator output values
                    d_Prompt_buffer[d_cn0_estimation_counter] = d_correlator_outs[1];  // prompt
                    d_cn0_estimation_counter++;
                }
            else
                {
                    d_cn0_estimation_counter = 0;
                    // Code lock indicator
                    d_CN0_SNV_dB_Hz = cn0_m2m4_estimator(d_Prompt_buffer.data(), FLAGS_cn0_samples, GPS_L1_CA_CODE_PERIOD_S);
                    // Carrier lock indicator
                    d_carrier_lock_test = carrier_lock_detector(d_Prompt_buffer.data(), FLAGS_cn0_samples);
                    // Loss of lock detection
                    if (d_carrier_lock_test < d_carrier_lock_threshold or d_CN0_SNV_dB_Hz < FLAGS_cn0_min)
                        {
                            // if (d_channel == 1)
                            // std::cout << "Carrier Lock Test Fail in channel " << d_channel << ": " << d_carrier_lock_test << " < " << d_carrier_lock_threshold << "," << nfail++ << '\n';
                            d_carrier_lock_fail_counter++;
                            // nfail++;
                        }
                    else
                        {
                            if (d_carrier_lock_fail_counter > 0)
                                {
                                    d_carrier_lock_fail_counter--;
                                }
                        }
                    if (d_carrier_lock_fail_counter > FLAGS_max_lock_fail)
                        {
                            std::cout << "Loss of lock in channel " << d_channel << "!\n";
                            LOG(INFO) << "Loss of lock in channel " << d_channel << "!";
                            this->message_port_pub(pmt::mp("events"), pmt::from_long(3));  // 3 -> loss of lock
                            d_carrier_lock_fail_counter = 0;
                            d_enable_tracking = false;
                            loss_of_lock = true;
                        }
                }
            // ########### Output the tracking data to navigation and PVT ##########
            current_synchro_data.Prompt_I = static_cast<double>((d_correlator_outs[1]).real());
            current_synchro_data.Prompt_Q = static_cast<double>((d_correlator_outs[1]).imag());
            current_synchro_data.Tracking_sample_counter = d_sample_counter + static_cast<uint64_t>(d_current_prn_length_samples);
            current_synchro_data.Code_phase_samples = d_rem_code_phase_samples;
            current_synchro_data.Carrier_phase_rads = d_acc_carrier_phase_rad;
            current_synchro_data.Carrier_Doppler_hz = d_carrier_doppler_hz;
            current_synchro_data.CN0_dB_hz = d_CN0_SNV_dB_Hz;
            current_synchro_data.Flag_valid_symbol_output = !loss_of_lock;
            current_synchro_data.correlation_length_ms = 1;

            kf_iter++;
        }
    else
        {
            for (int32_t n = 0; n < d_n_correlator_taps; n++)
                {
                    d_correlator_outs[n] = gr_complex(0, 0);
                }

            current_synchro_data.Tracking_sample_counter = d_sample_counter + static_cast<uint64_t>(d_current_prn_length_samples);
            current_synchro_data.System = {'G'};
            current_synchro_data.correlation_length_ms = 1;
        }

    // assign the GNU Radio block output data
    current_synchro_data.fs = d_fs_in;
    *out[0] = current_synchro_data;

    if (d_dump)
        {
            // MULTIPLEXED FILE RECORDING - Record results to file
            float prompt_I;
            float prompt_Q;
            float tmp_E;
            float tmp_P;
            float tmp_L;
            float tmp_VE = 0.0;
            float tmp_VL = 0.0;
            float tmp_float;
            double tmp_double;
            prompt_I = d_correlator_outs[1].real();
            prompt_Q = d_correlator_outs[1].imag();
            tmp_E = std::abs<float>(d_correlator_outs[0]);
            tmp_P = std::abs<float>(d_correlator_outs[1]);
            tmp_L = std::abs<float>(d_correlator_outs[2]);
            try
                {
                    // EPR
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
                    tmp_float = static_cast<float>(d_acc_carrier_phase_rad);
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    // carrier and code frequency
                    tmp_float = static_cast<float>(d_carrier_doppler_hz);
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    tmp_float = static_cast<float>(d_carrier_dopplerrate_hz2);
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    tmp_float = static_cast<float>(d_code_freq_chips);
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    // Kalman commands
                    tmp_float = static_cast<float>(d_carr_phase_error_rad * TWO_PI);
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    tmp_float = static_cast<float>(d_carr_phase_sigma2);
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    tmp_float = static_cast<float>(d_rem_carr_phase_rad * TWO_PI);
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    // DLL commands
                    tmp_float = static_cast<float>(code_error_chips);
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    tmp_float = static_cast<float>(code_error_filt_chips);
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    // CN0 and carrier lock test
                    tmp_float = static_cast<float>(d_CN0_SNV_dB_Hz);
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    tmp_float = static_cast<float>(d_carrier_lock_test);
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    // AUX vars (for debug purposes)
                    tmp_float = static_cast<float>(d_rem_code_phase_samples);
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    tmp_double = static_cast<double>(d_sample_counter + static_cast<uint64_t>(d_current_prn_length_samples));
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                    // PRN
                    uint32_t prn_ = d_acquisition_gnss_synchro->PRN;
                    d_dump_file.write(reinterpret_cast<char *>(&prn_), sizeof(uint32_t));
                }
            catch (const std::ofstream::failure &e)
                {
                    LOG(WARNING) << "Exception writing trk dump file " << e.what();
                }
        }

    consume_each(d_current_prn_length_samples);        // this is necessary in gr::block derivates
    d_sample_counter += d_current_prn_length_samples;  // count for the processed samples
    return 1;                                          // output tracking result ALWAYS even in the case of d_enable_tracking==false
}
