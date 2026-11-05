/*!
 * \file signal_generator_c.cc
 * \brief GNU Radio source block that generates synthesized GNSS signal.
 * \author Marc Molina, 2013. marc.molina.pena@gmail.com
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

#include "signal_generator_c.h"
#include "Beidou_B1I.h"
#include "GLONASS_L1_L2_CA.h"
#include "GPS_L1_CA.h"
#include "Galileo_E1.h"
#include "Galileo_E5a.h"
#include "Galileo_E5b.h"
#include "Galileo_E6.h"
#include "beidou_b1i_signal_replica.h"
#include "galileo_e1_signal_replica.h"
#include "galileo_e5_signal_replica.h"
#include "galileo_e6_signal_replica.h"
#include "glonass_l1_signal_replica.h"
#include "glonass_l2_signal_replica.h"
#include "gps_sdr_signal_replica.h"
#include <gnuradio/io_signature.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>


namespace
{
enum class Gen_Signal
{
    GPS_L1_CA,
    GLONASS_L1_CA,
    GLONASS_L2_CA,
    GALILEO_E1,
    GALILEO_E5A,
    GALILEO_E5B,
    GALILEO_E6,
    BEIDOU_B1I
};


// Same dispatch rules as the generation branches below
Gen_Signal classify_signal(const std::string &system, const std::string &signal)
{
    if (system == "G")
        {
            return Gen_Signal::GPS_L1_CA;
        }
    if (system == "R")
        {
            return (!signal.empty() && signal.at(0) == '2') ? Gen_Signal::GLONASS_L2_CA : Gen_Signal::GLONASS_L1_CA;
        }
    if (system == "E")
        {
            if (!signal.empty() && signal.at(0) == '5')
                {
                    return Gen_Signal::GALILEO_E5A;
                }
            if (!signal.empty() && signal.at(0) == '7')
                {
                    return Gen_Signal::GALILEO_E5B;
                }
            if (signal.size() > 1 && signal.at(1) == '6')
                {
                    return Gen_Signal::GALILEO_E6;
                }
            return Gen_Signal::GALILEO_E1;
        }
    if (system == "C" && signal == "B1")
        {
            return Gen_Signal::BEIDOU_B1I;
        }
    throw std::invalid_argument("Signal generator: unsupported signal \"" + signal + "\" for system \"" + system +
                                "\". Supported: G/1C, R/1G, R/2G, E/1B, E/5X, E/7X, E/E6, C/B1.");
}


// BeiDou GEO satellites (D2 navigation message, no NH code)
bool is_beidou_geo(unsigned int prn)
{
    return (prn > 0 && prn < 6) || (prn > 58);
}


struct Signal_Timing
{
    unsigned int samples_per_code;      // samples of one primary code period
    unsigned int code_period_ms;        // primary code period [ms]
    unsigned int min_vector_ms;         // shortest vector duration this signal can be generated in [ms]
    unsigned int data_bit_duration_ms;  // data symbol duration [ms]
};


unsigned int samples_per_code(unsigned int fs_in, double chip_rate_cps, double code_length_chips)
{
    return static_cast<unsigned int>(std::round(static_cast<double>(fs_in) / (chip_rate_cps / code_length_chips)));
}


unsigned int duration_ms(double rate_per_s)
{
    return static_cast<unsigned int>(std::lround(1e3 / rate_per_s));
}


Signal_Timing get_signal_timing(const std::string &system, const std::string &signal, unsigned int fs_in, unsigned int prn)
{
    // The code and secondary-code tables are indexed by PRN - 1
    if ((system == "E" && (prn < 1 || prn > 50)) || (system == "C" && (prn < 1 || prn > 63)))
        {
            throw std::invalid_argument("Signal generator: PRN " + std::to_string(prn) + " out of range for system \"" + system + "\"");
        }
    switch (classify_signal(system, signal))
        {
        case Gen_Signal::GPS_L1_CA:
            return {samples_per_code(fs_in, GPS_L1_CA_CODE_RATE_CPS, GPS_L1_CA_CODE_LENGTH_CHIPS), 1, 1, duration_ms(GPS_CA_TELEMETRY_RATE_BITS_SECOND)};
        case Gen_Signal::GLONASS_L1_CA:
            return {samples_per_code(fs_in, GLONASS_L1_CA_CODE_RATE_CPS, GLONASS_L1_CA_CODE_LENGTH_CHIPS), 1, 1, duration_ms(GLONASS_GNAV_TELEMETRY_RATE_BITS_SECOND)};
        case Gen_Signal::GLONASS_L2_CA:
            return {samples_per_code(fs_in, GLONASS_L2_CA_CODE_RATE_CPS, GLONASS_L2_CA_CODE_LENGTH_CHIPS), 1, 1, duration_ms(GLONASS_GNAV_TELEMETRY_RATE_BITS_SECOND)};
        case Gen_Signal::GALILEO_E1:
            {
                const auto code_period_ms = static_cast<unsigned int>(std::lround(1e3 * GALILEO_E1_CODE_PERIOD_S));
                // The E1-C pilot is pre-generated over one full secondary code period
                return {samples_per_code(fs_in, GALILEO_E1_CODE_CHIP_RATE_CPS, GALILEO_E1_B_CODE_LENGTH_CHIPS), code_period_ms,
                    code_period_ms * static_cast<unsigned int>(GALILEO_E1_C_SECONDARY_CODE_LENGTH), duration_ms(GALILEO_E1_B_SYMBOL_RATE_BPS)};
            }
        case Gen_Signal::GALILEO_E5A:
            return {samples_per_code(fs_in, GALILEO_E5A_CODE_CHIP_RATE_CPS, GALILEO_E5A_CODE_LENGTH_CHIPS), 1, 1, duration_ms(GALILEO_E5A_SYMBOL_RATE_BPS)};
        case Gen_Signal::GALILEO_E5B:
            return {samples_per_code(fs_in, GALILEO_E5B_CODE_CHIP_RATE_CPS, GALILEO_E5B_CODE_LENGTH_CHIPS), 1, 1, duration_ms(GALILEO_E5B_SYMBOL_RATE_BPS)};
        case Gen_Signal::GALILEO_E6:
            // The E6-C pilot is pre-generated over one full secondary code period
            return {samples_per_code(fs_in, GALILEO_E6_B_CODE_CHIP_RATE_CPS, GALILEO_E6_B_CODE_LENGTH_CHIPS), 1,
                static_cast<unsigned int>(GALILEO_E6_C_SECONDARY_CODE_LENGTH_CHIPS), 1};
        case Gen_Signal::BEIDOU_B1I:
            {
                // D1: 50 bit/s, each bit spread by the 20-chip NH code; D2 (GEO): 500 bit/s, no NH code
                const unsigned int symbols_per_bit = is_beidou_geo(prn) ? static_cast<unsigned int>(BEIDOU_B1I_GEO_TELEMETRY_SYMBOLS_PER_BIT) : static_cast<unsigned int>(BEIDOU_B1I_TELEMETRY_SYMBOLS_PER_BIT);
                return {samples_per_code(fs_in, BEIDOU_B1I_CODE_RATE_CPS, BEIDOU_B1I_CODE_LENGTH_CHIPS), 1, 1, symbols_per_bit * BEIDOU_B1I_CODE_PERIOD_MS};
            }
        }
    throw std::invalid_argument("Signal generator: unsupported signal");
}


// Output vector duration shared by all satellites: 100 ms if any Galileo E1 or E6
// signal is present, 1 ms otherwise. Every code period divides it.
unsigned int get_vector_duration_ms(const std::vector<std::string> &system, const std::vector<std::string> &signal,
    const std::vector<unsigned int> &PRN, unsigned int fs_in)
{
    if (system.size() != signal.size() || system.size() != PRN.size())
        {
            throw std::invalid_argument("Signal generator: inconsistent number of system, signal and PRN entries");
        }
    unsigned int vector_duration_ms = 1;
    for (size_t sat = 0; sat < system.size(); sat++)
        {
            vector_duration_ms = std::max(vector_duration_ms, get_signal_timing(system[sat], signal[sat], fs_in, PRN[sat]).min_vector_ms);
        }
    return vector_duration_ms;
}
}  // namespace


unsigned int signal_generator_c::compute_vector_length(const std::vector<std::string> &system,
    const std::vector<std::string> &signal,
    const std::vector<unsigned int> &PRN,
    unsigned int fs_in)
{
    const unsigned int vector_duration_ms = get_vector_duration_ms(system, signal, PRN, fs_in);
    unsigned int vector_length = 0;
    for (size_t sat = 0; sat < system.size(); sat++)
        {
            const Signal_Timing timing = get_signal_timing(system[sat], signal[sat], fs_in, PRN[sat]);
            vector_length = std::max(vector_length, (vector_duration_ms / timing.code_period_ms) * timing.samples_per_code);
        }
    return vector_length;
}


/*
 * Create a new instance of signal_generator_c and return
 * a boost shared_ptr. This is effectively the public constructor.
 */
signal_generator_c_sptr
signal_make_generator_c(const std::vector<std::string> &signal1, const std::vector<std::string> &system, const std::vector<unsigned int> &PRN,
    const std::vector<float> &CN0_dB, const std::vector<float> &doppler_Hz,
    const std::vector<unsigned int> &delay_chips, const std::vector<unsigned int> &delay_sec, bool data_flag, bool noise_flag,
    unsigned int fs_in, unsigned int vector_length, float BW_BB)
{
    return gnuradio::get_initial_sptr(new signal_generator_c(signal1, system, PRN, CN0_dB, doppler_Hz, delay_chips, delay_sec,
        data_flag, noise_flag, fs_in, vector_length, BW_BB));
}


/*
 * The private constructor
 */
signal_generator_c::signal_generator_c(std::vector<std::string> signal1,
    std::vector<std::string> system,
    const std::vector<unsigned int> &PRN,
    std::vector<float> CN0_dB,
    std::vector<float> doppler_Hz,
    std::vector<unsigned int> delay_chips,
    std::vector<unsigned int> delay_sec,
    bool data_flag,
    bool noise_flag,
    unsigned int fs_in,
    unsigned int vector_length,
    float BW_BB) : gr::block("signal_gen_cc", gr::io_signature::make(0, 0, sizeof(gr_complex)), gr::io_signature::make(1, 1, static_cast<int>(sizeof(gr_complex) * vector_length))),
                   signal_(std::move(signal1)),
                   system_(std::move(system)),
                   CN0_dB_(std::move(CN0_dB)),
                   doppler_Hz_(std::move(doppler_Hz)),
                   PRN_(PRN),
                   delay_chips_(std::move(delay_chips)),
                   delay_sec_(std::move(delay_sec)),
                   BW_BB_(BW_BB * static_cast<float>(fs_in) / 2.0F),
                   fs_in_(fs_in),
                   num_sats_(PRN.size()),
                   vector_length_(vector_length),
                   data_flag_(data_flag),
                   noise_flag_(noise_flag)
{
    init();
    generate_codes();
}


void signal_generator_c::init()
{
    work_counter_ = 0;

    complex_phase_ = std::vector<gr_complex>(vector_length_);
    start_phase_rad_.reserve(num_sats_);
    current_data_bit_int_.reserve(num_sats_);
    ms_counter_.reserve(num_sats_);
    data_modulation_.reserve(num_sats_);
    pilot_modulation_.reserve(num_sats_);
    samples_per_code_.reserve(num_sats_);
    num_of_codes_per_vector_.reserve(num_sats_);
    data_bit_duration_ms_.reserve(num_sats_);

    // Every satellite fills the whole output vector (see compute_vector_length())
    const unsigned int vector_duration_ms = get_vector_duration_ms(system_, signal_, PRN_, fs_in_);

    for (unsigned int sat = 0; sat < num_sats_; sat++)
        {
            start_phase_rad_.push_back(0);
            current_data_bit_int_.push_back(1);
            current_data_bits_.emplace_back(1, 0);
            ms_counter_.push_back(0);
            // E5a/E5b: modulation applied to the delay prefix of the first output
            // vector, i.e. to the tail of the code period preceding ms_counter_ = 0.
            // general_work() updates it at every subsequent code period.
            int32_t initial_data_modulation = 1;
            int32_t initial_pilot_modulation = 1;
            if (system_[sat] == "E" && (signal_[sat].at(0) == '5' || signal_[sat].at(0) == '7'))
                {
                    const bool is_e5a = (signal_[sat].at(0) == '5');
                    const auto i_len = static_cast<uint32_t>(is_e5a ? GALILEO_E5A_I_SECONDARY_CODE_LENGTH : GALILEO_E5B_I_SECONDARY_CODE_LENGTH);
                    const auto q_len = static_cast<uint32_t>(is_e5a ? GALILEO_E5A_Q_SECONDARY_CODE_LENGTH : GALILEO_E5B_Q_SECONDARY_CODE_LENGTH);
                    const char *i_code = is_e5a ? GALILEO_E5A_I_SECONDARY_CODE : GALILEO_E5B_I_SECONDARY_CODE;
                    initial_data_modulation = (i_code[(delay_sec_[sat] % i_len + i_len - 1) % i_len] == '0') ? 1 : -1;
                    const auto num_codes = static_cast<uint32_t>(is_e5a ? GALILEO_E5A_NUMBER_OF_CODES : GALILEO_E5B_NUMBER_OF_CODES);
                    if (PRN_[sat] >= 1 && PRN_[sat] <= num_codes)
                        {
                            const char *q_code = is_e5a ? GALILEO_E5A_Q_SECONDARY_CODE[PRN_[sat] - 1] : GALILEO_E5B_Q_SECONDARY_CODE[PRN_[sat] - 1];
                            initial_pilot_modulation = (q_code[(delay_sec_[sat] % q_len + q_len - 1) % q_len] == '0') ? 1 : -1;
                        }
                }
            else if (system_[sat] == "C" && !is_beidou_geo(PRN_[sat]))
                {
                    // B1I D1: NH code chip of the code period preceding ms_counter_ = 0
                    const auto nh_len = static_cast<uint32_t>(BEIDOU_B1I_SECONDARY_CODE_LENGTH);
                    initial_data_modulation = (BEIDOU_B1I_SECONDARY_CODE_STR[(delay_sec_[sat] % nh_len + nh_len - 1) % nh_len] == '0') ? 1 : -1;
                }
            data_modulation_.push_back(initial_data_modulation);
            pilot_modulation_.push_back(initial_pilot_modulation);

            const Signal_Timing timing = get_signal_timing(system_[sat], signal_[sat], fs_in_, PRN_[sat]);
            samples_per_code_.push_back(timing.samples_per_code);
            num_of_codes_per_vector_.push_back(vector_duration_ms / timing.code_period_ms);
            data_bit_duration_ms_.push_back(timing.data_bit_duration_ms);

            const unsigned int samples_per_vector = num_of_codes_per_vector_[sat] * samples_per_code_[sat];
            if (samples_per_vector > vector_length_)
                {
                    throw std::invalid_argument("Signal generator: output vector of " + std::to_string(vector_length_) +
                                                " samples is too short for satellite " + std::to_string(sat) + ", which needs " +
                                                std::to_string(samples_per_vector) + " samples");
                }
            if (samples_per_vector < vector_length_)
                {
                    std::cerr << "Warning: signal generator satellite " << sat << " fills " << samples_per_vector << " of "
                              << vector_length_ << " samples per output vector, so its code timing is not continuous. "
                              << "Use a sampling frequency that is a multiple of 1 kHz.\n";
                }
        }
}


void signal_generator_c::generate_codes()
{
    sampled_code_data_ = std::vector<std::vector<gr_complex>>(num_sats_, std::vector<gr_complex>(vector_length_));
    sampled_code_pilot_ = std::vector<std::vector<gr_complex>>(num_sats_, std::vector<gr_complex>(vector_length_));

    for (unsigned int sat = 0; sat < num_sats_; sat++)
        {
            // One code period. The code generators write floor(fs * T_code) samples,
            // which never exceeds samples_per_code_[sat] (computed with round()).
            // Guarded because init() does not push an entry for every system.
            std::vector<gr_complex> code(sat < samples_per_code_.size() ? samples_per_code_[sat] : 0);

            if (system_[sat] == "G")
                {
                    // Generate one code-period of 1C signal
                    gps_l1_ca_code_gen_complex_sampled(code, PRN_[sat], fs_in_,
                        static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS) - delay_chips_[sat]);

                    // Obtain the desired CN0 assuming that Pn = 1.
                    if (noise_flag_)
                        {
                            for (unsigned int i = 0; i < samples_per_code_[sat]; i++)
                                {
                                    code[i] *= std::sqrt(std::pow(10.0F, CN0_dB_[sat] / 10.0F) / BW_BB_);
                                }
                        }

                    // Concatenate "num_of_codes_per_vector_" codes
                    for (unsigned int i = 0; i < num_of_codes_per_vector_[sat]; i++)
                        {
                            std::copy_n(code.data(), samples_per_code_[sat],
                                &(sampled_code_data_[sat][i * samples_per_code_[sat]]));
                        }
                }
            else if (system_[sat] == "R")
                {
                    if (!signal_[sat].empty() && signal_[sat].at(0) == '2')
                        {
                            // Generate one code-period of 2G signal
                            glonass_l2_ca_code_gen_complex_sampled(code, fs_in_,
                                static_cast<int>(GLONASS_L2_CA_CODE_LENGTH_CHIPS) - delay_chips_[sat]);
                        }
                    else
                        {
                            // Generate one code-period of 1G signal
                            glonass_l1_ca_code_gen_complex_sampled(code, /*PRN_[sat],*/ fs_in_,
                                static_cast<int>(GLONASS_L1_CA_CODE_LENGTH_CHIPS) - delay_chips_[sat]);
                        }

                    // Obtain the desired CN0 assuming that Pn = 1.
                    if (noise_flag_)
                        {
                            for (unsigned int i = 0; i < samples_per_code_[sat]; i++)
                                {
                                    code[i] *= std::sqrt(std::pow(10.0F, CN0_dB_[sat] / 10.0F) / BW_BB_);
                                }
                        }

                    // Concatenate "num_of_codes_per_vector_" codes
                    for (unsigned int i = 0; i < num_of_codes_per_vector_[sat]; i++)
                        {
                            std::copy_n(code.data(), samples_per_code_[sat],
                                &(sampled_code_data_[sat][i * samples_per_code_[sat]]));
                        }
                }
            else if (system_[sat] == "C")
                {
                    // Generate one code-period of B1I signal
                    const auto codelen = static_cast<unsigned int>(BEIDOU_B1I_CODE_LENGTH_CHIPS);
                    beidou_b1i_code_gen_complex_sampled(code, PRN_[sat], static_cast<int32_t>(fs_in_),
                        (codelen - delay_chips_[sat] % codelen) % codelen);

                    // Obtain the desired CN0 assuming that Pn = 1.
                    if (noise_flag_)
                        {
                            for (unsigned int i = 0; i < samples_per_code_[sat]; i++)
                                {
                                    code[i] *= std::sqrt(std::pow(10.0F, CN0_dB_[sat] / 10.0F) / BW_BB_);
                                }
                        }

                    // Concatenate "num_of_codes_per_vector_" codes
                    for (unsigned int i = 0; i < num_of_codes_per_vector_[sat]; i++)
                        {
                            std::copy_n(code.data(), samples_per_code_[sat],
                                &(sampled_code_data_[sat][i * samples_per_code_[sat]]));
                        }
                }
            else if (system_[sat] == "E")
                {
                    if (signal_[sat].at(0) == '5')
                        {
                            std::array<char, 3> signal = {{'5', 'X', '\0'}};

                            galileo_e5_a_code_gen_complex_sampled(code, PRN_[sat], signal, fs_in_,
                                static_cast<int>(GALILEO_E5A_CODE_LENGTH_CHIPS) - delay_chips_[sat]);
                            // Obtain the desired CN0 assuming that Pn = 1.
                            if (noise_flag_)
                                {
                                    for (unsigned int i = 0; i < samples_per_code_[sat]; i++)
                                        {
                                            code[i] *= std::sqrt(std::pow(10.0F, CN0_dB_[sat] / 10.0F) / BW_BB_ / 2.0F);
                                        }
                                }
                            // Concatenate "num_of_codes_per_vector_" codes
                            for (unsigned int i = 0; i < num_of_codes_per_vector_[sat]; i++)
                                {
                                    std::copy_n(code.data(), samples_per_code_[sat],
                                        &(sampled_code_data_[sat][i * samples_per_code_[sat]]));
                                }
                        }
                    else if (signal_[sat].at(0) == '7')
                        {
                            std::array<char, 3> signal = {{'7', 'X', '\0'}};

                            galileo_e5_b_code_gen_complex_sampled(code, PRN_[sat], signal, fs_in_,
                                static_cast<int>(GALILEO_E5B_CODE_LENGTH_CHIPS) - delay_chips_[sat]);
                            // Obtain the desired CN0 assuming that Pn = 1.
                            if (noise_flag_)
                                {
                                    for (unsigned int i = 0; i < samples_per_code_[sat]; i++)
                                        {
                                            code[i] *= std::sqrt(std::pow(10.0F, CN0_dB_[sat] / 10.0F) / BW_BB_ / 2.0F);
                                        }
                                }
                            // Concatenate "num_of_codes_per_vector_" codes
                            for (unsigned int i = 0; i < num_of_codes_per_vector_[sat]; i++)
                                {
                                    std::copy_n(code.data(), samples_per_code_[sat],
                                        &(sampled_code_data_[sat][i * samples_per_code_[sat]]));
                                }
                        }
                    else if (signal_[sat].at(1) == '6')
                        {
                            // Generate one code-period of E&B signal
                            galileo_e6_b_code_gen_complex_sampled(code, PRN_[sat], fs_in_,
                                static_cast<int>(GALILEO_E6_B_CODE_LENGTH_CHIPS) - delay_chips_[sat]);
                            // Obtain the desired CN0 assuming that Pn = 1.
                            if (noise_flag_)
                                {
                                    for (unsigned int i = 0; i < samples_per_code_[sat]; i++)
                                        {
                                            code[i] *= std::sqrt(std::pow(10.0F, CN0_dB_[sat] / 10.0F) / BW_BB_ / 2.0F);
                                        }
                                }
                            // Concatenate "num_of_codes_per_vector_" codes
                            for (unsigned int i = 0; i < num_of_codes_per_vector_[sat]; i++)
                                {
                                    std::copy_n(code.data(), samples_per_code_[sat],
                                        &(sampled_code_data_[sat][i * samples_per_code_[sat]]));
                                }
                            // Generate E6C signal (100 code-periods, with secondary code)
                            galileo_e6_c_code_gen_complex_sampled(sampled_code_pilot_[sat], PRN_[sat], fs_in_,
                                static_cast<int>(GALILEO_E6_C_CODE_LENGTH_CHIPS) - delay_chips_[sat]);
                            // Obtain the desired CN0 assuming that Pn = 1.
                            if (noise_flag_)
                                {
                                    for (unsigned int i = 0; i < vector_length_; i++)
                                        {
                                            sampled_code_pilot_[sat][i] *= std::sqrt(std::pow(10.0F, CN0_dB_[sat] / 10.0F) / BW_BB_ / 2.0F);
                                        }
                                }
                        }
                    else
                        {
                            // Generate one code-period of E1B signal
                            bool cboc = true;
                            std::array<char, 3> signal = {{'1', 'B', '\0'}};

                            galileo_e1_code_gen_complex_sampled(code, signal, cboc, PRN_[sat], fs_in_,
                                static_cast<int>(GALILEO_E1_B_CODE_LENGTH_CHIPS) - delay_chips_[sat]);

                            // Obtain the desired CN0 assuming that Pn = 1.
                            if (noise_flag_)
                                {
                                    for (unsigned int i = 0; i < samples_per_code_[sat]; i++)
                                        {
                                            code[i] *= std::sqrt(std::pow(10.0F, CN0_dB_[sat] / 10.0F) / BW_BB_ / 2.0F);
                                        }
                                }

                            // Concatenate "num_of_codes_per_vector_" codes
                            for (unsigned int i = 0; i < num_of_codes_per_vector_[sat]; i++)
                                {
                                    std::copy_n(code.data(), samples_per_code_[sat],
                                        &(sampled_code_data_[sat][i * samples_per_code_[sat]]));
                                }

                            // Generate E1C signal (25 code-periods, with secondary code)

                            std::array<char, 3> signal_1C = {{'1', 'C', '\0'}};

                            galileo_e1_code_gen_complex_sampled(sampled_code_pilot_[sat], signal_1C, cboc, PRN_[sat], fs_in_,
                                static_cast<int>(GALILEO_E1_B_CODE_LENGTH_CHIPS) - delay_chips_[sat], true);

                            // Obtain the desired CN0 assuming that Pn = 1.
                            if (noise_flag_)
                                {
                                    for (unsigned int i = 0; i < vector_length_; i++)
                                        {
                                            sampled_code_pilot_[sat][i] *= std::sqrt(std::pow(10.0F, CN0_dB_[sat] / 10.0F) / BW_BB_ / 2.0F);
                                        }
                                }
                        }
                }
        }
}


int signal_generator_c::general_work(int noutput_items __attribute__((unused)),
    gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items __attribute__((unused)),
    gr_vector_void_star &output_items)
{
    auto *out = reinterpret_cast<gr_complex *>(output_items[0]);

    work_counter_++;

    std::default_random_engine e1(r());
    std::default_random_engine e2(r());
    unsigned int out_idx = 0;
    unsigned int i = 0;
    unsigned int k = 0;
    // the intermediate frequency must be set by the user
    unsigned int freq = 4e6;

    for (out_idx = 0; out_idx < vector_length_; out_idx++)
        {
            out[out_idx] = gr_complex(0.0, 0.0);
        }

    for (unsigned int sat = 0; sat < num_sats_; sat++)
        {
            // Carrier frequency of this satellite. GLONASS is generated at an IF plus its
            // FDMA channel offset; the other systems at baseband plus Doppler.
            double carrier_freq_hz = static_cast<double>(doppler_Hz_[sat]);
            if (system_[sat] == "R")
                {
                    const double fdma_step_hz = (!signal_[sat].empty() && signal_[sat].at(0) == '2') ? DFRQ2_GLO : DFRQ1_GLO;
                    carrier_freq_hz += static_cast<double>(freq) + fdma_step_hz * static_cast<double>(GLONASS_PRN.at(PRN_[sat]));
                }
            // Phase step per sample, computed in double and wrapped to [-pi, pi] before
            // the cast to float, to minimize frequency quantization at high IFs.
            const auto phase_step_rad = static_cast<float>(std::remainder(-TWO_PI * carrier_freq_hz / static_cast<double>(fs_in_), TWO_PI));
            std::array<float, 1> _phase{};
            _phase[0] = -start_phase_rad_[sat];
            volk_gnsssdr_s32f_sincos_32fc(complex_phase_.data(), -phase_step_rad, _phase.data(), vector_length_);
            // The kernel returns the phase after the last sample, wrapped to [-pi, pi).
            // Carrying it over keeps the carrier phase continuous across vectors.
            start_phase_rad_[sat] = -_phase[0];

            out_idx = 0;

            if (system_[sat] == "G")
                {
                    auto delay_samples = static_cast<unsigned int>((delay_chips_[sat] % static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS)) * samples_per_code_[sat] / GPS_L1_CA_CODE_LENGTH_CHIPS);

                    for (i = 0; i < num_of_codes_per_vector_[sat]; i++)
                        {
                            for (k = 0; k < delay_samples; k++)
                                {
                                    out[out_idx] += sampled_code_data_[sat][out_idx] * current_data_bits_[sat] * complex_phase_[out_idx];
                                    out_idx++;
                                }

                            if (ms_counter_[sat] == 0 && data_flag_)
                                {
                                    // New random data bit
                                    current_data_bits_[sat] = gr_complex((uniform_dist(e1) % 2) == 0 ? 1 : -1, 0);
                                }

                            for (k = delay_samples; k < samples_per_code_[sat]; k++)
                                {
                                    out[out_idx] += sampled_code_data_[sat][out_idx] * current_data_bits_[sat] * complex_phase_[out_idx];
                                    out_idx++;
                                }

                            ms_counter_[sat] = (ms_counter_[sat] + static_cast<int>(round(1e3 * GPS_L1_CA_CODE_PERIOD_S))) % data_bit_duration_ms_[sat];
                        }
                }

            else if (system_[sat] == "R")
                {
                    auto delay_samples = static_cast<unsigned int>((delay_chips_[sat] % static_cast<int>(GLONASS_L1_CA_CODE_LENGTH_CHIPS)) * samples_per_code_[sat] / GLONASS_L1_CA_CODE_LENGTH_CHIPS);

                    for (i = 0; i < num_of_codes_per_vector_[sat]; i++)
                        {
                            for (k = 0; k < delay_samples; k++)
                                {
                                    out[out_idx] += sampled_code_data_[sat][out_idx] * current_data_bits_[sat] * complex_phase_[out_idx];
                                    out_idx++;
                                }

                            if (ms_counter_[sat] == 0 && data_flag_)
                                {
                                    // New random data bit
                                    current_data_bits_[sat] = gr_complex((uniform_dist(e1) % 2) == 0 ? 1 : -1, 0);
                                }

                            for (k = delay_samples; k < samples_per_code_[sat]; k++)
                                {
                                    out[out_idx] += sampled_code_data_[sat][out_idx] * current_data_bits_[sat] * complex_phase_[out_idx];
                                    out_idx++;
                                }

                            ms_counter_[sat] = (ms_counter_[sat] + static_cast<int>(round(1e3 * GLONASS_L1_CA_CODE_PERIOD_S))) % data_bit_duration_ms_[sat];
                        }
                }

            else if (system_[sat] == "C")
                {
                    const auto codelen = static_cast<unsigned int>(BEIDOU_B1I_CODE_LENGTH_CHIPS);
                    const unsigned int delay_samples = (delay_chips_[sat] % codelen) * samples_per_code_[sat] / codelen;
                    const bool geo = is_beidou_geo(PRN_[sat]);

                    for (i = 0; i < num_of_codes_per_vector_[sat]; i++)
                        {
                            for (k = 0; k < delay_samples; k++)
                                {
                                    out[out_idx] += sampled_code_data_[sat][out_idx] * static_cast<float>(data_modulation_[sat]) * complex_phase_[out_idx];
                                    out_idx++;
                                }

                            // Data symbols start at the first chip of the NH code
                            if ((ms_counter_[sat] + delay_sec_[sat]) % data_bit_duration_ms_[sat] == 0 && data_flag_)
                                {
                                    // New random data bit
                                    current_data_bit_int_[sat] = (uniform_dist(e1) % 2) == 0 ? 1 : -1;
                                }
                            const int32_t nh_chip = geo ? 1 : (BEIDOU_B1I_SECONDARY_CODE_STR[(ms_counter_[sat] + delay_sec_[sat]) % static_cast<uint32_t>(BEIDOU_B1I_SECONDARY_CODE_LENGTH)] == '0' ? 1 : -1);
                            data_modulation_[sat] = current_data_bit_int_[sat] * nh_chip;

                            ms_counter_[sat] = ms_counter_[sat] + BEIDOU_B1I_CODE_PERIOD_MS;

                            for (k = delay_samples; k < samples_per_code_[sat]; k++)
                                {
                                    out[out_idx] += sampled_code_data_[sat][out_idx] * static_cast<float>(data_modulation_[sat]) * complex_phase_[out_idx];
                                    out_idx++;
                                }
                        }
                }

            else if (system_[sat] == "E")
                {
                    if (signal_[sat].at(0) == '5')
                        {
                            // Each work outputs num_of_codes_per_vector_ modulated primary codes
                            int codelen = static_cast<int>(GALILEO_E5A_CODE_LENGTH_CHIPS);
                            unsigned int delay_samples = (delay_chips_[sat] % codelen) * samples_per_code_[sat] / codelen;
                            for (i = 0; i < num_of_codes_per_vector_[sat]; i++)
                                {
                                    for (k = 0; k < delay_samples; k++)
                                        {
                                            out[out_idx] += (gr_complex(sampled_code_data_[sat][out_idx].real() * data_modulation_[sat],
                                                                sampled_code_data_[sat][out_idx].imag() * pilot_modulation_[sat])) *
                                                            complex_phase_[out_idx];
                                            out_idx++;
                                        }

                                    // Data symbols start at the first chip of the secondary code
                                    if ((ms_counter_[sat] + delay_sec_[sat]) % data_bit_duration_ms_[sat] == 0 && data_flag_)
                                        {
                                            // New random data bit
                                            current_data_bit_int_[sat] = (uniform_dist(e1) % 2) == 0 ? 1 : -1;
                                        }
                                    data_modulation_[sat] = current_data_bit_int_[sat] * (GALILEO_E5A_I_SECONDARY_CODE[(ms_counter_[sat] + delay_sec_[sat]) % 20] == '0' ? 1 : -1);
                                    pilot_modulation_[sat] = (GALILEO_E5A_Q_SECONDARY_CODE[PRN_[sat] - 1][((ms_counter_[sat] + delay_sec_[sat]) % 100)] == '0' ? 1 : -1);

                                    ms_counter_[sat] = ms_counter_[sat] + static_cast<int>(round(1e3 * GALILEO_E5A_CODE_PERIOD_S));

                                    for (k = delay_samples; k < samples_per_code_[sat]; k++)
                                        {
                                            out[out_idx] += (gr_complex(sampled_code_data_[sat][out_idx].real() * data_modulation_[sat],
                                                                sampled_code_data_[sat][out_idx].imag() * pilot_modulation_[sat])) *
                                                            complex_phase_[out_idx];
                                            out_idx++;
                                        }
                                }
                        }
                    else if (signal_[sat].at(0) == '7')
                        {
                            // Each work outputs num_of_codes_per_vector_ modulated primary codes
                            int codelen = static_cast<int>(GALILEO_E5B_CODE_LENGTH_CHIPS);
                            unsigned int delay_samples = (delay_chips_[sat] % codelen) * samples_per_code_[sat] / codelen;
                            for (i = 0; i < num_of_codes_per_vector_[sat]; i++)
                                {
                                    for (k = 0; k < delay_samples; k++)
                                        {
                                            out[out_idx] += (gr_complex(sampled_code_data_[sat][out_idx].real() * data_modulation_[sat],
                                                                sampled_code_data_[sat][out_idx].imag() * pilot_modulation_[sat])) *
                                                            complex_phase_[out_idx];
                                            out_idx++;
                                        }

                                    // Data symbols start at the first chip of the secondary code
                                    if ((ms_counter_[sat] + delay_sec_[sat]) % data_bit_duration_ms_[sat] == 0 && data_flag_)
                                        {
                                            // New random data bit
                                            current_data_bit_int_[sat] = (uniform_dist(e1) % 2) == 0 ? 1 : -1;
                                        }
                                    data_modulation_[sat] = current_data_bit_int_[sat] * (GALILEO_E5B_I_SECONDARY_CODE[((ms_counter_[sat] + delay_sec_[sat]) % 4)] == '0' ? 1 : -1);
                                    pilot_modulation_[sat] = (GALILEO_E5B_Q_SECONDARY_CODE[PRN_[sat] - 1][((ms_counter_[sat] + delay_sec_[sat]) % 100)] == '0' ? 1 : -1);

                                    ms_counter_[sat] = ms_counter_[sat] + static_cast<int>(round(1e3 * GALILEO_E5B_CODE_PERIOD_S));

                                    for (k = delay_samples; k < samples_per_code_[sat]; k++)
                                        {
                                            out[out_idx] += (gr_complex(sampled_code_data_[sat][out_idx].real() * data_modulation_[sat],
                                                                sampled_code_data_[sat][out_idx].imag() * pilot_modulation_[sat])) *
                                                            complex_phase_[out_idx];
                                            out_idx++;
                                        }
                                }
                        }
                    else if (signal_[sat].at(1) == '6')
                        {
                            // EACH WORK outputs 1 modulated primary code
                            int codelen = static_cast<int>(GALILEO_E6_C_CODE_LENGTH_CHIPS);
                            unsigned int delay_samples = (delay_chips_[sat] % codelen) * samples_per_code_[sat] / codelen;
                            for (i = 0; i < num_of_codes_per_vector_[sat]; i++)
                                {
                                    for (k = 0; k < delay_samples; k++)
                                        {
                                            out[out_idx] += (sampled_code_data_[sat][out_idx] * current_data_bits_[sat] - sampled_code_pilot_[sat][out_idx]) * complex_phase_[out_idx];
                                            out_idx++;
                                        }

                                    if (ms_counter_[sat] == 0 && data_flag_)
                                        {
                                            // New random data bit
                                            current_data_bits_[sat] = gr_complex((uniform_dist(e1) % 2) == 0 ? 1 : -1, 0);
                                        }

                                    for (k = delay_samples; k < samples_per_code_[sat]; k++)
                                        {
                                            out[out_idx] += (sampled_code_data_[sat][out_idx] * current_data_bits_[sat] - sampled_code_pilot_[sat][out_idx]) * complex_phase_[out_idx];
                                            out_idx++;
                                        }
                                    ms_counter_[sat] = (ms_counter_[sat] + 1) % data_bit_duration_ms_[sat];
                                }
                        }
                    else
                        {
                            auto delay_samples = static_cast<unsigned int>((delay_chips_[sat] % static_cast<int>(GALILEO_E1_B_CODE_LENGTH_CHIPS)) * samples_per_code_[sat] / GALILEO_E1_B_CODE_LENGTH_CHIPS);

                            for (i = 0; i < num_of_codes_per_vector_[sat]; i++)
                                {
                                    for (k = 0; k < delay_samples; k++)
                                        {
                                            out[out_idx] += (sampled_code_data_[sat][out_idx] * current_data_bits_[sat] - sampled_code_pilot_[sat][out_idx]) * complex_phase_[out_idx];
                                            out_idx++;
                                        }

                                    if (ms_counter_[sat] == 0 && data_flag_)
                                        {
                                            // New random data bit
                                            current_data_bits_[sat] = gr_complex((uniform_dist(e1) % 2) == 0 ? 1 : -1, 0);
                                        }

                                    for (k = delay_samples; k < samples_per_code_[sat]; k++)
                                        {
                                            out[out_idx] += (sampled_code_data_[sat][out_idx] * current_data_bits_[sat] - sampled_code_pilot_[sat][out_idx]) * complex_phase_[out_idx];
                                            out_idx++;
                                        }

                                    ms_counter_[sat] = (ms_counter_[sat] + static_cast<int>(round(1e3 * GALILEO_E1_CODE_PERIOD_S))) % data_bit_duration_ms_[sat];
                                }
                        }
                }
        }

    if (noise_flag_)
        {
            for (out_idx = 0; out_idx < vector_length_; out_idx++)
                {
                    out[out_idx] += gr_complex(normal_dist(e1), normal_dist(e2));
                }
        }

    // Tell runtime system how many output items we produced.
    return 1;
}
