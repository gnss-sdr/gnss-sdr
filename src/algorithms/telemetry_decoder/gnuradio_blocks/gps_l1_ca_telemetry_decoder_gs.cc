/*!
 * \file gps_l1_ca_telemetry_decoder_gs.cc
 * \brief Implementation of a NAV message demodulator block based on
 * Kay Borre book MATLAB-based GPS receiver
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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

#include "gps_l1_ca_telemetry_decoder_gs.h"
#include "gps_ephemeris.h"  // for Gps_Ephemeris
#include "gps_iono.h"       // for Gps_Iono
#include "gps_utc_model.h"  // for Gps_Utc_Model
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>        // for make_any
#include <pmt/pmt_sugar.h>  // for mp
#include <cmath>            // for round
#include <cstring>          // for memcpy
#include <exception>        // for exception
#include <iostream>         // for cout
#include <memory>           // for shared_ptr


#ifndef _rotl
#define _rotl(X, N) (((X) << (N)) ^ ((X) >> (32 - (N))))  // Used in the parity check algorithm
#endif


gps_l1_ca_telemetry_decoder_gs_sptr
gps_l1_ca_make_telemetry_decoder_gs(const Gnss_Satellite &satellite, bool dump)
{
    return gps_l1_ca_telemetry_decoder_gs_sptr(new gps_l1_ca_telemetry_decoder_gs(satellite, dump));
}


gps_l1_ca_telemetry_decoder_gs::gps_l1_ca_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    bool dump) : gr::block("gps_navigation_gs", gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
                     gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    // prevent telemetry symbols accumulation in output buffers
    this->set_max_noutput_items(1);

    // Ephemeris data port out
    this->message_port_register_out(pmt::mp("telemetry"));
    // Control messages to tracking block
    this->message_port_register_out(pmt::mp("telemetry_to_trk"));
    d_last_valid_preamble = 0;
    d_sent_tlm_failed_msg = false;

    // initialize internal vars
    d_dump = dump;
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Initializing GPS L1 TELEMETRY DECODER";

    d_bits_per_preamble = GPS_CA_PREAMBLE_LENGTH_BITS;
    d_samples_per_preamble = d_bits_per_preamble;
    d_preamble_period_symbols = GPS_SUBFRAME_BITS;
    // set the preamble
    d_required_symbols = GPS_SUBFRAME_BITS;
    // preamble bits to sampled symbols
    d_frame_length_symbols = GPS_SUBFRAME_BITS * GPS_CA_TELEMETRY_SYMBOLS_PER_BIT;
    d_max_symbols_without_valid_frame = d_required_symbols * 20;  // rise alarm 120 segs without valid tlm
    int32_t n = 0;
    for (int32_t i = 0; i < d_bits_per_preamble; i++)
        {
            if (GPS_CA_PREAMBLE.at(i) == '1')
                {
                    d_preamble_samples[n] = 1;
                    n++;
                }
            else
                {
                    d_preamble_samples[n] = -1;
                    n++;
                }
        }
    d_sample_counter = 0ULL;
    d_stat = 0;
    d_preamble_index = 0ULL;

    d_flag_frame_sync = false;

    d_flag_parity = false;
    d_TOW_at_current_symbol_ms = 0;
    d_TOW_at_Preamble_ms = 0;
    d_CRC_error_counter = 0;
    d_flag_preamble = false;
    d_channel = 0;
    flag_TOW_set = false;
    flag_PLL_180_deg_phase_locked = false;
    d_prev_GPS_frame_4bytes = 0;
    d_symbol_history.set_capacity(d_required_symbols);
}


gps_l1_ca_telemetry_decoder_gs::~gps_l1_ca_telemetry_decoder_gs()
{
    if (d_dump_file.is_open() == true)
        {
            try
                {
                    d_dump_file.close();
                }
            catch (const std::exception &ex)
                {
                    LOG(WARNING) << "Exception in destructor closing the dump file " << ex.what();
                }
        }
}


bool gps_l1_ca_telemetry_decoder_gs::gps_word_parityCheck(uint32_t gpsword)
{
    uint32_t d1, d2, d3, d4, d5, d6, d7, t, parity;
    // XOR as many bits in parallel as possible.  The magic constants pick
    //   up bits which are to be XOR'ed together to implement the GPS parity
    //   check algorithm described in IS-GPS-200E.  This avoids lengthy shift-
    //   and-xor loops.
    d1 = gpsword & 0xFBFFBF00U;
    d2 = _rotl(gpsword, 1U) & 0x07FFBF01U;
    d3 = _rotl(gpsword, 2U) & 0xFC0F8100U;
    d4 = _rotl(gpsword, 3U) & 0xF81FFE02U;
    d5 = _rotl(gpsword, 4U) & 0xFC00000EU;
    d6 = _rotl(gpsword, 5U) & 0x07F00001U;
    d7 = _rotl(gpsword, 6U) & 0x00003000U;
    t = d1 ^ d2 ^ d3 ^ d4 ^ d5 ^ d6 ^ d7;
    // Now XOR the 5 6-bit fields together to produce the 6-bit final result.
    parity = t ^ _rotl(t, 6U) ^ _rotl(t, 12U) ^ _rotl(t, 18U) ^ _rotl(t, 24U);
    parity = parity & 0x3FU;
    if (parity == (gpsword & 0x3FU))
        {
            return true;
        }

    return false;
}


void gps_l1_ca_telemetry_decoder_gs::set_satellite(const Gnss_Satellite &satellite)
{
    d_nav.reset();
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite " << d_satellite;
    d_nav.i_satellite_PRN = d_satellite.get_PRN();
    DLOG(INFO) << "Navigation Satellite set to " << d_satellite;
}


void gps_l1_ca_telemetry_decoder_gs::set_channel(int32_t channel)
{
    d_channel = channel;
    d_nav.i_channel_ID = channel;
    DLOG(INFO) << "Navigation channel set to " << channel;
    // ############# ENABLE DATA FILE LOG #################
    if (d_dump == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                        {
                            d_dump_filename = "telemetry";
                            d_dump_filename.append(std::to_string(d_channel));
                            d_dump_filename.append(".dat");
                            d_dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "Telemetry decoder dump enabled on channel " << d_channel
                                      << " Log file: " << d_dump_filename.c_str();
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump file " << e.what();
                        }
                }
        }
}


bool gps_l1_ca_telemetry_decoder_gs::decode_subframe()
{
    std::array<char, GPS_SUBFRAME_LENGTH> subframe{};
    int32_t frame_bit_index = 0;
    int32_t word_index = 0;
    uint32_t GPS_frame_4bytes = 0;
    bool subframe_synchro_confirmation = true;
    for (float subframe_symbol : d_symbol_history)
        {
            // ******* SYMBOL TO BIT *******
            // symbol to bit
            if (subframe_symbol > 0)
                {
                    GPS_frame_4bytes += 1;  // insert the telemetry bit in LSB
                }

            // ******* bits to words ******
            frame_bit_index++;
            if (frame_bit_index == 30)
                {
                    frame_bit_index = 0;
                    // parity check
                    // Each word in wordbuff is composed of:
                    //      Bits 0 to 29 = the GPS data word
                    //      Bits 30 to 31 = 2 LSBs of the GPS word ahead.
                    // prepare the extended frame [-2 -1 0 ... 30]
                    if (d_prev_GPS_frame_4bytes & 0x00000001U)
                        {
                            GPS_frame_4bytes = GPS_frame_4bytes | 0x40000000U;
                        }
                    if (d_prev_GPS_frame_4bytes & 0x00000002U)
                        {
                            GPS_frame_4bytes = GPS_frame_4bytes | 0x80000000U;
                        }
                    // Check that the 2 most recently logged words pass parity. Have to first
                    // invert the data bits according to bit 30 of the previous word.
                    if (GPS_frame_4bytes & 0x40000000U)
                        {
                            GPS_frame_4bytes ^= 0x3FFFFFC0U;  // invert the data bits (using XOR)
                        }
                    // check parity. If ANY word inside the subframe fails the parity, set subframe_synchro_confirmation = false
                    if (not gps_l1_ca_telemetry_decoder_gs::gps_word_parityCheck(GPS_frame_4bytes))
                        {
                            subframe_synchro_confirmation = false;
                        }
                    // add word to subframe
                    // insert the word in the correct position of the subframe
                    std::memcpy(&subframe[word_index * GPS_WORD_LENGTH], &GPS_frame_4bytes, sizeof(uint32_t));
                    word_index++;
                    d_prev_GPS_frame_4bytes = GPS_frame_4bytes;  // save the actual frame
                    GPS_frame_4bytes = 0;
                }
            else
                {
                    GPS_frame_4bytes <<= 1U;  // shift 1 bit left the telemetry word
                }
        }

    // decode subframe
    // NEW GPS SUBFRAME HAS ARRIVED!
    if (subframe_synchro_confirmation)
        {
            int32_t subframe_ID = d_nav.subframe_decoder(subframe.data());  // decode the subframe
            if (subframe_ID > 0 and subframe_ID < 6)
                {
                    std::cout << "New GPS NAV message received in channel " << this->d_channel << ": "
                              << "subframe "
                              << subframe_ID << " from satellite "
                              << Gnss_Satellite(std::string("GPS"), d_nav.i_satellite_PRN) << std::endl;

                    switch (subframe_ID)
                        {
                        case 3:  // we have a new set of ephemeris data for the current SV
                            if (d_nav.satellite_validation() == true)
                                {
                                    // get ephemeris object for this SV (mandatory)
                                    std::shared_ptr<Gps_Ephemeris> tmp_obj = std::make_shared<Gps_Ephemeris>(d_nav.get_ephemeris());
                                    this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
                                }
                            break;
                        case 4:  // Possible IONOSPHERE and UTC model update (page 18)
                            if (d_nav.flag_iono_valid == true)
                                {
                                    std::shared_ptr<Gps_Iono> tmp_obj = std::make_shared<Gps_Iono>(d_nav.get_iono());
                                    this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
                                }
                            if (d_nav.flag_utc_model_valid == true)
                                {
                                    std::shared_ptr<Gps_Utc_Model> tmp_obj = std::make_shared<Gps_Utc_Model>(d_nav.get_utc_model());
                                    this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
                                }
                            break;
                        case 5:
                            // get almanac (if available)
                            //TODO: implement almanac reader in navigation_message
                            break;
                        default:
                            break;
                        }
                    return true;
                }
        }
    return false;
}


void gps_l1_ca_telemetry_decoder_gs::reset()
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with work function called by the scheduler
    d_last_valid_preamble = d_sample_counter;
    d_sent_tlm_failed_msg = false;
    flag_TOW_set = false;
    d_symbol_history.clear();
    d_stat = 0;
    DLOG(INFO) << "Telemetry decoder reset for satellite " << d_satellite;
}


int gps_l1_ca_telemetry_decoder_gs::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    auto **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);            // Get the output buffer pointer
    const auto **in = reinterpret_cast<const Gnss_Synchro **>(&input_items[0]);  // Get the input buffer pointer

    // 1. Copy the current tracking output
    Gnss_Synchro current_symbol = in[0][0];
    // add new symbol to the symbol queue
    d_symbol_history.push_back(current_symbol.Prompt_I);
    d_sample_counter++;  // count for the processed symbols
    consume_each(1);
    d_flag_preamble = false;
    // check if there is a problem with the telemetry of the current satellite
    if (d_stat < 2 and d_sent_tlm_failed_msg == false)
        {
            if ((d_sample_counter - d_last_valid_preamble) > d_max_symbols_without_valid_frame)
                {
                    int message = 1;  // bad telemetry
                    this->message_port_pub(pmt::mp("telemetry_to_trk"), pmt::make_any(message));
                    d_sent_tlm_failed_msg = true;
                }
        }

    // ******* frame sync ******************
    switch (d_stat)
        {
        case 0:  // no preamble information
            {
                // correlate with preamble
                int32_t corr_value = 0;
                if (d_symbol_history.size() >= GPS_CA_PREAMBLE_LENGTH_BITS)
                    {
                        // ******* preamble correlation ********
                        for (int32_t i = 0; i < GPS_CA_PREAMBLE_LENGTH_BITS; i++)
                            {
                                if (d_symbol_history[i] < 0.0)  // symbols clipping
                                    {
                                        corr_value -= d_preamble_samples[i];
                                    }
                                else
                                    {
                                        corr_value += d_preamble_samples[i];
                                    }
                            }
                    }
                if (abs(corr_value) >= d_samples_per_preamble)
                    {
                        d_preamble_index = d_sample_counter;  // record the preamble sample stamp
                        DLOG(INFO) << "Preamble detection for GPS L1 satellite " << this->d_satellite;
                        decode_subframe();
                        d_stat = 1;  // enter into frame pre-detection status
                    }
                flag_TOW_set = false;
                break;
            }
        case 1:  // possible preamble lock
            {
                // correlate with preamble
                int32_t corr_value = 0;
                int32_t preamble_diff = 0;
                if (d_symbol_history.size() >= GPS_CA_PREAMBLE_LENGTH_BITS)
                    {
                        // ******* preamble correlation ********
                        for (int32_t i = 0; i < GPS_CA_PREAMBLE_LENGTH_BITS; i++)
                            {
                                if (d_symbol_history[i] < 0.0)  // symbols clipping
                                    {
                                        corr_value -= d_preamble_samples[i];
                                    }
                                else
                                    {
                                        corr_value += d_preamble_samples[i];
                                    }
                            }
                    }
                if (abs(corr_value) >= d_samples_per_preamble)
                    {
                        // check preamble separation
                        preamble_diff = static_cast<int32_t>(d_sample_counter - d_preamble_index);
                        if (abs(preamble_diff - d_preamble_period_symbols) == 0)
                            {
                                DLOG(INFO) << "Preamble confirmation for SAT " << this->d_satellite;
                                d_preamble_index = d_sample_counter;  // record the preamble sample stamp
                                if (corr_value < 0)
                                    {
                                        flag_PLL_180_deg_phase_locked = true;
                                    }
                                else
                                    {
                                        flag_PLL_180_deg_phase_locked = false;
                                    }
                                decode_subframe();
                                d_stat = 2;
                            }
                        else
                            {
                                if (preamble_diff > d_preamble_period_symbols)
                                    {
                                        d_stat = 0;  // start again
                                        flag_TOW_set = false;
                                    }
                            }
                    }
                break;
            }
        case 2:  // preamble acquired
            {
                if (d_sample_counter >= d_preamble_index + static_cast<uint64_t>(d_preamble_period_symbols))
                    {
                        DLOG(INFO) << "Preamble received for SAT " << this->d_satellite << "d_sample_counter=" << d_sample_counter << "\n";
                        // call the decoder
                        // 0. fetch the symbols into an array
                        d_preamble_index = d_sample_counter;  // record the preamble sample stamp (t_P)

                        if (decode_subframe())
                            {
                                d_CRC_error_counter = 0;
                                d_flag_preamble = true;  // valid preamble indicator (initialized to false every work())
                                gr::thread::scoped_lock lock(d_setlock);
                                d_last_valid_preamble = d_sample_counter;
                                if (!d_flag_frame_sync)
                                    {
                                        d_flag_frame_sync = true;
                                        DLOG(INFO) << " Frame sync SAT " << this->d_satellite;
                                    }
                            }
                        else
                            {
                                d_CRC_error_counter++;
                                if (d_CRC_error_counter > 2)
                                    {
                                        DLOG(INFO) << "Lost of frame sync SAT " << this->d_satellite;
                                        d_flag_frame_sync = false;
                                        d_stat = 0;
                                        d_TOW_at_current_symbol_ms = 0;
                                        d_TOW_at_Preamble_ms = 0;
                                        d_CRC_error_counter = 0;
                                        flag_TOW_set = false;
                                    }
                            }
                    }
                break;
            }
        }

    // 2. Add the telemetry decoder information
    if (d_flag_preamble == true)
        {
            if (!(d_nav.d_TOW == 0))
                {
                    d_TOW_at_current_symbol_ms = static_cast<uint32_t>(d_nav.d_TOW * 1000.0);
                    d_TOW_at_Preamble_ms = static_cast<uint32_t>(d_nav.d_TOW * 1000.0);
                    flag_TOW_set = true;
                }
            else
                {
                    DLOG(INFO) << "Received GPS L1 TOW equal to zero at sat " << d_nav.i_satellite_PRN;
                }
        }
    else
        {
            if (flag_TOW_set == true)
                {
                    d_TOW_at_current_symbol_ms += GPS_L1_CA_BIT_PERIOD_MS;
                }
        }

    if (flag_TOW_set == true)
        {
            current_symbol.TOW_at_current_symbol_ms = d_TOW_at_current_symbol_ms;
            current_symbol.Flag_valid_word = flag_TOW_set;

            if (flag_PLL_180_deg_phase_locked == true)
                {
                    // correct the accumulated phase for the Costas loop phase shift, if required
                    current_symbol.Carrier_phase_rads += GPS_PI;
                }

            if (d_dump == true)
                {
                    // MULTIPLEXED FILE RECORDING - Record results to file
                    try
                        {
                            double tmp_double;
                            uint64_t tmp_ulong_int;
                            tmp_double = static_cast<double>(d_TOW_at_current_symbol_ms) / 1000.0;
                            d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                            tmp_ulong_int = current_symbol.Tracking_sample_counter;
                            d_dump_file.write(reinterpret_cast<char *>(&tmp_ulong_int), sizeof(uint64_t));
                            tmp_double = static_cast<double>(d_TOW_at_Preamble_ms) / 1000.0;
                            d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            LOG(WARNING) << "Exception writing observables dump file " << e.what();
                        }
                }

            // 3. Make the output (copy the object contents to the GNU Radio reserved memory)
            *out[0] = current_symbol;

            return 1;
        }

    return 0;
}
