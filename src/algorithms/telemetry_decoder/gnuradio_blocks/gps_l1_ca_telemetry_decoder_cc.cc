/*!
 * \file gps_l1_ca_telemetry_decoder_cc.cc
 * \brief Implementation of a NAV message demodulator block based on
 * Kay Borre book MATLAB-based GPS receiver
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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

#include "gps_l1_ca_telemetry_decoder_cc.h"
#include "control_message_factory.h"
#include <boost/lexical_cast.hpp>
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <volk_gnsssdr/volk_gnsssdr.h>


#ifndef _rotl
#define _rotl(X, N) ((X << N) ^ (X >> (32 - N)))  // Used in the parity check algorithm
#endif

using google::LogMessage;

gps_l1_ca_telemetry_decoder_cc_sptr
gps_l1_ca_make_telemetry_decoder_cc(const Gnss_Satellite &satellite, bool dump)
{
    return gps_l1_ca_telemetry_decoder_cc_sptr(new gps_l1_ca_telemetry_decoder_cc(satellite, dump));
}


gps_l1_ca_telemetry_decoder_cc::gps_l1_ca_telemetry_decoder_cc(
    const Gnss_Satellite &satellite,
    bool dump) : gr::block("gps_navigation_cc", gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
                     gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    // Ephemeris data port out
    this->message_port_register_out(pmt::mp("telemetry"));
    // initialize internal vars
    d_dump = dump;
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());

    // set the preamble
    uint16_t preambles_bits[GPS_CA_PREAMBLE_LENGTH_BITS] = GPS_PREAMBLE;

    // preamble bits to sampled symbols
    d_preambles_symbols = static_cast<int32_t *>(volk_gnsssdr_malloc(GPS_CA_PREAMBLE_LENGTH_SYMBOLS * sizeof(int32_t), volk_gnsssdr_get_alignment()));
    int32_t n = 0;
    for (int32_t i = 0; i < GPS_CA_PREAMBLE_LENGTH_BITS; i++)
        {
            for (uint32_t j = 0; j < GPS_CA_TELEMETRY_SYMBOLS_PER_BIT; j++)
                {
                    if (preambles_bits[i] == 1)
                        {
                            d_preambles_symbols[n] = 1;
                        }
                    else
                        {
                            d_preambles_symbols[n] = -1;
                        }
                    n++;
                }
        }
    d_stat = 0;
    d_flag_frame_sync = false;
    d_prev_GPS_frame_4bytes = 0;
    d_TOW_at_Preamble_ms = 0;
    flag_TOW_set = false;
    d_flag_preamble = false;
    d_flag_new_tow_available = false;
    d_channel = 0;
    flag_PLL_180_deg_phase_locked = false;
    d_preamble_time_samples = 0ULL;
    d_TOW_at_current_symbol_ms = 0;
    d_symbol_history.resize(GPS_CA_PREAMBLE_LENGTH_SYMBOLS);  // Change fixed buffer size
    d_symbol_history.clear();                                 // Clear all the elements in the buffer
    d_crc_error_synchronization_counter = 0;
    d_current_subframe_symbol = 0;
}


gps_l1_ca_telemetry_decoder_cc::~gps_l1_ca_telemetry_decoder_cc()
{
    volk_gnsssdr_free(d_preambles_symbols);
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


bool gps_l1_ca_telemetry_decoder_cc::gps_word_parityCheck(uint32_t gpsword)
{
    uint32_t d1, d2, d3, d4, d5, d6, d7, t, parity;
    // XOR as many bits in parallel as possible.  The magic constants pick
    //   up bits which are to be XOR'ed together to implement the GPS parity
    //   check algorithm described in IS-GPS-200E.  This avoids lengthy shift-
    //   and-xor loops.
    d1 = gpsword & 0xFBFFBF00;
    d2 = _rotl(gpsword, 1) & 0x07FFBF01;
    d3 = _rotl(gpsword, 2) & 0xFC0F8100;
    d4 = _rotl(gpsword, 3) & 0xF81FFE02;
    d5 = _rotl(gpsword, 4) & 0xFC00000E;
    d6 = _rotl(gpsword, 5) & 0x07F00001;
    d7 = _rotl(gpsword, 6) & 0x00003000;
    t = d1 ^ d2 ^ d3 ^ d4 ^ d5 ^ d6 ^ d7;
    // Now XOR the 5 6-bit fields together to produce the 6-bit final result.
    parity = t ^ _rotl(t, 6) ^ _rotl(t, 12) ^ _rotl(t, 18) ^ _rotl(t, 24);
    parity = parity & 0x3F;
    if (parity == (gpsword & 0x3F))
        return (true);
    else
        return (false);
}


void gps_l1_ca_telemetry_decoder_cc::set_satellite(const Gnss_Satellite &satellite)
{
    d_nav.reset();
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite " << d_satellite;
    d_nav.i_satellite_PRN = d_satellite.get_PRN();
    DLOG(INFO) << "Navigation Satellite set to " << d_satellite;
}


void gps_l1_ca_telemetry_decoder_cc::set_channel(int32_t channel)
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
                            d_dump_filename.append(boost::lexical_cast<std::string>(d_channel));
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


bool gps_l1_ca_telemetry_decoder_cc::decode_subframe()
{
    char subframe[GPS_SUBFRAME_LENGTH];

    int32_t symbol_accumulator_counter = 0;
    int32_t frame_bit_index = 0;
    int32_t word_index = 0;
    uint32_t GPS_frame_4bytes = 0;
    float symbol_accumulator = 0;
    bool subframe_synchro_confirmation = false;
    bool CRC_ok = true;

    for (int32_t n = 0; n < GPS_SUBFRAME_MS; n++)
        {
            // ******* SYMBOL TO BIT *******
            // extended correlation to bit period is enabled in tracking!
            symbol_accumulator += d_subframe_symbols[n];  // accumulate the input value in d_symbol_accumulator
            symbol_accumulator_counter++;
            if (symbol_accumulator_counter == 20)
                {
                    // symbol to bit
                    if (symbol_accumulator > 0) GPS_frame_4bytes += 1;  // insert the telemetry bit in LSB
                    symbol_accumulator = 0;
                    symbol_accumulator_counter = 0;

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
                            if (d_prev_GPS_frame_4bytes & 0x00000001)
                                {
                                    GPS_frame_4bytes = GPS_frame_4bytes | 0x40000000;
                                }
                            if (d_prev_GPS_frame_4bytes & 0x00000002)
                                {
                                    GPS_frame_4bytes = GPS_frame_4bytes | 0x80000000;
                                }
                            // Check that the 2 most recently logged words pass parity. Have to first
                            // invert the data bits according to bit 30 of the previous word.
                            if (GPS_frame_4bytes & 0x40000000)
                                {
                                    GPS_frame_4bytes ^= 0x3FFFFFC0;  // invert the data bits (using XOR)
                                }
                            if (gps_l1_ca_telemetry_decoder_cc::gps_word_parityCheck(GPS_frame_4bytes))
                                {
                                    subframe_synchro_confirmation = true;
                                }
                            else
                                {
                                    // std::cout << "word invalid sat " << this->d_satellite << std::endl;
                                    CRC_ok = false;
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
                            GPS_frame_4bytes <<= 1;  // shift 1 bit left the telemetry word
                        }
                }
        }

    // decode subframe
    // NEW GPS SUBFRAME HAS ARRIVED!
    if (CRC_ok)
        {
            int32_t subframe_ID = d_nav.subframe_decoder(subframe);  //d ecode the subframe
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
                    d_flag_new_tow_available = true;
                }
            else
                {
                    return false;
                }
        }

    return subframe_synchro_confirmation;
}


int gps_l1_ca_telemetry_decoder_cc::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    int32_t preamble_diff_ms = 0;

    Gnss_Synchro **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);            // Get the output buffer pointer
    const Gnss_Synchro **in = reinterpret_cast<const Gnss_Synchro **>(&input_items[0]);  // Get the input buffer pointer

    Gnss_Synchro current_symbol;  // structure to save the synchronization information and send the output object to the next block
    // 1. Copy the current tracking output
    current_symbol = in[0][0];

    // record the oldest subframe symbol before inserting a new symbol into the circular buffer
    if (d_current_subframe_symbol < GPS_SUBFRAME_MS and d_symbol_history.size() > 0)
        {
            d_subframe_symbols[d_current_subframe_symbol] = d_symbol_history.at(0).Prompt_I;
            d_current_subframe_symbol++;
        }

    d_symbol_history.push_back(current_symbol);  // add new symbol to the symbol queue
    consume_each(1);

    d_flag_preamble = false;

    // ******* preamble correlation ********
    int32_t corr_value = 0;
    if ((d_symbol_history.size() == GPS_CA_PREAMBLE_LENGTH_SYMBOLS))  // and (d_make_correlation or !d_flag_frame_sync))
        {
            // std::cout << "-------\n";
            for (uint32_t i = 0; i < GPS_CA_PREAMBLE_LENGTH_SYMBOLS; i++)
                {
                    if (d_symbol_history.at(i).Flag_valid_symbol_output == true)
                        {
                            if (d_symbol_history.at(i).Prompt_I < 0)  // symbols clipping
                                {
                                    corr_value -= d_preambles_symbols[i];
                                }
                            else
                                {
                                    corr_value += d_preambles_symbols[i];
                                }
                        }
                }
        }

    // ******* frame sync ******************
    if (std::abs(corr_value) == GPS_CA_PREAMBLE_LENGTH_SYMBOLS)
        {
            //TODO: Rewrite with state machine
            if (d_stat == 0)
                {
                    // record the preamble sample stamp
                    d_preamble_time_samples = d_symbol_history.at(0).Tracking_sample_counter;  // record the preamble sample stamp
                    DLOG(INFO) << "Preamble detection for SAT " << this->d_satellite << "d_symbol_history.at(0).Tracking_sample_counter=" << d_symbol_history.at(0).Tracking_sample_counter;
                    d_stat = 1;  // enter into frame pre-detection status
                }
            else if (d_stat == 1)  // check 6 seconds of preamble separation
                {
                    preamble_diff_ms = std::round(((static_cast<double>(d_symbol_history.at(0).Tracking_sample_counter) - d_preamble_time_samples) / static_cast<double>(d_symbol_history.at(0).fs)) * 1000.0);
                    if (std::abs(preamble_diff_ms - GPS_SUBFRAME_MS) % GPS_SUBFRAME_MS == 0)
                        {
                            DLOG(INFO) << "Preamble confirmation for SAT " << this->d_satellite;
                            d_flag_preamble = true;
                            d_preamble_time_samples = d_symbol_history.at(0).Tracking_sample_counter;  // record the PRN start sample index associated to the preamble
                            if (!d_flag_frame_sync)
                                {
                                    d_flag_frame_sync = true;
                                    if (corr_value < 0)
                                        {
                                            flag_PLL_180_deg_phase_locked = true;  // PLL is locked to opposite phase!
                                            DLOG(INFO) << " PLL in opposite phase for Sat " << this->d_satellite.get_PRN();
                                        }
                                    else
                                        {
                                            flag_PLL_180_deg_phase_locked = false;
                                        }
                                    DLOG(INFO) << " Frame sync SAT " << this->d_satellite << " with preamble start at "
                                               << static_cast<double>(d_preamble_time_samples) / static_cast<double>(d_symbol_history.at(0).fs) << " [s]";
                                }

                            // try to decode the subframe:
                            if (decode_subframe() == false)
                                {
                                    d_crc_error_synchronization_counter++;
                                    if (d_crc_error_synchronization_counter > 3)
                                        {
                                            DLOG(INFO) << "TOO MANY CRC ERRORS: Lost of frame sync SAT " << this->d_satellite << std::endl;
                                            d_stat = 0;  // lost of frame sync
                                            d_flag_frame_sync = false;
                                            flag_TOW_set = false;
                                            d_crc_error_synchronization_counter = 0;
                                        }
                                }
                            d_current_subframe_symbol = 0;
                        }
                }
        }
    else
        {
            if (d_stat == 1)
                {
                    preamble_diff_ms = round(((static_cast<double>(d_symbol_history.at(0).Tracking_sample_counter) - static_cast<double>(d_preamble_time_samples)) / static_cast<double>(d_symbol_history.at(0).fs)) * 1000.0);
                    if (preamble_diff_ms > GPS_SUBFRAME_MS)
                        {
                            DLOG(INFO) << "Lost of frame sync SAT " << this->d_satellite << " preamble_diff= " << preamble_diff_ms;
                            //  std::cout << "Lost of frame sync SAT " << this->d_satellite << " preamble_diff= " << preamble_diff_ms << std::endl;
                            d_stat = 0;  // lost of frame sync
                            d_flag_frame_sync = false;
                            flag_TOW_set = false;
                            d_current_subframe_symbol = 0;
                            d_crc_error_synchronization_counter = 0;
                            d_TOW_at_current_symbol_ms = 0;
                        }
                }
        }

    // 2. Add the telemetry decoder information
    if (this->d_flag_preamble == true and d_flag_new_tow_available == true)
        {
            d_TOW_at_current_symbol_ms = static_cast<uint32_t>(d_nav.d_TOW * 1000.0) + GPS_CA_PREAMBLE_DURATION_MS;
            d_TOW_at_Preamble_ms = static_cast<uint32_t>(d_nav.d_TOW * 1000.0);
            flag_TOW_set = true;
            d_flag_new_tow_available = false;
        }
    else
        {
            if (flag_TOW_set == true)
                {
                    d_TOW_at_current_symbol_ms += GPS_L1_CA_CODE_PERIOD_MS;
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

            // 3. Make the output (copy the object contents to the GNURadio reserved memory)
            *out[0] = current_symbol;

            return 1;
        }
    else
        {
            return 0;
        }
}
