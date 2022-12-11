/*!
 * \file glonass_l1_ca_telemetry_decoder_gs.cc
 * \brief Implementation of a GLONASS L1 C/A NAV data decoder block
 * \note Code added as part of GSoC 2017 program
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
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


#include "glonass_l1_ca_telemetry_decoder_gs.h"
#include "glonass_gnav_almanac.h"
#include "glonass_gnav_ephemeris.h"
#include "glonass_gnav_utc_model.h"
#include "gnss_sdr_make_unique.h"  // for std::make_unique in C++11
#include "tlm_utils.h"
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>        // for make_any
#include <pmt/pmt_sugar.h>  // for mp
#include <cmath>            // for floor, round
#include <cstddef>          // for size_t
#include <cstdlib>          // for abs
#include <exception>        // for exception
#include <iostream>         // for cout
#include <memory>           // for shared_ptr, make_shared

#define CRC_ERROR_LIMIT 6


glonass_l1_ca_telemetry_decoder_gs_sptr
glonass_l1_ca_make_telemetry_decoder_gs(const Gnss_Satellite &satellite, const Tlm_Conf &conf)
{
    return glonass_l1_ca_telemetry_decoder_gs_sptr(new glonass_l1_ca_telemetry_decoder_gs(satellite, conf));
}


glonass_l1_ca_telemetry_decoder_gs::glonass_l1_ca_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    const Tlm_Conf &conf) : gr::block("glonass_l1_ca_telemetry_decoder_gs", gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
                                gr::io_signature::make(1, 1, sizeof(Gnss_Synchro))),
                            d_dump_filename(conf.dump_filename),
                            d_preamble_time_samples(0),
                            d_TOW_at_current_symbol(0),
                            d_sample_counter(0ULL),
                            d_preamble_index(0ULL),
                            d_stat(0),
                            d_CRC_error_counter(0),
                            d_channel(0),
                            d_flag_frame_sync(false),
                            d_flag_preamble(false),
                            d_dump(conf.dump),
                            d_dump_mat(conf.dump_mat),
                            d_remove_dat(conf.remove_dat),
                            d_enable_navdata_monitor(conf.enable_navdata_monitor),
                            d_dump_crc_stats(conf.dump_crc_stats)
{
    // prevent telemetry symbols accumulation in output buffers
    this->set_max_noutput_items(1);
    // Ephemeris data port out
    this->message_port_register_out(pmt::mp("telemetry"));
    // Control messages to tracking block
    this->message_port_register_out(pmt::mp("telemetry_to_trk"));
    this->message_port_register_out(pmt::mp("preamble_timestamp_samples"));

    if (d_enable_navdata_monitor)
        {
            // register nav message monitor out
            this->message_port_register_out(pmt::mp("Nav_msg_from_TLM"));
            d_nav_msg_packet.system = std::string("R");
            d_nav_msg_packet.signal = std::string("1G");
        }

    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    LOG(INFO) << "Initializing GLONASS L1 CA TELEMETRY DECODING";

    // preamble bits to sampled symbols
    int32_t n = 0;
    for (uint16_t d_preambles_bit : d_preambles_bits)
        {
            for (uint32_t j = 0; j < GLONASS_GNAV_TELEMETRY_SYMBOLS_PER_PREAMBLE_BIT; j++)
                {
                    if (d_preambles_bit == 1)
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

    d_symbol_history.set_capacity(GLONASS_GNAV_STRING_SYMBOLS);

    if (d_dump_crc_stats)
        {
            // initialize the telemetry CRC statistics class
            d_Tlm_CRC_Stats = std::make_unique<Tlm_CRC_Stats>();
            d_Tlm_CRC_Stats->initialize(conf.dump_crc_stats_filename);
        }
    else
        {
            d_Tlm_CRC_Stats = nullptr;
        }
}


glonass_l1_ca_telemetry_decoder_gs::~glonass_l1_ca_telemetry_decoder_gs()
{
    DLOG(INFO) << "Glonass L1 Telemetry decoder block (channel " << d_channel << ") destructor called.";
    size_t pos = 0;
    if (d_dump_file.is_open() == true)
        {
            pos = d_dump_file.tellp();
            try
                {
                    d_dump_file.close();
                }
            catch (const std::exception &ex)
                {
                    LOG(WARNING) << "Exception in destructor closing the dump file " << ex.what();
                }
            if (pos == 0)
                {
                    if (!tlm_remove_file(d_dump_filename))
                        {
                            LOG(WARNING) << "Error deleting temporary file";
                        }
                }
        }
    if (d_dump && (pos != 0) && d_dump_mat)
        {
            save_tlm_matfile(d_dump_filename);
            if (d_remove_dat)
                {
                    if (!tlm_remove_file(d_dump_filename))
                        {
                            LOG(WARNING) << "Error deleting temporary file";
                        }
                }
        }
}


void glonass_l1_ca_telemetry_decoder_gs::decode_string(const double *frame_symbols, int32_t frame_length)
{
    double chip_acc = 0.0;
    int32_t chip_acc_counter = 0;

    // 1. Transform from symbols to bits
    std::string bi_binary_code;
    bi_binary_code.reserve(frame_length / GLONASS_GNAV_TELEMETRY_SYMBOLS_PER_BIT);
    std::string relative_code;
    relative_code.reserve(GLONASS_GNAV_STRING_BITS);
    std::string data_bits;
    data_bits.reserve(GLONASS_GNAV_STRING_BITS + 1);

    // Group samples into bi-binary code
    for (int32_t i = 0; i < (frame_length); i++)
        {
            chip_acc += frame_symbols[i];
            chip_acc_counter += 1;

            if (chip_acc_counter == (GLONASS_GNAV_TELEMETRY_SYMBOLS_PER_BIT))
                {
                    if (chip_acc > 0)
                        {
                            bi_binary_code.push_back('1');
                            chip_acc_counter = 0;
                            chip_acc = 0;
                        }
                    else
                        {
                            bi_binary_code.push_back('0');
                            chip_acc_counter = 0;
                            chip_acc = 0;
                        }
                }
        }
    // Convert from bi-binary code to relative code
    for (int32_t i = 0; i < (GLONASS_GNAV_STRING_BITS); i++)
        {
            if (bi_binary_code[2 * i] == '1' && bi_binary_code[2 * i + 1] == '0')
                {
                    relative_code.push_back('1');
                }
            else
                {
                    relative_code.push_back('0');
                }
        }
    // Convert from relative code to data bits
    data_bits.push_back('0');
    for (int32_t i = 1; i < (GLONASS_GNAV_STRING_BITS); i++)
        {
            data_bits.push_back(((relative_code[i - 1] - '0') ^ (relative_code[i] - '0')) + '0');
        }

    if (d_enable_navdata_monitor)
        {
            d_nav_msg_packet.nav_message = data_bits;
        }

    // 2. Call the GLONASS GNAV string decoder
    d_nav.string_decoder(data_bits);

    // 3. Check operation executed correctly
    bool crc_ok = d_nav.get_flag_CRC_test();
    if (crc_ok)
        {
            LOG(INFO) << "GLONASS GNAV CRC correct in channel " << d_channel << " from satellite " << d_satellite;
        }
    else
        {
            LOG(INFO) << "GLONASS GNAV CRC error in channel " << d_channel << " from satellite " << d_satellite;
        }
    if (d_dump_crc_stats)
        {
            // update CRC statistics
            d_Tlm_CRC_Stats->update_CRC_stats(crc_ok);
        }

    // 4. Push the new navigation data to the queues
    if (d_nav.have_new_ephemeris() == true)
        {
            // get object for this SV (mandatory)
            d_nav.set_rf_link(d_satellite.get_rf_link());
            const std::shared_ptr<Glonass_Gnav_Ephemeris> tmp_obj = std::make_shared<Glonass_Gnav_Ephemeris>(d_nav.get_ephemeris());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            LOG(INFO) << "GLONASS GNAV Ephemeris have been received in channel" << d_channel << " from satellite " << d_satellite;
            std::cout << "New GLONASS L1 GNAV message received in channel " << d_channel << ": ephemeris from satellite " << d_satellite << '\n';
        }
    if (d_nav.have_new_utc_model() == true)
        {
            // get object for this SV (mandatory)
            const std::shared_ptr<Glonass_Gnav_Utc_Model> tmp_obj = std::make_shared<Glonass_Gnav_Utc_Model>(d_nav.get_utc_model());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            LOG(INFO) << "GLONASS GNAV UTC Model data have been received in channel" << d_channel << " from satellite " << d_satellite;
            std::cout << "New GLONASS L1 GNAV message received in channel " << d_channel << ": UTC model parameters from satellite " << d_satellite << '\n';
        }
    if (d_nav.have_new_almanac() == true)
        {
            const uint32_t slot_nbr = d_nav.get_alm_satellite_slot_number();
            const std::shared_ptr<Glonass_Gnav_Almanac>
                tmp_obj = std::make_shared<Glonass_Gnav_Almanac>(d_nav.get_almanac(slot_nbr));
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            LOG(INFO) << "GLONASS GNAV Almanac data have been received in channel" << d_channel << " in slot number " << slot_nbr;
            std::cout << "New GLONASS L1 GNAV almanac received in channel " << d_channel << " from satellite " << d_satellite << '\n';
        }
    // 5. Update satellite information on system
    if (d_nav.get_flag_update_slot_number() == true)
        {
            LOG(INFO) << "GLONASS GNAV Slot Number Identified in channel " << d_channel;
            d_satellite.update_PRN(d_nav.get_ephemeris().d_n);
            d_satellite.what_block(d_satellite.get_system(), d_nav.get_ephemeris().d_n);
            d_nav.set_flag_update_slot_number(false);
        }
}


void glonass_l1_ca_telemetry_decoder_gs::set_satellite(const Gnss_Satellite &satellite)
{
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite " << d_satellite;
    DLOG(INFO) << "Navigation Satellite set to " << d_satellite;
}


void glonass_l1_ca_telemetry_decoder_gs::set_channel(int32_t channel)
{
    d_channel = channel;
    LOG(INFO) << "Navigation channel set to " << channel;
    // ############# ENABLE DATA FILE LOG #################
    if (d_dump == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                        {
                            d_dump_filename.append(std::to_string(d_channel));
                            d_dump_filename.append(".dat");
                            d_dump_file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "Telemetry decoder dump enabled on channel " << d_channel << " Log file: " << d_dump_filename.c_str();
                        }
                    catch (const std::ofstream::failure &e)
                        {
                            LOG(WARNING) << "channel " << d_channel << ": exception opening Glonass TLM dump file. " << e.what();
                        }
                }
        }
    if (d_dump_crc_stats)
        {
            // set the channel number for the telemetry CRC statistics
            // disable the telemetry CRC statistics if there is a problem opening the output file
            d_dump_crc_stats = d_Tlm_CRC_Stats->set_channel(d_channel);
        }
}


int glonass_l1_ca_telemetry_decoder_gs::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    int32_t corr_value = 0;
    int32_t preamble_diff = 0;

    auto **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);            // Get the output buffer pointer
    const auto **in = reinterpret_cast<const Gnss_Synchro **>(&input_items[0]);  // Get the input buffer pointer

    Gnss_Synchro current_symbol{};  // structure to save the synchronization information and send the output object to the next block
    // 1. Copy the current tracking output
    current_symbol = in[0][0];
    d_symbol_history.push_back(current_symbol);  // add new symbol to the symbol queue
    d_sample_counter++;                          // count for the processed samples
    consume_each(1);

    d_flag_preamble = false;

    if (static_cast<int32_t>(d_symbol_history.size()) >= d_symbols_per_preamble)
        {
            // ******* preamble correlation ********
            for (int32_t i = 0; i < d_symbols_per_preamble; i++)
                {
                    if (d_symbol_history[i].Prompt_I < 0.0)  // symbols clipping
                        {
                            corr_value -= d_preambles_symbols[i];
                        }
                    else
                        {
                            corr_value += d_preambles_symbols[i];
                        }
                }
        }

    // ******* frame sync ******************
    if (d_stat == 0)  // no preamble information
        {
            if (abs(corr_value) >= d_symbols_per_preamble)
                {
                    // Record the preamble sample stamp
                    d_preamble_index = d_sample_counter;
                    LOG(INFO) << "Preamble detection for GLONASS L1 C/A SAT " << this->d_satellite;
                    // Enter into frame pre-detection status
                    d_stat = 1;
                    d_preamble_time_samples = d_symbol_history[0].Tracking_sample_counter;  // record the preamble sample stamp
                }
        }
    else if (d_stat == 1)  // possible preamble lock
        {
            if (abs(corr_value) >= d_symbols_per_preamble)
                {
                    // check preamble separation
                    preamble_diff = static_cast<int32_t>(d_sample_counter - d_preamble_index);
                    // Record the PRN start sample index associated to the preamble
                    d_preamble_time_samples = static_cast<double>(d_symbol_history[0].Tracking_sample_counter);
                    if (abs(preamble_diff - GLONASS_GNAV_PREAMBLE_PERIOD_SYMBOLS) == 0)
                        {
                            // try to decode frame
                            LOG(INFO) << "Starting string decoder for GLONASS L1 C/A SAT " << this->d_satellite;
                            d_preamble_index = d_sample_counter;  // record the preamble sample stamp
                            this->message_port_pub(pmt::mp("preamble_timestamp_samples"), pmt::mp(d_preamble_time_samples));
                            d_stat = 2;
                        }
                    else
                        {
                            if (preamble_diff > GLONASS_GNAV_PREAMBLE_PERIOD_SYMBOLS)
                                {
                                    d_stat = 0;  // start again
                                }
                            DLOG(INFO) << "Failed string decoder for GLONASS L1 C/A SAT " << this->d_satellite;
                        }
                }
        }
    else if (d_stat == 2)
        {
            // FIXME: The preamble index marks the first symbol of the string count. Here I just wait for another full string to be received before processing
            if (d_sample_counter == d_preamble_index + static_cast<uint64_t>(GLONASS_GNAV_STRING_SYMBOLS))
                {
                    // NEW GLONASS string received
                    // 0. fetch the symbols into an array
                    const int32_t string_length = GLONASS_GNAV_STRING_SYMBOLS - d_symbols_per_preamble;
                    std::array<double, GLONASS_GNAV_DATA_SYMBOLS> string_symbols{};

                    // ******* SYMBOL TO BIT *******
                    for (int32_t i = 0; i < string_length; i++)
                        {
                            if (corr_value > 0)
                                {
                                    string_symbols[i] = d_symbol_history[i + d_symbols_per_preamble].Prompt_I;  // because last symbol of the preamble is just received now!
                                }
                            else
                                {
                                    string_symbols[i] = -d_symbol_history[i + d_symbols_per_preamble].Prompt_I;  // because last symbol of the preamble is just received now!
                                }
                        }

                    // call the decoder
                    decode_string(string_symbols.data(), string_length);
                    bool crc_ok = d_nav.get_flag_CRC_test();
                    if (crc_ok == true)
                        {
                            d_CRC_error_counter = 0;
                            d_flag_preamble = true;               // valid preamble indicator (initialized to false every work())
                            d_preamble_index = d_sample_counter;  // record the preamble sample stamp (t_P)
                            if (!d_flag_frame_sync)
                                {
                                    d_flag_frame_sync = true;
                                    DLOG(INFO) << " Frame sync SAT " << this->d_satellite << " with preamble start at "
                                               << d_symbol_history[0].Tracking_sample_counter << " [samples]";
                                }
                        }
                    else
                        {
                            d_CRC_error_counter++;
                            d_preamble_index = d_sample_counter;  // record the preamble sample stamp
                            if (d_CRC_error_counter > CRC_ERROR_LIMIT)
                                {
                                    LOG(INFO) << "Lost of frame sync SAT " << this->d_satellite;
                                    d_flag_frame_sync = false;
                                    d_stat = 0;
                                }
                        }
                }
        }

    // UPDATE GNSS SYNCHRO DATA
    // 2. Add the telemetry decoder information
    if (this->d_flag_preamble == true && d_nav.get_flag_TOW_new() == true)
        // update TOW at the preamble instant
        {
            d_TOW_at_current_symbol = floor((d_nav.get_ephemeris().d_TOW - GLONASS_GNAV_PREAMBLE_DURATION_S) * 1000) / 1000;
            d_nav.set_flag_TOW_new(false);
        }
    else  // if there is not a new preamble, we define the TOW of the current symbol
        {
            d_TOW_at_current_symbol = d_TOW_at_current_symbol + GLONASS_L1_CA_CODE_PERIOD_S;
        }

    // if (d_flag_frame_sync == true && d_nav.flag_TOW_set==true && d_nav.get_flag_CRC_test() == true)

    // if(d_nav.flag_GGTO_1 == true  &&  d_nav.flag_GGTO_2 == true &&  d_nav.flag_GGTO_3 == true &&  d_nav.flag_GGTO_4 == true) // all GGTO parameters arrived
    //     {
    //         delta_t = d_nav.A_0G + d_nav.A_1G * (d_TOW_at_current_symbol - d_nav.t_0G + 604800.0 * (fmod((d_nav.WN_0 - d_nav.WN_0G), 64)));
    //     }

    current_symbol.PRN = this->d_satellite.get_PRN();
    current_symbol.TOW_at_current_symbol_ms = round(d_TOW_at_current_symbol * 1000.0);

    if (d_flag_frame_sync == true && d_nav.is_flag_TOW_set() == true)
        {
            current_symbol.Flag_valid_word = true;
            if (d_enable_navdata_monitor && !d_nav_msg_packet.nav_message.empty())
                {
                    d_nav_msg_packet.prn = static_cast<int32_t>(current_symbol.PRN);
                    d_nav_msg_packet.tow_at_current_symbol_ms = static_cast<int32_t>(current_symbol.TOW_at_current_symbol_ms);
                    const std::shared_ptr<Nav_Message_Packet> tmp_obj = std::make_shared<Nav_Message_Packet>(d_nav_msg_packet);
                    this->message_port_pub(pmt::mp("Nav_msg_from_TLM"), pmt::make_any(tmp_obj));
                    d_nav_msg_packet.nav_message = "";
                }
        }
    else
        {
            current_symbol.Flag_valid_word = false;
        }

    // todo: glonass time to gps time should be done in observables block
    // current_symbol.TOW_at_current_symbol_ms -= -= static_cast<uint32_t>(delta_t) * 1000;  // Galileo to GPS TOW

    if (d_dump == true)
        {
            // MULTIPLEXED FILE RECORDING - Record results to file
            try
                {
                    double tmp_double;
                    uint64_t tmp_ulong_int;
                    int32_t tmp_int;
                    tmp_double = d_TOW_at_current_symbol;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                    tmp_ulong_int = current_symbol.Tracking_sample_counter;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_ulong_int), sizeof(uint64_t));
                    tmp_double = 0;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                    tmp_int = (current_symbol.Prompt_I > 0.0 ? 1 : -1);
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_int), sizeof(int32_t));
                    tmp_int = static_cast<int32_t>(current_symbol.PRN);
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_int), sizeof(int32_t));
                }
            catch (const std::ofstream::failure &e)
                {
                    LOG(WARNING) << "Exception writing observables dump file " << e.what();
                }
        }

    // 3. Make the output (copy the object contents to the GNURadio reserved memory)
    *out[0] = current_symbol;

    return 1;
}
