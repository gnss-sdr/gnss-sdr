
/*!
 * \file beidou_b2a_telemetry_decoder_gs.cc
 * \brief Implementation of a BeiDou B2a CNAV2 data decoder block
  * \note Code added as part of GSoC 2018 program
 * \author Dong Kyeong Lee, 2018. dole7890(at)colorado.edu
 * \author Damian Miralles, 2019. dmiralles2009(at)gmail.com
 * \author Aloha Churchill, 2022. churchill.aloha(at)gmail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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


#include "beidou_b2a_telemetry_decoder_gs.h"
#include "Beidou_B2a.h"
#include "beidou_cnav2_almanac.h"
#include "beidou_cnav2_ephemeris.h"
#include "beidou_cnav2_iono.h"
#include "beidou_cnav2_utc_model.h"
#include "display.h"
#include "gnss_synchro.h"
#include "tlm_utils.h"
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>        // for make_any
#include <pmt/pmt_sugar.h>  // for mp
#include <cstdlib>          // for abs
#include <exception>        // for exception
#include <iostream>         // for cout
#include <memory>           // for shared_ptr, make_shared


#define CRC_ERROR_LIMIT 8


beidou_b2a_telemetry_decoder_gs_sptr
beidou_b2a_make_telemetry_decoder_gs(const Gnss_Satellite &satellite, const Tlm_Conf &conf)
{
    return beidou_b2a_telemetry_decoder_gs_sptr(new beidou_b2a_telemetry_decoder_gs(satellite, conf));
}


beidou_b2a_telemetry_decoder_gs::beidou_b2a_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    const Tlm_Conf &conf) : gr::block("beidou_b2a_telemetry_decoder_gs",
                     gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
                     gr::io_signature::make(1, 1, sizeof(Gnss_Synchro))),
                    d_dump_filename(conf.dump_filename),
                    d_sample_counter(0),
                    d_preamble_index(0),
                    d_channel(0),
                    d_symbols_per_preamble(BEIDOU_CNAV2_PREAMBLE_LENGTH_SYMBOLS),
                    d_samples_per_preamble(BEIDOU_CNAV2_PREAMBLE_LENGTH_SYMBOLS),
                    d_preamble_period_samples(BEIDOU_CNAV2_PREAMBLE_PERIOD_SYMBOLS),
                    d_CRC_error_counter(0),
                    d_required_symbols(BEIDOU_CNAV2_SUBFRAME_SYMBOLS + BEIDOU_CNAV2_PREAMBLE_LENGTH_SYMBOLS),
                    d_stat(0),
                    d_last_valid_preamble(0),
                    d_symbol_duration_ms(BEIDOU_B2A_TELEMETRY_SYMBOLS_PER_BIT * BEIDOU_B2A_CODE_PERIOD_MS),
                    d_TOW_at_Preamble_ms(0U),
                    d_TOW_at_current_symbol_ms(0U),
                    d_flag_SOW_set(false),
                    d_flag_frame_sync(false),
                    d_flag_preamble(false),
                    d_flag_valid_word(false),
                    d_sent_tlm_failed_msg(false),
                    d_dump(conf.dump),
                    d_dump_mat(conf.dump_mat),
                    d_remove_dat(conf.remove_dat),
                    d_enable_navdata_monitor(conf.enable_navdata_monitor),
                    d_dump_crc_stats(conf.dump_crc_stats)
{
    //prevent telemetry symbols accumulation in output buffers
    this->set_max_noutput_items(1);
    // Ephemeris data port out
    this->message_port_register_out(pmt::mp("telemetry"));
    // Control messages to tracking block
    this->message_port_register_out(pmt::mp("telemetry_to_trk"));
    // initialize internal vars

    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    LOG(INFO) << "Initializing BeiDou B2a Telemetry Decoding for satellite " << this->d_satellite;


    // Setting samples of preamble code
    for (int32_t i = 0; i < d_symbols_per_preamble; i++)
        {
            if (BEIDOU_CNAV2_PREAMBLE[i] == '1')
                {
                    d_preamble_samples[i] = 1;
                }
            else
                {
                    d_preamble_samples[i] = -1;
                }
        }

    d_symbol_history.set_capacity(d_required_symbols);

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


beidou_b2a_telemetry_decoder_gs::~beidou_b2a_telemetry_decoder_gs()
{
    DLOG(INFO) << "BeiDou B2a Telemetry decoder block (channel " << d_channel << ") destructor called.";
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


void beidou_b2a_telemetry_decoder_gs::decode_frame(float *frame_symbols)
{
    // 1. Transform from symbols to bits
    std::string data_bits;

    // we want data_bits = frame_symbols[24:24+288]
    for (uint32_t ii = 0; ii < (BEIDOU_CNAV2_DATA_BITS); ii++)
        {
            data_bits.push_back((frame_symbols[ii] > 0) ? ('1') : ('0'));
        }

    d_nav.frame_decoder(data_bits);

    // 3. Check operation executed correctly
    if (d_nav.flag_crc_test == true)
        {
            DLOG(INFO) << "BeiDou CNAV2 CRC correct in channel " << d_channel
                       << " from satellite " << d_satellite;
        }
    else
        {
            DLOG(INFO) << "BeiDou CNAV2 CRC error in channel " << d_channel
                       << " from satellite " << d_satellite;
        }
    // 4. Push the new navigation data to the queues
    if (d_nav.have_new_ephemeris() == true)
        {
            // get object for this SV (mandatory)
            std::shared_ptr<Beidou_Cnav2_Ephemeris> tmp_obj = std::make_shared<Beidou_Cnav2_Ephemeris>(d_nav.get_ephemeris());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            DLOG(INFO) << "BeiDou CNAV2 Ephemeris have been received in channel" << d_channel << " from satellite " << d_satellite;
            std::cout << TEXT_YELLOW << "New BeiDou B2a CNAV2 message received in channel " << d_channel << ": ephemeris from satellite " << d_satellite << TEXT_RESET << std::endl;
        }
    if (d_nav.have_new_utc_model() == true)
        {
            // get object for this SV (mandatory)
            std::shared_ptr<Beidou_Cnav2_Utc_Model> tmp_obj = std::make_shared<Beidou_Cnav2_Utc_Model>(d_nav.get_utc_model());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            DLOG(INFO) << "BeiDou CNAV2 UTC Model have been received in channel" << d_channel << " from satellite " << d_satellite;
            std::cout << TEXT_YELLOW << "New BeiDou B2a CNAV2 UTC model message received in channel " << d_channel << ": UTC model parameters from satellite " << d_satellite << TEXT_RESET << std::endl;
        }
    if (d_nav.have_new_iono() == true)
        {
            // get object for this SV (mandatory)
            std::shared_ptr<Beidou_Cnav2_Iono> tmp_obj = std::make_shared<Beidou_Cnav2_Iono>(d_nav.get_iono());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            DLOG(INFO) << "BeiDou CNAV2 Iono have been received in channel" << d_channel << " from satellite " << d_satellite;
            std::cout << TEXT_YELLOW << "New BeiDou B2a CNAV2 Iono message received in channel " << d_channel << ": UTC model parameters from satellite " << d_satellite << TEXT_RESET << std::endl;
        }
    if (d_nav.have_new_almanac() == true)
        {
            unsigned int slot_nbr = d_nav.i_alm_satellite_PRN;
            std::shared_ptr<Beidou_Cnav2_Almanac> tmp_obj = std::make_shared<Beidou_Cnav2_Almanac>(d_nav.get_almanac(slot_nbr));
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            DLOG(INFO) << "BeiDou CNAV2 Almanac have been received in channel" << d_channel << " in slot number " << slot_nbr;
            std::cout << TEXT_YELLOW << "New BeiDou B2a CNAV2 almanac received in channel " << d_channel << " from satellite " << d_satellite << TEXT_RESET << std::endl;
        }
}


void beidou_b2a_telemetry_decoder_gs::set_satellite(const Gnss_Satellite &satellite)
{
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite " << d_satellite;
    DLOG(INFO) << "Navigation Satellite set to " << d_satellite;
    // need to update info for 
}


void beidou_b2a_telemetry_decoder_gs::set_channel(int32_t channel)
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
                            d_dump_filename = "telemetry";
                            d_dump_filename.append(boost::lexical_cast<std::string>(d_channel));
                            d_dump_filename.append(".dat");
                            d_dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "Telemetry decoder dump enabled on channel " << d_channel << " Log file: " << d_dump_filename.c_str();
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            LOG(WARNING) << "channel " << d_channel << ": exception opening Beidou TLM dump file. " << e.what();
                        }
                }
        }
}


void beidou_b2a_telemetry_decoder_gs::reset()
{
    d_last_valid_preamble = d_sample_counter;
    d_TOW_at_current_symbol_ms = 0;
    d_sent_tlm_failed_msg = false;
    d_flag_valid_word = false;
    DLOG(INFO) << "BeiDou B2a Telemetry decoder reset for satellite " << d_satellite;
    return;
}


int beidou_b2a_telemetry_decoder_gs::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    int32_t corr_value = 0;
    int32_t preamble_diff = 0;

    auto **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);            // Get the output buffer pointer
    const auto **in = reinterpret_cast<const Gnss_Synchro **>(&input_items[0]);  // Get the input buffer pointer

    Gnss_Synchro current_symbol;  //structure to save the synchronization information and send the output object to the next block
    //1. Copy the current tracking output
    current_symbol = in[0][0];
    d_symbol_history.push_back(current_symbol.Prompt_I);  //add new symbol to the symbol queue
    d_sample_counter++;                                   //count for the processed samples
    consume_each(1);
    d_flag_preamble = false;

    if (d_symbol_history.size() >= d_required_symbols)
        {
            //******* preamble correlation ********
            for (int32_t i = 0; i < d_samples_per_preamble; i++)
                {
                    if (d_symbol_history[i] < 0)  // symbols clipping
                        {
                            corr_value -= d_preamble_samples[i];
                        }
                    else
                        {
                            corr_value += d_preamble_samples[i];
                        }
                }
        }

    //******* frame sync ******************
    if (d_stat == 0)  //no preamble information
        {
            if (abs(corr_value) >= d_samples_per_preamble)
                {
                    // Record the preamble sample stamp
                    d_preamble_index = d_sample_counter;
                    DLOG(INFO) << "Preamble detection for BeiDou B2a satellite " << this->d_satellite;
                    // Enter into frame pre-detection status
                    d_stat = 1;
                }
        }
    else if (d_stat == 1)  // possible preamble lock
        {
            if (abs(corr_value) >= d_samples_per_preamble)
                {
                    //check preamble separation
                    preamble_diff = static_cast<int32_t>(d_sample_counter - d_preamble_index);
                    if (abs(preamble_diff - d_preamble_period_samples) == 0)
                        {
                            //try to decode frame
                            DLOG(INFO) << "Starting BeiDou CNAV2 frame decoding for BeiDou B2a SAT " << this->d_satellite;
                            d_preamble_index = d_sample_counter;  //record the preamble sample stamp
                            d_stat = 2;

                            // ******* SAMPLES TO SYMBOLS *******
                            if (corr_value > 0)  //normal PLL lock
                                {
                                    for (uint32_t i = 0; i < BEIDOU_CNAV2_PREAMBLE_PERIOD_SYMBOLS; i++)
                                        {
                                            d_frame_symbols[i] = d_symbol_history.at(BEIDOU_CNAV2_PREAMBLE_LENGTH_SYMBOLS + i);
                                        }
                                }
                            else  // 180 deg. inverted carrier phase PLL lock
                                {
                                    for (uint32_t i = 0; i < BEIDOU_CNAV2_PREAMBLE_PERIOD_SYMBOLS; i++)
                                        {
                                            d_frame_symbols[i] = -d_symbol_history.at(BEIDOU_CNAV2_PREAMBLE_LENGTH_SYMBOLS + i);
                                        }
                                }

                            // call the decoder
                            decode_frame(d_frame_symbols.data());

                            if (d_nav.flag_crc_test == true)
                                {
                                    d_CRC_error_counter = 0;
                                    d_flag_preamble = true;               // valid preamble indicator (initialized to false every work())
                                    d_preamble_index = d_sample_counter;  // record the preamble sample stamp (t_P)
                                    if (!d_flag_frame_sync)
                                        {
                                            d_flag_frame_sync = true;
                                            DLOG(INFO) << "BeiDou CNAV2 frame sync found for satellite " << this->d_satellite;
                                        }
                                }
                            else
                                {
                                    d_CRC_error_counter++;
                                    d_preamble_index = d_sample_counter;  // record the preamble sample stamp
                                    if (d_CRC_error_counter > CRC_ERROR_LIMIT)
                                        {
                                            DLOG(INFO) << "BeiDou CNAV2 frame sync found for satellite " << this->d_satellite;
                                            d_flag_frame_sync = false;
                                            d_stat = 0;
                                            flag_TOW_set = false;
                                        }
                                }
                        }
                    else
                        {
                            if (preamble_diff > d_preamble_period_samples)
                                {
                                    d_stat = 0;  // start again
                                }
                            DLOG(INFO) << "Failed BeiDou CNAV2 frame decoding for BeiDou B2a SAT " << this->d_satellite;
                        }
                }
        }
    else if (d_stat == 2)  // preamble acquired
        {
            if (d_sample_counter == d_preamble_index + static_cast<uint64_t>(d_preamble_period_samples))
                {
                    //******* SAMPLES TO SYMBOLS *******
                    if (corr_value > 0)  //normal PLL lock
                        {
                            for (uint32_t i = 0; i < BEIDOU_CNAV2_PREAMBLE_PERIOD_SYMBOLS; i++)
                                {
                                    d_frame_symbols[i] = d_symbol_history.at(BEIDOU_CNAV2_PREAMBLE_LENGTH_SYMBOLS + i);
                                }
                        }
                    else  //180 deg. inverted carrier phase PLL lock
                        {
                            for (uint32_t i = 0; i < BEIDOU_CNAV2_PREAMBLE_PERIOD_SYMBOLS; i++)
                                {
                                    d_frame_symbols[i] = -d_symbol_history.at(BEIDOU_CNAV2_PREAMBLE_LENGTH_SYMBOLS + i);
                                }
                        }

                    //call the decoder
                    decode_frame(d_frame_symbols.data());

                    if (d_nav.flag_crc_test == true)
                        {
                            d_CRC_error_counter = 0;
                            d_flag_preamble = true;               //valid preamble indicator (initialized to false every work())
                            d_preamble_index = d_sample_counter;  //record the preamble sample stamp (t_P)
                            if (!d_flag_frame_sync)
                                {
                                    d_flag_frame_sync = true;
                                    DLOG(INFO) << "BeiDou CNAV2 frame sync found for SAT " << this->d_satellite;
                                }
                        }
                    else
                        {
                            d_CRC_error_counter++;
                            d_preamble_index = d_sample_counter;  //record the preamble sample stamp
                            if (d_CRC_error_counter > CRC_ERROR_LIMIT)
                                {
                                    LOG(INFO) << "BeiDou CNAV2 frame sync lost for SAT " << this->d_satellite;
                                    d_flag_frame_sync = false;
                                    d_stat = 0;
                                    flag_TOW_set = false;
                                }
                        }
                }
        }

    // UPDATE GNSS SYNCHRO DATA
    //2. Add the telemetry decoder information
    if (this->d_flag_preamble == true and d_nav.flag_TOW_set == true)
        //update TOW at the preamble instant
        {
            if (d_nav.flag_TOW_10 == true)
                {
                    d_TOW_at_Preamble_ms = static_cast<uint32_t>((d_nav.cnav2_ephemeris.SOW + BEIDOU_CNAV2_BDT2GPST_LEAP_SEC_OFFSET) * 1000.0);
                    flag_TOW_set = true;
                    d_nav.flag_TOW_10 = false;
                }
            else if (d_nav.flag_TOW_11 == true)
                {
                    d_TOW_at_Preamble_ms = static_cast<uint32_t>((d_nav.cnav2_ephemeris.SOW + BEIDOU_CNAV2_BDT2GPST_LEAP_SEC_OFFSET) * 1000.0);
                    flag_TOW_set = true;
                    d_nav.flag_TOW_11 = false;
                }
            else if (d_nav.flag_TOW_30 == true)
                {
                    d_TOW_at_Preamble_ms = static_cast<uint32_t>((d_nav.cnav2_ephemeris.SOW + BEIDOU_CNAV2_BDT2GPST_LEAP_SEC_OFFSET) * 1000.0);
                    flag_TOW_set = true;
                    d_nav.flag_TOW_30 = false;
                }
            else if (d_nav.flag_TOW_31 == true)
                {
                    d_TOW_at_Preamble_ms = static_cast<uint32_t>((d_nav.cnav2_ephemeris.SOW + BEIDOU_CNAV2_BDT2GPST_LEAP_SEC_OFFSET) * 1000.0);
                    flag_TOW_set = true;
                    d_nav.flag_TOW_31 = false;
                }
            else if (d_nav.flag_TOW_32 == true)
                {
                    d_TOW_at_Preamble_ms = static_cast<uint32_t>((d_nav.cnav2_ephemeris.SOW + BEIDOU_CNAV2_BDT2GPST_LEAP_SEC_OFFSET) * 1000.0);
                    flag_TOW_set = true;
                    d_nav.flag_TOW_32 = false;
                }
            else if (d_nav.flag_TOW_33 == true)
                {
                    d_TOW_at_Preamble_ms = static_cast<uint32_t>((d_nav.cnav2_ephemeris.SOW + BEIDOU_CNAV2_BDT2GPST_LEAP_SEC_OFFSET) * 1000.0);
                    flag_TOW_set = true;
                    d_nav.flag_TOW_33 = false;
                }
            else if (d_nav.flag_TOW_34 == true)
                {
                    d_TOW_at_Preamble_ms = static_cast<uint32_t>((d_nav.cnav2_ephemeris.SOW + BEIDOU_CNAV2_BDT2GPST_LEAP_SEC_OFFSET) * 1000.0);
                    flag_TOW_set = true;
                    d_nav.flag_TOW_34 = false;
                }
            else if (d_nav.flag_TOW_40 == true)
                {
                    d_TOW_at_Preamble_ms = static_cast<uint32_t>((d_nav.cnav2_ephemeris.SOW + BEIDOU_CNAV2_BDT2GPST_LEAP_SEC_OFFSET) * 1000.0);
                    flag_TOW_set = true;
                    d_nav.flag_TOW_40 = false;
                }

            uint32_t last_d_TOW_at_current_symbol_ms = d_TOW_at_current_symbol_ms;
            d_TOW_at_current_symbol_ms = d_TOW_at_Preamble_ms + d_required_symbols * d_symbol_duration_ms;

            if (last_d_TOW_at_current_symbol_ms != 0 and abs(static_cast<int64_t>(d_TOW_at_current_symbol_ms) - int64_t(last_d_TOW_at_current_symbol_ms)) > d_symbol_duration_ms)
                {
                    LOG(INFO) << "Warning: BeiDou B2a CNAV2 TOW update in ch " << d_channel
                              << " does not match the TLM TOW counter " << static_cast<int64_t>(d_TOW_at_current_symbol_ms) - int64_t(last_d_TOW_at_current_symbol_ms) << " ms \n";

                    d_TOW_at_current_symbol_ms = 0;
                    d_flag_valid_word = false;
                }
            else
                {
                    d_last_valid_preamble = d_sample_counter;
                    d_flag_valid_word = true;
                }
        }
    else  //if there is not a new preamble, we define the TOW of the current symbol
        {
            if (d_flag_valid_word)
                {
                    d_TOW_at_current_symbol_ms += d_symbol_duration_ms;
                    if (current_symbol.Flag_valid_symbol_output == false)
                        {
                            d_flag_valid_word = false;
                        }
                }
        }

    if (d_flag_valid_word == true)
        {
            current_symbol.TOW_at_current_symbol_ms = d_TOW_at_current_symbol_ms;
            current_symbol.Flag_valid_word = d_flag_valid_word;

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
                            LOG(WARNING) << "Exception writing Telemetry BeiDou B2a dump file " << e.what();
                        }
                }

            // 3. Make the output (copy the object contents to the GNURadio reserved memory)
            *out[0] = current_symbol;
            return 1;
        }
    return 0;
}