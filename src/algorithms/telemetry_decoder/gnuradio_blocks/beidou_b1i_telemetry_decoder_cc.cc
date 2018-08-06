/*!
 * \file beidou_b1i_telemetry_decoder_cc.cc
 * \brief Implementation of a NAV message demodulator block based on
 * Kay Borre book MATLAB-based GPS receiver
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.es
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

#include "beidou_b1i_telemetry_decoder_cc.h"
#include "control_message_factory.h"
#include <boost/lexical_cast.hpp>
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <volk_gnsssdr/volk_gnsssdr.h>


#ifndef _rotl
#define _rotl(X, N) ((X << N) ^ (X >> (32 - N)))  // Used in the parity check algorithm
#endif

using google::LogMessage;

beidou_b1i_telemetry_decoder_cc_sptr
beidou_b1i_make_telemetry_decoder_cc(const Gnss_Satellite &satellite, bool dump)
{
    return beidou_b1i_telemetry_decoder_cc_sptr(new beidou_b1i_telemetry_decoder_cc(satellite, dump));
}


beidou_b1i_telemetry_decoder_cc::beidou_b1i_telemetry_decoder_cc(
    const Gnss_Satellite &satellite,
    bool dump) : gr::block("beidou_navigation_cc", gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
                     gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{

    // Ephemeris data port out
    this->message_port_register_out(pmt::mp("telemetry"));
    // initialize internal vars
    d_dump = dump;
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());

    // set the preamble
    unsigned short int preambles_bits[BEIDOU_B1I_PREAMBLE_LENGTH_BITS] = BEIDOU_PREAMBLE;

    // preamble bits to sampled symbols
    d_preambles_symbols = static_cast<int *>(volk_gnsssdr_malloc(BEIDOU_B1I_PREAMBLE_LENGTH_SYMBOLS * sizeof(int), volk_gnsssdr_get_alignment()));
    int n = 0;
    for (int i = 0; i < BEIDOU_B1I_PREAMBLE_LENGTH_BITS; i++)
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
    d_stat = 0;
    d_symbol_accumulator = 0;
    d_symbol_accumulator_counter = 0;
    d_frame_bit_index = 0;
    d_flag_frame_sync = false;
    d_BEIDOU_frame_4bytes = 0;
    d_prev_BEIDOU_frame_4bytes = 0;
    d_flag_parity = false;
    d_TOW_at_Preamble_ms = 0;
    flag_TOW_set = false;
    d_flag_preamble = false;
    d_flag_new_tow_available = false;
    word_number = 0;
    d_channel = 0;
    flag_PLL_180_deg_phase_locked = false;
    d_preamble_time_samples = 0;
    d_TOW_at_current_symbol_ms = 0;
    d_symbol_history.resize(BEIDOU_B1I_PREAMBLE_LENGTH_BITS);  // Change fixed buffer size
    d_symbol_nh_history.resize(BEIDOU_B1I_NH_CODE_LENGTH + 1);  // Change fixed buffer size
    d_bit_buffer.resize(30);  // Change fixed buffer size
    d_symbol_history.clear();                                     // Clear all the elements in the buffer
    d_symbol_nh_history.clear();
    d_bit_buffer.clear();
    d_make_correlation = true;
    d_symbol_counter_corr = 0;
    for (int aux = 0; aux < BEIDOU_B1I_NH_CODE_LENGTH; aux++)
        {
            if (BEIDOU_B1I_NH_CODE[aux] == 0)
                {
                    bits_NH[aux] = -1.0;
                }
            else
                {
                    bits_NH[aux] = 1.0;
                }
        }
    sync_NH = false;
    new_sym = false;

}


beidou_b1i_telemetry_decoder_cc::~beidou_b1i_telemetry_decoder_cc()
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


bool beidou_b1i_telemetry_decoder_cc::beidou_word_parityCheck(unsigned int beidouword)
{

    unsigned int d1, d2, d3, d4, d5, d6, d7, t, parity;
    /* XOR as many bits in parallel as possible.  The magic constants pick
       up bits which are to be XOR'ed together to implement the GPS parity
       check algorithm described in IS-GPS-200E.  This avoids lengthy shift-
       and-xor loops. */
    d1 = beidouword & 0xFBFFBF00;
    d2 = _rotl(beidouword, 1) & 0x07FFBF01;
    d3 = _rotl(beidouword, 2) & 0xFC0F8100;
    d4 = _rotl(beidouword, 3) & 0xF81FFE02;
    d5 = _rotl(beidouword, 4) & 0xFC00000E;
    d6 = _rotl(beidouword, 5) & 0x07F00001;
    d7 = _rotl(beidouword, 6) & 0x00003000;
    t = d1 ^ d2 ^ d3 ^ d4 ^ d5 ^ d6 ^ d7;
    // Now XOR the 5 6-bit fields together to produce the 6-bit final result.
    parity = t ^ _rotl(t, 6) ^ _rotl(t, 12) ^ _rotl(t, 18) ^ _rotl(t, 24);
    parity = parity & 0x3F;
    if (parity == (beidouword & 0x3F))
        return (true);
    else
        return (false);
}


void beidou_b1i_telemetry_decoder_cc::set_satellite(const Gnss_Satellite &satellite)
{
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite " << d_satellite;
    d_BEIDOU_FSM.i_satellite_PRN = d_satellite.get_PRN();
    DLOG(INFO) << "Navigation Satellite set to " << d_satellite;
}


void beidou_b1i_telemetry_decoder_cc::set_channel(int channel)
{
    d_channel = channel;
    d_BEIDOU_FSM.i_channel_ID = channel;
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


void beidou_b1i_telemetry_decoder_cc::decode_word(int word_counter, boost::circular_buffer<signed int> *d_bit_buffer, unsigned int& d_BEIDOU_frame_4bytes)
{
//std::cout << word_counter << std::endl;
signed int firstBranch[15];
signed int secondBranch[15];

d_BEIDOU_frame_4bytes = 0;
    if (word_counter == 1)
        {
            for (unsigned int i = 0; i < 15 ; i++)
                {
                    if (d_bit_buffer->at(i) == 1)
                        {
                            d_BEIDOU_frame_4bytes++;
                        }
                    d_BEIDOU_frame_4bytes <<= 1;
                }
            for (unsigned int i = 15; i < 30 ; i++)
                {
                    if (d_bit_buffer->at(i) == 1)
                        {
                            d_BEIDOU_frame_4bytes++;
                        }
                    d_BEIDOU_frame_4bytes <<= 1;  
                }

                    d_BEIDOU_frame_4bytes >>= 1;
            
        }
    else
        {

            for (unsigned int i = 0; i < 30 ; i = i + 2)
                {
                    firstBranch[i/2] = d_bit_buffer->at(i);
                    secondBranch[i/2] = d_bit_buffer->at(i + 1);
                }
            for (unsigned int i = 0; i < 11 ; i++)
                {
                    if (firstBranch[i] == 1)
                        {   
                            d_BEIDOU_frame_4bytes++;
                        }
                    d_BEIDOU_frame_4bytes <<= 1;;
                }
            for (unsigned int i = 0; i < 11 ; i++)
                {
                    if (secondBranch[i] == 1)
                        {
                            d_BEIDOU_frame_4bytes++;
                        }
                    d_BEIDOU_frame_4bytes <<= 1;;
                }
            for (unsigned int i = 11; i < 15 ; i++)
                {
                    if (firstBranch[i] == 1)
                        {
                            d_BEIDOU_frame_4bytes++;
                        }
                    d_BEIDOU_frame_4bytes <<= 1;;
                }
            for (unsigned int i = 11; i < 15 ; i++)
                {
                    if (secondBranch[i] == 1)
                        {
                            d_BEIDOU_frame_4bytes++;
                        }
                    d_BEIDOU_frame_4bytes <<= 1;;
                }

                    d_BEIDOU_frame_4bytes >>= 1;

        }

            for (unsigned int i = 0; i < d_bit_buffer->size() ; i++)
                {
                                std::cout << d_bit_buffer->at(i);
                }

                                std::cout << std::endl;

//        std::cout << d_BEIDOU_frame_4bytes << std::endl;

}



int beidou_b1i_telemetry_decoder_cc::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    int corr_value = 0;
    int preamble_diff_ms = 0;
    int corr_NH = 0;
    Gnss_Synchro **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);            // Get the output buffer pointer
    const Gnss_Synchro **in = reinterpret_cast<const Gnss_Synchro **>(&input_items[0]);  // Get the input buffer pointer
    new_sym = false;
    Gnss_Synchro current_symbol;  //structure to save the synchronization information and send the output object to the next block
    //1. Copy the current tracking output
    current_symbol = in[0][0];
    double current_time_samples = current_symbol.Tracking_sample_counter;
    double current_samples_fs = current_symbol.fs;
    int symbol_value = 0;
    bool Flag_valid_symbol_output = false;
    d_symbol_nh_history.push_back(current_symbol.Prompt_I);  //add new symbol to the symbol queue
    consume_each(1);

    if (d_symbol_nh_history.size() == BEIDOU_B1I_NH_CODE_LENGTH)
        {
            for (int i = 0; i < BEIDOU_B1I_NH_CODE_LENGTH; i++)
                {
                    if ((bits_NH[i] * d_symbol_nh_history.at(i)) > 0.0)
                        {
                            corr_NH += 1;
                        }
                    else
                        {
                            corr_NH -= 1;
                        }
                }
           if (abs(corr_NH) == BEIDOU_B1I_NH_CODE_LENGTH)
                {
                    sync_NH = true;
                    if (corr_NH > 0)
                        {
                            symbol_value = 1;
                        }
                    else
                        {
                            symbol_value = -1;
                        }
//                   std::cout << "SUCCESSFUL NH  CORRELATION" << std::endl;

                    d_symbol_history.push_back(symbol_value);
                    new_sym = true;
                    d_symbol_nh_history.clear();
                }
            else
                {
                    d_symbol_nh_history.pop_front();
                    sync_NH = false;
                    new_sym = false;
                }
        }

    if ((d_symbol_history.size() >= BEIDOU_B1I_PREAMBLE_LENGTH_BITS) and (d_make_correlation or !d_flag_frame_sync))
        {
            //******* preamble correlation ********
            for (unsigned int i = 0; i < BEIDOU_B1I_PREAMBLE_LENGTH_BITS; i++)
                {
                            if (d_symbol_history.at(i) < 0)  // symbols clipping
                                {
                                    corr_value -= d_preambles_symbols[i];
                                }
                            else
                                {
                                    corr_value += d_preambles_symbols[i];
                                }
                }
                    //std::cout << corr_value << std::endl;

            if (std::abs(corr_value) >= BEIDOU_B1I_PREAMBLE_LENGTH_BITS)
                {
/*            for (unsigned int i = 0; i < d_symbol_history.size() ; i++)
                {
                                std::cout << d_symbol_history.at(i);
                }

                                std::cout << std::endl;
*/
//                   std::cout << "SUCCESSFUL PREAMBLE CORRELATION" << std::endl;

                    d_symbol_history.clear();
                    d_symbol_counter_corr++;
                }
        }

    /*if (new_sym and )
        {
            flag_new_cnav_frame = beidou_nav_msg_decoder_add_symbol(&d_cnav_decoder, symbol_clip, &msg, &delay);
            new_sym = false;
        }*/

    unsigned int required_symbols = BEIDOU_B1I_PREAMBLE_LENGTH_SYMBOLS;
    d_flag_preamble = false;


    //******* frame sync ******************
    if (std::abs(corr_value) == BEIDOU_B1I_PREAMBLE_LENGTH_BITS)
        {
//                   std::cout << "FRAME SYNC" << std::endl;

            //TODO: Rewrite with state machine
            if (d_stat == 0)
                {
//                   std::cout << "STATE MACHINE" << std::endl;

                    d_BEIDOU_FSM.Event_beidou_word_preamble();
                    //record the preamble sample stamp
                    d_preamble_time_samples = current_time_samples;  // record the preamble sample stamp
                    DLOG(INFO) << "Preamble detection for SAT " << this->d_satellite << "current_time_samples=" << current_time_samples;
                    //sync the symbol to bits integrator
                    d_symbol_accumulator = 0;
                    d_symbol_accumulator_counter = 0;
                    d_stat = 1;  // enter into frame pre-detection status
                }
            else if (d_stat == 1)  //check 6 seconds of preamble separation
                {
 //                  std::cout << "6 SECONDS" << std::endl;

                    preamble_diff_ms = std::round(((static_cast<double>(current_time_samples) - d_preamble_time_samples) / static_cast<double>(current_samples_fs)) * 1000.0);
                    if (std::abs(preamble_diff_ms - BEIDOU_SUBFRAME_MS) < 1)
                        {
                   std::cout << "Preamble confirmation for SAT" << std::endl;

                            DLOG(INFO) << "Preamble confirmation for SAT " << this->d_satellite;
                            d_BEIDOU_FSM.Event_beidou_word_preamble();
                            d_flag_preamble = true;
                            d_make_correlation = false;
                            d_symbol_counter_corr = 0;
                            d_preamble_time_samples = current_time_samples;  // record the PRN start sample index associated to the preamble
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
                                               << static_cast<double>(d_preamble_time_samples) / static_cast<double>(current_samples_fs) << " [s]";
                                }
                        }
                     d_frame_bit_index = 11;
                    d_symbol_history.clear();
   		    for (int i = 0; i < BEIDOU_B1I_PREAMBLE_LENGTH_BITS; i++)
       		        {
          		    d_bit_buffer.push_back(d_preambles_symbols[i]);
                        }
                    word_number = 0;
                }
        }
    else
        {
            d_symbol_counter_corr++;
            if (d_symbol_counter_corr > (BEIDOU_SUBFRAME_MS - BEIDOU_B1I_TELEMETRY_SYMBOLS_PER_BIT))
                {
                    d_make_correlation = true;
                }
            if (d_stat == 1)
                {
                    preamble_diff_ms = round(((static_cast<double>(current_time_samples) - static_cast<double>(d_preamble_time_samples)) / static_cast<double>(current_samples_fs)) * 1000.0);
                    if (preamble_diff_ms > BEIDOU_SUBFRAME_MS + 1)
                        {
                            DLOG(INFO) << "Lost of frame sync SAT " << this->d_satellite << " preamble_diff= " << preamble_diff_ms;
                            d_stat = 0;  //lost of frame sync
                            d_flag_frame_sync = false;
                            flag_TOW_set = false;
                            d_make_correlation = true;
                            d_symbol_counter_corr = 0;
                        }
                }
        }
	if (d_flag_frame_sync and new_sym)
	    {
//                std::cout << symbol_value << std::endl;
                if (flag_PLL_180_deg_phase_locked)
                    {
                        d_bit_buffer.push_back(-symbol_value);
                    }
                else
                    {
                        d_bit_buffer.push_back(symbol_value);
                    }
            //******* bits to words ******
            d_frame_bit_index++;
            if (d_frame_bit_index == 30)
                {
                    word_number++;
                    beidou_b1i_telemetry_decoder_cc::decode_word(word_number, &d_bit_buffer, d_BEIDOU_frame_4bytes);
//        std::cout << d_BEIDOU_frame_4bytes << std::endl;

                    d_bit_buffer.clear();
                    d_frame_bit_index = 0;
                            memcpy(&d_BEIDOU_FSM.d_BEIDOU_frame_4bytes, &d_BEIDOU_frame_4bytes, sizeof(char) * 4);
                            //d_BEIDOU_FSM.d_preamble_time_ms = d_preamble_time_seconds * 1000.0;
                            d_BEIDOU_FSM.Event_beidou_word_valid();
                            // send TLM data to PVT using asynchronous message queues
                            if (d_BEIDOU_FSM.d_flag_new_subframe == true)
                                {
                                    switch (d_BEIDOU_FSM.d_subframe_ID)
                                        {
                                        case 3:  //we have a new set of ephemeris data for the current SV
                                            if (d_BEIDOU_FSM.d_nav.satellite_validation() == true)
                                                {
                                                    // get ephemeris object for this SV (mandatory)
                                                    std::shared_ptr<Beidou_Ephemeris> tmp_obj = std::make_shared<Beidou_Ephemeris>(d_BEIDOU_FSM.d_nav.get_ephemeris());
                                                    this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
                                                }
                                            break;
                                        case 4:  // Possible IONOSPHERE and UTC model update (page 18)
                                            if (d_BEIDOU_FSM.d_nav.flag_iono_valid == true)
                                                {
                                                    std::shared_ptr<Beidou_Iono> tmp_obj = std::make_shared<Beidou_Iono>(d_BEIDOU_FSM.d_nav.get_iono());
                                                    this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
                                                }
                                            if (d_BEIDOU_FSM.d_nav.flag_utc_model_valid == true)
                                                {
                                                    std::shared_ptr<Beidou_Utc_Model> tmp_obj = std::make_shared<Beidou_Utc_Model>(d_BEIDOU_FSM.d_nav.get_utc_model());
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
                                    d_BEIDOU_FSM.clear_flag_new_subframe();
                                    d_flag_new_tow_available = true;
                                }
                }
            }
    //2. Add the telemetry decoder information
    if (this->d_flag_preamble == true and d_flag_new_tow_available == true)
        {
            d_TOW_at_current_symbol_ms = static_cast<unsigned int>(d_BEIDOU_FSM.d_nav.d_SOW) * 1000 + BEIDOU_B1I_CODE_PERIOD_MS + BEIDOU_B1I_PREAMBLE_DURATION_MS;
            d_TOW_at_Preamble_ms = d_TOW_at_current_symbol_ms;
            flag_TOW_set = true;
            d_flag_new_tow_available = false;
        }
    else
        {
            d_TOW_at_current_symbol_ms += BEIDOU_B1I_CODE_PERIOD_MS;
        }

    current_symbol.TOW_at_current_symbol_ms = d_TOW_at_current_symbol_ms;
    current_symbol.Flag_valid_word = flag_TOW_set;

    if (flag_PLL_180_deg_phase_locked == true)
        {
            //correct the accumulated phase for the Costas loop phase shift, if required
            current_symbol.Carrier_phase_rads += BEIDOU_PI;
        }

    if (d_dump == true)
        {
            // MULTIPLEXED FILE RECORDING - Record results to file
            try
                {
                    double tmp_double;
                    unsigned long int tmp_ulong_int;
                    tmp_double = static_cast<double>(d_TOW_at_current_symbol_ms) / 1000.0;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                    tmp_ulong_int = current_symbol.Tracking_sample_counter;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_ulong_int), sizeof(unsigned long int));
                    tmp_double = static_cast<double>(d_TOW_at_Preamble_ms) * 1000.0;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }
            catch (const std::ifstream::failure &e)
                {
                    LOG(WARNING) << "Exception writing observables dump file " << e.what();
                }
        }

    //3. Make the output (copy the object contents to the GNURadio reserved memory)
    *out[0] = current_symbol;

    return 1;
}
