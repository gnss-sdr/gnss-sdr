/*!
 * \file gps_l1_ca_telemetry_decoder_cc.cc
 * \brief Implementation of a NAV message demodulator block based on
 * Kay Borre book MATLAB-based GPS receiver
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
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

/*!
 * \todo Clean this code and move the telemetry definitions to GPS_L1_CA system definitions file
 */

#include "gnss_synchro.h"
#include "gps_l1_ca_telemetry_decoder_cc.h"
#include <iostream>
#include <sstream>
#include <bitset>
#include <gnuradio/gr_io_signature.h>
#include <glog/log_severity.h>
#include <glog/logging.h>
#include <boost/lexical_cast.hpp>
#include "control_message_factory.h"

#include "gnss_synchro.h"

#define _lrotl(X,N)  ((X << N) ^ (X >> (32-N)))  // Used in the parity check algorithm


using google::LogMessage;
/*!
 * \todo name and move the magic numbers to GPS_L1_CA.h
 */
gps_l1_ca_telemetry_decoder_cc_sptr
gps_l1_ca_make_telemetry_decoder_cc(Gnss_Satellite satellite, long if_freq, long fs_in, unsigned
        int vector_length, gr_msg_queue_sptr queue, bool dump)
{
    return gps_l1_ca_telemetry_decoder_cc_sptr(new gps_l1_ca_telemetry_decoder_cc(satellite, if_freq,
            fs_in, vector_length, queue, dump));
}



void gps_l1_ca_telemetry_decoder_cc::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
    for (unsigned i = 0; i < 3; i++)
        {
            ninput_items_required[i] = d_samples_per_bit * 8; //set the required sample history
        }
}



gps_l1_ca_telemetry_decoder_cc::gps_l1_ca_telemetry_decoder_cc(
        Gnss_Satellite satellite,
        long if_freq,
        long fs_in,
        unsigned
        int vector_length,
        gr_msg_queue_sptr queue,
        bool dump) :
        gr_block ("gps_navigation_cc", gr_make_io_signature (1, 1, sizeof(Gnss_Synchro)),
        gr_make_io_signature(1, 1, sizeof(Gnss_Synchro)))
{
    // initialize internal vars
    d_queue = queue;
    d_dump = dump;
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "TELEMETRY PROCESSING: satellite " << d_satellite;
    d_vector_length = vector_length;
    d_samples_per_bit = ( GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS ) / GPS_CA_TELEMETRY_RATE_BITS_SECOND;
    d_fs_in = fs_in;
    //d_preamble_duration_seconds = (1.0 / GPS_CA_TELEMETRY_RATE_BITS_SECOND) * GPS_CA_PREAMBLE_LENGTH_BITS;
    //std::cout<<"d_preamble_duration_seconds="<<d_preamble_duration_seconds<<"\r\n";
    // set the preamble
    unsigned short int preambles_bits[GPS_CA_PREAMBLE_LENGTH_BITS] = GPS_PREAMBLE;

    memcpy((unsigned short int*)this->d_preambles_bits, (unsigned short int*)preambles_bits, GPS_CA_PREAMBLE_LENGTH_BITS*sizeof(unsigned short int));

    // preamble bits to sampled symbols
    d_preambles_symbols = (signed int*)malloc(sizeof(signed int) * GPS_CA_PREAMBLE_LENGTH_BITS * d_samples_per_bit);
    int n = 0;
    for (int i=0; i<GPS_CA_PREAMBLE_LENGTH_BITS; i++)
        {
            for (unsigned int j=0; j<d_samples_per_bit; j++)
                {
                    if (d_preambles_bits[i] == 1)
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
    d_sample_counter = 0;
    d_preamble_code_phase_seconds = 0;
    d_stat = 0;
    d_preamble_index = 0;
    d_symbol_accumulator=0;
    d_symbol_accumulator_counter = 0;
    d_frame_bit_index = 0;
    d_preamble_time_seconds = 0;
    d_flag_frame_sync = false;
    d_GPS_frame_4bytes = 0;
    d_prev_GPS_frame_4bytes = 0;
    d_flag_parity = false;
    //set_history(d_samples_per_bit*8); // At least a history of 8 bits are needed to correlate with the preamble
}


gps_l1_ca_telemetry_decoder_cc::~gps_l1_ca_telemetry_decoder_cc()
{
    delete d_preambles_symbols;
    d_dump_file.close();
}



bool gps_l1_ca_telemetry_decoder_cc::gps_word_parityCheck(unsigned int gpsword)
{
    unsigned int d1, d2, d3, d4, d5, d6, d7, t, parity;
    /* XOR as many bits in parallel as possible.  The magic constants pick
       up bits which are to be XOR'ed together to implement the GPS parity
       check algorithm described in IS-GPS-200E.  This avoids lengthy shift-
       and-xor loops. */
    d1 = gpsword & 0xFBFFBF00;
    d2 = _lrotl(gpsword,1) & 0x07FFBF01;
    d3 = _lrotl(gpsword,2) & 0xFC0F8100;
    d4 = _lrotl(gpsword,3) & 0xF81FFE02;
    d5 = _lrotl(gpsword,4) & 0xFC00000E;
    d6 = _lrotl(gpsword,5) & 0x07F00001;
    d7 = _lrotl(gpsword,6) & 0x00003000;
    t = d1 ^ d2 ^ d3 ^ d4 ^ d5 ^ d6 ^ d7;
    // Now XOR the 5 6-bit fields together to produce the 6-bit final result.
    parity = t ^ _lrotl(t,6) ^ _lrotl(t,12) ^ _lrotl(t,18) ^ _lrotl(t,24);
    parity = parity & 0x3F;
    if (parity == (gpsword&0x3F)) return(true);
    else return(false);
}




int gps_l1_ca_telemetry_decoder_cc::general_work (int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,	gr_vector_void_star &output_items)
{
    int corr_value = 0;
    int preamble_diff = 0;

    Gnss_Synchro **out = (Gnss_Synchro **) &output_items[0];
    d_sample_counter++; //count for the processed samples

    // ########### Output the tracking data to navigation and PVT ##########
    const Gnss_Synchro **in = (const Gnss_Synchro **)  &input_items[0]; //Get the input samples pointer

    // TODO Optimize me!
    //******* preamble correlation ********
    for (unsigned int i=0; i<d_samples_per_bit*8; i++)
        {
            if (in[0][i].Prompt_I < 0)	// symbols clipping
                {
                    corr_value -= d_preambles_symbols[i];
                }
            else
                {
                    corr_value += d_preambles_symbols[i];
                }
        }
    d_flag_preamble=false;
    //******* frame sync ******************
    if (abs(corr_value) >= 160)
        {
            //TODO: Rewrite with state machine
            if (d_stat == 0)
                {
                    d_GPS_FSM.Event_gps_word_preamble();
                    d_preamble_index = d_sample_counter;//record the preamble sample stamp
                    std::cout << "Preamble detection for SAT " << this->d_satellite << std::endl;
                    d_symbol_accumulator = 0; //sync the symbol to bits integrator
                    d_symbol_accumulator_counter = 0;
                    d_frame_bit_index = 8;
                    d_stat = 1; // enter into frame pre-detection status
                }
            else if (d_stat == 1) //check 6 seconds of preample separation
                {
                    preamble_diff = abs(d_sample_counter - d_preamble_index);
                    if (abs(preamble_diff - 6000) < 1)
                        {
                            d_GPS_FSM.Event_gps_word_preamble();
                            d_flag_preamble = true;
                            d_preamble_index = d_sample_counter;  //record the preamble sample stamp (t_P)
                            d_preamble_time_seconds = in[0][0].Tracking_timestamp_secs;// - d_preamble_duration_seconds; //record the PRN start sample index associated to the preamble
                            d_preamble_code_phase_seconds = in[0][0].Code_phase_secs;

                            if (!d_flag_frame_sync)
                                {
                                    d_flag_frame_sync = true;
                                    std::cout <<" Frame sync SAT " << this->d_satellite << " with preamble start at " << d_preamble_time_seconds << " [s]" << std::endl;
                                }
                        }
                }
        }
    else
        {
            if (d_stat == 1)
                {
                    preamble_diff = d_sample_counter - d_preamble_index;
                    if (preamble_diff > 6001)
                        {
                            std::cout << "Lost of frame sync SAT " << this->d_satellite << " preamble_diff= " << preamble_diff << std::endl;
                            d_stat = 0; //lost of frame sync
                            d_flag_frame_sync = false;
                        }
                }
        }
    //******* SYMBOL TO BIT *******
    d_symbol_accumulator += in[0][d_samples_per_bit*8 - 1].Prompt_I; // accumulate the input value in d_symbol_accumulator
    d_symbol_accumulator_counter++;
    if (d_symbol_accumulator_counter == 20)
        {
            if (d_symbol_accumulator > 0)
                { //symbol to bit
                    d_GPS_frame_4bytes +=1; //insert the telemetry bit in LSB
                }
            d_symbol_accumulator = 0;
            d_symbol_accumulator_counter = 0;
            //******* bits to words ******
            d_frame_bit_index++;
            if (d_frame_bit_index == 30)
                {
                    d_frame_bit_index = 0;
                    // parity check
                    // Each word in wordbuff is composed of:
                    //      Bits 0 to 29 = the GPS data word
                    //      Bits 30 to 31 = 2 LSBs of the GPS word ahead.
                    // prepare the extended frame [-2 -1 0 ... 30]
                    if (d_prev_GPS_frame_4bytes & 0x00000001)
                        {
                            d_GPS_frame_4bytes = d_GPS_frame_4bytes | 0x40000000;
                        }
                    if (d_prev_GPS_frame_4bytes & 0x00000002)
                        {
                            d_GPS_frame_4bytes = d_GPS_frame_4bytes | 0x80000000;
                        }
                    /* Check that the 2 most recently logged words pass parity. Have to first
                     invert the data bits according to bit 30 of the previous word. */
                    if(d_GPS_frame_4bytes & 0x40000000)
                        {
                            d_GPS_frame_4bytes ^= 0x3FFFFFC0; // invert the data bits (using XOR)
                        }
                    if (gps_l1_ca_telemetry_decoder_cc::gps_word_parityCheck(d_GPS_frame_4bytes))
                        {
                            memcpy(&d_GPS_FSM.d_GPS_frame_4bytes, &d_GPS_frame_4bytes, sizeof(char)*4);
                            d_GPS_FSM.d_preamble_time_ms = d_preamble_time_seconds*1000.0;
                            d_GPS_FSM.Event_gps_word_valid();
                            d_flag_parity = true;
                        }
                    else
                        {
                            d_GPS_FSM.Event_gps_word_invalid();
                            d_flag_parity = false;
                        }
                    d_prev_GPS_frame_4bytes = d_GPS_frame_4bytes; // save the actual frame
                    d_GPS_frame_4bytes = d_GPS_frame_4bytes & 0;
                }
            else
                {
                    d_GPS_frame_4bytes<<=1; //shift 1 bit left the telemetry word
                }
        }
    // output the frame
    consume_each(1); //one by one
    Gnss_Synchro current_synchro_data; //structure to save the synchronization information and send the output object to the next block
    //1. Copy the current tracking output
    current_synchro_data=in[0][0];
    //2. Add the telemetry decoder information
    current_synchro_data.Flag_valid_word = (d_flag_frame_sync == true and d_flag_parity == true);
    current_synchro_data.Flag_preamble = d_flag_preamble;
    current_synchro_data.Preamble_timestamp_ms = d_preamble_time_seconds * 1000.0;
    current_synchro_data.Prn_timestamp_ms = in[0][0].Tracking_timestamp_secs * 1000.0;
    current_synchro_data.Preamble_symbol_counter = fmod((double)(d_sample_counter - d_preamble_index), 6000); //not corrected the preamble correlation lag! -> to be taken into account in TX Time
    if(d_dump == true)
        {
            // MULTIPLEXED FILE RECORDING - Record results to file
            try
            {
                    double tmp_double;
                    tmp_double = current_synchro_data.Preamble_timestamp_ms;
                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                    tmp_double = current_synchro_data.Prn_timestamp_ms;
                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                    tmp_double = current_synchro_data.Preamble_symbol_counter;
                    d_dump_file.write((char*)&tmp_double, sizeof(double));
            }
            catch (std::ifstream::failure e)
            {
                    std::cout << "Exception writing observables dump file " << e.what() << std::endl;
            }
        }
    //3. Make the output (copy the object contents to the GNURadio reserved memory)
    *out[0] = current_synchro_data;
    return 1;
}


void gps_l1_ca_telemetry_decoder_cc::set_satellite(Gnss_Satellite satellite)
{
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite "  << d_satellite;
    d_GPS_FSM.i_satellite_PRN = d_satellite.get_PRN();
    DLOG(INFO) << "Navigation Satellite set to " << d_satellite;
}


void gps_l1_ca_telemetry_decoder_cc::set_channel(int channel)
{
    d_channel = channel;
    d_GPS_FSM.i_channel_ID = channel;
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
                            d_dump_file.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            std::cout << "Telemetry decoder dump enabled on channel " << d_channel << " Log file: " << d_dump_filename.c_str() << std::endl;
                    }
                    catch (std::ifstream::failure e)
                    {
                            std::cout << "channel " << d_channel << " Exception opening trk dump file " << e.what() << std::endl;
                    }
                }
        }
}

