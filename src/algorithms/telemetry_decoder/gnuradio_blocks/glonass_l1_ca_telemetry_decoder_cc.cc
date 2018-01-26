/*!
 * \file glonass_l1_ca_telemetry_decoder_cc.cc
 * \brief Implementation of an adapter of a GLONASS L1 C/A NAV data decoder block
 * to a TelemetryDecoderInterface
 * \note Code added as part of GSoC 2017 program
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
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


#include "glonass_l1_ca_telemetry_decoder_cc.h"
#include <boost/lexical_cast.hpp>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>


#define CRC_ERROR_LIMIT 6

using google::LogMessage;


glonass_l1_ca_telemetry_decoder_cc_sptr
glonass_l1_ca_make_telemetry_decoder_cc(const Gnss_Satellite & satellite, bool dump)
{
    return glonass_l1_ca_telemetry_decoder_cc_sptr(new glonass_l1_ca_telemetry_decoder_cc(satellite, dump));
}


glonass_l1_ca_telemetry_decoder_cc::glonass_l1_ca_telemetry_decoder_cc(
        const Gnss_Satellite & satellite,
        bool dump) :
        gr::block("glonass_l1_ca_telemetry_decoder_cc", gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
        gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    // Telemetry Bit transition synchronization port out
    this->message_port_register_out(pmt::mp("preamble_timestamp_s"));
    // Ephemeris data port out
    this->message_port_register_out(pmt::mp("telemetry"));
    // initialize internal vars
    d_dump = dump;
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    LOG(INFO) << "Initializing GLONASS L1 CA TELEMETRY DECODING";
    // Define the number of sampes per symbol. Notice that GLONASS has 2 rates,
    //one for the navigation data and the other for the preamble information
    d_samples_per_symbol = ( GLONASS_L1_CA_CODE_RATE_HZ / GLONASS_L1_CA_CODE_LENGTH_CHIPS ) / GLONASS_L1_CA_SYMBOL_RATE_BPS;

    // Set the preamble information
    unsigned short int preambles_bits[GLONASS_GNAV_PREAMBLE_LENGTH_BITS] = GLONASS_GNAV_PREAMBLE;
    // Since preamble rate is different than navigation data rate we use a constant
    d_symbols_per_preamble = GLONASS_GNAV_PREAMBLE_LENGTH_SYMBOLS;

    memcpy((unsigned short int*)this->d_preambles_bits, (unsigned short int*)preambles_bits, GLONASS_GNAV_PREAMBLE_LENGTH_BITS*sizeof(unsigned short int));

    // preamble bits to sampled symbols
    d_preambles_symbols = (signed int*)malloc(sizeof(signed int) * d_symbols_per_preamble);
    int n = 0;
    for (int i = 0; i < GLONASS_GNAV_PREAMBLE_LENGTH_BITS; i++)
        {
            for (unsigned int j = 0; j < GLONASS_GNAV_TELEMETRY_SYMBOLS_PER_PREAMBLE_BIT; j++)
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
    d_stat = 0;
    d_preamble_index = 0;

    d_flag_frame_sync = false;

    d_flag_parity = false;
    d_TOW_at_current_symbol = 0;
    Flag_valid_word = false;
    delta_t = 0;
    d_CRC_error_counter = 0;
    d_flag_preamble = false;
    d_channel = 0;
    flag_TOW_set = false;
    d_preamble_time_samples = 0;
}


glonass_l1_ca_telemetry_decoder_cc::~glonass_l1_ca_telemetry_decoder_cc()
{
    delete d_preambles_symbols;
    if(d_dump_file.is_open() == true)
        {
            try
            {
                    d_dump_file.close();
            }
            catch(const std::exception & ex)
            {
                    LOG(WARNING) << "Exception in destructor closing the dump file " << ex.what();
            }
        }
}


void glonass_l1_ca_telemetry_decoder_cc::decode_string(double *frame_symbols,int frame_length)
{
    double chip_acc = 0.0;
    int chip_acc_counter = 0;

    // 1. Transform from symbols to bits
    std::string bi_binary_code;
    std::string relative_code;
    std::string data_bits;

    // Group samples into bi-binary code
    for(int i = 0; i < (frame_length); i++)
        {
            chip_acc += frame_symbols[i];
            chip_acc_counter += 1;

            if(chip_acc_counter == (GLONASS_GNAV_TELEMETRY_SYMBOLS_PER_BIT))
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
    for(int i = 0; i < (GLONASS_GNAV_STRING_BITS); i++)
        {
            if(bi_binary_code[2*i] == '1' && bi_binary_code[2*i + 1] == '0')
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
    for(int i = 1; i < (GLONASS_GNAV_STRING_BITS); i++)
        {
            data_bits.push_back(((relative_code[i-1]-'0') ^ (relative_code[i]-'0')) + '0');
        }

    // 2. Call the GLONASS GNAV string decoder
    d_nav.string_decoder(data_bits);

    // 3. Check operation executed correctly
    if(d_nav.flag_CRC_test == true)
        {
            LOG(INFO) << "GLONASS GNAV CRC correct on channel " << d_channel << " from satellite "<< d_satellite;
        }
    else
        {
            LOG(INFO) << "GLONASS GNAV CRC error on channel " << d_channel <<  " from satellite " << d_satellite;
        }
    // 4. Push the new navigation data to the queues
    if (d_nav.have_new_ephemeris() == true)
        {
            // get object for this SV (mandatory)
            std::shared_ptr<Glonass_Gnav_Ephemeris> tmp_obj = std::make_shared<Glonass_Gnav_Ephemeris>(d_nav.get_ephemeris());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            LOG(INFO) << "GLONASS GNAV Ephemeris have been received on channel" << d_channel << " from satellite " << d_satellite;
        }
    if (d_nav.have_new_utc_model() == true)
        {
            // get object for this SV (mandatory)
            std::shared_ptr<Glonass_Gnav_Utc_Model> tmp_obj = std::make_shared<Glonass_Gnav_Utc_Model>(d_nav.get_utc_model());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            LOG(INFO) << "GLONASS GNAV UTC Model have been received on channel" << d_channel << " from satellite " << d_satellite;
        }
    if (d_nav.have_new_almanac() == true)
        {
            unsigned int slot_nbr = d_nav.i_alm_satellite_slot_number;
            std::shared_ptr<Glonass_Gnav_Almanac> tmp_obj= std::make_shared<Glonass_Gnav_Almanac>(d_nav.get_almanac(slot_nbr));
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            LOG(INFO) << "GLONASS GNAV Almanac have been received on channel" << d_channel << " in slot number " << slot_nbr;
        }
    // 5. Update satellite information on system
    if(d_nav.flag_update_slot_number == true)
        {
            LOG(INFO) << "GLONASS GNAV Slot Number Identified on channel " << d_channel;
            d_satellite.update_PRN(d_nav.gnav_ephemeris.d_n);
            d_satellite.what_block(d_satellite.get_system(), d_nav.gnav_ephemeris.d_n);
            d_nav.flag_update_slot_number = false;
        }
}


int glonass_l1_ca_telemetry_decoder_cc::general_work (int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    int corr_value = 0;
    int preamble_diff = 0;

    Gnss_Synchro **out = (Gnss_Synchro **) &output_items[0];
    const Gnss_Synchro **in = (const Gnss_Synchro **)  &input_items[0]; //Get the input samples pointer

    Gnss_Synchro current_symbol; //structure to save the synchronization information and send the output object to the next block
    //1. Copy the current tracking output
    current_symbol = in[0][0];
    d_symbol_history.push_back(current_symbol); //add new symbol to the symbol queue
    d_sample_counter++; //count for the processed samples
    consume_each(1);

    d_flag_preamble = false;
    unsigned int required_symbols=GLONASS_GNAV_STRING_SYMBOLS;

    if (d_symbol_history.size()>required_symbols)
        {
            //******* preamble correlation ********
            for (int i = 0; i < d_symbols_per_preamble; i++)
                {
                    if (d_symbol_history.at(i).Prompt_I < 0)    // symbols clipping
                        {
                            corr_value -= d_preambles_symbols[i];
                        }
                    else
                        {
                            corr_value += d_preambles_symbols[i];
                        }
                }
        }

    //******* frame sync ******************
    if (d_stat == 0) //no preamble information
        {
            if (abs(corr_value) >= d_symbols_per_preamble)
                {
                    // Record the preamble sample stamp
                    d_preamble_index = d_sample_counter;
                    LOG(INFO) << "Preamble detection for GLONASS L1 C/A SAT " << this->d_satellite;
                    // Enter into frame pre-detection status
                    d_stat = 1;
                    d_preamble_time_samples = d_symbol_history.at(0).Tracking_sample_counter; // record the preamble sample stamp
                }
        }
    else if (d_stat == 1) // posible preamble lock
        {
            if (abs(corr_value) >= d_symbols_per_preamble)
                {
                    //check preamble separation
                    preamble_diff = d_sample_counter - d_preamble_index;
                    // Record the PRN start sample index associated to the preamble
                    d_preamble_time_samples = d_symbol_history.at(0).Tracking_sample_counter;
                    if (abs(preamble_diff - GLONASS_GNAV_PREAMBLE_PERIOD_SYMBOLS) == 0)
                        {
                            //try to decode frame
                            LOG(INFO) << "Starting string decoder for GLONASS L1 C/A SAT " << this->d_satellite;
                            d_preamble_index = d_sample_counter; //record the preamble sample stamp
                            d_stat = 2;
                            // send asynchronous message to tracking to inform of frame sync and extend correlation time
                            pmt::pmt_t value = pmt::from_double(static_cast<double>(d_preamble_time_samples) / static_cast<double>(d_symbol_history.at(0).fs) - 0.001);
                            this->message_port_pub(pmt::mp("preamble_timestamp_s"), value);
                        }
                    else
                        {
                            if (preamble_diff > GLONASS_GNAV_PREAMBLE_PERIOD_SYMBOLS)
                                {
                                    d_stat = 0; // start again
                                }
                            DLOG(INFO) << "Failed string decoder for GLONASS L1 C/A SAT " << this->d_satellite;
                        }
                }
        }
    else if (d_stat == 2)
        {
            // FIXME: The preamble index marks the first symbol of the string count. Here I just wait for another full string to be received before processing
            if (d_sample_counter == d_preamble_index + GLONASS_GNAV_STRING_SYMBOLS)
                {
                    // NEW GLONASS string received
                    // 0. fetch the symbols into an array
                    int string_length = GLONASS_GNAV_STRING_SYMBOLS - d_symbols_per_preamble;
                    double string_symbols[GLONASS_GNAV_DATA_SYMBOLS] = {0};

                    //******* SYMBOL TO BIT *******
                    for (int i = 0; i < string_length; i++)
                        {
                            if (corr_value > 0)
                                {
                                    string_symbols[i] = d_symbol_history.at(i + d_symbols_per_preamble).Prompt_I; // because last symbol of the preamble is just received now!

                                }
                            else
                                {
                                    string_symbols[i] = -d_symbol_history.at(i + d_symbols_per_preamble).Prompt_I; // because last symbol of the preamble is just received now!
                                }
                        }

                    //call the decoder
                    decode_string(string_symbols, string_length);
                    if (d_nav.flag_CRC_test == true)
                        {
                            d_CRC_error_counter = 0;
                            d_flag_preamble = true; //valid preamble indicator (initialized to false every work())
                            d_preamble_index = d_sample_counter;  //record the preamble sample stamp (t_P)
                            if (!d_flag_frame_sync)
                                {
                                    d_flag_frame_sync = true;
                                    DLOG(INFO) << " Frame sync SAT " << this->d_satellite << " with preamble start at "
                                            << d_symbol_history.at(0).Tracking_sample_counter << " [samples]";
                                }
                        }
                    else
                        {
                            d_CRC_error_counter++;
                            d_preamble_index = d_sample_counter;  //record the preamble sample stamp
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
    //2. Add the telemetry decoder information
    if (this->d_flag_preamble == true and d_nav.flag_TOW_new == true)
        //update TOW at the preamble instant
        {
            d_TOW_at_current_symbol = floor((d_nav.gnav_ephemeris.d_TOW-GLONASS_GNAV_PREAMBLE_DURATION_S)*1000)/1000;
            d_nav.flag_TOW_new = false;

        }
    else //if there is not a new preamble, we define the TOW of the current symbol
        {
            d_TOW_at_current_symbol = d_TOW_at_current_symbol + GLONASS_L1_CA_CODE_PERIOD;
        }

    //if (d_flag_frame_sync == true and d_nav.flag_TOW_set==true and d_nav.flag_CRC_test == true)

    // if(d_nav.flag_GGTO_1 == true  and  d_nav.flag_GGTO_2 == true and  d_nav.flag_GGTO_3 == true and  d_nav.flag_GGTO_4 == true) //all GGTO parameters arrived
    //     {
    //         delta_t = d_nav.A_0G_10 + d_nav.A_1G_10 * (d_TOW_at_current_symbol - d_nav.t_0G_10 + 604800.0 * (fmod((d_nav.WN_0 - d_nav.WN_0G_10), 64)));
    //     }

    if (d_flag_frame_sync == true and d_nav.flag_TOW_set == true)
        {
            current_symbol.Flag_valid_word = true;
        }
    else
        {
            current_symbol.Flag_valid_word = false;
        }

    current_symbol.PRN = this->d_satellite.get_PRN();
    current_symbol.TOW_at_current_symbol_s = d_TOW_at_current_symbol;
    current_symbol.TOW_at_current_symbol_s -=delta_t; //Galileo to GPS TOW

    if(d_dump == true)
        {
            // MULTIPLEXED FILE RECORDING - Record results to file
            try
            {
                    double tmp_double;
                    unsigned long int tmp_ulong_int;
                    tmp_double = d_TOW_at_current_symbol;
                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                    tmp_ulong_int = current_symbol.Tracking_sample_counter;
                    d_dump_file.write((char*)&tmp_ulong_int, sizeof(unsigned long int));
                    tmp_double = 0;
                    d_dump_file.write((char*)&tmp_double, sizeof(double));
            }
            catch (const std::ifstream::failure & e)
            {
                    LOG(WARNING) << "Exception writing observables dump file " << e.what();
            }
        }

    // remove used symbols from history
    if (d_symbol_history.size()>required_symbols)
        {
            d_symbol_history.pop_front();
        }
    //3. Make the output (copy the object contents to the GNURadio reserved memory)
    *out[0] = current_symbol;

    return 1;
}


void glonass_l1_ca_telemetry_decoder_cc::set_satellite(const Gnss_Satellite & satellite)
{
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite "<< d_satellite;
    DLOG(INFO) << "Navigation Satellite set to " << d_satellite;
}


void glonass_l1_ca_telemetry_decoder_cc::set_channel(int channel)
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
                            d_dump_file.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "Telemetry decoder dump enabled on channel " << d_channel << " Log file: " << d_dump_filename.c_str();
                    }
                    catch (const std::ifstream::failure& e)
                    {
                            LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump file " << e.what();
                    }
                }
        }
}
