/*!
 * \file gps_l1_ca_telemetry_decoder_gs.cc
 * \brief Implementation of a NAV message demodulator block based on
 * Kay Borre book MATLAB-based GPS receiver
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "gps_l1_ca_telemetry_decoder_gs.h"
#include "gnss_sdr_make_unique.h"  // for std::make_unique in C++11
#include "gps_ephemeris.h"         // for Gps_Ephemeris
#include "gps_iono.h"              // for Gps_Iono
#include "gps_utc_model.h"         // for Gps_Utc_Model
#include "tlm_utils.h"
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>        // for make_any
#include <pmt/pmt_sugar.h>  // for mp
#include <bitset>           // for bitset
#include <cmath>            // for round
#include <cstddef>          // for size_t
#include <cstring>          // for memcpy
#include <exception>        // for exception
#include <iostream>         // for cout
#include <memory>           // for shared_ptr
#include <vector>

#ifdef COMPILER_HAS_ROTL
#include <bit>
namespace my_rotl = std;
#else
namespace my_rotl
{
#if HAS_GENERIC_LAMBDA
auto rotl = [](auto x, auto n) { return (((x) << (n)) ^ ((x) >> (32 - (n)))); };
#else
auto rotl = [](uint32_t x, uint32_t n) { return (((x) << (n)) ^ ((x) >> (32 - (n)))); };
#endif
}  // namespace my_rotl
#endif

#if PMT_USES_BOOST_ANY
#include <boost/any.hpp>
namespace wht = boost;
#else
#include <any>
namespace wht = std;
#endif

gps_l1_ca_telemetry_decoder_gs_sptr
gps_l1_ca_make_telemetry_decoder_gs(const Gnss_Satellite &satellite, const Tlm_Conf &conf)
{
    return gps_l1_ca_telemetry_decoder_gs_sptr(new gps_l1_ca_telemetry_decoder_gs(satellite, conf));
}


gps_l1_ca_telemetry_decoder_gs::gps_l1_ca_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    const Tlm_Conf &conf) : gr::block("gps_navigation_gs", gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
                                gr::io_signature::make(1, 1, sizeof(Gnss_Synchro))),
                            d_dump_filename(conf.dump_filename),
                            d_sample_counter(0ULL),
                            d_preamble_index(0ULL),
                            d_last_valid_preamble(0),
                            d_bits_per_preamble(GPS_CA_PREAMBLE_LENGTH_BITS),
                            d_samples_per_preamble(GPS_CA_PREAMBLE_LENGTH_BITS),
                            d_preamble_period_symbols(GPS_SUBFRAME_BITS),
                            d_CRC_error_counter(0),
                            d_channel(0),
                            d_required_symbols(GPS_SUBFRAME_BITS),
                            d_prev_GPS_frame_4bytes(0),
                            d_stat(0),
                            d_TOW_at_Preamble_ms(0),
                            d_TOW_at_current_symbol_ms(0),
                            d_flag_frame_sync(false),
                            d_flag_preamble(false),
                            d_sent_tlm_failed_msg(false),
                            d_flag_PLL_180_deg_phase_locked(false),
                            d_flag_TOW_set(false),
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

    if (d_enable_navdata_monitor)
        {
            // register nav message monitor out
            this->message_port_register_out(pmt::mp("Nav_msg_from_TLM"));
            d_nav_msg_packet.system = std::string("G");
            d_nav_msg_packet.signal = std::string("1C");
        }

    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Initializing GPS L1 TELEMETRY DECODER";

    // set the preamble
    // preamble bits to sampled symbols
    d_max_symbols_without_valid_frame = d_required_symbols * 20;  // rise alarm 120 segs without valid tlm
    int32_t n = 0;
    for (int32_t i = 0; i < d_bits_per_preamble; i++)
        {
            if (GPS_CA_PREAMBLE[i] == '1')
                {
                    d_preamble_samples[n] = 1;
                }
            else
                {
                    d_preamble_samples[n] = -1;
                }
            n++;
        }

    d_symbol_history.set_capacity(d_required_symbols);

    set_tag_propagation_policy(TPP_DONT);  // no tag propagation, the time tag will be adjusted and regenerated in work()

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


gps_l1_ca_telemetry_decoder_gs::~gps_l1_ca_telemetry_decoder_gs()
{
    DLOG(INFO) << "GPS L1 C/A Telemetry decoder block (channel " << d_channel << ") destructor called.";
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


bool gps_l1_ca_telemetry_decoder_gs::gps_word_parityCheck(uint32_t gpsword)
{
    // XOR as many bits in parallel as possible.  The magic constants pick
    //   up bits which are to be XOR'ed together to implement the GPS parity
    //   check algorithm described in IS-GPS-200M.  This avoids lengthy shift-
    //   and-xor loops.
    const uint32_t d1 = gpsword & 0xFBFFBF00U;
    const uint32_t d2 = my_rotl::rotl(gpsword, 1U) & 0x07FFBF01U;
    const uint32_t d3 = my_rotl::rotl(gpsword, 2U) & 0xFC0F8100U;
    const uint32_t d4 = my_rotl::rotl(gpsword, 3U) & 0xF81FFE02U;
    const uint32_t d5 = my_rotl::rotl(gpsword, 4U) & 0xFC00000EU;
    const uint32_t d6 = my_rotl::rotl(gpsword, 5U) & 0x07F00001U;
    const uint32_t d7 = my_rotl::rotl(gpsword, 6U) & 0x00003000U;
    const uint32_t t = d1 ^ d2 ^ d3 ^ d4 ^ d5 ^ d6 ^ d7;
    // Now XOR the 5 6-bit fields together to produce the 6-bit final result.
    uint32_t parity = t ^ my_rotl::rotl(t, 6U) ^ my_rotl::rotl(t, 12U) ^ my_rotl::rotl(t, 18U) ^ my_rotl::rotl(t, 24U);
    parity = parity & 0x3FU;
    if (parity == (gpsword & 0x3FU))
        {
            return true;
        }

    return false;
}


void gps_l1_ca_telemetry_decoder_gs::set_satellite(const Gnss_Satellite &satellite)
{
    d_nav = Gps_Navigation_Message();
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite " << d_satellite;
    d_nav.set_satellite_PRN(d_satellite.get_PRN());
    DLOG(INFO) << "Navigation Satellite set to " << d_satellite;
}


void gps_l1_ca_telemetry_decoder_gs::set_channel(int32_t channel)
{
    d_channel = channel;
    d_nav.set_channel(channel);
    DLOG(INFO) << "Navigation channel set to " << channel;
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
                            LOG(INFO) << "Telemetry decoder dump enabled on channel " << d_channel
                                      << " Log file: " << d_dump_filename.c_str();
                        }
                    catch (const std::ofstream::failure &e)
                        {
                            LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump file " << e.what();
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


bool gps_l1_ca_telemetry_decoder_gs::decode_subframe(bool flag_invert)
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
            if (flag_invert == false)
                {
                    if (subframe_symbol > 0)
                        {
                            GPS_frame_4bytes += 1;  // insert the telemetry bit in LSB
                        }
                }
            else
                {
                    if (subframe_symbol < 0)
                        {
                            GPS_frame_4bytes += 1;  // insert the inverted telemetry bit in LSB
                        }
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
                    bool crc_ok = gps_l1_ca_telemetry_decoder_gs::gps_word_parityCheck(GPS_frame_4bytes);
                    if (d_dump_crc_stats)
                        {
                            // update CRC statistics
                            d_Tlm_CRC_Stats->update_CRC_stats(crc_ok);
                        }
                    if (!crc_ok)
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
            if (d_enable_navdata_monitor)
                {
                    uint32_t gps_word;
                    std::bitset<GPS_SUBFRAME_BITS> subframe_bits;
                    std::bitset<GPS_WORD_BITS + 2> word_bits;
                    for (int32_t i = 0; i < 10; i++)
                        {
                            memcpy(&gps_word, &subframe[i * 4], sizeof(char) * 4);
                            word_bits = std::bitset<(GPS_WORD_BITS + 2)>(gps_word);
                            for (int32_t j = 0; j < GPS_WORD_BITS; j++)
                                {
                                    subframe_bits[GPS_WORD_BITS * (9 - i) + j] = word_bits[j];
                                }
                        }
                    d_nav_msg_packet.nav_message = subframe_bits.to_string();
                }
            const int32_t subframe_ID = d_nav.subframe_decoder(subframe.data());  // decode the subframe
            if (subframe_ID > 0 && subframe_ID < 6)
                {
                    std::cout << "New GPS NAV message received in channel " << this->d_channel << ": "
                              << "subframe "
                              << subframe_ID << " from satellite "
                              << Gnss_Satellite(std::string("GPS"), d_nav.get_satellite_PRN()) << '\n';

                    switch (subframe_ID)
                        {
                        case 1:
                            if (d_nav.satellite_validation() == true)
                                {
                                    // get ephemeris object for this SV (mandatory)
                                    const std::shared_ptr<Gps_Ephemeris> tmp_obj = std::make_shared<Gps_Ephemeris>(d_nav.get_ephemeris());
                                    this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
                                }

                            break;
                        case 2:
                            if (d_nav.satellite_validation() == true)
                                {
                                    // get ephemeris object for this SV (mandatory)
                                    const std::shared_ptr<Gps_Ephemeris> tmp_obj = std::make_shared<Gps_Ephemeris>(d_nav.get_ephemeris());
                                    this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
                                }

                            break;
                        case 3:  // we have a new set of ephemeris data for the current SV
                            if (d_nav.satellite_validation() == true)
                                {
                                    // get ephemeris object for this SV (mandatory)
                                    const std::shared_ptr<Gps_Ephemeris> tmp_obj = std::make_shared<Gps_Ephemeris>(d_nav.get_ephemeris());
                                    this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
                                }
                            break;
                        case 4:  // Possible IONOSPHERE and UTC model update (page 18)
                            if (d_nav.get_flag_iono_valid() == true)
                                {
                                    const std::shared_ptr<Gps_Iono> tmp_obj = std::make_shared<Gps_Iono>(d_nav.get_iono());
                                    this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
                                }
                            if (d_nav.get_flag_utc_model_valid() == true)
                                {
                                    const std::shared_ptr<Gps_Utc_Model> tmp_obj = std::make_shared<Gps_Utc_Model>(d_nav.get_utc_model());
                                    this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
                                }
                            break;
                        case 5:
                        // get almanac (if available)
                        // TODO: implement almanac reader in navigation_message
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
    d_flag_TOW_set = false;
    d_symbol_history.clear();
    d_stat = 0;
    DLOG(INFO) << "Telemetry decoder reset for satellite " << d_satellite;
}


int gps_l1_ca_telemetry_decoder_gs::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    auto **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);            // Get the output buffer pointer
    const auto **in = reinterpret_cast<const Gnss_Synchro **>(&input_items[0]);  // Get the input buffer pointer

    Gnss_Synchro current_symbol{};
    // 1. Copy the current tracking output
    current_symbol = in[0][0];
    if (d_symbol_history.empty())
        {
            // Tracking synchronizes the tlm bit boundaries by acquiring the preamble
            // inserting the preamble to the new tracked satellite (history empty) before the first synchronized symbol
            // may speed up the tlm decoding by not discarding the first received frame
            for (int32_t i = 0; i < GPS_CA_PREAMBLE_LENGTH_BITS; i++)
                {
                    if (current_symbol.Flag_PLL_180_deg_phase_locked == true)
                        {
                            d_symbol_history.push_back(static_cast<float>(-d_preamble_samples[i]));
                        }
                    else
                        {
                            d_symbol_history.push_back(static_cast<float>(d_preamble_samples[i]));
                        }
                    d_sample_counter++;
                }
        }
    // add new symbol to the symbol queue
    d_symbol_history.push_back(current_symbol.Prompt_I);

    d_sample_counter++;  // count for the processed symbols
    consume_each(1);
    d_flag_preamble = false;
    // check if there is a problem with the telemetry of the current satellite
    if (d_stat < 2 && d_sent_tlm_failed_msg == false)
        {
            if ((d_sample_counter - d_last_valid_preamble) > d_max_symbols_without_valid_frame)
                {
                    const int message = 1;  // bad telemetry
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
                if (d_symbol_history.size() >= d_required_symbols)
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
                        if (corr_value < 0)
                            {
                                d_flag_PLL_180_deg_phase_locked = true;
                            }
                        else
                            {
                                d_flag_PLL_180_deg_phase_locked = false;
                            }
                        DLOG(INFO) << "Preamble detection for GPS L1 satellite " << this->d_satellite;
                        d_prev_GPS_frame_4bytes = 0;
                        if (decode_subframe(d_flag_PLL_180_deg_phase_locked))
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
                                d_stat = 1;  // preamble acquired
                            }
                    }
                d_flag_TOW_set = false;
                break;
            }
        case 1:  // preamble acquired
            {
                if (d_sample_counter >= d_preamble_index + static_cast<uint64_t>(d_preamble_period_symbols))
                    {
                        DLOG(INFO) << "Preamble received for SAT " << this->d_satellite << "d_sample_counter=" << d_sample_counter << "\n";
                        // call the decoder
                        // 0. fetch the symbols into an array
                        d_preamble_index = d_sample_counter;  // record the preamble sample stamp (t_P)

                        if (decode_subframe(d_flag_PLL_180_deg_phase_locked))
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
                                        d_flag_TOW_set = false;
                                    }
                            }
                    }
                break;
            }
        }

    // 2. Add the telemetry decoder information
    if (d_flag_preamble == true)
        {
            if (!(d_nav.get_TOW() == 0))
                {
                    d_TOW_at_current_symbol_ms = static_cast<uint32_t>(d_nav.get_TOW() * 1000.0);
                    d_TOW_at_Preamble_ms = static_cast<uint32_t>(d_nav.get_TOW() * 1000.0);
                    d_flag_TOW_set = true;
                }
            else
                {
                    DLOG(INFO) << "Received GPS L1 TOW equal to zero at sat " << d_nav.get_satellite_PRN();
                }
        }
    else
        {
            if (d_flag_TOW_set == true)
                {
                    d_TOW_at_current_symbol_ms += GPS_L1_CA_BIT_PERIOD_MS;
                }
        }

    if (d_flag_TOW_set == true)
        {
            current_symbol.TOW_at_current_symbol_ms = d_TOW_at_current_symbol_ms;
            current_symbol.Flag_valid_word = d_flag_TOW_set;

            if (d_enable_navdata_monitor && !d_nav_msg_packet.nav_message.empty())
                {
                    d_nav_msg_packet.prn = static_cast<int32_t>(current_symbol.PRN);
                    d_nav_msg_packet.tow_at_current_symbol_ms = static_cast<int32_t>(d_TOW_at_current_symbol_ms);
                    const std::shared_ptr<Nav_Message_Packet> tmp_obj = std::make_shared<Nav_Message_Packet>(d_nav_msg_packet);
                    this->message_port_pub(pmt::mp("Nav_msg_from_TLM"), pmt::make_any(tmp_obj));
                    d_nav_msg_packet.nav_message = "";
                }

            if (d_flag_PLL_180_deg_phase_locked == true)
                {
                    // correct the accumulated phase for the Costas loop phase shift, if required
                    current_symbol.Carrier_phase_rads += GNSS_PI;
                    current_symbol.Flag_PLL_180_deg_phase_locked = true;
                }
            else
                {
                    current_symbol.Flag_PLL_180_deg_phase_locked = false;
                }

            // time tags
            std::vector<gr::tag_t> tags_vec;
            this->get_tags_in_range(tags_vec, 0, this->nitems_read(0), this->nitems_read(0) + 1);
            for (const auto &it : tags_vec)
                {
                    try
                        {
                            if (pmt::any_ref(it.value).type().hash_code() == typeid(const std::shared_ptr<GnssTime>).hash_code())
                                {
                                    const auto timetag = wht::any_cast<const std::shared_ptr<GnssTime>>(pmt::any_ref(it.value));
                                    // std::cout << "[" << this->nitems_written(0) + 1 << "] TLM RX TimeTag Week: " << timetag->week << ", TOW: " << timetag->tow_ms << " [ms], TOW fraction: " << timetag->tow_ms_fraction
                                    //           << " [ms], DELTA TLM TOW: " << static_cast<double>(timetag->tow_ms - current_symbol.TOW_at_current_symbol_ms) + timetag->tow_ms_fraction << " [ms] \n";
                                    add_item_tag(0, this->nitems_written(0) + 1, pmt::mp("timetag"), pmt::make_any(timetag));
                                }
                            else
                                {
                                    std::cout << "hash code not match\n";
                                }
                        }
                    catch (const wht::bad_any_cast &e)
                        {
                            std::cout << "msg Bad any_cast: " << e.what();
                        }
                }
            if (d_dump == true)
                {
                    // MULTIPLEXED FILE RECORDING - Record results to file
                    try
                        {
                            double tmp_double;
                            uint64_t tmp_ulong_int;
                            int32_t tmp_int;
                            tmp_double = static_cast<double>(d_TOW_at_current_symbol_ms) / 1000.0;
                            d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                            tmp_ulong_int = current_symbol.Tracking_sample_counter;
                            d_dump_file.write(reinterpret_cast<char *>(&tmp_ulong_int), sizeof(uint64_t));
                            tmp_double = static_cast<double>(d_TOW_at_Preamble_ms) / 1000.0;
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

            // 3. Make the output (copy the object contents to the GNU Radio reserved memory)
            *out[0] = current_symbol;

            return 1;
        }

    return 0;
}
