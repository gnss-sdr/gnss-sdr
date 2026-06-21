/*!
 * \file galileo_telemetry_decoder_gs.h
 * \brief Implementation of a GPS L1C message demodulator
 * block
 * \author José Antonio Mayo 2026. contact(at)tatjam.eu
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "GPS_L1C.h"
#include "display.h"
#include "gnss_synchro.h"
#include "gps_cnav2_navigation_message.h"
#include "gps_cnav_navigation_message.h"
#include "gps_l1c_telemetry_decoder_gs.h"
#include "tow_utils.h"
#include <gflags/gflags.h>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

gps_l1c_telemetry_decoder_gs_sptr
gps_l1c_make_telemetry_decoder_gs(const Tlm_Conf &conf, CnavSystem system)
{
    return gps_l1c_telemetry_decoder_gs_sptr(new gps_l1c_telemetry_decoder_gs(conf, system));
}

gps_l1c_telemetry_decoder_gs::gps_l1c_telemetry_decoder_gs(const Tlm_Conf &conf, CnavSystem system)
    : telemetry_impl_interface("gps_l1c_telemetry_decoder_gs",
          gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
          gr::io_signature::make(1, 1, sizeof(Gnss_Synchro))),
      d_system(system),
      d_channel(0),
      d_stat(0),
      d_frame_position(0),
      d_frames_since_last_valid_tow(0),
      d_has_valid_tow(false),
      d_sf2_decoder(
          GPS_L1C_LDPC_SF2_CN_NEIGHBORS_STR, GPS_L1C_LDPC_SF2_CN_NEIGHBORS_LENGTH,
          GPS_L1C_LDPC_SF2_CN_ROW_PTR_STR, GPS_L1C_LDPC_SF2_CN_ROW_PTR_LENGTH,
          GPS_L1C_LDPC_SF2_VN_COL_PTR_STR, GPS_L1C_LDPC_SF2_VN_COL_PTR_LENGTH,
          GPS_L1C_LDPC_SF2_PERMUTATION_STR, GPS_L1C_LDPC_SF2_PERMUTATION_LENGTH,
          GPS_L1C_LDPC_SF2_IS_16_BIT_HEXSTR),
      d_sf3_decoder(
          GPS_L1C_LDPC_SF3_CN_NEIGHBORS_STR, GPS_L1C_LDPC_SF3_CN_NEIGHBORS_LENGTH,
          GPS_L1C_LDPC_SF3_CN_ROW_PTR_STR, GPS_L1C_LDPC_SF3_CN_ROW_PTR_LENGTH,
          GPS_L1C_LDPC_SF3_VN_COL_PTR_STR, GPS_L1C_LDPC_SF3_VN_COL_PTR_LENGTH,
          GPS_L1C_LDPC_SF3_PERMUTATION_STR, GPS_L1C_LDPC_SF3_PERMUTATION_LENGTH,
          GPS_L1C_LDPC_SF3_IS_16_BIT_HEXSTR)
{
    configure_basic_outputs();

    d_symbol_history.set_capacity(GPS_L1C_FRAME_BITS);
    d_cnav2_message = std::make_unique<Gps_CNAV2_Navigation_Message>(0, d_system);
}


void gps_l1c_telemetry_decoder_gs::set_satellite(const Gnss_Satellite &satellite)
{
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << ((d_system == CnavSystem::GPS) ? "GPS" : "QZSS")
               << " L1C CNAV2 telemetry decoder in channel "
               << d_channel << " set to satellite " << d_satellite;
    d_cnav2_message = std::make_unique<Gps_CNAV2_Navigation_Message>(d_satellite.get_PRN(), d_system);
}
void gps_l1c_telemetry_decoder_gs::set_channel(int32_t channel)
{
    d_channel = channel;
    DLOG(INFO) << ((d_system == CnavSystem::GPS) ? "GPS" : "QZSS")
               << " L1C CNAV2 channel set to " << channel;

    // TODO
    // configure_dump_file(d_channel, d_dump, d_dump_filename, d_dump_file);
    // configure_crc_stats_channel(d_channel, d_dump_crc_stats, d_Tlm_CRC_Stats);
    d_cnav2_message = std::make_unique<Gps_CNAV2_Navigation_Message>(d_satellite.get_PRN(), d_system);
}

void gps_l1c_telemetry_decoder_gs::reset()
{
    d_stat = 0;
    DLOG(INFO) << "Telemetry decoder reset for satellite " << d_satellite;
}

int gps_l1c_telemetry_decoder_gs::general_work(int noutput_items, gr_vector_int &ninput_items,
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    // TODO: This assumes the secondary is locked and thus bits arrive start with the first bit of the
    // frame. If we have no pilot, we will need some alternative solution.

    auto **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);            // Get the output buffer pointer
    const auto **in = reinterpret_cast<const Gnss_Synchro **>(&input_items[0]);  // Get the input buffer pointer

    Gnss_Synchro current_symbol{};
    current_symbol = in[0][0];
    consume_each(1);

    // MSB is the first bit to arrive, thus d_symbol_history is ordered from MSB to LSB
    d_symbol_history.push_back(current_symbol.Prompt_I);

    switch (d_stat)
        {
        case 0:
            state_find_align(current_symbol);
            break;
        case 1:
            state_aligned(current_symbol);
            break;
        }

    if (d_has_valid_tow)
        {
            // Frame position refers to the bit parsed in this call
            int64_t frame_position_ms = static_cast<int64_t>(d_frame_position) * GPS_L1C_CODE_PERIOD_MS;
            int64_t delta_ms = static_cast<int64_t>(d_frames_since_last_valid_tow) * 18000 + frame_position_ms - 18000;

            // TODO: It appears the TOW_at_current_symbol_ms is for the falling edge? Or
            // is this a timing issue elsewhere? A similar +1 exists in the galileo inav_current_symbol_delay_ms function.
            delta_ms += GPS_L1C_CODE_PERIOD_MS;

            uint32_t tow_now = gnss_tow::add_ms(static_cast<int64_t>(d_last_valid_tow) * 1000, delta_ms);

            current_symbol.Flag_valid_word = true;
            current_symbol.TOW_at_current_symbol_ms = tow_now;

            out[0][0] = std::move(current_symbol);
            return 1;
        }

    return 0;
}

std::pair<uint16_t, float> gps_l1c_telemetry_decoder_gs::compute_toi_hypothesis(size_t start_index, uint16_t hypothesis, float &normalization)
{
    // Note that we ignore the "true MSB" bit of TOI, and deduce it from the correlation sign
    assert(start_index + GPS_L1C_TOI_BCH_BITS - 1 < d_symbol_history.size());

    const bool compute_normalization = normalization <= 0.0F;
    float hypothesis_value = 0.0F;

    // LSB only
    hypothesis &= 0x00FF;

    for (size_t j = 0; j < GPS_L1C_TOI_BCH_BITS - 1; j++)
        {
            // Note, the +1 because we ignore the MSB of the encoded TOI, which is the first bit to arrive
            const float bit = d_symbol_history[start_index + j + 1];
            const float expected = GPS_L1C_TOI_BCH_LSB_CODES[hypothesis][j] == '1' ? -1.0F : 1.0F;
            hypothesis_value += bit * expected;

            // Collect normalization on first run only
            if (compute_normalization)
                {
                    normalization += std::abs(bit);
                }
        }

    uint16_t toi = hypothesis;

    if (hypothesis_value < 0.0F)
        {
            // MSB is 1
            toi |= 0x0100;
        }

    return std::make_pair(toi, hypothesis_value / normalization);
}

int16_t gps_l1c_telemetry_decoder_gs::test_toi_hypotheses(size_t start_index, const Gnss_Synchro &synchro)
{
    if (start_index + GPS_L1C_TOI_BCH_BITS >= d_symbol_history.size())
        {
            return -1;
        }

    float normalization = 0.0F;
    float max_hypothesis_abs = -1.0F;
    uint16_t toi = 0;
    float toi_hypothesis_val = 0.0F;

    // TODO: Make configurable
    constexpr float min_hypothesis_abs = 0.95F;

    // Correlate the last received 51 bits against every possible LSB of TOI encoding
    for (int16_t i = 0; i < GPS_L1C_TOI_LSB_VALUES; i++)
        {
            const auto &[toi_deduced, hypothesis_val] = compute_toi_hypothesis(
                start_index, i, normalization);

            const float abs = std::abs(hypothesis_val);

            if (abs > min_hypothesis_abs && abs > max_hypothesis_abs)
                {
                    max_hypothesis_abs = abs;
                    toi = toi_deduced;
                    toi_hypothesis_val = hypothesis_val;
                }
        }

    if (max_hypothesis_abs < 0.0F)
        {
            // We were unable to confirm any of the hypotheses
            return -1;
        }

    DLOG(INFO) << "GPS L1C telemetry decoder for PRN " << synchro.PRN
               << " found valid TOI hypothesis = " << toi << " hypothesis value " << toi_hypothesis_val;

    return toi;
}

void gps_l1c_telemetry_decoder_gs::state_find_align(const Gnss_Synchro &synchro)
{
    if (d_symbol_history.size() < GPS_L1C_FRAME_BITS)
        {
            // Skip checking for TOI until we have enough bits as to decode the full frame
            return;
        }

    // const size_t maybe_first_subframe_bit = d_symbol_history.size() - GPS_L1C_FRAME_BITS;
    const size_t maybe_first_subframe_bit = 0;

    // Check if first 52 bits are a valid TOI
    int16_t toi = test_toi_hypotheses(maybe_first_subframe_bit, synchro);
    if (toi < 0)
        {
            // No TOI hypothesis, skip frame search
            return;
        }

    // Try to decode the remainder of the frame
    // TODO: Try to decode the inverted one too, and if that one matches,
    // invert the entire bits (i.e. change TOI MSB) as we have locked 180 out of phase
    GpsL1cFrame frame = try_parse_frame(maybe_first_subframe_bit, toi);
    if (!frame.has_any_sf())
        {
            // No frame found, the toi was not properly aligned, keep searching
            return;
        }

    parse_new_subframe_data(frame, synchro);

    // Frame found! We are aligned, switch to aligned state.
    d_frame_position = GPS_L1C_FRAME_BITS - 1;
    d_stat = 1;
}

void gps_l1c_telemetry_decoder_gs::state_aligned(const Gnss_Synchro &synchro)
{
    d_frame_position++;

    if (d_frame_position == GPS_L1C_FRAME_BITS)
        {
            // Wrap around do d_frame_position is always in range [0, GPS_L1C_FRAME_BITS)
            d_frame_position = 0;
            if (d_has_valid_tow)
                {
                    d_frames_since_last_valid_tow++;
                }
        }

    if (d_frame_position != GPS_L1C_FRAME_BITS - 1)
        {
            // No full frame read yet, do nothing
            return;
        }

    const size_t first_subframe_bit = d_symbol_history.size() - GPS_L1C_FRAME_BITS;

    // A full frame should have been received by now
    int16_t toi = test_toi_hypotheses(first_subframe_bit, synchro);

    if (toi < 0)
        {
            // We lost lock!
            // TODO: We could search around a bit just in case we are offset by a few samples
            d_stat = 0;
            d_has_valid_tow = false;
            return;
        }

    GpsL1cFrame frame = try_parse_frame(first_subframe_bit, toi);
    if (!frame.has_any_sf())
        {
            // TODO: Same as before
            d_stat = 0;
            d_has_valid_tow = false;
            return;
        }

    parse_new_subframe_data(frame, synchro);

    // Reset frame pointer if frame parsed successfully
    d_frame_position = GPS_L1C_FRAME_BITS - 1;
}


GpsL1cFrame gps_l1c_telemetry_decoder_gs::try_parse_frame(size_t start_index, uint16_t toi)
{
    assert(start_index + GPS_L1C_FRAME_BITS - 1 < d_symbol_history.size());

    // TODO: Make configurable
    constexpr int MAX_LDPC_ITERATIONS = 200;
    constexpr float ATTENUATION = 0.5F;

    std::array<float, GPS_L1C_SF_2_AND_3_ENCODED_BITS> deinterleaved = deinterlave_frame(start_index);

    GpsL1cFrame out{};
    out.toi = toi;

    // SF2
    std::vector<float> &sf2_inputs = d_sf2_decoder.get_inputs();
    for (size_t i = 0; i < GPS_L1C_SF_2_ENCODED_BITS; i++)
        {
            sf2_inputs[i] = deinterleaved[i];
        }

    d_sf2_decoder.prepare_iteration();
    std::vector<int> sf2_decoded = d_sf2_decoder.run_decoder(MAX_LDPC_ITERATIONS, ATTENUATION);

    if (!sf2_decoded.empty())
        {
            out.sf2 = std::bitset<GPS_L1C_SF_2_DATA_BITS>();
            out.has_sf2 = true;
            extract_bit_vector_to_bitset(sf2_decoded, out.sf2);
        }

    // SF3
    std::vector<float> &sf3_inputs = d_sf3_decoder.get_inputs();
    for (size_t i = 0; i < GPS_L1C_SF_3_ENCODED_BITS; i++)
        {
            sf3_inputs[i] = deinterleaved[i + GPS_L1C_SF_2_ENCODED_BITS];
        }

    d_sf3_decoder.prepare_iteration();
    std::vector<int> sf3_decoded = d_sf3_decoder.run_decoder(MAX_LDPC_ITERATIONS, ATTENUATION);

    if (!sf3_decoded.empty())
        {
            out.sf3 = std::bitset<GPS_L1C_SF_3_DATA_BITS>();
            out.has_sf3 = true;
            extract_bit_vector_to_bitset(sf3_decoded, out.sf3);
        }

    return out;
}

std::array<float, GPS_L1C_SF_2_AND_3_ENCODED_BITS> gps_l1c_telemetry_decoder_gs::deinterlave_frame(size_t start_index)
{
    assert(start_index + GPS_L1C_FRAME_BITS - 1 < d_symbol_history.size());

    std::array<float, GPS_L1C_SF_2_AND_3_ENCODED_BITS> out{};

    for (size_t rel = 0; rel < GPS_L1C_SF_2_AND_3_ENCODED_BITS; rel++)
        {
            const size_t abs = rel + GPS_L1C_TOI_BCH_BITS;
            const float bit = d_symbol_history[start_index + abs];

            size_t row = rel % GPS_L1C_INTERLEAVE_ROWS;
            size_t col = rel / GPS_L1C_INTERLEAVE_ROWS;

            out[row * GPS_L1C_INTERLEAVE_COLS + col] = bit;
        }

    return out;
}

template <size_t SIZE>
bool gps_l1c_telemetry_decoder_gs::check_subframe_crc(const std::bitset<SIZE> &bits)
{
    CRC_GPS_L1C_type crc(GPS_L1C_CRC_POLYNOMIAL);

    // CRC expects bit in "left to right" order, hence we revert again
    for (size_t i = 0; i < bits.size(); i++)
        {
            crc.process_bit(bits[bits.size() - 1 - i]);
        }

    printf("CRC: %i\n", crc.checksum());
    // If we feed a message + CRC into itself, it must give 0
    return crc.checksum() == 0;
}

void gps_l1c_telemetry_decoder_gs::parse_new_subframe_data(const GpsL1cFrame &frame, const Gnss_Synchro &synchro)
{
    if (frame.has_sf2 && check_subframe_crc(frame.sf2))
        {
            d_cnav2_message->decode_sf2(frame.toi, frame.sf2);

            // tow is in seconds since start of week, but goes in multiple of 2 hours, as it's actually ITOW,
            // TOI is in 18s intervals since last ITOW, up to 399 (included) which is 2 hours
            // Note that TOI is for the next subframe rising edge!
            d_last_valid_tow = d_cnav2_message->get_ephemeris().tow + frame.toi * 18;
            d_has_valid_tow = true;
            d_frames_since_last_valid_tow = 0;
        }

    if (frame.has_sf3 && check_subframe_crc(frame.sf3))
        {
            d_cnav2_message->decode_sf3(frame.toi, frame.sf3);
        }

    if (d_cnav2_message->have_new_ephemeris())
        {
            const std::shared_ptr<Gps_CNAV2_Ephemeris> tmp_obj = std::make_shared<Gps_CNAV2_Ephemeris>(d_cnav2_message->get_ephemeris());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));

            const auto default_precision = std::cout.precision();
            std::cout << TEXT_MAGENTA << "New " << ((d_system == CnavSystem::GPS) ? "GPS" : "QZSS")
                      << " L1C CNAV2 message received in channel " << d_channel
                      << ": ephemeris from satellite " << d_satellite
                      << " with CN0=" << std::setprecision(2) << synchro.CN0_dB_hz
                      << std::setprecision(default_precision) << " dB-Hz" << TEXT_RESET << std::endl;
        }

    if (d_cnav2_message->have_new_ephemeris())
        {
            const std::shared_ptr<Gps_CNAV2_Ephemeris> tmp_obj = std::make_shared<Gps_CNAV2_Ephemeris>(d_cnav2_message->get_ephemeris());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));

            const auto default_precision = std::cout.precision();
            std::cout << TEXT_MAGENTA << "New " << ((d_system == CnavSystem::GPS) ? "GPS" : "QZSS")
                      << " L1C CNAV2 message received in channel " << d_channel
                      << ": ephemeris from satellite " << d_satellite
                      << " with CN0=" << std::setprecision(2) << synchro.CN0_dB_hz
                      << std::setprecision(default_precision) << " dB-Hz" << TEXT_RESET << std::endl;
        }
}

template <size_t SIZE>
void gps_l1c_telemetry_decoder_gs::extract_bit_vector_to_bitset(const std::vector<int> &bits, std::bitset<SIZE> &target)
{
    assert(SIZE >= bits.size());

    target.reset();

    // Notice the reverse order of the bits sequence, required by the message decoder
    for (size_t i = 0; i < bits.size(); i++)
        {
            target[bits.size() - 1 - i] = bits[i] > 0;
        }
}
