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
#include "gnss_synchro.h"
#include "gps_l1c_telemetry_decoder_gs.h"
#include <optional>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

gps_l1c_telemetry_decoder_gs_sptr
gps_l1c_make_telemetry_decoder_gs(const Tlm_Conf &conf)
{
    return gps_l1c_telemetry_decoder_gs_sptr(new gps_l1c_telemetry_decoder_gs(conf));
}

gps_l1c_telemetry_decoder_gs::gps_l1c_telemetry_decoder_gs(const Tlm_Conf &conf)
    : telemetry_impl_interface("gps_l1c_telemetry_decoder_gs",
          gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
          gr::io_signature::make(1, 1, sizeof(Gnss_Synchro))),
      d_stat(0),
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
}


void gps_l1c_telemetry_decoder_gs::set_satellite(const Gnss_Satellite &satellite)
{
}
void gps_l1c_telemetry_decoder_gs::set_channel(int32_t channel)
{
}

void gps_l1c_telemetry_decoder_gs::reset()
{
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

    consume_each(1);
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
    // The TOI is transmitted at the start of the subframe,
    // i.e. if we are aligned to the secondary code, the first 52 bits received are TOI; not the last as Figure 3.2-3
    // in IS-GPS-800J 01-AUG-2022 would lead one to believe at first.

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
    GpsL1cFrame frame = try_parse_frame(maybe_first_subframe_bit, toi);
    if (!frame.has_any_sf())
        {
            // No frame found, the toi was not properly aligned, keep searching
            return;
        }

    parse_new_subframe_data(frame);

    // Frame found! We are aligned, switch to aligned state
    d_frame_position = 0;
    d_stat = 1;
}

void gps_l1c_telemetry_decoder_gs::state_aligned(const Gnss_Synchro &synchro)
{
    d_frame_position++;

    if (d_frame_position != GPS_L1C_FRAME_BITS)
        {
            // No frame read yet, do nothing
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
            return;
        }

    GpsL1cFrame frame = try_parse_frame(first_subframe_bit, toi);
    if (!frame.has_any_sf())
        {
            // TODO: Same as before
            d_stat = 0;
            return;
        }

    parse_new_subframe_data(frame);

    // Reset frame pointer if frame parsed successfully
    d_frame_position = 0;
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
            out.sf2 = std::array<uint8_t, GPS_L1C_SF_2_DATA_BYTES>();
            out.has_sf2 = true;
            extract_bit_vector_to_array(sf2_decoded, out.sf2);
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
            out.sf3 = std::array<uint8_t, GPS_L1C_SF_3_DATA_BYTES>();
            out.has_sf3 = true;
            extract_bit_vector_to_array(sf3_decoded, out.sf3);
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
bool gps_l1c_telemetry_decoder_gs::check_subframe_crc(const std::array<uint8_t, SIZE> &bytes, size_t size_in_bits)
{
    assert(SIZE * 8 >= size_in_bits);

    CRC_GPS_L1C_type crc(GPS_L1C_CRC_POLYNOMIAL);

    const size_t whole_bytes = size_in_bits / 8;
    crc.process_bytes(bytes.data(), whole_bytes);

    if (size_in_bits != SIZE * 8)
        {
            const size_t rem_bits = size_in_bits - whole_bytes * 8;
            crc.process_bits(bytes[whole_bytes], rem_bits);
        }

    printf("CRC: %i\n", crc.checksum());
    // If we feed a message + CRC into itself, it must give 0
    return crc.checksum() == 0;
}

void gps_l1c_telemetry_decoder_gs::parse_new_subframe_data(const GpsL1cFrame &frame)
{
    if (frame.has_sf2 && check_subframe_crc(frame.sf2, GPS_L1C_SF_2_DATA_BITS))
        {
            parse_sf2_clock_ephemeris_tow(frame.sf2);
        }

    if (frame.has_sf3 && check_subframe_crc(frame.sf3, GPS_L1C_SF_3_DATA_BITS))
        {
        }
}

template <size_t SIZE>
void gps_l1c_telemetry_decoder_gs::extract_bit_vector_to_array(const std::vector<int> &bits, std::array<uint8_t, SIZE> &target)
{
    assert(SIZE >= bits.size() / 8);

    target.fill(0);

    for (size_t i = 0; i < bits.size(); i++)
        {
            if (bits[i] > 0)
                {
                    const size_t byte = i / 8;
                    const size_t bit_pos = i % 8;
                    // Bits are read "left to right", so we fill MSB first
                    const size_t shift = 7 - bit_pos;
                    target[byte] |= 1 << shift;
                }
        }
}


void gps_l1c_telemetry_decoder_gs::parse_sf2_clock_ephemeris_tow(const std::array<uint8_t, GPS_L1C_SF_2_DATA_BYTES> &sf2)
{
    auto wn = static_cast<uint16_t>(extract_unsigned(sf2, 0, 13));
    auto itow = static_cast<uint8_t>(extract_unsigned(sf2, 13, 21));
    auto top = static_cast<uint16_t>(extract_unsigned(sf2, 21, 32));
    auto l1c_health = static_cast<bool>(extract_unsigned(sf2, 32, 33));
    auto uraed = static_cast<int16_t>(extract_signed(sf2, 33, 38));
    auto toe = static_cast<uint16_t>(extract_unsigned(sf2, 38, 49));
    auto delta_A = static_cast<int32_t>(extract_signed(sf2, 49, 75));
    auto dot_A = static_cast<int32_t>(extract_signed(sf2, 75, 100));
    auto delta_n = static_cast<int32_t>(extract_signed(sf2, 100, 117));
    auto delta_dot_n = static_cast<int32_t>(extract_signed(sf2, 117, 140));
    auto m_0 = static_cast<int64_t>(extract_signed(sf2, 140, 173));
    auto e = static_cast<uint64_t>(extract_unsigned(sf2, 173, 206));
    auto w = static_cast<int64_t>(extract_signed(sf2, 206, 239));
    auto Omega = static_cast<int64_t>(extract_signed(sf2, 239, 272));
    auto i = static_cast<int64_t>(extract_signed(sf2, 272, 305));
    auto delta_dot_Omega = static_cast<int32_t>(extract_signed(sf2, 305, 322));
    auto idot = static_cast<int16_t>(extract_signed(sf2, 322, 337));
    auto cisn = static_cast<int16_t>(extract_signed(sf2, 337, 353));
    auto cicn = static_cast<int16_t>(extract_signed(sf2, 353, 369));
    auto crsn = static_cast<int32_t>(extract_signed(sf2, 369, 393));
    auto crcn = static_cast<int32_t>(extract_signed(sf2, 393, 417));
    auto cusn = static_cast<int32_t>(extract_signed(sf2, 417, 438));
    auto cucn = static_cast<int32_t>(extract_signed(sf2, 438, 459));
    auto uraned0 = static_cast<int8_t>(extract_signed(sf2, 459, 464));
    auto uraned1 = static_cast<uint8_t>(extract_unsigned(sf2, 464, 467));
    auto uraned2 = static_cast<uint8_t>(extract_unsigned(sf2, 467, 470));
    auto af0n = static_cast<int32_t>(extract_signed(sf2, 470, 496));
    auto af1n = static_cast<int32_t>(extract_signed(sf2, 496, 516));
    auto af2n = static_cast<int16_t>(extract_signed(sf2, 516, 526));
    auto tgd = static_cast<int16_t>(extract_signed(sf2, 526, 539));
    auto isc_l1cp = static_cast<int16_t>(extract_signed(sf2, 539, 552));
    auto isc_l1cd = static_cast<int16_t>(extract_signed(sf2, 552, 565));
    auto isf = static_cast<bool>(extract_unsigned(sf2, 565, 566));
    auto wnop = static_cast<uint8_t>(extract_unsigned(sf2, 567, 574));
    auto reserved = static_cast<uint16_t>(extract_unsigned(sf2, 574, 576));
}

template <size_t SIZE>
uint64_t gps_l1c_telemetry_decoder_gs::extract_unsigned(const std::array<uint8_t, SIZE> &bytes, size_t first_bit, size_t last_bit)
{
    uint64_t out = 0;
    const size_t numbits = last_bit - first_bit;

    for (size_t bit = first_bit; bit < last_bit; bit++)
        {
            const size_t byte = bit / 8;
            const size_t bitpos = bit % 8;

            // Array is MSB to LSB
            const uint8_t mask = 1 << (7 - bitpos);
            if ((bytes[byte] & mask) != 0)
                {
                    const size_t progress = bit - first_bit;

                    // We fill the number MSB to LSB
                    out |= 1ULL << (numbits - 1 - progress);
                }
        }

    return out;
}

template <size_t SIZE>
int64_t gps_l1c_telemetry_decoder_gs::extract_signed(const std::array<uint8_t, SIZE> &bytes, size_t first_bit, size_t last_bit)
{
    auto out = static_cast<int64_t>(extract_unsigned(bytes, first_bit, last_bit));

    const size_t byte = first_bit / 8;
    const size_t bitpos = first_bit % 8;
    const uint8_t mask = 1 << (7 - bitpos);

    const size_t numbits = last_bit - first_bit;

    // Check if the number was negative
    if ((bytes[byte] & mask) != 0)
        {
            out -= 1LL << numbits;
        }

    return out;
}
