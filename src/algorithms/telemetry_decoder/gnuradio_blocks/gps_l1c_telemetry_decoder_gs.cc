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

int16_t gps_l1c_telemetry_decoder_gs::test_toi_hypotheses(const Gnss_Synchro &synchro)
{
    if (d_symbol_history.size() < GPS_L1C_FRAME_BITS)
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
                0, i, normalization);

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

    printf("Current TOI: %i | Val: %f\n", toi, toi_hypothesis_val);

    DLOG(INFO) << "GPS L1C telemetry decoder for PRN " << synchro.PRN
               << " found valid TOI hypothesis = " << toi << " hypothesis value " << toi_hypothesis_val;

    return toi;
}

void gps_l1c_telemetry_decoder_gs::state_find_align(const Gnss_Synchro &synchro)
{
    // The TOI is transmitted at the start of the subframe,
    // i.e. if we are aligned to the secondary code, the first 52 bits received are TOI; not the last as Figure 3.2-3
    // in IS-GPS-800J 01-AUG-2022 would lead one to believe at first.
    int16_t toi = test_toi_hypotheses(synchro);
    if (toi >= 0)
        {
            std::optional<GpsL1cFrame> frame = try_parse_frame(0, toi);
        }
}

void gps_l1c_telemetry_decoder_gs::state_aligned(const Gnss_Synchro &synchro)
{
}


std::optional<GpsL1cFrame> gps_l1c_telemetry_decoder_gs::try_parse_frame(size_t start_index, uint16_t toi)
{
    assert(start_index + GPS_L1C_FRAME_BITS - 1 < d_symbol_history.size());

    // TODO: Make configurable
    constexpr int MAX_LDPC_ITERATIONS = 200;
    constexpr float ATTENUATION = 0.5F;

    std::array<float, GPS_L1C_SF_2_AND_3_ENCODED_BITS> deinterleaved = deinterlave_frame(start_index);

    GpsL1cFrame out;
    out.toi = toi;

    // SF2
    std::vector<float> &sf2_inputs = d_sf2_decoder.get_inputs();
    for (size_t i = 0; i < GPS_L1C_SF_2_ENCODED_BITS; i++)
        {
            sf2_inputs[i] = deinterleaved[i];
        }

    d_sf2_decoder.prepare_iteration();
    std::optional<std::vector<int>> sf2_decoded = d_sf2_decoder.run_decoder(MAX_LDPC_ITERATIONS, ATTENUATION);

    // TODO: Parse to bitset, and set in sf2
    if (sf2_decoded.has_value())
        {
            printf("DECODED SF2\n");
        }

    // SF3
    std::vector<float> &sf3_inputs = d_sf3_decoder.get_inputs();
    for (size_t i = 0; i < GPS_L1C_SF_3_ENCODED_BITS; i++)
        {
            sf3_inputs[i] = deinterleaved[i + GPS_L1C_SF_2_ENCODED_BITS];
        }

    d_sf3_decoder.prepare_iteration();
    std::optional<std::vector<int>> sf3_decoded = d_sf3_decoder.run_decoder(MAX_LDPC_ITERATIONS, ATTENUATION);

    if (sf3_decoded.has_value())
        {
            printf("DECODED SF3\n");
        }

    // TODO: Parse to bitset, and set in sf3

    return std::nullopt;
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
