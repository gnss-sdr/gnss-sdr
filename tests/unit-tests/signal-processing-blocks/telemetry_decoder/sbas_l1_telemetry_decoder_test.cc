/*!
 * \file sbas_l1_telemetry_decoder_test.cc
 * \brief  This class implements a deterministic unit test for the message
 *  timestamp computation of the SBAS L1 telemetry decoder.
 * \author Miguel Gómez López, 2026. mgomezl(at)ing.uc3m.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "SBAS_L1.h"
#include "sbas_l1_telemetry_decoder_gs.h"
#include <gtest/gtest.h>

TEST(SbasL1TelemetryDecoderTest, TimestampMatchesFirstBitWhenFullyAligned)
{
    // When both sample and symbol alignment already match the chosen
    // hypothesis, no residual correction is applied: the message timestamp
    // must equal the first bit's own absolute timestamp.
    const double first_bit_stamp_s = 123.456;
    const double message_stamp_s = sbas_l1_telemetry_decoder_gs::compute_message_timestamp(first_bit_stamp_s, true, true);
    EXPECT_DOUBLE_EQ(first_bit_stamp_s, message_stamp_s);
}


TEST(SbasL1TelemetryDecoderTest, TimestampCorrectionIsBoundedBySymbolPeriod)
{
    // The residual sample/symbol alignment correction must never exceed one
    // encoded symbol period: it only resolves a small, sub-bit ambiguity, not
    // the block-level offset that used to cause ~1 s errors.
    const double first_bit_stamp_s = 1000.0;
    const double max_correction_s = 2.0 * SBAS_L1_CODE_PERIOD_S;  // 1 symbol = 2 code periods

    for (const bool sample_aligned : {false, true})
        {
            for (const bool symbol_aligned : {false, true})
                {
                    const double message_stamp_s = sbas_l1_telemetry_decoder_gs::compute_message_timestamp(first_bit_stamp_s, sample_aligned, symbol_aligned);
                    EXPECT_NEAR(first_bit_stamp_s, message_stamp_s, max_correction_s);
                }
        }
}


TEST(SbasL1TelemetryDecoderTest, TimestampIsNeverInTheFuture)
{
    // A decoded message can only be timestamped at or before the time its
    // first bit was received; the previous implementation could report a
    // timestamp roughly 1 s ahead of the true preamble reception time.
    const double first_bit_stamp_s = 42.0;

    for (const bool sample_aligned : {false, true})
        {
            for (const bool symbol_aligned : {false, true})
                {
                    const double message_stamp_s = sbas_l1_telemetry_decoder_gs::compute_message_timestamp(first_bit_stamp_s, sample_aligned, symbol_aligned);
                    EXPECT_LE(message_stamp_s, first_bit_stamp_s);
                }
        }
}
