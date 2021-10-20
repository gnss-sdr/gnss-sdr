/*!
 * \file galileo_fnav_inav_decoder_test.cc
 * \brief  This class implements the unit test for the Galileo FNAV and INAV frames
 *  according to the Galileo ICD
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "galileo_fnav_message.h"
#include "galileo_inav_message.h"
#include "gnss_sdr_make_unique.h"  // for std::make_unique in C++11
#include "viterbi_decoder.h"
#include <gtest/gtest.h>
#include <algorithm>  // for copy
#include <array>
#include <chrono>
#include <exception>
#include <iterator>  // for std::back_inserter
#include <string>
#include <unistd.h>


class Galileo_FNAV_INAV_test : public ::testing::Test
{
public:
    Galileo_FNAV_INAV_test()
    {
        viterbi_fnav = std::make_unique<Viterbi_Decoder>(KK, nn, ((488 / nn) - mm), g_encoder);
        viterbi_inav = std::make_unique<Viterbi_Decoder>(KK, nn, ((240 / nn) - mm), g_encoder);
        flag_even_word_arrived = 0;
    }

    ~Galileo_FNAV_INAV_test() = default;

    Galileo_Inav_Message INAV_decoder;
    Galileo_Fnav_Message FNAV_decoder;
    const int32_t nn = 2;  // Coding rate 1/n
    const int32_t KK = 7;  // Constraint Length
    const int32_t mm = KK - 1;
    const std::array<int32_t, 2> g_encoder{{121, 91}};
    std::unique_ptr<Viterbi_Decoder> viterbi_fnav;
    std::unique_ptr<Viterbi_Decoder> viterbi_inav;
    int32_t flag_even_word_arrived;

    void deinterleaver(int32_t rows, int32_t cols, const float *in, float *out)
    {
        for (int32_t r = 0; r < rows; r++)
            {
                for (int32_t c = 0; c < cols; c++)
                    {
                        out[c * rows + r] = in[r * cols + c];
                    }
            }
    }

    bool decode_INAV_word(float *page_part_symbols, int32_t frame_length)
    {
        // 1. De-interleave
        std::vector<float> page_part_symbols_deint = std::vector<float>(frame_length / 2);
        std::copy(&page_part_symbols[0], &page_part_symbols[frame_length / 2], std::back_inserter(page_part_symbols_deint));
        deinterleaver(GALILEO_INAV_INTERLEAVER_ROWS, GALILEO_INAV_INTERLEAVER_COLS, page_part_symbols, page_part_symbols_deint.data());

        // 2. Viterbi decoder
        // 2.1 Take into account the NOT gate in G2 polynomial (Galileo ICD Figure 13, FEC encoder)
        // 2.2 Take into account the possible inversion of the polarity due to PLL lock at 180º
        for (int32_t i = 0; i < frame_length; i++)
            {
                if ((i + 1) % 2 == 0)
                    {
                        page_part_symbols_deint[i] = -page_part_symbols_deint[i];
                    }
            }

        std::vector<int32_t> page_part_bits = std::vector<int32_t>(frame_length / 2);
        viterbi_inav->decode(page_part_bits, page_part_symbols_deint);

        // 3. Call the Galileo page decoder
        std::string page_String;
        for (int32_t i = 0; i < (frame_length / 2); i++)
            {
                if (page_part_bits[i] > 0)
                    {
                        page_String.push_back('1');
                    }
                else
                    {
                        page_String.push_back('0');
                    }
            }

        bool crc_ok = false;
        if (page_part_bits[0] == 1)
            {
                // DECODE COMPLETE WORD (even + odd) and TEST CRC
                INAV_decoder.split_page(page_String, flag_even_word_arrived);
                if (INAV_decoder.get_flag_CRC_test() == true)
                    {
                        crc_ok = true;
                    }
                flag_even_word_arrived = 0;
            }
        else
            {
                // STORE HALF WORD (even page)
                INAV_decoder.split_page(page_String.c_str(), flag_even_word_arrived);
                flag_even_word_arrived = 1;
            }
        return crc_ok;
    }

    bool decode_FNAV_word(float *page_symbols, int32_t frame_length)
    {
        // 1. De-interleave
        std::vector<float> page_symbols_deint = std::vector<float>(frame_length);
        std::copy(&page_symbols[0], &page_symbols[frame_length / 2], std::back_inserter(page_symbols_deint));
        deinterleaver(GALILEO_FNAV_INTERLEAVER_ROWS, GALILEO_FNAV_INTERLEAVER_COLS, page_symbols, page_symbols_deint.data());

        // 2. Viterbi decoder
        // 2.1 Take into account the NOT gate in G2 polynomial (Galileo ICD Figure 13, FEC encoder)
        // 2.2 Take into account the possible inversion of the polarity due to PLL lock at 180
        for (int32_t i = 0; i < frame_length; i++)
            {
                if ((i + 1) % 2 == 0)
                    {
                        page_symbols_deint[i] = -page_symbols_deint[i];
                    }
            }

        std::vector<int32_t> page_bits = std::vector<int32_t>(frame_length);
        viterbi_fnav->decode(page_bits, page_symbols_deint);

        // 3. Call the Galileo page decoder
        std::string page_String;
        for (int32_t i = 0; i < frame_length; i++)
            {
                if (page_bits[i] > 0)
                    {
                        page_String.push_back('1');
                    }
                else
                    {
                        page_String.push_back('0');
                    }
            }

        // DECODE COMPLETE WORD (even + odd) and TEST CRC
        FNAV_decoder.split_page(page_String);
        if (FNAV_decoder.get_flag_CRC_test() == true)
            {
                return true;
            }
        return false;
    }
};


TEST_F(Galileo_FNAV_INAV_test, ValidationOfResults)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    start = std::chrono::system_clock::now();
    int repetitions = 10;
    // FNAV FULLY ENCODED FRAME
    float FNAV_frame[488] = {-1, 1, -1, -1, 1, -1, 1, 1, 1, -1, -1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        1, -1, -1, 1, -1, -1, 1, 1, 1, -1, 1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, -1, -1, 1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1,
        -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, 1, 1, -1, 1, -1, 1, -1,
        -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1, 1, -1, 1, -1, 1, 1, -1, 1, -1, 1, 1, -1, 1, 1, 1, -1, -1, 1, 1, -1, -1, -1, -1,
        -1, -1, -1, -1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1, -1, 1, 1, 1, 1, -1, -1, -1, -1, -1, 1, 1,
        -1, -1, -1, 1, -1, -1, -1, -1, -1, 1, -1, 1, 1, 1, 1, 1, -1, -1, 1, -1, -1, 1, 1, 1, 1, -1, -1, -1, -1, 1, -1, -1, -1,
        -1, 1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, 1, 1, -1, -1, -1, -1, -1, 1, -1, -1, 1, 1, 1, 1, 1, 1, -1,
        -1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1, -1, 1, -1, 1, 1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, 1, 1, 1, -1, 1, -1, -1, 1, 1, 1, 1, 1, -1, -1, 1, 1, -1, -1, -1, 1, -1, 1, -1, 1, 1, -1,
        1, -1, 1, 1, -1, -1, -1, 1, 1, -1, 1, 1, 1, -1, -1, -1, -1, 1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1,
        -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, -1, 1, 1, -1, -1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 1, 1, 1, -1, -1, -1, 1, -1, -1, -1, 1,
        1, -1, 1, -1, -1, 1, 1, -1, -1, -1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1, -1, 1, 1, 1, -1,
        -1, 1, -1, -1, -1, -1, 1, -1, -1, -1, -1, 1, 1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, 1, 1, 1, -1, -1, 1,
        1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, 1, -1, -1, -1, 1, 1, 1, -1, 1, 1, 1, 1, -1, -1, -1, 1, 1, 1, 1, 1,
        1, -1, 1, -1, -1, 1, 1, 1, -1, -1, 1, -1, 1, 1};

    ASSERT_NO_THROW({
        for (int n = 0; n < repetitions; n++)
            {
                EXPECT_EQ(decode_FNAV_word(&FNAV_frame[0], 488), true);
            }
    }) << "Exception during FNAV frame decoding";


    // INAV FULLY ENCODED FRAME
    float INAV_frame_even[240] = {-1, -1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, 1, 1, -1, -1, -1, 1, 1,
        -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, -1, -1, 1, -1, -1, -1, 1, -1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, -1, 1, 1, -1, 1, 1, 1, 1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, 1, -1, 1, -1, -1, -1, -1, 1, 1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, 1, -1, -1, -1, -1, 1, 1, -1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1,
        1, -1, 1, 1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, 1,
        -1, -1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1, 1, -1, 1, 1, 1};

    float INAV_frame_odd[240] = {1, -1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        1, 1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1,
        1, 1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1,
        -1, -1, 1, 1, -1, 1, 1, 1, -1, 1, 1, 1, 1, -1, -1, 1,
        1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, 1, 1, 1, -1, -1, -1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 1, -1, -1,
        1, -1, 1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, 1, -1, -1,
        -1, -1, -1, -1, -1, -1, 1, -1, 1, 1, 1, 1, -1, 1, -1, 1,
        1, 1, -1, 1, 1, 1, 1, 1, -1, -1, -1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, -1, 1, -1, -1, -1, -1, -1, 1, -1, 1, -1, 1,
        -1, -1, -1, -1, -1, -1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1,
        -1, -1, 1, -1, -1, -1, -1, -1, -1, 1, -1, 1, 1, 1, 1, 1,
        1, -1, -1, -1, -1, 1, 1, -1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, -1, -1, -1, -1, 1, 1, -1, -1, 1, 1};

    ASSERT_NO_THROW({
        for (int n = 0; n < repetitions; n++)
            {
                decode_INAV_word(&INAV_frame_even[0], 240);
                EXPECT_EQ(decode_INAV_word(&INAV_frame_odd[0], 240), true);
            }
    }) << "Exception during INAV frame decoding";
    end = std::chrono::system_clock::now();
    elapsed_seconds = end - start;
    std::cout << "Galileo INAV/FNAV CRC and Viterbi decoder test completed in " << elapsed_seconds.count() * 1e6 << " microseconds\n";
}
