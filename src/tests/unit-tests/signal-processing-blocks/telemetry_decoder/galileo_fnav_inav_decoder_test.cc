/*!
 * \file galileo_fnav_inav_decoder_test.cc
 * \brief  This class implements the unit test for the Galileo FNAV and INAV frames
 *  according to the Galileo ICD
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012-2018  (see AUTHORS file for a list of contributors)
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

#include "galileo_navigation_message.h"
#include "galileo_fnav_message.h"
#include "convolutional.h"
#include <unistd.h>
#include <chrono>
#include <exception>
#include <string>
#include <armadillo>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <gtest/gtest.h>


class Galileo_FNAV_INAV_test : public ::testing::Test
{
public:
    Galileo_Navigation_Message INAV_decoder;
    Galileo_Fnav_Message FNAV_decoder;
    // vars for Viterbi decoder
    int32_t *out0, *out1, *state0, *state1;
    int32_t g_encoder[2];
    const int32_t nn = 2;  // Coding rate 1/n
    const int32_t KK = 7;  // Constraint Length
    int32_t mm = KK - 1;
    int32_t flag_even_word_arrived;
    void viterbi_decoder(double *page_part_symbols, int32_t *page_part_bits, int32_t _datalength)
    {
        Viterbi(page_part_bits, out0, state0, out1, state1,
            page_part_symbols, KK, nn, _datalength);
    }


    void deinterleaver(int32_t rows, int32_t cols, double *in, double *out)
    {
        for (int32_t r = 0; r < rows; r++)
            {
                for (int32_t c = 0; c < cols; c++)
                    {
                        out[c * rows + r] = in[r * cols + c];
                    }
            }
    }


    bool decode_INAV_word(double *page_part_symbols, int32_t frame_length)
    {
        // 1. De-interleave
        double *page_part_symbols_deint = static_cast<double *>(volk_gnsssdr_malloc(frame_length * sizeof(double), volk_gnsssdr_get_alignment()));
        deinterleaver(GALILEO_INAV_INTERLEAVER_ROWS, GALILEO_INAV_INTERLEAVER_COLS, page_part_symbols, page_part_symbols_deint);

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

        int32_t *page_part_bits = static_cast<int32_t *>(volk_gnsssdr_malloc((frame_length / 2) * sizeof(int32_t), volk_gnsssdr_get_alignment()));

        const int32_t CodeLength = 240;
        int32_t DataLength = (CodeLength / nn) - mm;
        viterbi_decoder(page_part_symbols_deint, page_part_bits, DataLength);
        volk_gnsssdr_free(page_part_symbols_deint);

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
                if (INAV_decoder.flag_CRC_test == true)
                    {
                        std::cout << "Galileo E1 INAV PAGE CRC correct \n";
                        //std::cout << "Galileo E1 CRC correct on channel " << d_channel << " from satellite " << d_satellite << std::endl;
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
        volk_gnsssdr_free(page_part_bits);
        return crc_ok;
    }

    bool decode_FNAV_word(double *page_symbols, int32_t frame_length)
    {
        // 1. De-interleave
        double *page_symbols_deint = static_cast<double *>(volk_gnsssdr_malloc(frame_length * sizeof(double), volk_gnsssdr_get_alignment()));
        deinterleaver(GALILEO_FNAV_INTERLEAVER_ROWS, GALILEO_FNAV_INTERLEAVER_COLS, page_symbols, page_symbols_deint);

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
        int32_t *page_bits = static_cast<int32_t *>(volk_gnsssdr_malloc((frame_length / 2) * sizeof(int32_t), volk_gnsssdr_get_alignment()));

        const int32_t CodeLength = 488;
        int32_t DataLength = (CodeLength / nn) - mm;
        viterbi_decoder(page_symbols_deint, page_bits, DataLength);

        volk_gnsssdr_free(page_symbols_deint);

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
        volk_gnsssdr_free(page_bits);

        // DECODE COMPLETE WORD (even + odd) and TEST CRC
        FNAV_decoder.split_page(page_String);
        if (FNAV_decoder.flag_CRC_test == true)
            {
                std::cout << "Galileo E5a FNAV PAGE CRC correct \n";
                return true;
            }
        else
            {
                return false;
            }
    }

    Galileo_FNAV_INAV_test()
    {
        // vars for Viterbi decoder
        int32_t max_states = 1 << mm;  // 2^mm
        g_encoder[0] = 121;            // Polynomial G1
        g_encoder[1] = 91;             // Polynomial G2
        out0 = static_cast<int32_t *>(volk_gnsssdr_malloc(max_states * sizeof(int32_t), volk_gnsssdr_get_alignment()));
        out1 = static_cast<int32_t *>(volk_gnsssdr_malloc(max_states * sizeof(int32_t), volk_gnsssdr_get_alignment()));
        state0 = static_cast<int32_t *>(volk_gnsssdr_malloc(max_states * sizeof(int32_t), volk_gnsssdr_get_alignment()));
        state1 = static_cast<int32_t *>(volk_gnsssdr_malloc(max_states * sizeof(int32_t), volk_gnsssdr_get_alignment()));
        // create appropriate transition matrices
        nsc_transit(out0, state0, 0, g_encoder, KK, nn);
        nsc_transit(out1, state1, 1, g_encoder, KK, nn);
        flag_even_word_arrived = 0;
    }

    ~Galileo_FNAV_INAV_test()
    {
        volk_gnsssdr_free(out0);
        volk_gnsssdr_free(out1);
        volk_gnsssdr_free(state0);
        volk_gnsssdr_free(state1);
    }
};

TEST_F(Galileo_FNAV_INAV_test, ValidationOfResults)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);

    int repetitions = 10;
    // FNAV FULLY ENCODED FRAME
    double FNAV_frame[488] = {-1, 1, -1, -1, 1, -1, 1, 1, 1, -1, -1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
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
    double INAV_frame_even[240] = {-1, -1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
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

    double INAV_frame_odd[240] = {1, -1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
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


    std::cout << "Galileo FNAV/INAV Test completed in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}
