/*!
* \file osnma_data.cc
* \brief Class for Galileo OSNMA data storage
* \author Carles Fernandez-Prades, 2020-2023 cfernandez(at)cttc.es
*
* -----------------------------------------------------------------------------
*
* GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
* This file is part of GNSS-SDR.
*
* Copyright (C) 2010-2023  (see AUTHORS file for a list of contributors)
* SPDX-License-Identifier: GPL-3.0-or-later
*
* -----------------------------------------------------------------------------
*/

#include "osnma_data.h"
#include <cstring>
#include <iostream>

/**
 * @brief Constructs a NavData object with the given osnma_msg.
 * \details Packs the ephemeris, iono and utc data from the current subframe into the NavData structure. It also gets the PRNa and the GST.
 * @param osnma_msg The shared pointer to the OSNMA_msg object.
 */
void NavData::init(const std::shared_ptr<OSNMA_msg> &osnma_msg)
{
    EphemerisData = osnma_msg->EphemerisData;
    IonoData = osnma_msg->IonoData;
    UtcData = osnma_msg->UtcModelData;
    generate_eph_iono_vector();
    generate_utc_vector();
    PRNa = osnma_msg->PRN;
    WN_sf0 = osnma_msg->WN_sf0;
    TOW_sf0 = osnma_msg->TOW_sf0;
};
void NavData::generate_eph_iono_vector()
{
    ephemeris_iono_vector.clear();
    uint64_t bit_buffer = 0; // variable to store the bits to be extracted, it can contain bits from different variables
    int bit_count = 0; // Number of bits in the buffer, i.e. to be extracted

    // create structure to hold the variables to store into the vector along with their bit size
    std::vector<std::pair<void*, int>> variables = {
        // data from word type 1
        {static_cast<void*>(&EphemerisData.IOD_nav), sizeof(EphemerisData.IOD_nav) * 8},
        {static_cast<void*>(&EphemerisData.toe), sizeof(EphemerisData.toe) * 8},
        {static_cast<void*>(&EphemerisData.M_0), sizeof(EphemerisData.M_0) * 8},
        {static_cast<void*>(&EphemerisData.ecc), sizeof(EphemerisData.ecc) * 8},
        {static_cast<void*>(&EphemerisData.sqrtA), sizeof(EphemerisData.sqrtA) * 8},
        // data from word type 2
        {static_cast<void*>(&EphemerisData.IOD_nav), sizeof(EphemerisData.IOD_nav) * 8},
        {static_cast<void*>(&EphemerisData.OMEGA_0), sizeof(EphemerisData.OMEGA_0) * 8},
        {static_cast<void*>(&EphemerisData.i_0), sizeof(EphemerisData.i_0) * 8},
        {static_cast<void*>(&EphemerisData.omega), sizeof(EphemerisData.omega) * 8},
        {static_cast<void*>(&EphemerisData.idot), sizeof(EphemerisData.idot) * 8},
        {static_cast<void*>(&EphemerisData.IOD_nav), sizeof(EphemerisData.IOD_nav) * 8},
        // data from word type 3
        {static_cast<void*>(&EphemerisData.OMEGAdot), sizeof(EphemerisData.OMEGAdot) * 8},
        {static_cast<void*>(&EphemerisData.delta_n), sizeof(EphemerisData.delta_n) * 8},
        {static_cast<void*>(&EphemerisData.Cuc), sizeof(EphemerisData.Cuc) * 8},
        {static_cast<void*>(&EphemerisData.Cus), sizeof(EphemerisData.Cus) * 8},
        {static_cast<void*>(&EphemerisData.Crc), sizeof(EphemerisData.Crc) * 8},
        {static_cast<void*>(&EphemerisData.Crs), sizeof(EphemerisData.Crs) * 8},
        {static_cast<void*>(&EphemerisData.SISA), sizeof(EphemerisData.SISA) * 8},
        // data from word type 4
        {static_cast<void*>(&EphemerisData.IOD_nav), sizeof(EphemerisData.IOD_nav) * 8},
        {static_cast<void*>(&EphemerisData.PRN), sizeof(EphemerisData.PRN) * 8},
        {static_cast<void*>(&EphemerisData.Cic), sizeof(EphemerisData.Cic) * 8},
        {static_cast<void*>(&EphemerisData.Cis), sizeof(EphemerisData.Cis) * 8},
        {static_cast<void*>(&EphemerisData.toe), sizeof(EphemerisData.toe) * 8},
        {static_cast<void*>(&EphemerisData.af0), sizeof(EphemerisData.af0) * 8},
        {static_cast<void*>(&EphemerisData.af1), sizeof(EphemerisData.af1) * 8},
        {static_cast<void*>(&EphemerisData.af2), sizeof(EphemerisData.af2) * 8},
        // data from word type 5
        {static_cast<void*>(&IonoData.ai0), sizeof(IonoData.ai0) * 8},
        {static_cast<void*>(&IonoData.ai1), sizeof(IonoData.ai1) * 8},
        {static_cast<void*>(&IonoData.ai2), sizeof(IonoData.ai2) * 8},
        {static_cast<void*>(&IonoData.Region1_flag), sizeof(IonoData.Region1_flag) * 8},
        {static_cast<void*>(&IonoData.Region2_flag), sizeof(IonoData.Region2_flag) * 8},
        {static_cast<void*>(&IonoData.Region3_flag), sizeof(IonoData.Region3_flag) * 8},
        {static_cast<void*>(&IonoData.Region4_flag), sizeof(IonoData.Region4_flag) * 8},
        {static_cast<void*>(&IonoData.Region5_flag), sizeof(IonoData.Region5_flag) * 8},
        {static_cast<void*>(&EphemerisData.BGD_E1E5a), sizeof(EphemerisData.BGD_E1E5a) * 8},
        {static_cast<void*>(&EphemerisData.BGD_E1E5b), sizeof(EphemerisData.BGD_E1E5b) * 8},
        {static_cast<void*>(&EphemerisData.E5b_HS), sizeof(EphemerisData.E5b_HS) * 8},
        {static_cast<void*>(&EphemerisData.E1B_HS), sizeof(EphemerisData.E1B_HS) * 8},
        {static_cast<void*>(&EphemerisData.E5b_DVS), sizeof(EphemerisData.E5b_DVS) * 8},
        {static_cast<void*>(&EphemerisData.E1B_DVS), sizeof(EphemerisData.E1B_DVS) * 8},
    };

    for (auto& var : variables)
    {
        // extract the bits from the variable
        uint64_t binary_representation;
        memcpy(&binary_representation, var.first, var.second / 8);

        // Append the bits to the buffer and update the bit count
        bit_buffer = (bit_buffer << var.second) | binary_representation;
        bit_count += var.second;
        // While there are 8 or more bits in the buffer
        while (bit_count >= 8)
        {
            // Extract the 8 bits starting from last bit position and add them to the vector
            uint8_t extracted_bits = (bit_buffer >> (bit_count - 8)) & 0xFF;
            ephemeris_iono_vector.push_back(extracted_bits);

            // Remove the extracted bits from the buffer
            bit_count -= 8;
            bit_buffer = bit_buffer & ~(0xFF << bit_count);
        }

    }

    // If there are any bits left in the buffer, add them to the vector
    if (bit_count > 0)
    {
        ephemeris_iono_vector.push_back(static_cast<uint8_t>(bit_buffer));
    }
}

void NavData::generate_utc_vector()
{
    utc_vector.clear();
    uint64_t bit_buffer = 0;
    int bit_count = 0;

    std::vector<std::pair<void*, int>> variables = {
        {static_cast<void*>(&UtcData.A0), sizeof(UtcData.A0) * 8},
        {static_cast<void*>(&UtcData.A1), sizeof(UtcData.A1) * 8},
        {static_cast<void*>(&UtcData.Delta_tLS), sizeof(UtcData.Delta_tLS) * 8},
        {static_cast<void*>(&UtcData.tot), sizeof(UtcData.tot) * 8},
        {static_cast<void*>(&UtcData.WNot), sizeof(UtcData.WNot) * 8},
        {static_cast<void*>(&UtcData.WN_LSF), sizeof(UtcData.WN_LSF) * 8},
        {static_cast<void*>(&UtcData.DN), sizeof(UtcData.DN) * 8},
        {static_cast<void*>(&UtcData.Delta_tLSF), sizeof(UtcData.Delta_tLSF) * 8},
        {static_cast<void*>(&UtcData.A_0G), sizeof(UtcData.A_0G) * 8},
        {static_cast<void*>(&UtcData.A_1G), sizeof(UtcData.A_1G) * 8},
        {static_cast<void*>(&UtcData.t_0G), sizeof(UtcData.t_0G) * 8},
        {static_cast<void*>(&UtcData.WN_0G), sizeof(UtcData.WN_0G) * 8},
    };

    for (auto& var : variables)
        {
            uint64_t binary_representation;
            memcpy(&binary_representation, var.first, var.second / 8);

            bit_buffer = (bit_buffer << var.second) | binary_representation;
            bit_count += var.second;

            while (bit_count >= 8)
                {
                    uint8_t extracted_bits = (bit_buffer >> (bit_count - 8)) & 0xFF;
                    utc_vector.push_back(extracted_bits);

                    bit_count -= 8;
                    bit_buffer = bit_buffer & ~(0xFF << bit_count);
                }
        }

    if (bit_count > 0)
        {
            utc_vector.push_back(static_cast<uint8_t>(bit_buffer));
        }
}

