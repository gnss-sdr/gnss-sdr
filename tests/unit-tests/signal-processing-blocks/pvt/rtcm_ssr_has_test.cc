/*!
 * \file rtcm_ssr_has_test.cc
 * \brief Unit tests for the RTCM SSR and IGS SSR messages built from Galileo
 * HAS corrections
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
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

#include "Galileo_CNAV.h"
#include "galileo_has_data.h"
#include "rtcm.h"
#include "rtklib_rtcm.h"
#include "rtklib_rtkcmn.h"
#include <gtest/gtest.h>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// MT1 for GPS PRNs 3, 7 and 12 and Galileo PRNs 5 and 11, where
// - GPS PRN 7 has no orbit correction available,
// - GPS PRN 12 shall not be used (clock correction flag),
// - Galileo PRN 11 has no clock correction available.
// GPS clock corrections use a Delta Clock Multiplier of 2.
class RtcmSsrHasTest : public ::testing::Test
{
protected:
    // Used by value only: C++11 would need out-of-class definitions to bind
    // them to references
    static constexpr int GPS_WEEK = 2400;
    static constexpr uint32_t TOW = 345600U;           // HAS message reference time, and SSR epoch time
    static constexpr uint32_t RECEPTION_DELAY_S = 5U;  // the HAS message is fully received later
    static constexpr uint8_t IOD_SET_ID = 3U;

    static Galileo_HAS_data make_message()
    {
        Galileo_HAS_data has_data;
        has_data.week = GPS_WEEK;
        has_data.tow = TOW + RECEPTION_DELAY_S;
        has_data.has_status = 1;
        has_data.header = mt1_header{};
        has_data.header.toh = static_cast<uint16_t>(TOW % 3600U);
        has_data.header.mask_id = 1;
        has_data.header.iod_set_id = IOD_SET_ID;
        has_data.header.mask_flag = true;
        has_data.header.orbit_correction_flag = true;
        has_data.header.clock_fullset_flag = true;

        has_data.Nsys = 2;
        has_data.gnss_id_mask = {HAS_MSG_GPS_SYSTEM, HAS_MSG_GALILEO_SYSTEM};
        has_data.satellite_mask = {satellite_bit(3) | satellite_bit(7) | satellite_bit(12),
            satellite_bit(5) | satellite_bit(11)};
        has_data.signal_mask = {0, 0};
        has_data.cell_mask_availability_flag = {false, false};
        has_data.nav_message = {0, 0};

        has_data.validity_interval_index_orbit_corrections = 10;
        has_data.validity_interval_index_clock_fullset_corrections = 10;
        has_data.validity_interval_index_clock_subset_corrections = 10;

        //                                 G03   G07    G12  E05  E11
        has_data.gnss_iod = {45, 46, 47, 300, 301};
        has_data.delta_radial = {100, -4096, -200, 40, 80};  // 0.0025 m
        has_data.delta_in_track = {-50, 10, 25, -12, 5};     // 0.008 m
        has_data.delta_cross_track = {20, 3, -7, 30, -2};    // 0.008 m
        has_data.delta_clock_multiplier = {2, 1};
        has_data.delta_clock_correction = {-2048, 400, HAS_MSG_DELTA_CLOCK_DO_NOT_USE_RAW,
            -40, HAS_MSG_DELTA_CLOCK_UNAVAILABLE_RAW};  // 0.0025 m
        return has_data;
    }

    // Clock subset for GPS PRNs 7 and 12, the latter not available
    static Galileo_HAS_data make_clock_subset_message()
    {
        Galileo_HAS_data has_data = make_message();
        has_data.header.orbit_correction_flag = false;
        has_data.header.clock_fullset_flag = false;
        has_data.header.clock_subset_flag = true;
        has_data.Nsys_sub = 1;
        has_data.gnss_id_clock_subset = {HAS_MSG_GPS_SYSTEM};
        has_data.satellite_submask = {0x3};  // MSB first over the 3 GPS satellites of the mask
        has_data.delta_clock_multiplier_clock_subset = {3};
        has_data.delta_clock_correction_clock_subset = {{100, HAS_MSG_DELTA_CLOCK_UNAVAILABLE_RAW}};
        return has_data;
    }

    static uint64_t satellite_bit(int prn)
    {
        return uint64_t{1} << (HAS_MSG_NUMBER_SATELLITE_IDS - prn);  // MSB is PRN 1
    }

    struct Rtklib_Decoder
    {
        Rtklib_Decoder()
        {
            initialized = init_rtcm(&rtcm) == 1;
            rtcm.time = gpst2time(GPS_WEEK, TOW);
        }
        ~Rtklib_Decoder()
        {
            if (initialized)
                {
                    free_rtcm(&rtcm);
                }
        }
        Rtklib_Decoder(const Rtklib_Decoder&) = delete;
        Rtklib_Decoder& operator=(const Rtklib_Decoder&) = delete;

        int input(const std::string& message)
        {
            int status = 0;
            for (const char byte : message)
                {
                    status = input_rtcm3(&rtcm, static_cast<unsigned char>(byte));
                }
            return status;
        }

        const ssr_t& gps(int prn) const { return rtcm.ssr[satno(SYS_GPS, prn) - 1]; }

        rtcm_t rtcm{};
        bool initialized{false};
    };

    // Sequential reader of the fields of an RTCM 3 frame
    class Bit_Reader
    {
    public:
        Bit_Reader(const Rtcm& rtcm, const std::string& message)
            : bits_(rtcm.binary_data_to_bin(message)), rtcm_(rtcm)
        {
        }
        uint32_t uint(size_t length) { return rtcm_.bin_to_uint(next(length)); }
        int32_t sint(size_t length) { return rtcm_.bin_to_int(next(length)); }
        void skip(size_t length) { pos_ += length; }

    private:
        std::string next(size_t length)
        {
            std::string field = bits_.substr(pos_, length);
            pos_ += length;
            return field;
        }
        std::string bits_;
        const Rtcm& rtcm_;
        size_t pos_{24};  // after preamble, reserved bits and message length
    };

    struct Igm_Header
    {
        uint32_t message_number;
        uint32_t subtype;
        uint32_t tow;
        uint32_t multiple_message;
        uint32_t iod_ssr;
        uint32_t num_satellites;
    };

    // IGS SSR v1.00 Table 7, with the Regional/Global indicator (IDF006) in
    // the orbit messages
    static Igm_Header read_igm_header(Bit_Reader& reader, bool has_regional_indicator)
    {
        Igm_Header header{};
        header.message_number = reader.uint(12);   // DF002
        reader.skip(3);                            // IDF001
        header.subtype = reader.uint(8);           // IDF002
        header.tow = reader.uint(20);              // IDF003
        reader.skip(4);                            // IDF004
        header.multiple_message = reader.uint(1);  // IDF005
        header.iod_ssr = reader.uint(4);           // IDF007
        reader.skip(16 + 4);                       // IDF008, IDF009
        if (has_regional_indicator)
            {
                reader.skip(1);  // IDF006
            }
        header.num_satellites = reader.uint(6);  // IDF010
        EXPECT_EQ(static_cast<uint32_t>(TOW), header.tow);
        return header;
    }

    // IDF011 to IDF018. The signs are those of the SSR formats: the opposite
    // of HAS for the orbit
    static void expect_igm_orbit(Bit_Reader& reader, uint32_t prn, uint32_t iod_lsb,
        int32_t radial_01mm, int32_t along_04mm, int32_t cross_04mm)
    {
        EXPECT_EQ(prn, reader.uint(6));           // IDF011
        EXPECT_EQ(iod_lsb, reader.uint(8));       // IDF012
        EXPECT_EQ(radial_01mm, reader.sint(22));  // IDF013
        EXPECT_EQ(along_04mm, reader.sint(20));   // IDF014
        EXPECT_EQ(cross_04mm, reader.sint(20));   // IDF015
        EXPECT_EQ(0, reader.sint(21));            // IDF016
        EXPECT_EQ(0, reader.sint(19));            // IDF017
        EXPECT_EQ(0, reader.sint(19));            // IDF018
    }

    // IDF019 to IDF021
    static void expect_igm_clock(Bit_Reader& reader, int32_t c0_01mm)
    {
        EXPECT_EQ(c0_01mm, reader.sint(22));  // IDF019
        EXPECT_EQ(0, reader.sint(21));        // IDF020
        EXPECT_EQ(0, reader.sint(27));        // IDF021
    }

    std::unique_ptr<Rtcm> rtcm = std::unique_ptr<Rtcm>(new Rtcm());
};


TEST_F(RtcmSsrHasTest, Mt1057InvertsOrbitSignAndSkipsUnavailableOrbits)
{
    const std::string message = rtcm->print_MT1057(make_message());
    ASSERT_FALSE(message.empty());

    Rtklib_Decoder decoder;
    ASSERT_TRUE(decoder.initialized);
    EXPECT_EQ(10, decoder.input(message));

    const ssr_t& g03 = decoder.gps(3);
    EXPECT_EQ(gpst2time(GPS_WEEK, TOW).time, g03.t0[0].time);  // reference time, not reception time
    EXPECT_EQ(45, g03.iode);
    EXPECT_EQ(static_cast<int>(IOD_SET_ID), g03.iod[0]);
    EXPECT_NEAR(-0.25, g03.deph[0], 1e-9);  // HAS: +0.25 m
    EXPECT_NEAR(0.4, g03.deph[1], 1e-9);    // HAS: -0.4 m
    EXPECT_NEAR(-0.16, g03.deph[2], 1e-9);  // HAS: +0.16 m

    const ssr_t& g12 = decoder.gps(12);
    EXPECT_NE(0, g12.t0[0].time);
    EXPECT_EQ(47, g12.iode);
    EXPECT_NEAR(0.5, g12.deph[0], 1e-9);
    EXPECT_NEAR(-0.2, g12.deph[1], 1e-9);
    EXPECT_NEAR(0.056, g12.deph[2], 1e-9);

    EXPECT_EQ(0, decoder.gps(7).t0[0].time);  // orbit not available
}


TEST_F(RtcmSsrHasTest, Mt1058AppliesClockMultiplierAndSkipsUnusableClocks)
{
    const std::string message = rtcm->print_MT1058(make_message());
    ASSERT_FALSE(message.empty());

    Rtklib_Decoder decoder;
    ASSERT_TRUE(decoder.initialized);
    EXPECT_EQ(10, decoder.input(message));

    EXPECT_NEAR(-10.24, decoder.gps(3).dclk[0], 1e-9);  // -5.12 m x 2
    EXPECT_NEAR(2.0, decoder.gps(7).dclk[0], 1e-9);     // 1.0 m x 2
    EXPECT_EQ(gpst2time(GPS_WEEK, TOW).time, decoder.gps(3).t0[1].time);
    EXPECT_NE(0, decoder.gps(7).t0[1].time);
    EXPECT_EQ(0, decoder.gps(12).t0[1].time);  // do not use
}


TEST_F(RtcmSsrHasTest, Mt1058ClockSubset)
{
    const Galileo_HAS_data has_data = make_clock_subset_message();
    EXPECT_TRUE(rtcm->print_MT1058(has_data).empty());  // no full set

    const std::string message = rtcm->print_MT1058(has_data, true);
    ASSERT_FALSE(message.empty());

    Rtklib_Decoder decoder;
    ASSERT_TRUE(decoder.initialized);
    EXPECT_EQ(10, decoder.input(message));

    EXPECT_NEAR(0.75, decoder.gps(7).dclk[0], 1e-9);  // 0.25 m x 3
    EXPECT_NE(0, decoder.gps(7).t0[1].time);
    EXPECT_EQ(0, decoder.gps(3).t0[1].time);   // not in the subset
    EXPECT_EQ(0, decoder.gps(12).t0[1].time);  // not available
}


TEST_F(RtcmSsrHasTest, Mt1060OnlyHasSatellitesWithBothCorrections)
{
    const std::string message = rtcm->print_MT1060(make_message());
    ASSERT_FALSE(message.empty());

    Rtklib_Decoder decoder;
    ASSERT_TRUE(decoder.initialized);
    EXPECT_EQ(10, decoder.input(message));

    const ssr_t& g03 = decoder.gps(3);
    EXPECT_NE(0, g03.t0[0].time);
    EXPECT_EQ(45, g03.iode);
    EXPECT_NEAR(-0.25, g03.deph[0], 1e-9);
    EXPECT_NEAR(0.4, g03.deph[1], 1e-9);
    EXPECT_NEAR(-0.16, g03.deph[2], 1e-9);
    EXPECT_NEAR(-10.24, g03.dclk[0], 1e-9);

    EXPECT_EQ(0, decoder.gps(7).t0[0].time);   // orbit not available
    EXPECT_EQ(0, decoder.gps(12).t0[0].time);  // do not use
}


TEST_F(RtcmSsrHasTest, Igm01FieldOrderAndSatelliteSelection)
{
    const std::vector<std::string> messages = rtcm->print_IGM01(make_message());
    ASSERT_EQ(2U, messages.size());

    Bit_Reader gps(*rtcm, messages[0]);
    const Igm_Header gps_header = read_igm_header(gps, true);
    EXPECT_EQ(4076U, gps_header.message_number);
    EXPECT_EQ(21U, gps_header.subtype);
    EXPECT_EQ(1U, gps_header.multiple_message);
    EXPECT_EQ(static_cast<uint32_t>(IOD_SET_ID), gps_header.iod_ssr);
    ASSERT_EQ(2U, gps_header.num_satellites);
    expect_igm_orbit(gps, 3, 45, -2500, 1000, -400);
    expect_igm_orbit(gps, 12, 47, 5000, -500, 140);

    Bit_Reader galileo(*rtcm, messages[1]);
    const Igm_Header galileo_header = read_igm_header(galileo, true);
    EXPECT_EQ(61U, galileo_header.subtype);
    EXPECT_EQ(0U, galileo_header.multiple_message);
    ASSERT_EQ(2U, galileo_header.num_satellites);
    expect_igm_orbit(galileo, 5, 300 & 0xFF, -1000, 240, -600);
    expect_igm_orbit(galileo, 11, 301 & 0xFF, -2000, -100, 40);
}


TEST_F(RtcmSsrHasTest, Igm02SatelliteSelection)
{
    const std::vector<std::string> messages = rtcm->print_IGM02(make_message());
    ASSERT_EQ(2U, messages.size());

    Bit_Reader gps(*rtcm, messages[0]);
    const Igm_Header gps_header = read_igm_header(gps, false);
    EXPECT_EQ(22U, gps_header.subtype);
    EXPECT_EQ(1U, gps_header.multiple_message);
    ASSERT_EQ(2U, gps_header.num_satellites);
    EXPECT_EQ(3U, gps.uint(6));
    expect_igm_clock(gps, -102400);
    EXPECT_EQ(7U, gps.uint(6));
    expect_igm_clock(gps, 20000);

    Bit_Reader galileo(*rtcm, messages[1]);
    const Igm_Header galileo_header = read_igm_header(galileo, false);
    EXPECT_EQ(62U, galileo_header.subtype);
    EXPECT_EQ(0U, galileo_header.multiple_message);
    ASSERT_EQ(1U, galileo_header.num_satellites);
    EXPECT_EQ(5U, galileo.uint(6));
    expect_igm_clock(galileo, -1000);
}


TEST_F(RtcmSsrHasTest, Igm03OnlyHasSatellitesWithBothCorrections)
{
    const std::vector<std::string> messages = rtcm->print_IGM03(make_message());
    ASSERT_EQ(2U, messages.size());

    Bit_Reader gps(*rtcm, messages[0]);
    const Igm_Header gps_header = read_igm_header(gps, true);
    EXPECT_EQ(23U, gps_header.subtype);
    EXPECT_EQ(1U, gps_header.multiple_message);
    ASSERT_EQ(1U, gps_header.num_satellites);
    expect_igm_orbit(gps, 3, 45, -2500, 1000, -400);
    expect_igm_clock(gps, -102400);

    Bit_Reader galileo(*rtcm, messages[1]);
    const Igm_Header galileo_header = read_igm_header(galileo, true);
    EXPECT_EQ(63U, galileo_header.subtype);
    EXPECT_EQ(0U, galileo_header.multiple_message);
    ASSERT_EQ(1U, galileo_header.num_satellites);
    expect_igm_orbit(galileo, 5, 300 & 0xFF, -1000, 240, -600);
    expect_igm_clock(galileo, -1000);
}


TEST_F(RtcmSsrHasTest, SystemsWithoutValidCorrectionsAreOmitted)
{
    Galileo_HAS_data has_data = make_message();
    has_data.delta_clock_correction[3] = HAS_MSG_DELTA_CLOCK_DO_NOT_USE_RAW;  // E05

    const std::vector<std::string> messages = rtcm->print_IGM03(has_data);
    ASSERT_EQ(1U, messages.size());
    Bit_Reader gps(*rtcm, messages[0]);
    const Igm_Header header = read_igm_header(gps, true);
    EXPECT_EQ(23U, header.subtype);
    EXPECT_EQ(0U, header.multiple_message);  // last message of the sequence
    EXPECT_EQ(2U, rtcm->print_IGM01(has_data).size());
    EXPECT_EQ(1U, rtcm->print_IGM02(has_data).size());
}


TEST_F(RtcmSsrHasTest, EpochTimeIsTheReferenceTimeOfTheCorrections)
{
    // Received one second into an hour, with a TOH in the previous hour
    // (HAS SIS ICD 1.0 Section 7.7)
    Galileo_HAS_data has_data = make_message();
    has_data.tow = TOW + 1U;
    has_data.header.toh = 3598;
    const uint32_t reference_tow = TOW - 2U;

    const std::vector<std::string> messages = rtcm->print_IGM01(has_data);
    ASSERT_FALSE(messages.empty());
    Bit_Reader reader(*rtcm, messages[0]);
    reader.skip(12 + 3 + 8);                    // DF002, IDF001, IDF002
    EXPECT_EQ(reference_tow, reader.uint(20));  // IDF003

    const std::string message = rtcm->print_MT1057(has_data);
    ASSERT_FALSE(message.empty());
    Rtklib_Decoder decoder;
    ASSERT_TRUE(decoder.initialized);
    EXPECT_EQ(10, decoder.input(message));
    EXPECT_EQ(gpst2time(GPS_WEEK, reference_tow).time, decoder.gps(3).t0[0].time);
}


TEST_F(RtcmSsrHasTest, CombinedMessagesNeedACommonUpdateInterval)
{
    Galileo_HAS_data has_data = make_message();
    has_data.validity_interval_index_clock_fullset_corrections = 2;  // 15 s, orbit 300 s

    EXPECT_TRUE(rtcm->print_MT1060(has_data).empty());
    EXPECT_TRUE(rtcm->print_IGM03(has_data).empty());
    EXPECT_FALSE(rtcm->print_MT1057(has_data).empty());
    EXPECT_FALSE(rtcm->print_MT1058(has_data).empty());
    EXPECT_EQ(2U, rtcm->print_IGM01(has_data).size());
    EXPECT_EQ(2U, rtcm->print_IGM02(has_data).size());
}
