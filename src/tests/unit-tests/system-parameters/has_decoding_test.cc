/*!
 * \file has_decoding_test.cc
 * \brief Tests for HAS message decoder
 * \author Carles Fernandez-Prades, 2021. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "galileo_e6_has_msg_receiver.h"
#include "galileo_has_data.h"
#include "galileo_has_page.h"
#include "gnss_sdr_make_unique.h"
#include "has_simple_printer.h"
#include <gflags/gflags.h>
#include <gtest/gtest.h>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>


// Usage:
// ./run_tests --gtest_filter=HAS_Test.Decoder
// ./run_tests --gtest_filter=HAS_Test.Decoder --has_data_test_file=../data/HAS_Messages_sample/encoded/Sample_HAS_Pages_Encoded_20210713_08.txt --start_page_test_file=70
DEFINE_string(has_data_test_file, std::string(""), "File containing encoded HAS pages (format: [time sat_id HAS_page_in_hex] in each line)");
DEFINE_int32(start_page_test_file, 0, "Starting page in case of reading HAS pages from a file");

#if PMT_USES_BOOST_ANY
namespace wht = boost;
#else
namespace wht = std;
#endif

// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class HasDecoderTester;

using HasDecoderTester_sptr = gnss_shared_ptr<HasDecoderTester>;

HasDecoderTester_sptr HasDecoderTester_make();

class HasDecoderTester : public gr::block
{
private:
    friend HasDecoderTester_sptr HasDecoderTester_make();
    HasDecoderTester();

public:
    std::shared_ptr<Galileo_HAS_page> generate_has_page(const std::string& page, int rx_time);
    ~HasDecoderTester();  //!< Default destructor
};


HasDecoderTester_sptr HasDecoderTester_make()
{
    return HasDecoderTester_sptr(new HasDecoderTester());
}


std::shared_ptr<Galileo_HAS_page> HasDecoderTester::generate_has_page(const std::string& page, int rx_time)
{
    auto gh = std::make_shared<Galileo_HAS_page>();

    std::string bits;
    for (size_t i = 0; i < page.size(); i++)
        {
            std::string s = page.substr(i, 1);
            std::stringstream ss;
            ss << std::hex << s;
            unsigned n;
            ss >> n;
            std::bitset<4> b(n);
            bits += b.to_string();
        }

    if (page.substr(0, 6) != "AF3BC3")  // if not dummy
        {
            gh->has_message_string = bits.substr(24, 424);
        }
    gh->time_stamp = rx_time;

    std::bitset<2> b_has_status(bits.substr(0, 2));
    gh->has_status = b_has_status.to_ulong();

    std::bitset<2> b_has_reserved(bits.substr(2, 2));
    gh->reserved = b_has_reserved.to_ulong();

    std::bitset<2> b_message_type(bits.substr(4, 2));
    gh->message_type = b_message_type.to_ulong();

    std::bitset<5> b_message_id(bits.substr(6, 5));
    gh->message_id = b_message_id.to_ulong();

    std::bitset<5> b_message_size(bits.substr(11, 5));
    gh->message_size = b_message_size.to_ulong() + 1;

    std::bitset<8> b_message_page_id(bits.substr(16, 8));
    gh->message_page_id = b_message_page_id.to_ulong();

    return gh;
}


HasDecoderTester::HasDecoderTester() : gr::block("HasDecoderTester", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
}


HasDecoderTester::~HasDecoderTester() = default;


// ###########################################################


class Read_Encoded_Pages
{
public:
    Read_Encoded_Pages() = default;
    std::vector<std::string> get_pages() const
    {
        return page;
    };

    std::vector<std::string> get_sats() const
    {
        return sat;
    };

    std::vector<int> get_time() const
    {
        return rx_time;
    };

    size_t get_number_pages()
    {
        return page.size();
    };

    bool is_known_data()
    {
        return known_test_data;
    };

    inline bool read_data(const std::string& input_filename)
    {
        if (input_filename.empty())
            {
                page = {
                    "07AC6195E44FBA30126D3291D1D973CF3A2C066B77C0816068769E6232772D8086B652DEA8DFC9470B7E12891F27458B84EE1A3D391E20EE",
                    "07AC746F1DA0B7DE403789910C247DCC8CB7AACBA0A395086EEF44091385FF7625FF276A0DF4131195F28794D812A987C53186CE332C2417",
                    "07AC877DCF08B51A8CDB5B1D98790DED6E2365123139CFAD31BC6A84D1D9F0FEB70BC74064DD533D069EEF4D3850B500A7CDAAB1B8F56AA4",
                    "07AC9AAD3485E17627D3697D8537A1DD44BC9923088078182A808E99DE334BEEFF560152B986035325EFDC7D0A43913D926C71C3B12AB757",
                    "07ACAD94E50D544B7130F3840FCEFD4D2AA5A5A66876ECBFFCB0869C5127401B125F3021E70012ECECC4CF6AC1CE65737585095885067A92",
                    "07AC263E0B467BD3AB999E76A9A8D05B18E6B818307C81EEF6DD14722388B72D5651E78F04D1D52E131494D624518F6358B19F508F3C7950",
                    "07AC39CC3470C0F3EEA678A635BA8D7D3FAD16BB0879549DB6835C2DBA92D53A06C919E702C0242F523CCCD5F88FC45AF21DA0519F28E506",
                    "07AC4C87FB8058D90D518A035849C341E421F0DD280A77FC57FE5823B9AD68A6D8EE7D7EC0FD6208DD969F4FAC3E4D7B4214FF808E2EDC7B",
                    "07AC5FD3E1D40A6E958E9C139DF60252A67F6DA37B7E416557058E6D979EC4DEB9802EDE9AD49E47B522A133A05FABFAF1212BF3219AED56",
                    "07AC72FEB3494D685DA492DA72E415A1255BE7DC08AD4EB1617428636CC30EAAB6A9948E03ED69887717D81F2324E48D717847FF29CEE115",
                    "07AC8553C81674CA989EE3B762BFE9F7113F9458A5FD2749D0B685A4F49012532088C872254C881194C7641762A7B9495A02BCD6686CE17F",
                    "07AC987E7D3A9854AA56BCCD7170CB6939966DA4F2199A0C6C5F9CAB5B24539786CCB299DA69DE4EEE9698EEDD2D7BD409565C27674B4268",
                    "07ACAB286D5CA9F01FEC5F5105132F0A41EFCFB5E970C06395B3FE72C3D3B476BADF27DC9CA50ED9EC997AB8BED648DF1424EE56FFAD35B1"};
                rx_time = {690883223, 690883223, 690883223, 690883223, 690883223, 690883224, 690883224, 690883224, 690883224, 690883224, 690883224, 690883224};
                known_test_data = true;
                return true;
            }

        known_test_data = false;
        indata.open(input_filename);

        if (!indata)
            {  // file couldn't be opened
                std::cerr << "Error: file could not be opened\n";
                return false;
            }

        while (!indata.eof())
            {
                indata >> time_instant;
                indata >> sat_id;
                indata >> page_content;
                rx_time.push_back(time_instant);
                sat.push_back(sat_id);
                page.push_back(page_content);
            }
        return true;
    }

private:
    std::vector<int> rx_time;
    std::vector<std::string> sat;
    std::vector<std::string> page;
    std::ifstream indata;
    std::string sat_id;
    std::string page_content;
    int time_instant{};
    bool known_test_data{};
};


TEST(HAS_Test, Decoder)
{
    Read_Encoded_Pages read_pages{};
    EXPECT_TRUE(read_pages.read_data(FLAGS_has_data_test_file));
    auto gal_e6_has_rx_ = galileo_e6_has_msg_receiver_make();
    auto has_tester = HasDecoderTester_make();
    std::unique_ptr<Has_Simple_Printer> has_simple_printer = nullptr;

    auto pages = read_pages.get_pages();
    auto rx_time = read_pages.get_time();
    bool known_data = read_pages.is_known_data();
    int init = 0;
    if (!known_data)
        {
            has_simple_printer = std::make_unique<Has_Simple_Printer>();
            if (static_cast<size_t>(FLAGS_start_page_test_file) < read_pages.get_number_pages())
                {
                    init = FLAGS_start_page_test_file;
                }
            else
                {
                    std::cerr << "The flag --start_page_test_file is set beyond the total number of pages in the file (" << read_pages.get_number_pages() << "), ignoring it.\n";
                }
        }

    for (size_t p = init; p < read_pages.get_number_pages() - 1; p++)
        {
            auto has_page = has_tester->generate_has_page(pages[p], rx_time[p]);
            if (!has_page->has_message_string.empty())  // if not dummy
                {
                    auto has_message = gal_e6_has_rx_->process_test_page(pmt::make_any(has_page));
                    if (has_message != nullptr)
                        {
                            if (known_data)
                                {
                                    EXPECT_EQ(has_message->header.toh, 0);
                                    EXPECT_EQ(has_message->header.mask_flag, true);
                                    EXPECT_EQ(has_message->header.orbit_correction_flag, true);
                                    EXPECT_EQ(has_message->header.clock_fullset_flag, false);
                                    EXPECT_EQ(has_message->header.clock_subset_flag, false);
                                    EXPECT_EQ(has_message->header.code_bias_flag, true);
                                    EXPECT_EQ(has_message->header.phase_bias_flag, true);
                                    EXPECT_EQ(has_message->header.reserved, 0);
                                    EXPECT_EQ(has_message->header.mask_id, 1);
                                    EXPECT_EQ(has_message->header.iod_set_id, 4);

                                    EXPECT_EQ(has_message->Nsys, 2);
                                    EXPECT_EQ(has_message->gnss_id_mask[0], 0);
                                    std::vector<int> prns = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
                                    EXPECT_EQ(has_message->get_PRNs_in_mask(0), prns);
                                    EXPECT_EQ(has_message->get_signals_in_mask(0)[0], "L1 C/A");
                                    EXPECT_EQ(has_message->cell_mask_availability_flag[0], false);
                                    EXPECT_EQ(has_message->nav_message[0], 0);

                                    EXPECT_EQ(has_message->gnss_id_mask[1], 2);
                                    prns = {1, 2, 3, 4, 5, 7, 8, 9, 11, 12, 13, 15, 19, 21, 24, 25, 26, 27, 30, 31, 33, 36};
                                    EXPECT_EQ(has_message->get_PRNs_in_mask(1), prns);
                                    EXPECT_EQ(has_message->get_signals_in_mask(1)[0], "E1-C");
                                    EXPECT_EQ(has_message->get_signals_in_mask(1)[1], "E5a-Q");
                                    EXPECT_EQ(has_message->get_signals_in_mask(1)[2], "E5b-Q");
                                    EXPECT_EQ(has_message->get_signals_in_mask(1)[3], "E6-C");
                                    EXPECT_EQ(has_message->cell_mask_availability_flag[1], false);
                                    EXPECT_EQ(has_message->nav_message[1], 0);

                                    EXPECT_EQ(has_message->validity_interval_index_orbit_corrections, 10);

                                    std::vector<uint16_t> gnss_iod_expected = {17, 92, 1, 169, 13, 15, 67, 10, 84, 108, 34, 22, 203, 75, 91, 10, 60, 42, 35, 92, 48, 74, 81, 21, 19, 6, 0, 4,
                                        86, 22, 78, 78, 74, 78, 76, 78, 78, 78, 78, 75, 78, 76, 74, 78, 78, 78, 78, 78, 78, 78, 78, 78, 77};
                                    std::vector<float> delta_radial_expected = {-0.355, -0.0925, -0.3, -0.8675, 0.1, 0.075, -10.24, -0.0575, 0.0725, 0.22, -0.0825, 0.03, -0.95, -0.1125,
                                        -0.045, -0.1725, -0.975, 0.0725, -0.045, -0.13, -0.335, -0.88, -0.0375, 0.2925, 0.2375, 0.1125, -10.24, 0.13, -0.08, -0.1125, -0.115};
                                    std::vector<float> delta_in_track_expected = {1.552, -0.544, 0.16, -0.272, 0.048, -0.784, -16.384, -0.16, -0.728, -1.528, 0.656, -0.216, -0.552, 0.984,
                                        -0.216, -0.712, 0.184, -0.984, 0.376, 1.136, -0.072, -0.352, 0.488, -0.136, -0.456, 0.04, -16.384, 0.024, -1.616, 1.336, 0.472};
                                    std::vector<float> delta_cross_track_expected = {0.832, 0.056, -0.024, -0.176, 0.04, -0.352, -16.384, -0.44, 0.016, 0.736, 0.104, 0.136, 0.696, 0.08,
                                        -0.192, 0.32, -0.552, -0.304, 0.464, -0.184, 0.576, 0.864, 0.4, -0.896, -1.288, -0.808, -16.384, -0.304, 1.104, 0.576, 0.576};
                                    EXPECT_EQ(has_message->gnss_iod, gnss_iod_expected);
                                    EXPECT_EQ(has_message->get_delta_radial_m(0).size(), delta_radial_expected.size());
                                    EXPECT_EQ(has_message->get_delta_in_track_m(0).size(), delta_in_track_expected.size());
                                    EXPECT_EQ(has_message->get_delta_cross_track_m(0).size(), delta_cross_track_expected.size());
                                    for (size_t i = 0; i < has_message->get_delta_radial_m(0).size(); i++)
                                        {
                                            EXPECT_FLOAT_EQ(has_message->get_delta_radial_m(0)[i], delta_radial_expected[i]);
                                            EXPECT_FLOAT_EQ(has_message->get_delta_in_track_m(0)[i], delta_in_track_expected[i]);
                                            EXPECT_FLOAT_EQ(has_message->get_delta_cross_track_m(0)[i], delta_cross_track_expected[i]);
                                        }

                                    delta_radial_expected = {-0.1325, -0.0025, -0.1325, 0.0675, -0.0525, -0.2675, -0.1475, 0.0475, 0.17, 0.185, -0.195, -0.01, -0.0925, -0.155, 0.0375,
                                        0.2075, 0.08, -0.175, -0.1925, -0.0375, -0.0575, 0.0875};
                                    delta_in_track_expected = {0.056, -0.072, 0.184, 0.064, 0.256, -0.088, 0.024, 0.344, -0.16, -0.056, 0.112, 0.176, 0.448, -0.104, 0.264, -0.072,
                                        -0.248, -0.128, 0.456, 0.352, -0.0960, -0.1360};
                                    delta_cross_track_expected = {0.104, 0.192, 0.312, 0.104, 0.024, 0.392, 0.352, -0.048, -0.12, -0.032, -0.152, -0.328, -0.088, 0.16, 0.464, 0.256,
                                        -0.064, 0.104, -0.128, 0.28, -0.104, -0.288};
                                    for (size_t i = 0; i < has_message->get_delta_radial_m(1).size(); i++)
                                        {
                                            EXPECT_FLOAT_EQ(has_message->get_delta_radial_m(1)[i], delta_radial_expected[i]);
                                            EXPECT_FLOAT_EQ(has_message->get_delta_in_track_m(1)[i], delta_in_track_expected[i]);
                                            EXPECT_FLOAT_EQ(has_message->get_delta_cross_track_m(1)[i], delta_cross_track_expected[i]);
                                        }

                                    std::vector<float> code_bias_gps_expected = {-3.76, 4.38, -2.82, -0.60, 1.28, -3.46, 1.48, -3.04, -2.04, -2.64, 1.82, 1.60, 0.36, 1.08, 2.04, 1.44,
                                        0.42, 4.10, 1.92, 2.28, 4.86, 0.46, -2.94, -3.04, -3.76, -2.08, 2.20, 1.12, -2.58, 1.94, -2.44};
                                    for (size_t i = 0; i < has_message->get_PRNs_in_mask(0).size(); i++)
                                        {
                                            EXPECT_FLOAT_EQ(has_message->get_code_bias_m()[i][0], code_bias_gps_expected[i]);
                                        }
                                    std::vector<float> code_bias_galileo_e1_expected = {-0.08, 0.56, -1.04, 1.04, -1.22, -1.68, 1.3, -0.54, 4.38, 3.04, -0.06, -2.14, 1.16, -0.74, 1.34,
                                        -1.22, -2.02, -0.7, -0.58, -1.6, 1.12, -1.98};
                                    std::vector<float> code_bias_galileo_e5a_expected = {-0.12, 0.8, -1.76, 1.80, -2.04, -2.9, 2.18, -0.94, 7.6, 5.64, -0.2, -3.7, 1.96, -1.28, 2.18,
                                        -2.08, -3.44, -1.20, -1.04, -2.72, 1.92, -3.36};
                                    std::vector<float> code_bias_galileo_e5b_expected = {-0.12, 0.94, -1.76, 1.78, -2.06, -2.88, 2.22, -0.92, 7.46, 5.20, -0.12, -3.64, 1.96, -1.26, 2.28,
                                        -2.06, -3.46, -1.20, -0.98, -2.7, 1.9, -3.38};
                                    std::vector<float> code_bias_galileo_e6c_expected = {-1.04, -0.16, -0.94, 0.96, -0.28, -1.42, -0.58, -0.86, 7.18, 5.72, -1.26, -2.48, 1.4, -2.00, 1.10,
                                        -0.66, -2.72, -2.0, -0.22, -0.58, 0.58, -2.26};

                                    for (size_t i = 0; i < has_message->get_PRNs_in_mask(1).size(); i++)
                                        {
                                            EXPECT_FLOAT_EQ(has_message->get_code_bias_m()[has_message->get_PRNs_in_mask(0).size() + i][0], code_bias_galileo_e1_expected[i]);
                                            EXPECT_FLOAT_EQ(has_message->get_code_bias_m()[has_message->get_PRNs_in_mask(0).size() + i][1], code_bias_galileo_e5a_expected[i]);
                                            EXPECT_FLOAT_EQ(has_message->get_code_bias_m()[has_message->get_PRNs_in_mask(0).size() + i][2], code_bias_galileo_e5b_expected[i]);
                                            EXPECT_FLOAT_EQ(has_message->get_code_bias_m()[has_message->get_PRNs_in_mask(0).size() + i][3], code_bias_galileo_e6c_expected[i]);
                                        }

                                    std::vector<float> phase_bias_gps_expected(has_message->get_PRNs_in_mask(0).size(), -10.24);
                                    std::vector<float> phase_bias_galileo_e1_expected(has_message->get_PRNs_in_mask(1).size(), -10.24);
                                    std::vector<float> phase_bias_galileo_e5a_expected(has_message->get_PRNs_in_mask(1).size(), -10.24);
                                    std::vector<float> phase_bias_galileo_e5b_expected(has_message->get_PRNs_in_mask(1).size(), -10.24);
                                    std::vector<float> phase_bias_galileo_e6c_expected(has_message->get_PRNs_in_mask(1).size(), -10.24);
                                    std::vector<uint8_t> phase_discontinuity_indicator_expected_gps(has_message->get_PRNs_in_mask(0).size(), 0);
                                    std::vector<uint8_t> phase_discontinuity_indicator_expected_gal(has_message->get_PRNs_in_mask(1).size(), 0);
                                    for (size_t i = 0; i < has_message->get_PRNs_in_mask(0).size(); i++)
                                        {
                                            EXPECT_FLOAT_EQ(has_message->get_phase_bias_cycle()[i][0], phase_bias_gps_expected[i]);
                                            EXPECT_EQ(has_message->phase_discontinuity_indicator[i][0], phase_discontinuity_indicator_expected_gps[i]);
                                        }
                                    for (size_t i = 0; i < has_message->get_PRNs_in_mask(1).size(); i++)
                                        {
                                            EXPECT_FLOAT_EQ(has_message->get_phase_bias_cycle()[has_message->get_PRNs_in_mask(0).size() + i][0], phase_bias_galileo_e1_expected[i]);
                                            EXPECT_FLOAT_EQ(has_message->get_phase_bias_cycle()[has_message->get_PRNs_in_mask(0).size() + i][1], phase_bias_galileo_e5a_expected[i]);
                                            EXPECT_FLOAT_EQ(has_message->get_phase_bias_cycle()[has_message->get_PRNs_in_mask(0).size() + i][2], phase_bias_galileo_e5b_expected[i]);
                                            EXPECT_FLOAT_EQ(has_message->get_phase_bias_cycle()[has_message->get_PRNs_in_mask(0).size() + i][3], phase_bias_galileo_e6c_expected[i]);
                                            EXPECT_EQ(has_message->phase_discontinuity_indicator[has_message->get_PRNs_in_mask(0).size() + i][0], phase_discontinuity_indicator_expected_gal[i]);
                                            EXPECT_EQ(has_message->phase_discontinuity_indicator[has_message->get_PRNs_in_mask(0).size() + i][1], phase_discontinuity_indicator_expected_gal[i]);
                                            EXPECT_EQ(has_message->phase_discontinuity_indicator[has_message->get_PRNs_in_mask(0).size() + i][2], phase_discontinuity_indicator_expected_gal[i]);
                                            EXPECT_EQ(has_message->phase_discontinuity_indicator[has_message->get_PRNs_in_mask(0).size() + i][3], phase_discontinuity_indicator_expected_gal[i]);
                                        }
                                    EXPECT_FLOAT_EQ(has_message->get_code_bias_m("L1 C/A", 10), -2.64 * HAS_MSG_CODE_BIAS_SCALE_FACTOR);
                                    EXPECT_FLOAT_EQ(has_message->get_code_bias_m("L1 C/A", 11), 0.0);
                                    EXPECT_FLOAT_EQ(has_message->get_code_bias_m("E1-C", 36), -1.98 * HAS_MSG_CODE_BIAS_SCALE_FACTOR);
                                    EXPECT_FLOAT_EQ(has_message->get_code_bias_m("E1-C", 37), 0.0);
                                }
                            else
                                {
                                    has_simple_printer->print_message(has_message.get());
                                }
                        }
                }
        }
}
