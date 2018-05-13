/*!
 * \file adapter_test.cc
 * \brief  This file implements tests for the DataTypeAdapter block
 * \author Carles Fernandez-Prades, 2017. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <system_error>
#include <vector>
#include <gnuradio/blocks/file_source.h>
#include <gtest/gtest.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include "byte_to_short.h"
#include "ibyte_to_cbyte.h"
#include "ibyte_to_complex.h"
#include "ibyte_to_cshort.h"
#include "ishort_to_complex.h"
#include "ishort_to_cshort.h"
#include "in_memory_configuration.h"


class DataTypeAdapter : public ::testing::Test
{
public:
    DataTypeAdapter();
    ~DataTypeAdapter();
    int run_byte_to_short_block();
    int run_ibyte_to_cbyte_block();
    int run_ibyte_to_complex_block();
    int run_ibyte_to_cshort_block();
    int run_ishort_to_complex_block();
    int run_ishort_to_cshort_block();
    std::string file_name_input;
    std::string file_name_output;
    std::vector<int8_t> input_data_bytes;
    std::vector<short> input_data_shorts;
};


DataTypeAdapter::DataTypeAdapter()
{
    file_name_input = "adapter_test_input.dat";
    file_name_output = "adapter_test_output.dat";
    int8_t input_bytes[] = {2, 23, -1, 127, -127, 0};
    short input_shorts[] = {2, 23, -1, 127, -127, 0, 255, 255};

    const std::vector<int8_t> input_data_bytes_(input_bytes, input_bytes + sizeof(input_bytes) / sizeof(int8_t));
    input_data_bytes = input_data_bytes_;

    const std::vector<short> input_data_shorts_(input_shorts, input_shorts + sizeof(input_shorts) / sizeof(short));
    input_data_shorts = input_data_shorts_;
}


DataTypeAdapter::~DataTypeAdapter()
{
}


int DataTypeAdapter::run_ishort_to_cshort_block()
{
    std::shared_ptr<ConfigurationInterface> config = std::make_shared<InMemoryConfiguration>();
    config->set_property("Test.implementation", "Ishort_To_Cshort");
    std::shared_ptr<IshortToCshort> ishort_to_cshort = std::make_shared<IshortToCshort>(config.get(), "Test", 1, 1);
    std::string expected_implementation = "Ishort_To_Cshort";
    EXPECT_EQ(expected_implementation, ishort_to_cshort->implementation());

    std::ofstream ofs(file_name_input.c_str(), std::ofstream::binary);
    for (std::vector<short>::const_iterator i = input_data_shorts.cbegin(); i != input_data_shorts.cend(); ++i)
        {
            short aux = *i;
            ofs.write(reinterpret_cast<const char*>(&aux), sizeof(short));
        }
    ofs.close();

    auto top_block = gr::make_top_block("Ishort_To_Cshort test");
    auto file_source = gr::blocks::file_source::make(sizeof(short), file_name_input.c_str());
    auto sink = gr::blocks::file_sink::make(sizeof(lv_16sc_t), file_name_output.c_str(), false);

    EXPECT_NO_THROW({
        top_block->connect(file_source, 0, ishort_to_cshort->get_left_block(), 0);
        top_block->connect(ishort_to_cshort->get_right_block(), 0, sink, 0);
        top_block->run();
    });
    return 0;
}


int DataTypeAdapter::run_ishort_to_complex_block()
{
    std::shared_ptr<ConfigurationInterface> config = std::make_shared<InMemoryConfiguration>();
    config->set_property("Test.implementation", "Ishort_To_Complex");
    std::shared_ptr<IshortToComplex> ishort_to_complex = std::make_shared<IshortToComplex>(config.get(), "Test", 1, 1);
    std::string expected_implementation = "Ishort_To_Complex";
    EXPECT_EQ(expected_implementation, ishort_to_complex->implementation());

    std::ofstream ofs(file_name_input.c_str(), std::ofstream::binary);
    for (std::vector<short>::const_iterator i = input_data_shorts.cbegin(); i != input_data_shorts.cend(); ++i)
        {
            short aux = *i;
            ofs.write(reinterpret_cast<const char*>(&aux), sizeof(short));
        }
    ofs.close();

    auto top_block = gr::make_top_block("Ishort_To_Complex test");
    auto file_source = gr::blocks::file_source::make(sizeof(short), file_name_input.c_str());
    auto sink = gr::blocks::file_sink::make(sizeof(gr_complex), file_name_output.c_str(), false);

    EXPECT_NO_THROW({
        top_block->connect(file_source, 0, ishort_to_complex->get_left_block(), 0);
        top_block->connect(ishort_to_complex->get_right_block(), 0, sink, 0);
        top_block->run();
    });
    return 0;
}


int DataTypeAdapter::run_ibyte_to_cshort_block()
{
    std::shared_ptr<ConfigurationInterface> config = std::make_shared<InMemoryConfiguration>();
    config->set_property("Test.implementation", "Ibyte_To_Cshort");
    std::shared_ptr<IbyteToCshort> ibyte_to_cshort = std::make_shared<IbyteToCshort>(config.get(), "Test", 1, 1);
    std::string expected_implementation = "Ibyte_To_Cshort";
    EXPECT_EQ(expected_implementation, ibyte_to_cshort->implementation());

    std::ofstream ofs(file_name_input.c_str());
    for (std::vector<int8_t>::const_iterator i = input_data_bytes.cbegin(); i != input_data_bytes.cend(); ++i)
        {
            ofs << *i;
        }
    ofs.close();

    auto top_block = gr::make_top_block("Ibyte_To_Cshort test");
    auto file_source = gr::blocks::file_source::make(sizeof(int8_t), file_name_input.c_str());
    auto sink = gr::blocks::file_sink::make(sizeof(lv_16sc_t), file_name_output.c_str(), false);

    EXPECT_NO_THROW({
        top_block->connect(file_source, 0, ibyte_to_cshort->get_left_block(), 0);
        top_block->connect(ibyte_to_cshort->get_right_block(), 0, sink, 0);
        top_block->run();
    });
    return 0;
}


int DataTypeAdapter::run_ibyte_to_complex_block()
{
    std::shared_ptr<ConfigurationInterface> config = std::make_shared<InMemoryConfiguration>();
    config->set_property("Test.implementation", "Ibyte_To_Complex");
    std::shared_ptr<IbyteToComplex> ibyte_to_complex = std::make_shared<IbyteToComplex>(config.get(), "Test", 1, 1);
    std::string expected_implementation = "Ibyte_To_Complex";
    EXPECT_EQ(expected_implementation, ibyte_to_complex->implementation());

    std::ofstream ofs(file_name_input.c_str());
    for (std::vector<int8_t>::const_iterator i = input_data_bytes.cbegin(); i != input_data_bytes.cend(); ++i)
        {
            ofs << *i;
        }
    ofs.close();

    auto top_block = gr::make_top_block("Ibyte_To_Complex test");
    auto file_source = gr::blocks::file_source::make(sizeof(int8_t), file_name_input.c_str());
    auto sink = gr::blocks::file_sink::make(sizeof(gr_complex), file_name_output.c_str(), false);

    EXPECT_NO_THROW({
        top_block->connect(file_source, 0, ibyte_to_complex->get_left_block(), 0);
        top_block->connect(ibyte_to_complex->get_right_block(), 0, sink, 0);
        top_block->run();
    });
    return 0;
}


int DataTypeAdapter::run_ibyte_to_cbyte_block()
{
    std::shared_ptr<ConfigurationInterface> config = std::make_shared<InMemoryConfiguration>();
    config->set_property("Test.implementation", "Ibyte_To_Cbyte");
    std::shared_ptr<IbyteToCbyte> ibyte_to_cbyte = std::make_shared<IbyteToCbyte>(config.get(), "Test", 1, 1);
    std::string expected_implementation = "Ibyte_To_Cbyte";
    EXPECT_EQ(expected_implementation, ibyte_to_cbyte->implementation());

    std::ofstream ofs(file_name_input.c_str());
    for (std::vector<int8_t>::const_iterator i = input_data_bytes.cbegin(); i != input_data_bytes.cend(); ++i)
        {
            ofs << *i;
        }
    ofs.close();

    auto top_block = gr::make_top_block("Ibyte_To_Cbyte test");
    auto file_source = gr::blocks::file_source::make(sizeof(int8_t), file_name_input.c_str());
    auto sink = gr::blocks::file_sink::make(sizeof(short), file_name_output.c_str(), false);

    EXPECT_NO_THROW({
        top_block->connect(file_source, 0, ibyte_to_cbyte->get_left_block(), 0);
        top_block->connect(ibyte_to_cbyte->get_right_block(), 0, sink, 0);
        top_block->run();
    });
    return 0;
}


int DataTypeAdapter::run_byte_to_short_block()
{
    std::shared_ptr<ConfigurationInterface> config = std::make_shared<InMemoryConfiguration>();
    config->set_property("Test.implementation", "Byte_To_Short");
    std::shared_ptr<ByteToShort> byte_to_short = std::make_shared<ByteToShort>(config.get(), "Test", 1, 1);
    std::string expected_implementation = "Byte_To_Short";
    EXPECT_EQ(expected_implementation, byte_to_short->implementation());

    std::ofstream ofs(file_name_input.c_str());
    for (std::vector<int8_t>::const_iterator i = input_data_bytes.cbegin(); i != input_data_bytes.cend(); ++i)
        {
            ofs << *i;
        }
    ofs.close();

    auto top_block = gr::make_top_block("Byte_To_Short test");
    auto file_source = gr::blocks::file_source::make(sizeof(int8_t), file_name_input.c_str());
    auto sink = gr::blocks::file_sink::make(sizeof(int16_t), file_name_output.c_str(), false);

    EXPECT_NO_THROW({
        top_block->connect(file_source, 0, byte_to_short->get_left_block(), 0);
        top_block->connect(byte_to_short->get_right_block(), 0, sink, 0);
        top_block->run();
    });
    return 0;
}


TEST_F(DataTypeAdapter, ByteToShortValidationOfResults)
{
    run_byte_to_short_block();
    std::ifstream ifs(file_name_output.data(), std::ifstream::binary | std::ifstream::in);

    int16_t iSample;
    int i = 0;
    try
        {
            while (ifs.read(reinterpret_cast<char*>(&iSample), sizeof(int16_t)))
                {
                    EXPECT_EQ(input_data_bytes.at(i), static_cast<int8_t>(iSample / 256));  // Scale down!
                    i++;
                }
        }
    catch (std::system_error& e)
        {
            std::cerr << e.code().message() << std::endl;
        }
    ifs.close();
    ASSERT_EQ(remove(file_name_input.c_str()), 0) << "Problem deleting temporary file";
    ASSERT_EQ(remove(file_name_output.c_str()), 0) << "Problem deleting temporary file";
}


TEST_F(DataTypeAdapter, IbyteToCbyteValidationOfResults)
{
    run_ibyte_to_cbyte_block();
    std::ifstream ifs(file_name_output.data(), std::ifstream::binary | std::ifstream::in);
    lv_8sc_t iSample;
    int i = 0;
    try
        {
            while (ifs.read(reinterpret_cast<char*>(&iSample), sizeof(lv_8sc_t)))
                {
                    EXPECT_EQ(input_data_bytes.at(i), iSample.real());
                    i++;
                    EXPECT_EQ(input_data_bytes.at(i), iSample.imag());
                    i++;
                }
        }
    catch (std::system_error& e)
        {
            std::cerr << e.code().message() << std::endl;
        }
    ifs.close();
    ASSERT_EQ(remove(file_name_input.c_str()), 0) << "Problem deleting temporary file";
    ASSERT_EQ(remove(file_name_output.c_str()), 0) << "Problem deleting temporary file";
}


TEST_F(DataTypeAdapter, IbyteToComplexValidationOfResults)
{
    run_ibyte_to_cbyte_block();
    std::ifstream ifs(file_name_output.data(), std::ifstream::binary | std::ifstream::in);
    gr_complex iSample;
    int i = 0;
    try
        {
            while (ifs.read(reinterpret_cast<char*>(&iSample), sizeof(gr_complex)))
                {
                    EXPECT_EQ(input_data_bytes.at(i), static_cast<int8_t>(iSample.real()));
                    i++;
                    EXPECT_EQ(input_data_bytes.at(i), static_cast<int8_t>(iSample.imag()));
                    i++;
                }
        }
    catch (std::system_error& e)
        {
            std::cerr << e.code().message() << std::endl;
        }
    ifs.close();
    ASSERT_EQ(remove(file_name_input.c_str()), 0) << "Problem deleting temporary file";
    ASSERT_EQ(remove(file_name_output.c_str()), 0) << "Problem deleting temporary file";
}


TEST_F(DataTypeAdapter, IbyteToCshortValidationOfResults)
{
    run_ibyte_to_cshort_block();
    std::ifstream ifs(file_name_output.data(), std::ifstream::binary | std::ifstream::in);
    lv_16sc_t iSample;
    int i = 0;
    try
        {
            while (ifs.read(reinterpret_cast<char*>(&iSample), sizeof(lv_16sc_t)))
                {
                    EXPECT_EQ(input_data_bytes.at(i), static_cast<int8_t>(iSample.real()));
                    i++;
                    EXPECT_EQ(input_data_bytes.at(i), static_cast<int8_t>(iSample.imag()));
                    i++;
                }
        }
    catch (std::system_error& e)
        {
            std::cerr << e.code().message() << std::endl;
        }
    ifs.close();
    ASSERT_EQ(remove(file_name_input.c_str()), 0) << "Problem deleting temporary file";
    ASSERT_EQ(remove(file_name_output.c_str()), 0) << "Problem deleting temporary file";
}


TEST_F(DataTypeAdapter, IshortToComplexValidationOfResults)
{
    run_ishort_to_complex_block();
    std::ifstream ifs(file_name_output.data(), std::ifstream::binary | std::ifstream::in);
    gr_complex iSample;
    int i = 0;
    try
        {
            while (ifs.read(reinterpret_cast<char*>(&iSample), sizeof(gr_complex)))
                {
                    EXPECT_EQ(input_data_shorts.at(i), static_cast<short>(iSample.real()));
                    i++;
                    EXPECT_EQ(input_data_shorts.at(i), static_cast<short>(iSample.imag()));
                    i++;
                }
        }
    catch (std::system_error& e)
        {
            std::cerr << e.code().message() << std::endl;
        }
    ifs.close();
    ASSERT_EQ(remove(file_name_input.c_str()), 0) << "Problem deleting temporary file";
    ASSERT_EQ(remove(file_name_output.c_str()), 0) << "Problem deleting temporary file";
}


TEST_F(DataTypeAdapter, IshortToCshortValidationOfResults)
{
    run_ishort_to_cshort_block();
    std::ifstream ifs(file_name_output.data(), std::ifstream::binary | std::ifstream::in);
    lv_16sc_t iSample;
    int i = 0;
    try
        {
            while (ifs.read(reinterpret_cast<char*>(&iSample), sizeof(lv_16sc_t)))
                {
                    EXPECT_EQ(input_data_shorts.at(i), static_cast<short>(iSample.real()));
                    i++;
                    EXPECT_EQ(input_data_shorts.at(i), static_cast<short>(iSample.imag()));
                    i++;
                }
        }
    catch (std::system_error& e)
        {
            std::cerr << e.code().message() << std::endl;
        }
    ifs.close();
    ASSERT_EQ(remove(file_name_input.c_str()), 0) << "Problem deleting temporary file";
    ASSERT_EQ(remove(file_name_output.c_str()), 0) << "Problem deleting temporary file";
}
