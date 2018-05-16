/*!
 * \file file_signal_source_test.cc
 * \brief  This class implements a Unit Test for the class FileSignalSource.
 * \author Carlos Avilés, 2010. carlos.avilesr(at)googlemail.com
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

#include <stdexcept>
#include <gnuradio/msg_queue.h>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/null_sink.h>
#include <gtest/gtest.h>
#include "file_signal_source.h"
#include "in_memory_configuration.h"

TEST(FileSignalSource, Instantiate)
{
    boost::shared_ptr<gr::msg_queue> queue = gr::msg_queue::make(0);
    std::shared_ptr<InMemoryConfiguration> config = std::make_shared<InMemoryConfiguration>();

    config->set_property("Test.samples", "0");
    config->set_property("Test.sampling_frequency", "0");
    std::string path = std::string(TEST_PATH);
    std::string filename = path + "signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat";
    config->set_property("Test.filename", filename);
    config->set_property("Test.item_type", "gr_complex");
    config->set_property("Test.repeat", "false");

    std::unique_ptr<FileSignalSource> signal_source(new FileSignalSource(config.get(), "Test", 1, 1, queue));

    EXPECT_STREQ("gr_complex", signal_source->item_type().c_str());
    EXPECT_TRUE(signal_source->repeat() == false);
}

TEST(FileSignalSource, InstantiateFileNotExists)
{
    boost::shared_ptr<gr::msg_queue> queue = gr::msg_queue::make(0);
    std::shared_ptr<InMemoryConfiguration> config = std::make_shared<InMemoryConfiguration>();

    config->set_property("Test.samples", "0");
    config->set_property("Test.sampling_frequency", "0");
    config->set_property("Test.filename", "./signal_samples/i_dont_exist.dat");
    config->set_property("Test.item_type", "gr_complex");
    config->set_property("Test.repeat", "false");

    EXPECT_THROW({ auto uptr = std::make_shared<FileSignalSource>(config.get(), "Test", 1, 1, queue); }, std::exception);
}
