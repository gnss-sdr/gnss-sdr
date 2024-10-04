/*!
 * \file file_signal_source_test.cc
 * \brief  This class implements a Unit Test for the class FileSignalSource.
 * \author Carlos Avilés, 2010. carlos.avilesr(at)googlemail.com
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "concurrent_queue.h"
#include "file_signal_source.h"
#include "gnss_sdr_make_unique.h"
#include "in_memory_configuration.h"
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <stdexcept>
#include <utility>

TEST(FileSignalSource, Instantiate)
{
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    auto config = std::make_shared<InMemoryConfiguration>();

    config->set_property("Test.samples", "0");
    config->set_property("Test.sampling_frequency", "0");
    std::string path = std::string(TEST_PATH);
    std::string filename = path + "signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat";
    config->set_property("Test.filename", std::move(filename));
    config->set_property("Test.item_type", "gr_complex");
    config->set_property("Test.repeat", "false");

    auto signal_source = std::make_unique<FileSignalSource>(config.get(), "Test", 0, 1, queue.get());

    EXPECT_STREQ("gr_complex", signal_source->item_type().c_str());
    EXPECT_TRUE(signal_source->repeat() == false);
}

TEST(FileSignalSource, InstantiateFileNotExists)
{
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    auto config = std::make_shared<InMemoryConfiguration>();

    config->set_property("Test.samples", "0");
    config->set_property("Test.sampling_frequency", "0");
    config->set_property("Test.filename", "./signal_samples/i_dont_exist.dat");
    config->set_property("Test.item_type", "gr_complex");
    config->set_property("Test.repeat", "false");

    // the file existence test was moved from the ctor to the connect() call. The argument to
    // connect doesn't matter, since the exception should be thrown sooner than any connections
    auto top = gr::make_top_block("GNSSUnitTest");
    auto uptr = std::make_shared<FileSignalSource>(config.get(), "Test", 0, 1, queue.get());
    EXPECT_THROW({ uptr->connect(std::move(top)); }, std::exception);
}
