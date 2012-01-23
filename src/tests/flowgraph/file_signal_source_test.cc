/*!
 * \file file_signal_source_test.cc
 * \brief  This file implements tests for the FileSignalSourceClass
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include <gtest/gtest.h>
#include <gr_block.h>
#include <gr_msg_queue.h>
#include <gr_top_block.h>
#include <gr_null_sink.h>
#include <stdexcept>
#include "file_signal_source.h"
#include "in_memory_configuration.h"



TEST(File_Signal_Source_Test, Instantiate)
{
	gr_msg_queue_sptr queue = gr_make_msg_queue(0);

	InMemoryConfiguration* config = new InMemoryConfiguration();

	config->set_property("Test.samples", "0");
	config->set_property("Test.sampling_frequency", "0");

	        config->set_property("Test.filename", "/Users/carlesfernandez/Documents/workspace/gnss-sdr/trunk/data/sc2_d16.dat");

	config->set_property("Test.item_type", "gr_complex");
	config->set_property("Test.repeat", "false");

	try
	{
	    FileSignalSource *signal_source = new FileSignalSource(config, "Test", 1, 1, queue);
	}
	catch(std::exception& e){}
	EXPECT_STREQ("/Users/carlesfernandez/Documents/workspace/gnss-sdr/trunk/data/sc2_d16.dat", signal_source->filename().c_str());
	EXPECT_STREQ("gr_complex", signal_source->item_type().c_str());
	EXPECT_TRUE(signal_source->repeat() == false);

	delete signal_source;
}



TEST(File_Signal_Source_Test, InstantiateFileNotExists)
{
	gr_msg_queue_sptr queue = gr_make_msg_queue(0);

	InMemoryConfiguration* config = new InMemoryConfiguration();

	config->set_property("Test.samples", "0");
	config->set_property("Test.sampling_frequency", "0");
	config->set_property("Test.filename", "./signal_samples/i_dont_exist.dat");

	config->set_property("Test.item_type", "gr_complex");
	config->set_property("Test.repeat", "false");

	EXPECT_THROW(new FileSignalSource(config, "Test", 1, 1, queue), std::exception);
}
