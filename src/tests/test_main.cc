/*!
 * \file test_main.cc
 * \brief  This file implements all system tests.
 * \author Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
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




#include <iostream>
#include <queue>
#include <gtest/gtest.h>
#include <glog/log_severity.h>
#include <glog/logging.h>
#include <gnuradio/gr_msg_queue.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>
#include "concurrent_queue.h"
#include "gps_navigation_message.h"
#include "control_thread.h"


#include "control_thread/control_message_factory_test.cc"
//#include "control_thread/control_thread_test.cc"
#include "configuration/file_configuration_test.cc"
//#include "flowgraph/file_output_filter_test.cc"
//#include "flowgraph/file_signal_source_test.cc"
#include "flowgraph/pass_through_test.cc"
//#include "flowgraph/gnss_flowgraph_test.cc"
//#include "gnss_block/file_output_filter_test.cc"
//#include "gnss_block/gnss_block_factory_test.cc"
#include "gnuradio_block/gnss_sdr_valve_test.cc"
#include "gnuradio_block/direct_resampler_conditioner_cc_test.cc"
#include "string_converter/string_converter_test.cc"
#include "arithmetic/complex_arithmetic_libc.cc"
#include "arithmetic/correlations_libc.cc"
#include "arithmetic/cordic_test.cc"
#include "gnss_block/fir_filter_test.cc"
//#include "gnss_block/direct_resampler_conditioner_test.cc"

concurrent_queue<Gps_Navigation_Message> global_gps_nav_msg_queue;

/*
class Control_Message_Factory_Test : public ::testing:: Test
{
protected:
    Control_Message_Factory_Test(){}
};



class File_Configuration_Test : public ::testing:: Test
{
protected:
    File_Configuration_Test(){}
};


class Control_Thread_Test : public ::testing:: Test
{
protected:
    Control_Thread_Test(){}
};
*/





int main(int argc, char **argv)
{
    std::cout << "Running main() from test_main.cc" << std::endl;
    testing::InitGoogleTest(&argc, argv);
    google::InitGoogleLogging(argv[0]);
    return RUN_ALL_TESTS();
}
