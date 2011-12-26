/*!
 * \file control_thread.cc
 * \brief Brief description of the file here
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
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

#include "control_thread.h"

#include <unistd.h>

#include <gnuradio/gr_message.h>

#include <gflags/gflags.h>

#include <glog/log_severity.h>
#include <glog/logging.h>

#include "gnss_flowgraph.h"
#include "file_configuration.h"
#include "control_message_factory.h"

using google::LogMessage;

DEFINE_string(config_file, "../conf/gnss-sdr.conf",
        "Path to the file containing the configuration parameters");

ControlThread::ControlThread()
{
    configuration_ = new FileConfiguration(FLAGS_config_file);
    delete_configuration_ = true;
    init();
}

ControlThread::ControlThread(ConfigurationInterface *configuration)
{
    configuration_ = configuration;
    delete_configuration_ = false;
    init();
}

ControlThread::~ControlThread()
{
    delete flowgraph_;
    if (delete_configuration_) delete configuration_;
    delete control_message_factory_;
}

/*! Main loop to read and process the control messages
 *
 *  1- Connect the GNSS receiver flowgraph
 *  2- Start the GNSS receiver flowgraph
 *  while (flowgraph_->running() && !stop)_{
 *  3- Read control messages and process them }
 */
void ControlThread::run()
{

    flowgraph_->connect();
    if (flowgraph_->connected())
    {
        LOG_AT_LEVEL(INFO) << "Flowgraph connected";
    }
    else
    {
        LOG_AT_LEVEL(ERROR) << "Unable to connect flowgraph";
        return;
    }

    flowgraph_->start();
    if (flowgraph_->running())
    {
        LOG_AT_LEVEL(INFO) << "Flowgraph started";
    }
    else
    {
        LOG_AT_LEVEL(ERROR) << "Unable to start flowgraph";
        return;
    }

    // Main loop to read and process the control messages
    while (flowgraph_->running() && !stop_)
    {
        //TODO re-enable the blocking read messages functions and fork the process
        read_control_messages();
        if (control_messages_ != 0) process_control_messages();
    }

    flowgraph_->stop();

    LOG_AT_LEVEL(INFO) << "Flowgraph stopped";
}

void ControlThread::set_control_queue(gr_msg_queue_sptr control_queue)
{
    if (flowgraph_->running())
    {
        LOG_AT_LEVEL(WARNING)
                << "Unable to set control queue while flowgraph is running";
        return;
    }

    control_queue_ = control_queue;
}

/*! Creates a control_queue_, a flowgraph_ and a control_message_factory,
 *  sets stop_ to false and initializes processed_control_messages_ and
 *  applied_actions_ to zero
 */
void ControlThread::init()
{
    control_queue_ = gr_make_msg_queue(0);
    flowgraph_ = new GNSSFlowgraph(configuration_, control_queue_);
    control_message_factory_ = new ControlMessageFactory();
    stop_ = false;
    processed_control_messages_ = 0;
    applied_actions_ = 0;
}

void ControlThread::read_control_messages()
{

    DLOG(INFO) << "Reading control messages from queue";
    gr_message_sptr queue_message = control_queue_->delete_head();
    if (queue_message != 0)
    {
        control_messages_ = control_message_factory_->GetControlMessages(
                queue_message);
    }
    else
    {
        control_messages_ = 0;
    }
}

// Apply the corresponding control actions
// TODO:  May be it is better to move the apply_action state machine to the control_thread
void ControlThread::process_control_messages()
{

    for (unsigned int i = 0; i < control_messages_->size(); i++)
    {
        if (stop_) break;
        if (control_messages_->at(i)->who == 200)
        {
            apply_action(control_messages_->at(i)->what);
        }
        else
        {
            flowgraph_->apply_action(control_messages_->at(i)->who,
                    control_messages_->at(i)->what);
        }

        delete control_messages_->at(i);
        processed_control_messages_++;
    }

    control_messages_->clear();
    delete control_messages_;

    DLOG(INFO) << "Processed all control messages";
}

void ControlThread::apply_action(unsigned int what)
{

    switch (what)
    {
        case 0:
            DLOG(INFO) << "Received action STOP";
            stop_ = true;
            applied_actions_++;
            break;
        default:
            DLOG(INFO) << "Unrecognized action.";
    }

}

