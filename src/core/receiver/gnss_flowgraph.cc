/*!
 * \file gnss_flowgraph.cc
 * \brief Implementation of a GNSS receiver flowgraph
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2011. luis(at)epsilon-formacion.com
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

#include "gnss_flowgraph.h"
#include <exception>
#include "unistd.h"
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <glog/log_severity.h>
#include <glog/logging.h>
#include <set>
#include "configuration_interface.h"
#include "gnss_block_interface.h"
#include "channel_interface.h"
#include "gnss_block_factory.h"


using google::LogMessage;

GNSSFlowgraph::GNSSFlowgraph(ConfigurationInterface *configuration,
        gr_msg_queue_sptr queue)
{
    connected_ = false;
    running_ = false;
    configuration_ = configuration;
    blocks_ = new std::vector<GNSSBlockInterface*>();
    block_factory_ = new GNSSBlockFactory();
    queue_ = queue;
    available_GPS_satellites_IDs_ = new std::list<Gnss_Satellite>();

    init();
}

GNSSFlowgraph::~GNSSFlowgraph()
{

    delete block_factory_;

    for (unsigned int i = 0; i < blocks_->size(); i++)
        {
            delete blocks_->at(i);
        }
    blocks_->clear();

    delete blocks_;
}

void GNSSFlowgraph::start()
{
    if (running_)
        {
            LOG_AT_LEVEL(WARNING) << "Already running";
            return;
        }

    try
    {
            top_block_->start();
    }
    catch (std::exception& e)
    {
            LOG_AT_LEVEL(ERROR) << "Unable to start flowgraph";
            LOG_AT_LEVEL(ERROR) << e.what();
            return;
    }

    running_ = true;
}

void GNSSFlowgraph::stop()
{
    for (unsigned int i = 0; i < channels_count_; i++)
        {
            channel(i)->stop();
        }
    DLOG(INFO) << "Threads finished. Return to main program.";
    top_block_->stop();
    running_ = false;
}

void GNSSFlowgraph::connect()
{
    /* Connects the blocks in the flowgraph
     *
     * Signal Source > Signal conditioner >> Channels >> Observables >> PVT > Output filter
     */
    DLOG(INFO) << "Connecting flowgraph";
    if (connected_)
        {
            LOG_AT_LEVEL(WARNING) << "flowgraph already connected";
            return;
        }


    try
    {
            signal_source()->connect(top_block_);
    }
    catch (std::exception& e)
    {
            LOG_AT_LEVEL(ERROR) << "Can't connect signal source block internally";
            LOG_AT_LEVEL(ERROR) << e.what();
            top_block_->disconnect_all();
            return;
    }

    // Signal Source > Signal conditioner >
    try
    {
            signal_conditioner()->connect(top_block_);
    }
    catch (std::exception& e)
    {
            LOG_AT_LEVEL(ERROR)
                                << "Can't connect signal conditioner block internally";
            LOG_AT_LEVEL(ERROR) << e.what();
            top_block_->disconnect_all();
            return;
    }


    for (unsigned int i = 0; i < channels_count_; i++)
        {
            try
            {
                    channel(i)->connect(top_block_);
            }
            catch (std::exception& e)
            {
                    LOG_AT_LEVEL(ERROR) << "Can't connect channel " << i
                            << " internally";
                    LOG_AT_LEVEL(ERROR) << e.what();
                    top_block_->disconnect_all();
                    return;
            }
        }


    try
    {
            observables()->connect(top_block_);
    }
    catch (std::exception& e)
    {
            LOG_AT_LEVEL(ERROR) << "Can't connect observables block internally";
            LOG_AT_LEVEL(ERROR) << e.what();
            top_block_->disconnect_all();
            return;
    }

    // Signal Source > Signal conditioner >> Channels >> Observables > PVT
    try
    {
            pvt()->connect(top_block_);
    }
    catch (std::exception& e)
    {
            LOG_AT_LEVEL(ERROR) << "Can't connect PVT block internally";
            LOG_AT_LEVEL(ERROR) << e.what();
            top_block_->disconnect_all();
            return;
    }

    // Signal Source > Signal conditioner >> Channels >> Observables > PVT > Output Filter
    try
    {
            output_filter()->connect(top_block_);
    }
    catch (std::exception& e)
    {
            LOG_AT_LEVEL(ERROR) << "Can't connect output filter block internally";
            LOG_AT_LEVEL(ERROR) << e.what();
            top_block_->disconnect_all();
            return;
    }

    DLOG(INFO) << "blocks connected internally";

    // Signal Source >  Signal conditioner >
    try
    {
            top_block_->connect(signal_source()->get_right_block(), 0,
                    signal_conditioner()->get_left_block(), 0);
    }
    catch (std::exception& e)
    {
            LOG_AT_LEVEL(ERROR)
                                << "Can't connect signal source to signal conditioner";
            LOG_AT_LEVEL(ERROR) << e.what();
            top_block_->disconnect_all();
            return;
    }
    DLOG(INFO) << "Signal source connected to signal conditioner";

    // Signal Source > Signal conditioner >> channels_count_ number of Channels in parallel
    for (unsigned int i = 0; i < channels_count_; i++)
        {
            try
            {
                    top_block_->connect(signal_conditioner()->get_right_block(), 0,
                            channel(i)->get_left_block(), 0);
            }
            catch (std::exception& e)
            {
                    LOG_AT_LEVEL(ERROR)
                                    << "Can't connect signal conditioner to channel " << i;
                    LOG_AT_LEVEL(ERROR) << e.what();
                    top_block_->disconnect_all();
                    return;
            }

            DLOG(INFO) << "signal conditioner connected to channel " << i;

            // Signal Source > Signal conditioner >> Channels >> Observables
            try
            {
                    top_block_->connect(channel(i)->get_right_block(), 0,
                            observables()->get_left_block(), i);
            }
            catch (std::exception& e)
            {
                    LOG_AT_LEVEL(ERROR) << "Can't connect channel " << i
                            << " to observables";
                    LOG_AT_LEVEL(ERROR) << e.what();
                    top_block_->disconnect_all();
                    return;
            }

            channel(i)->set_satellite(available_GPS_satellites_IDs_->front());
            std::cout << "Channel " << i << " satellite "
                    << available_GPS_satellites_IDs_->front() << std::endl;
            available_GPS_satellites_IDs_->pop_front();
            channel(i)->start();
            //channel(i)->start_acquisition();

            DLOG(INFO) << "Channel " << i
                    << " connected to observables and ready for acquisition";
        }
    /*
     * Connect the observables output of each channel to the PVT block
     */
    try
    {
            for (unsigned int i = 0; i < channels_count_; i++)
                {
                    top_block_->connect(observables()->get_right_block(), i,
                            pvt()->get_left_block(), i);
                }
    }
    catch (std::exception& e)
    {
            LOG_AT_LEVEL(ERROR) << "Can't connect observables to PVT";
            LOG_AT_LEVEL(ERROR) << e.what();
            top_block_->disconnect_all();
            return;
    }

    try
    {
            top_block_->connect(pvt()->get_right_block(), 0,
                    output_filter()->get_left_block(), 0);
    }
    catch (std::exception& e)
    {
            LOG_AT_LEVEL(ERROR) << "Can't connect PVT to output filter";
            LOG_AT_LEVEL(ERROR) << e.what();
            top_block_->disconnect_all();
            return;
    }

    DLOG(INFO) << "PVT connected to output filter";

    connected_ = true;
    DLOG(INFO) << "Flowgraph connected";
    top_block_->dump();

}

void GNSSFlowgraph::wait()
{
    if (!running_)
        {
            LOG_AT_LEVEL(WARNING) << "Can't apply wait. Flowgraph is not running";
            return;
        }
    top_block_->wait();
    DLOG(INFO) << "Flowgraph finished calculations";
    running_ = false;
}

/*
 * Applies an action to the flowgraph
 *
 * \param[in] who   Who generated the action
 * \param[in] what  What is the action 0: acquisition failed
 */
void GNSSFlowgraph::apply_action(unsigned int who, unsigned int what)
{
    DLOG(INFO) << "received " << what << " from " << who;

    switch (what)
    {
    case 0:

        LOG_AT_LEVEL(INFO) << "Channel " << who
        << " ACQ FAILED satellite " << channel(who)->get_satellite();
        available_GPS_satellites_IDs_->push_back(
                channel(who)->get_satellite());
        channel(who)->set_satellite(
                available_GPS_satellites_IDs_->front());
        available_GPS_satellites_IDs_->pop_front();
        channel(who)->start_acquisition();
        break;
        // TODO: Tracking messages

    default:
        break;
    }

    DLOG(INFO) << "available channels "
            << available_GPS_satellites_IDs_->size();
}

void GNSSFlowgraph::set_configuration(ConfigurationInterface* configuration)
{
    if (running_)
        {
            LOG_AT_LEVEL(WARNING)
                                << "Unable to update configuration while flowgraph running";
            return;
        }

    if (connected_)
        {
            LOG_AT_LEVEL(WARNING)
                                << "Unable to update configuration while flowgraph connected";
        }

    configuration_ = configuration;
}

GNSSBlockInterface* GNSSFlowgraph::signal_source()
{
    return blocks_->at(0);
}

GNSSBlockInterface* GNSSFlowgraph::signal_conditioner()
{
    return blocks_->at(1);
}

ChannelInterface* GNSSFlowgraph::channel(unsigned int index)
{
    return (ChannelInterface*)blocks_->at(index + 5);
}

GNSSBlockInterface* GNSSFlowgraph::observables()
{
    return blocks_->at(2);
}

GNSSBlockInterface* GNSSFlowgraph::pvt()
{
    return blocks_->at(3);
}

GNSSBlockInterface* GNSSFlowgraph::output_filter()
{
    return blocks_->at(4);
}




void GNSSFlowgraph::init()
{
    /*
     * Instantiates the receiver blocks
     */
    blocks_->push_back(
            block_factory_->GetSignalSource(configuration_, queue_));
    blocks_->push_back(block_factory_->GetSignalConditioner(configuration_,
            queue_));
    blocks_->push_back(block_factory_->GetObservables(configuration_, queue_));
    blocks_->push_back(block_factory_->GetPVT(configuration_, queue_));
    blocks_->push_back(
            block_factory_->GetOutputFilter(configuration_, queue_));

    std::vector<GNSSBlockInterface*>* channels = block_factory_->GetChannels(
            configuration_, queue_);

    channels_count_ = channels->size();

    for (unsigned int i = 0; i < channels_count_; i++)
        {
            blocks_->push_back(channels->at(i));
        }

    top_block_ = gr_make_top_block("GNSSFlowgraph");

    delete channels;

    // fill the available_GPS_satellites_IDs_ queue with the satellites ID's to be searched by the acquisition

    set_satellites_list();

    applied_actions_ = 0;

    DLOG(INFO) << "Blocks instantiated. " << channels_count_
            << " channels.";
}

void GNSSFlowgraph::set_satellites_list()
{

    /*
     * Sets a sequential list of satellites (1...33)
     */

    /*!
     * \TODO Describe GNSS satellites more nicely, with RINEX notation
     * See http://igscb.jpl.nasa.gov/igscb/data/format/rinex301.pdf (page 5)
     */
    std::set<unsigned int> available_gps_prn = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                                            11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
                                            21, 22, 23    , 25, 26, 27, 28, 29, 30,
                                            31, 32};


    std::set<unsigned int>::iterator available_gps_prn_iter;

    for (available_gps_prn_iter = available_gps_prn.begin();
            available_gps_prn_iter != available_gps_prn.end();
            available_gps_prn_iter++)
        {
            sv = Gnss_Satellite(std::string("GPS"), *available_gps_prn_iter);
            available_GPS_satellites_IDs_->push_back(sv);
        }

    std::list<Gnss_Satellite>::iterator it =
            available_GPS_satellites_IDs_->begin();

    for (unsigned int i = 0; i < channels_count_; i++)
        {
            unsigned int sat = configuration_->property("Acquisition" + boost::lexical_cast<std::string>(i) + ".satellite", 0);
            if ((sat == 0) || (sat == it->get_PRN())) // 0 = not PRN in configuration file
                {
                    it++;
                }
            else
                {
                    sv = Gnss_Satellite(std::string("GPS"), it->get_PRN());
                    available_GPS_satellites_IDs_->remove(sv);
                    available_GPS_satellites_IDs_->insert(it, sv);
                }
        }

    //    std::cout << "Satellite queue: ";
    //    for (std::list<unsigned int>::iterator it =
    //            available_GPS_satellites_IDs_->begin(); it
    //            != available_GPS_satellites_IDs_->end(); it++)
    //    {
    //        std::cout << *it << ", ";
    //    }
    //    std::cout << std::endl;

}

void GNSSFlowgraph::apply_action(unsigned int what)
{
    DLOG(INFO) << "Applied action " << what << " to flowgraph";
    applied_actions_++;
}

