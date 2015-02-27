/*!
 * \file gnss_flowgraph.cc
 * \brief Implementation of a GNSS receiver flowgraph
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *         Carles Fernandez-Prades, 2014. cfernandez(at)cttc.es
 *
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "gnss_flowgraph.h"
#include "unistd.h"
#include <exception>
#include <iostream>
#include <set>
#include <boost/lexical_cast.hpp>
#include <glog/logging.h>
#include "configuration_interface.h"
#include "gnss_block_interface.h"
#include "channel_interface.h"
#include "gnss_block_factory.h"

#define GNSS_SDR_ARRAY_SIGNAL_CONDITIONER_CHANNELS 8

using google::LogMessage;

GNSSFlowgraph::GNSSFlowgraph(std::shared_ptr<ConfigurationInterface> configuration,
        boost::shared_ptr<gr::msg_queue> queue)
{
    connected_ = false;
    running_ = false;
    configuration_ = configuration;
    //std::shared_ptr<std::vector<std::shared_ptr<GNSSBlockInterface>>> blocks_ = std::make_shared<std::vector<std::shared_ptr<GNSSBlockInterface>>>();
    queue_ = queue;
    init();
}


GNSSFlowgraph::~GNSSFlowgraph()
{
    //blocks_->clear();
}

void GNSSFlowgraph::start()
{
    if (running_)
        {
            LOG(WARNING) << "Already running";
            return;
        }

    try
    {
            top_block_->start();
    }
    catch (std::exception& e)
    {
            LOG(WARNING) << "Unable to start flowgraph";
            LOG(ERROR) << e.what();
            return;
    }

    running_ = true;
}

void GNSSFlowgraph::stop()
{
    for (unsigned int i = 0; i < channels_count_; i++)
        {
            channels_.at(i)->stop();
            LOG(INFO) << "Channel " << i << " in state " << channels_state_[i] << std::endl;
        }
    LOG(INFO) << "Threads finished. Return to main program.";
    top_block_->stop();
    running_ = false;
}


void GNSSFlowgraph::connect()
{
    /* Connects the blocks in the flowgraph
     *
     * Signal Source > Signal conditioner >> Channels >> Observables >> PVT > Output filter
     */
    LOG(INFO) << "Connecting flowgraph";
    if (connected_)
        {
            LOG(WARNING) << "flowgraph already connected";
            return;
        }

		for (int i = 0; i < sources_count_; i++)
		{
			try
			{
				sig_source_.at(i)->connect(top_block_);
			}
			catch (std::exception& e)
			{
				LOG(INFO) << "Can't connect signal source block " << i << " internally";
				LOG(ERROR) << e.what();
				top_block_->disconnect_all();
				return;
			}
		}

    // Signal Source > Signal conditioner >

		for (int i = 0; i < sources_count_; i++)
		{
			try
			{
				sig_conditioner_.at(i)->connect(top_block_);
			}
			catch (std::exception& e)
			{
				LOG(INFO) << "Can't connect signal conditioner block " << i << " internally";
				LOG(ERROR) << e.what();
				top_block_->disconnect_all();
				return;
			}
		}

    for (unsigned int i = 0; i < channels_count_; i++)
        {
            try
            {
                    //auto chan_ = std::move(blocks_->at(i));
                    //std::shared_ptr<ChannelInterface> chan = std::dynamic_pointer_cast<ChannelInterface>(chan_);
                    //channels_.push_back(chan);
                    channels_.at(i)->connect(top_block_);
            }
            catch (std::exception& e)
            {
                    LOG(WARNING) << "Can't connect channel " << i << " internally";
                    LOG(ERROR) << e.what();
                    top_block_->disconnect_all();
                    return;
            }
        }

    try
    {
            //observables_ = std::move(blocks_->at(2));
            observables_->connect(top_block_);
    }
    catch (std::exception& e)
    {
            LOG(WARNING) << "Can't connect observables block internally";
            LOG(ERROR) << e.what();
            top_block_->disconnect_all();
            return;
    }

    // Signal Source > Signal conditioner >> Channels >> Observables > PVT
    try
    {
            //pvt_ = std::move(blocks_->at(3));
            pvt_->connect(top_block_);
    }
    catch (std::exception& e)
    {
            LOG(WARNING) << "Can't connect PVT block internally";
            LOG(ERROR) << e.what();
            top_block_->disconnect_all();
            return;
    }

    // Signal Source > Signal conditioner >> Channels >> Observables > PVT > Output Filter
    try
    {
            //output_filter_ = std::move(blocks_->at(4));
            output_filter_->connect(top_block_);
    }
    catch (std::exception& e)
    {
            LOG(WARNING) << "Can't connect output filter block internally";
            LOG(ERROR) << e.what();
            top_block_->disconnect_all();
            return;
    }

    DLOG(INFO) << "blocks connected internally";

    // Signal Source (i) >  Signal conditioner (i) >

	for (int i = 0; i < sources_count_; i++)
	{

			try
			{
					//TODO: Remove this array implementation and create generic multistream connector
				    //(if a signal source has more than 1 stream, then connect it to the multistream signal conditioner)
					if(sig_source_.at(i)->implementation().compare("Raw_Array_Signal_Source") == 0)
						{
							//Multichannel Array
							std::cout << "ARRAY MODE" << std::endl;
							for (int j = 0; j < GNSS_SDR_ARRAY_SIGNAL_CONDITIONER_CHANNELS; j++)
								{
									std::cout << "connecting ch "<< j << std::endl;
									top_block_->connect(sig_source_.at(i)->get_right_block(), j, sig_conditioner_.at(i)->get_left_block(), j);
								}
						}
					else
						{
							//single channel
							top_block_->connect(sig_source_.at(i)->get_right_block(), 0, sig_conditioner_.at(i)->get_left_block(), 0);
						}

			}
			catch (std::exception& e)
			{
					LOG(WARNING) << "Can't connect signal source " << i << " to signal conditioner " << i;
					LOG(ERROR) << e.what();
					top_block_->disconnect_all();
					return;
			}
    }
    DLOG(INFO) << "Signal source connected to signal conditioner";

    // Signal conditioner (selected_signal_source) >> channels (i) (dependent of their associated SignalSource_ID)
    int selected_signal_source;
    for (unsigned int i = 0; i < channels_count_; i++)
        {

            selected_signal_source = configuration_->property("Channel" + boost::lexical_cast<std::string>(i) +".SignalSource_ID", 0);
			try
			{
			top_block_->connect(sig_conditioner_.at(selected_signal_source)->get_right_block(), 0,
					channels_.at(i)->get_left_block(), 0);
            }
            catch (std::exception& e)
            {
                    LOG(WARNING) << "Can't connect signal conditioner "<<selected_signal_source<<" to channel " << i;
                    LOG(ERROR) << e.what();
                    top_block_->disconnect_all();
                    return;
            }

            DLOG(INFO) << "signal conditioner "<<selected_signal_source<<" connected to channel " << i;

            // Signal Source > Signal conditioner >> Channels >> Observables
            try
            {
                    top_block_->connect(channels_.at(i)->get_right_block(), 0,
                            observables_->get_left_block(), i);
            }
            catch (std::exception& e)
            {
                    LOG(WARNING) << "Can't connect channel " << i << " to observables";
                    LOG(ERROR) << e.what();
                    top_block_->disconnect_all();
                    return;
            }

            //discriminate between systems
            //TODO: add a specific string member to the channel template, and not re-use the implementation field!
            while (channels_.at(i)->implementation()!= available_GNSS_signals_.front().get_satellite().get_system())
                {
                    available_GNSS_signals_.push_back(available_GNSS_signals_.front());
                    available_GNSS_signals_.pop_front();
                }
            channels_.at(i)->set_signal(available_GNSS_signals_.front());
            LOG(INFO) << "Channel " << i << " assigned to " << available_GNSS_signals_.front();
            available_GNSS_signals_.pop_front();
            channels_.at(i)->start();

            if (channels_state_[i] == 1)
                {
                    channels_.at(i)->start_acquisition();
                    LOG(INFO) << "Channel " << i
                              << " connected to observables and ready for acquisition";
                }
            else
                {
                    LOG(INFO) << "Channel " << i
                              << " connected to observables in standby mode";
                }

        }
    /*
     * Connect the observables output of each channel to the PVT block
     */
    try
    {
            for (unsigned int i = 0; i < channels_count_; i++)
                {
                    top_block_->connect(observables_->get_right_block(), i, pvt_->get_left_block(), i);
                }
    }
    catch (std::exception& e)
    {
            LOG(WARNING) << "Can't connect observables to PVT";
            LOG(ERROR) << e.what();
            top_block_->disconnect_all();
            return;
    }

    try
    {
            top_block_->connect(pvt_->get_right_block(), 0, output_filter_->get_left_block(), 0);
    }
    catch (std::exception& e)
    {
            LOG(WARNING) << "Can't connect PVT to output filter";
            LOG(ERROR) << e.what();
            top_block_->disconnect_all();
            return;
    }

    DLOG(INFO) << "PVT connected to output filter";
    connected_ = true;
    LOG(INFO) << "Flowgraph connected";
    top_block_->dump();
}




void GNSSFlowgraph::wait()
{
    if (!running_)
        {
            LOG(WARNING) << "Can't apply wait. Flowgraph is not running";
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
    LOG(INFO) << "received " << what << " from " << who;

    switch (what)
    {
    case 0:
        LOG(INFO) << "Channel " << who << " ACQ FAILED satellite " << channels_.at(who)->get_signal().get_satellite();
        available_GNSS_signals_.push_back(channels_.at(who)->get_signal());

        while (channels_.at(who)->get_signal().get_satellite().get_system() != available_GNSS_signals_.front().get_satellite().get_system())
            {
                available_GNSS_signals_.push_back(available_GNSS_signals_.front());
                available_GNSS_signals_.pop_front();
            }
        channels_.at(who)->set_signal(available_GNSS_signals_.front());
        available_GNSS_signals_.pop_front();
        channels_.at(who)->start_acquisition();

        break;
        // TODO: Tracking messages

    case 1:
        LOG(INFO) << "Channel " << who << " ACQ SUCCESS satellite " << channels_.at(who)->get_signal().get_satellite();
        channels_state_[who] = 2;
        acq_channels_count_--;
        if (acq_channels_count_ < max_acq_channels_)
            {
                for (unsigned int i = 0; i < channels_count_; i++)
                    {
                        if (channels_state_[i] == 0)
                            {
                                channels_state_[i] = 1;
                                acq_channels_count_++;
                                channels_.at(i)->start_acquisition();
                                break;
                            }
                    }
            }

        for (unsigned int i = 0; i < channels_count_; i++)
            {
                LOG(INFO) << "Channel " << i << " in state " << channels_state_[i] << std::endl;
            }
        break;

    case 2:
        LOG(INFO) << "Channel " << who << " TRK FAILED satellite " << channels_.at(who)->get_signal().get_satellite();
        if (acq_channels_count_ < max_acq_channels_)
            {
                channels_state_[who] = 1;
                acq_channels_count_++;
                channels_.at(who)->start_acquisition();
            }
        else
            {
                channels_state_[who] = 0;
                channels_.at(who)->standby();
            }

        for (unsigned int i = 0; i < channels_count_; i++)
            {
                LOG(INFO) << "Channel " << i << " in state " << channels_state_[i] << std::endl;
            }
        break;

    default:
        break;
    }
    LOG(INFO) << "Number of available satellites: " << available_GNSS_signals_.size();
}



void GNSSFlowgraph::set_configuration(std::shared_ptr<ConfigurationInterface> configuration)
{
    if (running_)
        {
            LOG(WARNING) << "Unable to update configuration while flowgraph running";
            return;
        }

    if (connected_)
        {
            LOG(WARNING) << "Unable to update configuration while flowgraph connected";
        }
    configuration_ = configuration;
}



void GNSSFlowgraph::init()
{
    /*
     * Instantiates the receiver blocks
     */
    std::shared_ptr<GNSSBlockFactory> block_factory_ = std::make_shared<GNSSBlockFactory>();

    // 1. read the number of RF front-ends available (one file_source per RF front-end)
    sources_count_ = configuration_->property("Receiver.sources_count", 1);

    if (sources_count_>1)
    {
		for (int i = 0; i < sources_count_; i++)
			{
			std::cout<<"creating source "<<i<<std::endl;
				sig_source_.push_back(block_factory_->GetSignalSource(configuration_, queue_,i));
				sig_conditioner_.push_back(block_factory_->GetSignalConditioner(configuration_, queue_, i));
			}
    }else{
    	//backwards compatibility for old config files
		sig_source_.push_back(block_factory_->GetSignalSource(configuration_, queue_,-1));
		sig_conditioner_.push_back(block_factory_->GetSignalConditioner(configuration_, queue_, -1));
	}

    observables_ = block_factory_->GetObservables(configuration_, queue_);
    pvt_ = block_factory_->GetPVT(configuration_, queue_);
    output_filter_ = block_factory_->GetOutputFilter(configuration_, queue_);

    std::shared_ptr<std::vector<std::unique_ptr<GNSSBlockInterface>>> channels = block_factory_->GetChannels(configuration_, queue_);

    channels_count_ = channels->size();
    for (unsigned int i = 0; i < channels_count_; i++)
        {
        	std::shared_ptr<GNSSBlockInterface> chan_ = std::move(channels->at(i));
            channels_.push_back(std::dynamic_pointer_cast<ChannelInterface>(chan_));
        }

    top_block_ = gr::make_top_block("GNSSFlowgraph");

    // fill the available_GNSS_signals_ queue with the satellites ID's to be searched by the acquisition
    set_signals_list();
    set_channels_state();
    applied_actions_ = 0;

    DLOG(INFO) << "Blocks instantiated. " << channels_count_ << " channels.";
}

void GNSSFlowgraph::set_signals_list()
{
    /*
     * Sets a sequential list of GNSS satellites
     */
    std::set<unsigned int>::iterator available_gnss_prn_iter;

    /*
     * \TODO Describe GNSS satellites more nicely, with RINEX notation
     * See http://igscb.jpl.nasa.gov/igscb/data/format/rinex301.pdf (page 5)
     */

    /*
     * Read GNSS-SDR default GNSS system and signal
     */
    std::string default_system = configuration_->property("Channel.system", std::string("GPS"));
    std::string default_signal = configuration_->property("Channel.signal", std::string("1C"));

    /*
     * Loop to create the list of GNSS Signals
     * To add signals from other systems, add another loop 'for'
     */
    if (default_system.find(std::string("GPS")) != std::string::npos )
        {
            /*
             * Loop to create GPS L1 C/A signals
             */
            std::set<unsigned int> available_gps_prn = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                    11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 25, 26, 27, 28,
                    29, 30, 31, 32 };

            for (available_gnss_prn_iter = available_gps_prn.begin();
                    available_gnss_prn_iter != available_gps_prn.end();
                    available_gnss_prn_iter++)
                {
                    available_GNSS_signals_.push_back(Gnss_Signal(Gnss_Satellite(std::string("GPS"),
                            *available_gnss_prn_iter), std::string("1C")));
                }
        }


    if (default_system.find(std::string("SBAS")) != std::string::npos)
        {
            /*
             * Loop to create SBAS L1 C/A signals
             */
            std::set<unsigned int> available_sbas_prn = {120, 124, 126};

            for (available_gnss_prn_iter = available_sbas_prn.begin();
                    available_gnss_prn_iter != available_sbas_prn.end();
                    available_gnss_prn_iter++)
                {
                    available_GNSS_signals_.push_back(Gnss_Signal(Gnss_Satellite(std::string("SBAS"),
                            *available_gnss_prn_iter), std::string("1C")));
                }
        }


    if (default_system.find(std::string("Galileo")) != std::string::npos)
        {
            /*
             * Loop to create the list of Galileo E1 B signals
             */
            std::set<unsigned int> available_galileo_prn = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                    11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 25, 26, 27, 28,
                    29, 30, 31, 32, 33, 34, 35, 36};

            for (available_gnss_prn_iter = available_galileo_prn.begin();
                    available_gnss_prn_iter != available_galileo_prn.end();
                    available_gnss_prn_iter++)
                {
//                    available_GNSS_signals_.push_back(Gnss_Signal(Gnss_Satellite(std::string("Galileo"),
//                            *available_gnss_prn_iter), std::string("1B")));
                    available_GNSS_signals_.push_back(Gnss_Signal(Gnss_Satellite(std::string("Galileo"),
                            *available_gnss_prn_iter), default_signal));
                }
        }

    /*
     * Ordering the list of signals from configuration file
     */

    std::list<Gnss_Signal>::iterator gnss_it = available_GNSS_signals_.begin();

    for (unsigned int i = 0; i < channels_count_; i++)
        {
            std::string gnss_system = (configuration_->property("Channel"
                    + boost::lexical_cast<std::string>(i) + ".system",
                    default_system));
            LOG(INFO) << "Channel " << i << " system " << gnss_system;

            std::string gnss_signal = (configuration_->property("Channel"
                    + boost::lexical_cast<std::string>(i) + ".signal",
                    default_signal));
            LOG(INFO) << "Channel " << i << " signal " << gnss_signal;

            unsigned int sat = configuration_->property("Channel"
                    + boost::lexical_cast<std::string>(i) + ".satellite", 0);

            if ((sat == 0) || (sat == gnss_it->get_satellite().get_PRN())) // 0 = not PRN in configuration file
                {
                    gnss_it++;
                }
            else
                {
                    Gnss_Signal signal_value = Gnss_Signal(Gnss_Satellite(gnss_system, sat), gnss_signal);
                    DLOG(INFO) << "Channel " << i << " " << signal_value;
                    available_GNSS_signals_.remove(signal_value);
                    available_GNSS_signals_.insert(gnss_it, signal_value);
                }
        }


//    **** FOR DEBUGGING THE LIST OF GNSS SIGNALS ****

//    std::cout<<"default_system="<<default_system<<std::endl;
//    std::cout<<"default_signal="<<default_signal<<std::endl;
//        std::list<Gnss_Signal>::iterator available_gnss_list_iter;
//        for (available_gnss_list_iter = available_GNSS_signals_.begin(); available_gnss_list_iter
//        != available_GNSS_signals_.end(); available_gnss_list_iter++)
//        {
//          std::cout << *available_gnss_list_iter << std::endl;
//        }

}


void GNSSFlowgraph::set_channels_state()
{
    max_acq_channels_ = (configuration_->property("Channels.in_acquisition", channels_count_));
    if (max_acq_channels_ > channels_count_)
        {
            max_acq_channels_ = channels_count_;
            LOG(WARNING) << "Channels_in_acquisition is bigger than number of channels. Variable acq_channels_count_ is set to "
                         << channels_count_;
        }
    channels_state_.reserve(channels_count_);
    for (unsigned int i = 0; i < channels_count_; i++)
        {
            if (i < max_acq_channels_)
                {
                    channels_state_.push_back(1);
                }
            else
                channels_state_.push_back(0);
        }
    acq_channels_count_ = max_acq_channels_;
    DLOG(INFO) << acq_channels_count_ << " channels in acquisition state";
    for (unsigned int i = 0; i < channels_count_; i++)
        {
            LOG(INFO) << "Channel " << i << " in state " << channels_state_[i];
        }
}
