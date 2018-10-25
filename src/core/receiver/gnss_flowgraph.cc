/*!
 * \file gnss_flowgraph.cc
 * \brief Implementation of a GNSS receiver flow graph
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *         Carles Fernandez-Prades, 2014. cfernandez(at)cttc.es
 *         Álvaro Cebrián Juan, 2018. acebrianjuan(at)gmail.com
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

#include "gnss_flowgraph.h"
#include "gnss_synchro.h"
#include "configuration_interface.h"
#include "gnss_block_interface.h"
#include "channel_interface.h"
#include "gnss_block_factory.h"
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <glog/logging.h>
#include <algorithm>
#include <exception>
#include <iostream>
#include <set>


#define GNSS_SDR_ARRAY_SIGNAL_CONDITIONER_CHANNELS 8

using google::LogMessage;

GNSSFlowgraph::GNSSFlowgraph(std::shared_ptr<ConfigurationInterface> configuration, gr::msg_queue::sptr queue)
{
    connected_ = false;
    running_ = false;
    configuration_ = configuration;
    queue_ = queue;
    init();
}


GNSSFlowgraph::~GNSSFlowgraph()
{
    if (connected_)
        {
            GNSSFlowgraph::disconnect();
        }
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
    catch (const std::exception& e)
        {
            LOG(WARNING) << "Unable to start flowgraph";
            LOG(ERROR) << e.what();
            return;
        }

    running_ = true;
}


void GNSSFlowgraph::stop()
{
    top_block_->stop();
    running_ = false;
}


void GNSSFlowgraph::connect()
{
    // Connects the blocks in the flow graph
    // Signal Source > Signal conditioner >> Channels >> Observables >> PVT

    LOG(INFO) << "Connecting flowgraph";
    if (connected_)
        {
            LOG(WARNING) << "flowgraph already connected";
            return;
        }

    for (int i = 0; i < sources_count_; i++)
        {
            if (configuration_->property(sig_source_.at(i)->role() + ".enable_FPGA", false) == false)
                {
                    try
                        {
                            sig_source_.at(i)->connect(top_block_);
                        }
                    catch (const std::exception& e)
                        {
                            LOG(INFO) << "Can't connect signal source block " << i << " internally";
                            LOG(ERROR) << e.what();
                            top_block_->disconnect_all();
                            return;
                        }
                }
        }

    // Signal Source > Signal conditioner >
    for (unsigned int i = 0; i < sig_conditioner_.size(); i++)
        {
            if (configuration_->property(sig_conditioner_.at(i)->role() + ".enable_FPGA", false) == false)
                {
                    try
                        {
                            sig_conditioner_.at(i)->connect(top_block_);
                        }
                    catch (const std::exception& e)
                        {
                            LOG(INFO) << "Can't connect signal conditioner block " << i << " internally";
                            LOG(ERROR) << e.what();
                            top_block_->disconnect_all();
                            return;
                        }
                }
        }

    for (unsigned int i = 0; i < channels_count_; i++)
        {
            try
                {
                    channels_.at(i)->connect(top_block_);
                }
            catch (const std::exception& e)
                {
                    LOG(WARNING) << "Can't connect channel " << i << " internally";
                    LOG(ERROR) << e.what();
                    top_block_->disconnect_all();
                    return;
                }
        }

    try
        {
            observables_->connect(top_block_);
        }
    catch (const std::exception& e)
        {
            LOG(WARNING) << "Can't connect observables block internally";
            LOG(ERROR) << e.what();
            top_block_->disconnect_all();
            return;
        }

    // Signal Source > Signal conditioner >> Channels >> Observables > PVT
    try
        {
            pvt_->connect(top_block_);
        }
    catch (const std::exception& e)
        {
            LOG(WARNING) << "Can't connect PVT block internally";
            LOG(ERROR) << e.what();
            top_block_->disconnect_all();
            return;
        }

    DLOG(INFO) << "blocks connected internally";
    // Signal Source (i) >  Signal conditioner (i) >
    int RF_Channels = 0;
    int signal_conditioner_ID = 0;

    for (int i = 0; i < sources_count_; i++)
        {
            //FPGA Accelerators do not need signal sources or conditioners
            //as the samples are feed directly to the FPGA fabric, so, if enabled, do not connect any source
            if (configuration_->property(sig_source_.at(i)->role() + ".enable_FPGA", false) == false)
                {
                    try
                        {
                            //TODO: Remove this array implementation and create generic multistream connector
                            //(if a signal source has more than 1 stream, then connect it to the multistream signal conditioner)
                            if (sig_source_.at(i)->implementation().compare("Raw_Array_Signal_Source") == 0)
                                {
                                    //Multichannel Array
                                    std::cout << "ARRAY MODE" << std::endl;
                                    for (int j = 0; j < GNSS_SDR_ARRAY_SIGNAL_CONDITIONER_CHANNELS; j++)
                                        {
                                            std::cout << "connecting ch " << j << std::endl;
                                            top_block_->connect(sig_source_.at(i)->get_right_block(), j, sig_conditioner_.at(i)->get_left_block(), j);
                                        }
                                }
                            else
                                {
                                    //TODO: Create a class interface for SignalSources, derived from GNSSBlockInterface.
                                    //Include GetRFChannels in the interface to avoid read config parameters here
                                    //read the number of RF channels for each front-end
                                    RF_Channels = configuration_->property(sig_source_.at(i)->role() + ".RF_channels", 1);

                                    for (int j = 0; j < RF_Channels; j++)
                                        {
                                            //Connect the multichannel signal source to multiple signal conditioners
                                            // GNURADIO max_streams=-1 means infinite ports!
                                            LOG(INFO) << "sig_source_.at(i)->get_right_block()->output_signature()->max_streams()=" << sig_source_.at(i)->get_right_block()->output_signature()->max_streams();
                                            LOG(INFO) << "sig_conditioner_.at(signal_conditioner_ID)->get_left_block()->input_signature()=" << sig_conditioner_.at(signal_conditioner_ID)->get_left_block()->input_signature()->max_streams();

                                            if (sig_source_.at(i)->get_right_block()->output_signature()->max_streams() > 1)
                                                {
                                                    LOG(INFO) << "connecting sig_source_ " << i << " stream " << j << " to conditioner " << j;
                                                    top_block_->connect(sig_source_.at(i)->get_right_block(), j, sig_conditioner_.at(signal_conditioner_ID)->get_left_block(), 0);
                                                }
                                            else
                                                {
                                                    if (j == 0)
                                                        {
                                                            // RF_channel 0 backward compatibility with single channel sources
                                                            LOG(INFO) << "connecting sig_source_ " << i << " stream " << 0 << " to conditioner " << j;
                                                            top_block_->connect(sig_source_.at(i)->get_right_block(), 0, sig_conditioner_.at(signal_conditioner_ID)->get_left_block(), 0);
                                                        }
                                                    else
                                                        {
                                                            // Multiple channel sources using multiple output blocks of single channel (requires RF_channel selector in call)
                                                            LOG(INFO) << "connecting sig_source_ " << i << " stream " << j << " to conditioner " << j;
                                                            top_block_->connect(sig_source_.at(i)->get_right_block(j), 0, sig_conditioner_.at(signal_conditioner_ID)->get_left_block(), 0);
                                                        }
                                                }
                                            signal_conditioner_ID++;
                                        }
                                }
                        }
                    catch (const std::exception& e)
                        {
                            LOG(WARNING) << "Can't connect signal source " << i << " to signal conditioner " << i;
                            LOG(ERROR) << e.what();
                            top_block_->disconnect_all();
                            return;
                        }
                }
        }
    DLOG(INFO) << "Signal source connected to signal conditioner";
    bool FPGA_enabled = configuration_->property(sig_source_.at(0)->role() + ".enable_FPGA", false);

#if ENABLE_FPGA
    if (FPGA_enabled == false)
        {
            //connect the signal source to sample counter
            //connect the sample counter to Observables
            try
                {
                    double fs = static_cast<double>(configuration_->property("GNSS-SDR.internal_fs_sps", 0));
                    if (fs == 0.0)
                        {
                            LOG(WARNING) << "Set GNSS-SDR.internal_fs_sps in configuration file";
                            std::cout << "Set GNSS-SDR.internal_fs_sps in configuration file" << std::endl;
                            throw(std::invalid_argument("Set GNSS-SDR.internal_fs_sps in configuration"));
                        }
                    int observable_interval_ms = static_cast<double>(configuration_->property("GNSS-SDR.observable_interval_ms", 20));
                    ch_out_sample_counter = gnss_sdr_make_sample_counter(fs, observable_interval_ms, sig_conditioner_.at(0)->get_right_block()->output_signature()->sizeof_stream_item(0));
                    top_block_->connect(sig_conditioner_.at(0)->get_right_block(), 0, ch_out_sample_counter, 0);
                    top_block_->connect(ch_out_sample_counter, 0, observables_->get_left_block(), channels_count_);  //extra port for the sample counter pulse
                }
            catch (const std::exception& e)
                {
                    LOG(WARNING) << "Can't connect sample counter";
                    LOG(ERROR) << e.what();
                    top_block_->disconnect_all();
                    return;
                }
        }
    else
        {
            //create a hardware-defined gnss_synchro pulse for the observables block
            try
                {
                    double fs = static_cast<double>(configuration_->property("GNSS-SDR.internal_fs_sps", 0));
                    if (fs == 0.0)
                        {
                            LOG(WARNING) << "Set GNSS-SDR.internal_fs_sps in configuration file";
                            std::cout << "Set GNSS-SDR.internal_fs_sps in configuration file" << std::endl;
                            throw(std::invalid_argument("Set GNSS-SDR.internal_fs_sps in configuration"));
                        }
                    int observable_interval_ms = static_cast<double>(configuration_->property("GNSS-SDR.observable_interval_ms", 20));
                    ch_out_fpga_sample_counter = gnss_sdr_make_fpga_sample_counter(fs, observable_interval_ms);
                    top_block_->connect(ch_out_fpga_sample_counter, 0, observables_->get_left_block(), channels_count_);  //extra port for the sample counter pulse
                }
            catch (const std::exception& e)
                {
                    LOG(WARNING) << "Can't connect FPGA sample counter";
                    LOG(ERROR) << e.what();
                    top_block_->disconnect_all();
                    return;
                }
        }
#else
    // connect the signal source to sample counter
    // connect the sample counter to Observables
    try
        {
            double fs = static_cast<double>(configuration_->property("GNSS-SDR.internal_fs_sps", 0));
            if (fs == 0.0)
                {
                    LOG(WARNING) << "Set GNSS-SDR.internal_fs_sps in configuration file";
                    std::cout << "Set GNSS-SDR.internal_fs_sps in configuration file" << std::endl;
                    throw(std::invalid_argument("Set GNSS-SDR.internal_fs_sps in configuration"));
                }

            int observable_interval_ms = static_cast<double>(configuration_->property("GNSS-SDR.observable_interval_ms", 20));
            ch_out_sample_counter = gnss_sdr_make_sample_counter(fs, observable_interval_ms, sig_conditioner_.at(0)->get_right_block()->output_signature()->sizeof_stream_item(0));
            top_block_->connect(sig_conditioner_.at(0)->get_right_block(), 0, ch_out_sample_counter, 0);
            top_block_->connect(ch_out_sample_counter, 0, observables_->get_left_block(), channels_count_);  //extra port for the sample counter pulse
        }
    catch (const std::exception& e)
        {
            LOG(WARNING) << "Can't connect sample counter";
            LOG(ERROR) << e.what();
            top_block_->disconnect_all();
            return;
        }
#endif
    // Signal conditioner (selected_signal_source) >> channels (i) (dependent of their associated SignalSource_ID)
    int selected_signal_conditioner_ID = 0;
    for (unsigned int i = 0; i < channels_count_; i++)
        {
            if (FPGA_enabled == false)
                {
                    try
                        {
                            selected_signal_conditioner_ID = configuration_->property("Channel" + std::to_string(i) + ".RF_channel_ID", 0);
                        }
                    catch (const std::exception& e)
                        {
                            LOG(WARNING) << e.what();
                        }
                    try
                        {
                            top_block_->connect(sig_conditioner_.at(selected_signal_conditioner_ID)->get_right_block(), 0,
                                channels_.at(i)->get_left_block(), 0);
                        }
                    catch (const std::exception& e)
                        {
                            LOG(WARNING) << "Can't connect signal conditioner " << selected_signal_conditioner_ID << " to channel " << i;
                            LOG(ERROR) << e.what();
                            top_block_->disconnect_all();
                            return;
                        }

                    DLOG(INFO) << "signal conditioner " << selected_signal_conditioner_ID << " connected to channel " << i;
                }
            // Signal Source > Signal conditioner >> Channels >> Observables
            try
                {
                    top_block_->connect(channels_.at(i)->get_right_block(), 0,
                        observables_->get_left_block(), i);
                }
            catch (const std::exception& e)
                {
                    LOG(WARNING) << "Can't connect channel " << i << " to observables";
                    LOG(ERROR) << e.what();
                    top_block_->disconnect_all();
                    return;
                }
        }

    // Put channels fixed to a given satellite at the beginning of the vector, then the rest
    std::vector<unsigned int> vector_of_channels;
    for (unsigned int i = 0; i < channels_count_; i++)
        {
            unsigned int sat = 0;
            try
                {
                    sat = configuration_->property("Channel" + std::to_string(i) + ".satellite", 0);
                }
            catch (const std::exception& e)
                {
                    LOG(WARNING) << e.what();
                }
            if (sat == 0)
                {
                    vector_of_channels.push_back(i);
                }
            else
                {
                    auto it = vector_of_channels.begin();
                    it = vector_of_channels.insert(it, i);
                }
        }

    // Assign satellites to channels in the initialization
    for (unsigned int& i : vector_of_channels)
        {
            std::string gnss_signal = channels_.at(i)->get_signal().get_signal_str();  // use channel's implicit signal
            unsigned int sat = 0;
            try
                {
                    sat = configuration_->property("Channel" + std::to_string(i) + ".satellite", 0);
                }
            catch (const std::exception& e)
                {
                    LOG(WARNING) << e.what();
                }
            if (sat == 0)
                {
                    channels_.at(i)->set_signal(search_next_signal(gnss_signal, false));
                }
            else
                {
                    std::string gnss_system;
                    Gnss_Signal signal_value;
                    switch (mapStringValues_[gnss_signal])
                        {
                        case evGPS_1C:
                            gnss_system = "GPS";
                            signal_value = Gnss_Signal(Gnss_Satellite(gnss_system, sat), gnss_signal);
                            available_GPS_1C_signals_.remove(signal_value);
                            break;

                        case evGPS_2S:
                            gnss_system = "GPS";
                            signal_value = Gnss_Signal(Gnss_Satellite(gnss_system, sat), gnss_signal);
                            available_GPS_2S_signals_.remove(signal_value);
                            break;

                        case evGPS_L5:
                            gnss_system = "GPS";
                            signal_value = Gnss_Signal(Gnss_Satellite(gnss_system, sat), gnss_signal);
                            available_GPS_L5_signals_.remove(signal_value);
                            break;

                        case evGAL_1B:
                            gnss_system = "Galileo";
                            signal_value = Gnss_Signal(Gnss_Satellite(gnss_system, sat), gnss_signal);
                            available_GAL_1B_signals_.remove(signal_value);
                            break;

                        case evGAL_5X:
                            gnss_system = "Galileo";
                            signal_value = Gnss_Signal(Gnss_Satellite(gnss_system, sat), gnss_signal);
                            available_GAL_5X_signals_.remove(signal_value);
                            break;

                        case evGLO_1G:
                            gnss_system = "Glonass";
                            signal_value = Gnss_Signal(Gnss_Satellite(gnss_system, sat), gnss_signal);
                            available_GLO_1G_signals_.remove(signal_value);
                            break;

                        case evGLO_2G:
                            gnss_system = "Glonass";
                            signal_value = Gnss_Signal(Gnss_Satellite(gnss_system, sat), gnss_signal);
                            available_GLO_2G_signals_.remove(signal_value);
                            break;

                        default:
                            LOG(ERROR) << "This should not happen :-(";
                            gnss_system = "GPS";
                            signal_value = Gnss_Signal(Gnss_Satellite(gnss_system, sat), gnss_signal);
                            available_GPS_1C_signals_.remove(signal_value);
                            break;
                        }

                    channels_.at(i)->set_signal(signal_value);
                }
        }

    // Connect the observables output of each channel to the PVT block
    try
        {
            for (unsigned int i = 0; i < channels_count_; i++)
                {
                    top_block_->connect(observables_->get_right_block(), i, pvt_->get_left_block(), i);
                    top_block_->msg_connect(channels_.at(i)->get_right_block(), pmt::mp("telemetry"), pvt_->get_left_block(), pmt::mp("telemetry"));
                }
        }
    catch (const std::exception& e)
        {
            LOG(WARNING) << "Can't connect observables to PVT";
            LOG(ERROR) << e.what();
            top_block_->disconnect_all();
            return;
        }

    // GNSS SYNCHRO MONITOR
    if (enable_monitor_)
        {
            try
                {
                    for (unsigned int i = 0; i < channels_count_; i++)
                        {
                            top_block_->connect(observables_->get_right_block(), i, GnssSynchroMonitor_, i);
                        }
                }
            catch (const std::exception& e)
                {
                    LOG(WARNING) << "Can't connect observables to Monitor block";
                    LOG(ERROR) << e.what();
                    top_block_->disconnect_all();
                    return;
                }
        }

    // Activate acquisition in enabled channels
    for (unsigned int i = 0; i < channels_count_; i++)
        {
            LOG(INFO) << "Channel " << i << " assigned to " << channels_.at(i)->get_signal();
            if (channels_state_[i] == 1)
                {
                    if (FPGA_enabled == false)
                        {
                            channels_.at(i)->start_acquisition();
                        }
                    LOG(INFO) << "Channel " << i << " connected to observables and ready for acquisition";
                }
            else
                {
                    LOG(INFO) << "Channel " << i << " connected to observables in standby mode";
                }
        }

    connected_ = true;
    LOG(INFO) << "Flowgraph connected";
    top_block_->dump();
}


void GNSSFlowgraph::disconnect()
{
    LOG(INFO) << "Disconnecting flowgraph";

    if (!connected_)
        {
            LOG(INFO) << "flowgraph was not connected";
            return;
        }
    connected_ = false;
    // Signal Source (i) >  Signal conditioner (i) >
    int RF_Channels = 0;
    int signal_conditioner_ID = 0;

    for (int i = 0; i < sources_count_; i++)
        {
            try
                {
                    // TODO: Remove this array implementation and create generic multistream connector
                    // (if a signal source has more than 1 stream, then connect it to the multistream signal conditioner)
                    if (sig_source_.at(i)->implementation().compare("Raw_Array_Signal_Source") == 0)
                        {
                            //Multichannel Array
                            for (int j = 0; j < GNSS_SDR_ARRAY_SIGNAL_CONDITIONER_CHANNELS; j++)
                                {
                                    top_block_->disconnect(sig_source_.at(i)->get_right_block(), j, sig_conditioner_.at(i)->get_left_block(), j);
                                }
                        }
                    else
                        {
                            // TODO: Create a class interface for SignalSources, derived from GNSSBlockInterface.
                            // Include GetRFChannels in the interface to avoid read config parameters here
                            // read the number of RF channels for each front-end
                            RF_Channels = configuration_->property(sig_source_.at(i)->role() + ".RF_channels", 1);

                            for (int j = 0; j < RF_Channels; j++)
                                {
                                    if (sig_source_.at(i)->get_right_block()->output_signature()->max_streams() > 1)
                                        {
                                            top_block_->disconnect(sig_source_.at(i)->get_right_block(), j, sig_conditioner_.at(signal_conditioner_ID)->get_left_block(), 0);
                                        }
                                    else
                                        {
                                            if (j == 0)
                                                {
                                                    // RF_channel 0 backward compatibility with single channel sources
                                                    top_block_->disconnect(sig_source_.at(i)->get_right_block(), 0, sig_conditioner_.at(signal_conditioner_ID)->get_left_block(), 0);
                                                }
                                            else
                                                {
                                                    // Multiple channel sources using multiple output blocks of single channel (requires RF_channel selector in call)
                                                    top_block_->disconnect(sig_source_.at(i)->get_right_block(j), 0, sig_conditioner_.at(signal_conditioner_ID)->get_left_block(), 0);
                                                }
                                        }
                                    signal_conditioner_ID++;
                                }
                        }
                }
            catch (const std::exception& e)
                {
                    LOG(INFO) << "Can't disconnect signal source " << i << " to signal conditioner " << i << ": " << e.what();
                    top_block_->disconnect_all();
                    return;
                }
        }

#if ENABLE_FPGA
    bool FPGA_enabled = configuration_->property(sig_source_.at(0)->role() + ".enable_FPGA", false);
    if (FPGA_enabled == false)
        {
            // disconnect the signal source to sample counter
            // disconnect the sample counter to Observables
            try
                {
                    top_block_->disconnect(sig_conditioner_.at(0)->get_right_block(), 0, ch_out_sample_counter, 0);
                    top_block_->disconnect(ch_out_sample_counter, 0, observables_->get_left_block(), channels_count_);  // extra port for the sample counter pulse
                }
            catch (const std::exception& e)
                {
                    LOG(WARNING) << "Can't disconnect sample counter";
                    LOG(ERROR) << e.what();
                    top_block_->disconnect_all();
                    return;
                }
        }
    else
        {
            try
                {
                    top_block_->disconnect(ch_out_fpga_sample_counter, 0, observables_->get_left_block(), channels_count_);
                }
            catch (const std::exception& e)
                {
                    LOG(WARNING) << "Can't connect FPGA sample counter";
                    LOG(ERROR) << e.what();
                    top_block_->disconnect_all();
                    return;
                }
        }
#else
    // disconnect the signal source to sample counter
    // disconnect the sample counter to Observables
    try
        {
            top_block_->disconnect(sig_conditioner_.at(0)->get_right_block(), 0, ch_out_sample_counter, 0);
            top_block_->disconnect(ch_out_sample_counter, 0, observables_->get_left_block(), channels_count_);  // extra port for the sample counter pulse
        }
    catch (const std::exception& e)
        {
            LOG(WARNING) << "Can't connect sample counter";
            LOG(ERROR) << e.what();
            top_block_->disconnect_all();
            return;
        }
#endif
    // Signal conditioner (selected_signal_source) >> channels (i) (dependent of their associated SignalSource_ID)
    int selected_signal_conditioner_ID;
    for (unsigned int i = 0; i < channels_count_; i++)
        {
            try
                {
                    selected_signal_conditioner_ID = configuration_->property("Channel" + std::to_string(i) + ".RF_channel_ID", 0);
                }
            catch (const std::exception& e)
                {
                    LOG(WARNING) << e.what();
                    top_block_->disconnect_all();
                    return;
                }
            try
                {
                    top_block_->disconnect(sig_conditioner_.at(selected_signal_conditioner_ID)->get_right_block(), 0,
                        channels_.at(i)->get_left_block(), 0);
                }
            catch (const std::exception& e)
                {
                    LOG(INFO) << "Can't disconnect signal conditioner " << selected_signal_conditioner_ID << " to channel " << i << ": " << e.what();
                    top_block_->disconnect_all();
                    return;
                }

            // Signal Source > Signal conditioner >> Channels >> Observables
            try
                {
                    top_block_->disconnect(channels_.at(i)->get_right_block(), 0,
                        observables_->get_left_block(), i);
                }
            catch (const std::exception& e)
                {
                    LOG(INFO) << "Can't disconnect channel " << i << " to observables: " << e.what();
                    top_block_->disconnect_all();
                    return;
                }
        }

    try
        {
            for (unsigned int i = 0; i < channels_count_; i++)
                {
                    top_block_->disconnect(observables_->get_right_block(), i, pvt_->get_left_block(), i);
                    top_block_->msg_disconnect(channels_.at(i)->get_right_block(), pmt::mp("telemetry"), pvt_->get_left_block(), pmt::mp("telemetry"));
                }
        }
    catch (const std::exception& e)
        {
            LOG(INFO) << "Can't disconnect observables to PVT: " << e.what();
            top_block_->disconnect_all();
            return;
        }

    for (int i = 0; i < sources_count_; i++)
        {
            try
                {
                    sig_source_.at(i)->disconnect(top_block_);
                }
            catch (const std::exception& e)
                {
                    LOG(INFO) << "Can't disconnect signal source block " << i << " internally: " << e.what();
                    top_block_->disconnect_all();
                    return;
                }
        }

    // Signal Source > Signal conditioner >
    for (unsigned int i = 0; i < sig_conditioner_.size(); i++)
        {
            try
                {
                    sig_conditioner_.at(i)->disconnect(top_block_);
                }
            catch (const std::exception& e)
                {
                    LOG(INFO) << "Can't disconnect signal conditioner block " << i << " internally: " << e.what();
                    top_block_->disconnect_all();
                    return;
                }
        }

    for (unsigned int i = 0; i < channels_count_; i++)
        {
            try
                {
                    channels_.at(i)->disconnect(top_block_);
                }
            catch (const std::exception& e)
                {
                    LOG(INFO) << "Can't disconnect channel " << i << " internally: " << e.what();
                    top_block_->disconnect_all();
                    return;
                }
        }

    try
        {
            observables_->disconnect(top_block_);
        }
    catch (const std::exception& e)
        {
            LOG(INFO) << "Can't disconnect observables block internally: " << e.what();
            top_block_->disconnect_all();
            return;
        }

    // Signal Source > Signal conditioner >> Channels >> Observables > PVT
    try
        {
            pvt_->disconnect(top_block_);
        }
    catch (const std::exception& e)
        {
            LOG(INFO) << "Can't disconnect PVT block internally: " << e.what();
            top_block_->disconnect_all();
            return;
        }

    DLOG(INFO) << "blocks disconnected internally";
    LOG(INFO) << "Flowgraph disconnected";
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


bool GNSSFlowgraph::send_telemetry_msg(pmt::pmt_t msg)
{
    // Push ephemeris to PVT telemetry msg in port using a channel out port
    // it uses the first channel as a message producer (it is already connected to PVT)
    channels_.at(0)->get_right_block()->message_port_pub(pmt::mp("telemetry"), msg);
    return true;
}


/*
 * Applies an action to the flow graph
 *
 * \param[in] who   Who generated the action
 * \param[in] what  What is the action 0: acquisition failed
 */
void GNSSFlowgraph::apply_action(unsigned int who, unsigned int what)
{
    std::lock_guard<std::mutex> lock(signal_list_mutex);
    DLOG(INFO) << "Received " << what << " from " << who << ". Number of applied actions = " << applied_actions_;
    unsigned int sat = 0;
    try
        {
            sat = configuration_->property("Channel" + std::to_string(who) + ".satellite", 0);
        }
    catch (const std::exception& e)
        {
            LOG(WARNING) << e.what();
        }
    switch (what)
        {
        case 0:
            DLOG(INFO) << "Channel " << who << " ACQ FAILED satellite " << channels_[who]->get_signal().get_satellite() << ", Signal " << channels_[who]->get_signal().get_signal_str();
            if (sat == 0)
                {
                    Gnss_Signal gs = channels_[who]->get_signal();
                    switch (mapStringValues_[gs.get_signal_str()])
                        {
                        case evGPS_1C:
                            available_GPS_1C_signals_.remove(gs);
                            available_GPS_1C_signals_.push_back(gs);
                            break;

                        case evGPS_2S:
                            available_GPS_2S_signals_.remove(gs);
                            available_GPS_2S_signals_.push_back(gs);
                            break;

                        case evGPS_L5:
                            available_GPS_L5_signals_.remove(gs);
                            available_GPS_L5_signals_.push_back(gs);
                            break;

                        case evGAL_1B:
                            available_GAL_1B_signals_.remove(gs);
                            available_GAL_1B_signals_.push_back(gs);
                            break;

                        case evGAL_5X:
                            available_GAL_5X_signals_.remove(gs);
                            available_GAL_5X_signals_.push_back(gs);
                            break;

                        case evGLO_1G:
                            available_GLO_1G_signals_.remove(gs);
                            available_GLO_1G_signals_.push_back(gs);
                            break;

                        case evGLO_2G:
                            available_GLO_2G_signals_.remove(gs);
                            available_GLO_2G_signals_.push_back(gs);
                            break;

                        default:
                            LOG(ERROR) << "This should not happen :-(";
                            break;
                        }
                }
            channels_state_[who] = 0;
            acq_channels_count_--;
            for (unsigned int i = 0; i < channels_count_; i++)
                {
                    unsigned int ch_index = (who + i + 1) % channels_count_;
                    unsigned int sat_ = 0;
                    try
                        {
                            sat_ = configuration_->property("Channel" + std::to_string(ch_index) + ".satellite", 0);
                        }
                    catch (const std::exception& e)
                        {
                            LOG(WARNING) << e.what();
                        }
                    if ((acq_channels_count_ < max_acq_channels_) && (channels_state_[ch_index] == 0))
                        {
                            channels_state_[ch_index] = 1;
                            if (sat_ == 0)
                                {
                                    channels_[ch_index]->set_signal(search_next_signal(channels_[ch_index]->get_signal().get_signal_str(), true));
                                }
                            acq_channels_count_++;
                            DLOG(INFO) << "Channel " << ch_index << " Starting acquisition " << channels_[ch_index]->get_signal().get_satellite() << ", Signal " << channels_[ch_index]->get_signal().get_signal_str();
                            channels_[ch_index]->start_acquisition();
                        }
                    DLOG(INFO) << "Channel " << ch_index << " in state " << channels_state_[ch_index];
                }
            break;

        case 1:
            LOG(INFO) << "Channel " << who << " ACQ SUCCESS satellite " << channels_[who]->get_signal().get_satellite();

            // If the satellite is in the list of available ones, remove it.
            switch (mapStringValues_[channels_[who]->get_signal().get_signal_str()])
                {
                case evGPS_1C:
                    available_GPS_1C_signals_.remove(channels_[who]->get_signal());
                    break;

                case evGPS_2S:
                    available_GPS_2S_signals_.remove(channels_[who]->get_signal());
                    break;

                case evGPS_L5:
                    available_GPS_L5_signals_.remove(channels_[who]->get_signal());
                    break;

                case evGAL_1B:
                    available_GAL_1B_signals_.remove(channels_[who]->get_signal());
                    break;

                case evGAL_5X:
                    available_GAL_5X_signals_.remove(channels_[who]->get_signal());
                    break;

                case evGLO_1G:
                    available_GLO_1G_signals_.remove(channels_[who]->get_signal());
                    break;

                case evGLO_2G:
                    available_GLO_2G_signals_.remove(channels_[who]->get_signal());
                    break;

                default:
                    LOG(ERROR) << "This should not happen :-(";
                    break;
                }

            channels_state_[who] = 2;
            acq_channels_count_--;
            for (unsigned int i = 0; i < channels_count_; i++)
                {
                    unsigned int sat_ = 0;
                    try
                        {
                            sat_ = configuration_->property("Channel" + std::to_string(i) + ".satellite", 0);
                        }
                    catch (const std::exception& e)
                        {
                            LOG(WARNING) << e.what();
                        }
                    if ((acq_channels_count_ < max_acq_channels_) && (channels_state_[i] == 0))
                        {
                            channels_state_[i] = 1;
                            if (sat_ == 0)
                                {
                                    channels_[i]->set_signal(search_next_signal(channels_[i]->get_signal().get_signal_str(), true, true));
                                }
                            acq_channels_count_++;
                            DLOG(INFO) << "Channel " << i << " Starting acquisition " << channels_[i]->get_signal().get_satellite() << ", Signal " << channels_[i]->get_signal().get_signal_str();
                            channels_[i]->start_acquisition();
                        }
                    DLOG(INFO) << "Channel " << i << " in state " << channels_state_[i];
                }
            break;

        case 2:
            LOG(INFO) << "Channel " << who << " TRK FAILED satellite " << channels_[who]->get_signal().get_satellite();
            DLOG(INFO) << "Number of channels in acquisition = " << acq_channels_count_;

            if (acq_channels_count_ < max_acq_channels_)
                {
                    channels_state_[who] = 1;
                    acq_channels_count_++;
                    LOG(INFO) << "Channel " << who << " Starting acquisition " << channels_[who]->get_signal().get_satellite() << ", Signal " << channels_[who]->get_signal().get_signal_str();
                    channels_[who]->start_acquisition();
                }
            else
                {
                    channels_state_[who] = 0;
                    LOG(INFO) << "Channel " << who << " Idle state";
                    if (sat == 0)
                        {
                            switch (mapStringValues_[channels_[who]->get_signal().get_signal_str()])
                                {
                                case evGPS_1C:
                                    available_GPS_1C_signals_.push_back(channels_[who]->get_signal());
                                    break;

                                case evGPS_2S:
                                    available_GPS_2S_signals_.push_back(channels_[who]->get_signal());
                                    break;

                                case evGPS_L5:
                                    available_GPS_L5_signals_.push_back(channels_[who]->get_signal());
                                    break;

                                case evGAL_1B:
                                    available_GAL_1B_signals_.push_back(channels_[who]->get_signal());
                                    break;

                                case evGAL_5X:
                                    available_GAL_5X_signals_.push_back(channels_[who]->get_signal());
                                    break;

                                case evGLO_1G:
                                    available_GLO_1G_signals_.push_back(channels_[who]->get_signal());
                                    break;

                                case evGLO_2G:
                                    available_GLO_2G_signals_.push_back(channels_[who]->get_signal());
                                    break;

                                default:
                                    LOG(ERROR) << "This should not happen :-(";
                                    break;
                                }
                        }
                }
            break;

        default:
            break;
        }
    applied_actions_++;
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


void GNSSFlowgraph::start_acquisition_helper()
{
    for (unsigned int i = 0; i < channels_count_; i++)
        {
            if (channels_state_[i] == 1)
                {
                    channels_.at(i)->start_acquisition();
                }
        }
}


void GNSSFlowgraph::init()
{
    /*
     * Instantiates the receiver blocks
     */
    std::unique_ptr<GNSSBlockFactory> block_factory_(new GNSSBlockFactory());

    // 1. read the number of RF front-ends available (one file_source per RF front-end)
    sources_count_ = configuration_->property("Receiver.sources_count", 1);

    int RF_Channels = 0;
    int signal_conditioner_ID = 0;

    if (sources_count_ > 1)
        {
            for (int i = 0; i < sources_count_; i++)
                {
                    std::cout << "Creating source " << i << std::endl;
                    sig_source_.push_back(block_factory_->GetSignalSource(configuration_, queue_, i));
                    // TODO: Create a class interface for SignalSources, derived from GNSSBlockInterface.
                    // Include GetRFChannels in the interface to avoid read config parameters here
                    // read the number of RF channels for each front-end
                    RF_Channels = configuration_->property(sig_source_.at(i)->role() + ".RF_channels", 1);
                    std::cout << "RF Channels " << RF_Channels << std::endl;
                    for (int j = 0; j < RF_Channels; j++)
                        {
                            sig_conditioner_.push_back(block_factory_->GetSignalConditioner(configuration_, signal_conditioner_ID));
                            signal_conditioner_ID++;
                        }
                }
        }
    else
        {
            // backwards compatibility for old config files
            sig_source_.push_back(block_factory_->GetSignalSource(configuration_, queue_, -1));
            // TODO: Create a class interface for SignalSources, derived from GNSSBlockInterface.
            // Include GetRFChannels in the interface to avoid read config parameters here
            // read the number of RF channels for each front-end
            RF_Channels = configuration_->property(sig_source_.at(0)->role() + ".RF_channels", 0);
            if (RF_Channels != 0)
                {
                    for (int j = 0; j < RF_Channels; j++)
                        {
                            sig_conditioner_.push_back(block_factory_->GetSignalConditioner(configuration_, signal_conditioner_ID));
                            signal_conditioner_ID++;
                        }
                }
            else
                {
                    // old config file, single signal source and single channel, not specified
                    sig_conditioner_.push_back(block_factory_->GetSignalConditioner(configuration_, -1));
                }
        }

    observables_ = block_factory_->GetObservables(configuration_);
    // Mark old implementations as deprecated
    std::string default_str("Default");
    std::string obs_implementation = configuration_->property("Observables.implementation", default_str);
    if ((obs_implementation.compare("GPS_L1_CA_Observables") == 0) || (obs_implementation.compare("GPS_L2C_Observables") == 0) ||
        (obs_implementation.compare("Galileo_E1B_Observables") == 0) || (obs_implementation.compare("Galileo_E5A_Observables") == 0))
        {
            std::cout << "WARNING: Implementation '" << obs_implementation << "' of the Observables block has been replaced by 'Hybrid_Observables'." << std::endl;
            std::cout << "Please update your configuration file." << std::endl;
        }

    pvt_ = block_factory_->GetPVT(configuration_);
    // Mark old implementations as deprecated
    std::string pvt_implementation = configuration_->property("PVT.implementation", default_str);
    if ((pvt_implementation.compare("GPS_L1_CA_PVT") == 0) || (pvt_implementation.compare("Galileo_E1_PVT") == 0) || (pvt_implementation.compare("Hybrid_PVT") == 0))
        {
            std::cout << "WARNING: Implementation '" << pvt_implementation << "' of the PVT block has been replaced by 'RTKLIB_PVT'." << std::endl;
            std::cout << "Please update your configuration file." << std::endl;
        }

    std::shared_ptr<std::vector<std::unique_ptr<GNSSBlockInterface>>> channels = block_factory_->GetChannels(configuration_, queue_);

    channels_count_ = channels->size();
    for (unsigned int i = 0; i < channels_count_; i++)
        {
            std::shared_ptr<GNSSBlockInterface> chan_ = std::move(channels->at(i));
            channels_.push_back(std::dynamic_pointer_cast<ChannelInterface>(chan_));
        }

    top_block_ = gr::make_top_block("GNSSFlowgraph");

    mapStringValues_["1C"] = evGPS_1C;
    mapStringValues_["2S"] = evGPS_2S;
    mapStringValues_["L5"] = evGPS_L5;
    mapStringValues_["1B"] = evGAL_1B;
    mapStringValues_["5X"] = evGAL_5X;
    mapStringValues_["1G"] = evGLO_1G;
    mapStringValues_["2G"] = evGLO_2G;

    // fill the signals queue with the satellites ID's to be searched by the acquisition
    set_signals_list();
    set_channels_state();
    applied_actions_ = 0;
    DLOG(INFO) << "Blocks instantiated. " << channels_count_ << " channels.";

    /*
	 * Instantiate the receiver monitor block, if required
	 */
    enable_monitor_ = configuration_->property("Monitor.enable_monitor", false);

    std::string address_string = configuration_->property("Monitor.client_addresses", std::string("127.0.0.1"));
    std::vector<std::string> udp_addr_vec = split_string(address_string, '_');
    std::sort(udp_addr_vec.begin(), udp_addr_vec.end());
    udp_addr_vec.erase(std::unique(udp_addr_vec.begin(), udp_addr_vec.end()), udp_addr_vec.end());

    if (enable_monitor_)
        {
            GnssSynchroMonitor_ = gr::basic_block_sptr(new gnss_synchro_monitor(channels_count_,
                configuration_->property("Monitor.output_rate_ms", 1),
                configuration_->property("Monitor.udp_port", 1234),
                udp_addr_vec));
        }
}


void GNSSFlowgraph::set_signals_list()
{
    // Set a sequential list of GNSS satellites
    std::set<unsigned int>::const_iterator available_gnss_prn_iter;

    // Create the lists of GNSS satellites
    std::set<unsigned int> available_gps_prn = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
        11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28,
        29, 30, 31, 32};

    std::set<unsigned int> available_sbas_prn = {123, 131, 135, 136, 138};

    std::set<unsigned int> available_galileo_prn = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
        11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28,
        29, 30, 31, 32, 33, 34, 35, 36};

    // Removing satellites sharing same frequency number(1 and 5, 2 and 6, 3 and 7, 4 and 6, 11 and 15, 12 and 16, 14 and 18, 17 and 21
    std::set<unsigned int> available_glonass_prn = {1, 2, 3, 4, 9, 10, 11, 12, 18, 19, 20, 21, 24};

    std::string sv_list = configuration_->property("Galileo.prns", std::string(""));

    if (sv_list.length() > 0)
        {
            // Reset the available prns:
            std::set<unsigned int> tmp_set;
            boost::tokenizer<> tok(sv_list);
            std::transform(tok.begin(), tok.end(), std::inserter(tmp_set, tmp_set.begin()),
                boost::lexical_cast<unsigned int, std::string>);

            if (tmp_set.size() > 0)
                {
                    available_galileo_prn = tmp_set;
                }
        }

    sv_list = configuration_->property("GPS.prns", std::string(""));

    if (sv_list.length() > 0)
        {
            // Reset the available prns:
            std::set<unsigned int> tmp_set;
            boost::tokenizer<> tok(sv_list);
            std::transform(tok.begin(), tok.end(), std::inserter(tmp_set, tmp_set.begin()),
                boost::lexical_cast<unsigned int, std::string>);

            if (tmp_set.size() > 0)
                {
                    available_gps_prn = tmp_set;
                }
        }

    sv_list = configuration_->property("SBAS.prns", std::string(""));

    if (sv_list.length() > 0)
        {
            // Reset the available prns:
            std::set<unsigned int> tmp_set;
            boost::tokenizer<> tok(sv_list);
            std::transform(tok.begin(), tok.end(), std::inserter(tmp_set, tmp_set.begin()),
                boost::lexical_cast<unsigned int, std::string>);

            if (tmp_set.size() > 0)
                {
                    available_sbas_prn = tmp_set;
                }
        }

    sv_list = configuration_->property("Glonass.prns", std::string(""));

    if (sv_list.length() > 0)
        {
            // Reset the available prns:
            std::set<unsigned int> tmp_set;
            boost::tokenizer<> tok(sv_list);
            std::transform(tok.begin(), tok.end(), std::inserter(tmp_set, tmp_set.begin()),
                boost::lexical_cast<unsigned int, std::string>);

            if (tmp_set.size() > 0)
                {
                    available_glonass_prn = tmp_set;
                }
        }

    if (configuration_->property("Channels_1C.count", 0) > 0)
        {
            // Loop to create GPS L1 C/A signals
            for (available_gnss_prn_iter = available_gps_prn.cbegin();
                 available_gnss_prn_iter != available_gps_prn.cend();
                 available_gnss_prn_iter++)
                {
                    available_GPS_1C_signals_.push_back(Gnss_Signal(
                        Gnss_Satellite(std::string("GPS"), *available_gnss_prn_iter),
                        std::string("1C")));
                }
        }

    if (configuration_->property("Channels_2S.count", 0) > 0)
        {
            // Loop to create GPS L2C M signals
            for (available_gnss_prn_iter = available_gps_prn.cbegin();
                 available_gnss_prn_iter != available_gps_prn.cend();
                 available_gnss_prn_iter++)
                {
                    available_GPS_2S_signals_.push_back(Gnss_Signal(
                        Gnss_Satellite(std::string("GPS"), *available_gnss_prn_iter),
                        std::string("2S")));
                }
        }

    if (configuration_->property("Channels_L5.count", 0) > 0)
        {
            // Loop to create GPS L5 signals
            for (available_gnss_prn_iter = available_gps_prn.cbegin();
                 available_gnss_prn_iter != available_gps_prn.cend();
                 available_gnss_prn_iter++)
                {
                    available_GPS_L5_signals_.push_back(Gnss_Signal(
                        Gnss_Satellite(std::string("GPS"), *available_gnss_prn_iter),
                        std::string("L5")));
                }
        }

    if (configuration_->property("Channels_SBAS.count", 0) > 0)
        {
            // Loop to create SBAS L1 C/A signals
            for (available_gnss_prn_iter = available_sbas_prn.cbegin();
                 available_gnss_prn_iter != available_sbas_prn.cend();
                 available_gnss_prn_iter++)
                {
                    available_SBAS_1C_signals_.push_back(Gnss_Signal(
                        Gnss_Satellite(std::string("SBAS"), *available_gnss_prn_iter),
                        std::string("1C")));
                }
        }

    if (configuration_->property("Channels_1B.count", 0) > 0)
        {
            // Loop to create the list of Galileo E1B signals
            for (available_gnss_prn_iter = available_galileo_prn.cbegin();
                 available_gnss_prn_iter != available_galileo_prn.cend();
                 available_gnss_prn_iter++)
                {
                    available_GAL_1B_signals_.push_back(Gnss_Signal(
                        Gnss_Satellite(std::string("Galileo"), *available_gnss_prn_iter),
                        std::string("1B")));
                }
        }

    if (configuration_->property("Channels_5X.count", 0) > 0)
        {
            // Loop to create the list of Galileo E5a signals
            for (available_gnss_prn_iter = available_galileo_prn.cbegin();
                 available_gnss_prn_iter != available_galileo_prn.cend();
                 available_gnss_prn_iter++)
                {
                    available_GAL_5X_signals_.push_back(Gnss_Signal(
                        Gnss_Satellite(std::string("Galileo"), *available_gnss_prn_iter),
                        std::string("5X")));
                }
        }

    if (configuration_->property("Channels_1G.count", 0) > 0)
        {
            // Loop to create the list of GLONASS L1 C/A signals
            for (available_gnss_prn_iter = available_glonass_prn.cbegin();
                 available_gnss_prn_iter != available_glonass_prn.cend();
                 available_gnss_prn_iter++)
                {
                    available_GLO_1G_signals_.push_back(Gnss_Signal(
                        Gnss_Satellite(std::string("Glonass"), *available_gnss_prn_iter),
                        std::string("1G")));
                }
        }

    if (configuration_->property("Channels_2G.count", 0) > 0)
        {
            // Loop to create the list of GLONASS L2 C/A signals
            for (available_gnss_prn_iter = available_glonass_prn.cbegin();
                 available_gnss_prn_iter != available_glonass_prn.cend();
                 available_gnss_prn_iter++)
                {
                    available_GLO_2G_signals_.push_back(Gnss_Signal(
                        Gnss_Satellite(std::string("Glonass"), *available_gnss_prn_iter),
                        std::string("2G")));
                }
        }
}


void GNSSFlowgraph::set_channels_state()
{
    std::lock_guard<std::mutex> lock(signal_list_mutex);
    max_acq_channels_ = configuration_->property("Channels.in_acquisition", channels_count_);
    if (max_acq_channels_ > channels_count_)
        {
            max_acq_channels_ = channels_count_;
            LOG(WARNING) << "Channels_in_acquisition is bigger than number of channels. Variable acq_channels_count_ is set to " << channels_count_;
        }
    channels_state_.reserve(channels_count_);
    for (unsigned int i = 0; i < channels_count_; i++)
        {
            if (i < max_acq_channels_)
                {
                    channels_state_.push_back(1);
                }
            else
                {
                    channels_state_.push_back(0);
                }
            DLOG(INFO) << "Channel " << i << " in state " << channels_state_[i];
        }
    acq_channels_count_ = max_acq_channels_;
    DLOG(INFO) << acq_channels_count_ << " channels in acquisition state";
}


Gnss_Signal GNSSFlowgraph::search_next_signal(std::string searched_signal, bool pop, bool tracked)
{
    Gnss_Signal result;
    bool untracked_satellite = true;
    switch (mapStringValues_[searched_signal])
        {
        case evGPS_1C:
            result = available_GPS_1C_signals_.front();
            available_GPS_1C_signals_.pop_front();
            if (!pop)
                {
                    available_GPS_1C_signals_.push_back(result);
                }
            if (tracked)
                {
                    if ((configuration_->property("Channels_2S.count", 0) > 0) or (configuration_->property("Channels_L5.count", 0) > 0))
                        {
                            for (unsigned int ch = 0; ch < channels_count_; ch++)
                                {
                                    if ((channels_[ch]->get_signal().get_satellite() == result.get_satellite()) and (channels_[ch]->get_signal().get_signal_str().compare("1C") != 0)) untracked_satellite = false;
                                }
                            if (untracked_satellite and configuration_->property("Channels_2S.count", 0) > 0)
                                {
                                    Gnss_Signal gs = Gnss_Signal(result.get_satellite(), "2S");
                                    available_GPS_2S_signals_.remove(gs);
                                    available_GPS_2S_signals_.push_front(gs);
                                }
                            if (untracked_satellite and configuration_->property("Channels_L5.count", 0) > 0)
                                {
                                    Gnss_Signal gs = Gnss_Signal(result.get_satellite(), "L5");
                                    available_GPS_L5_signals_.remove(gs);
                                    available_GPS_L5_signals_.push_front(gs);
                                }
                        }
                }
            break;

        case evGPS_2S:
            result = available_GPS_2S_signals_.front();
            available_GPS_2S_signals_.pop_front();
            if (!pop)
                {
                    available_GPS_2S_signals_.push_back(result);
                }
            if (tracked)
                {
                    if ((configuration_->property("Channels_1C.count", 0) > 0) or (configuration_->property("Channels_L5.count", 0) > 0))
                        {
                            for (unsigned int ch = 0; ch < channels_count_; ch++)
                                {
                                    if ((channels_[ch]->get_signal().get_satellite() == result.get_satellite()) and (channels_[ch]->get_signal().get_signal_str().compare("2S") != 0)) untracked_satellite = false;
                                }
                            if (untracked_satellite and configuration_->property("Channels_1C.count", 0) > 0)
                                {
                                    Gnss_Signal gs = Gnss_Signal(result.get_satellite(), "1C");
                                    available_GPS_1C_signals_.remove(gs);
                                    available_GPS_1C_signals_.push_front(gs);
                                }
                            if (untracked_satellite and configuration_->property("Channels_L5.count", 0) > 0)
                                {
                                    Gnss_Signal gs = Gnss_Signal(result.get_satellite(), "L5");
                                    available_GPS_L5_signals_.remove(gs);
                                    available_GPS_L5_signals_.push_front(gs);
                                }
                        }
                }
            break;

        case evGPS_L5:
            result = available_GPS_L5_signals_.front();
            available_GPS_L5_signals_.pop_front();
            if (!pop)
                {
                    available_GPS_L5_signals_.push_back(result);
                }
            if (tracked)
                {
                    if ((configuration_->property("Channels_1C.count", 0) > 0) or (configuration_->property("Channels_2S.count", 0) > 0))
                        {
                            for (unsigned int ch = 0; ch < channels_count_; ch++)
                                {
                                    if ((channels_[ch]->get_signal().get_satellite() == result.get_satellite()) and (channels_[ch]->get_signal().get_signal_str().compare("L5") != 0)) untracked_satellite = false;
                                }
                            if (untracked_satellite and configuration_->property("Channels_1C.count", 0) > 0)
                                {
                                    Gnss_Signal gs = Gnss_Signal(result.get_satellite(), "1C");
                                    available_GPS_1C_signals_.remove(gs);
                                    available_GPS_1C_signals_.push_front(gs);
                                }
                            if (untracked_satellite and configuration_->property("Channels_2S.count", 0) > 0)
                                {
                                    Gnss_Signal gs = Gnss_Signal(result.get_satellite(), "2S");
                                    available_GPS_2S_signals_.remove(gs);
                                    available_GPS_2S_signals_.push_front(gs);
                                }
                        }
                }
            break;

        case evGAL_1B:
            result = available_GAL_1B_signals_.front();
            available_GAL_1B_signals_.pop_front();
            if (!pop)
                {
                    available_GAL_1B_signals_.push_back(result);
                }
            if (tracked)
                {
                    if (configuration_->property("Channels_5X.count", 0) > 0)
                        {
                            for (unsigned int ch = 0; ch < channels_count_; ch++)
                                {
                                    if ((channels_[ch]->get_signal().get_satellite() == result.get_satellite()) and (channels_[ch]->get_signal().get_signal_str().compare("1B") != 0)) untracked_satellite = false;
                                }
                            if (untracked_satellite)
                                {
                                    Gnss_Signal gs = Gnss_Signal(result.get_satellite(), "5X");
                                    available_GAL_5X_signals_.remove(gs);
                                    available_GAL_5X_signals_.push_front(gs);
                                }
                        }
                }
            break;

        case evGAL_5X:
            result = available_GAL_5X_signals_.front();
            available_GAL_5X_signals_.pop_front();
            if (!pop)
                {
                    available_GAL_5X_signals_.push_back(result);
                }
            if (tracked)
                {
                    if (configuration_->property("Channels_1B.count", 0) > 0)
                        {
                            for (unsigned int ch = 0; ch < channels_count_; ch++)
                                {
                                    if ((channels_[ch]->get_signal().get_satellite() == result.get_satellite()) and (channels_[ch]->get_signal().get_signal_str().compare("5X") != 0)) untracked_satellite = false;
                                }
                            if (untracked_satellite)
                                {
                                    Gnss_Signal gs = Gnss_Signal(result.get_satellite(), "1B");
                                    available_GAL_1B_signals_.remove(gs);
                                    available_GAL_1B_signals_.push_front(gs);
                                }
                        }
                }
            break;

        case evGLO_1G:
            result = available_GLO_1G_signals_.front();
            available_GLO_1G_signals_.pop_front();
            if (!pop)
                {
                    available_GLO_1G_signals_.push_back(result);
                }
            if (tracked)
                {
                    if (configuration_->property("Channels_2G.count", 0) > 0)
                        {
                            for (unsigned int ch = 0; ch < channels_count_; ch++)
                                {
                                    if ((channels_[ch]->get_signal().get_satellite() == result.get_satellite()) and (channels_[ch]->get_signal().get_signal_str().compare("1G") != 0)) untracked_satellite = false;
                                }
                            if (untracked_satellite)
                                {
                                    Gnss_Signal gs = Gnss_Signal(result.get_satellite(), "2G");
                                    available_GLO_2G_signals_.remove(gs);
                                    available_GLO_2G_signals_.push_front(gs);
                                }
                        }
                }
            break;

        case evGLO_2G:
            result = available_GLO_2G_signals_.front();
            available_GLO_2G_signals_.pop_front();
            if (!pop)
                {
                    available_GLO_2G_signals_.push_back(result);
                }
            if (tracked)
                {
                    if (configuration_->property("Channels_1G.count", 0) > 0)
                        {
                            for (unsigned int ch = 0; ch < channels_count_; ch++)
                                {
                                    if ((channels_[ch]->get_signal().get_satellite() == result.get_satellite()) and (channels_[ch]->get_signal().get_signal_str().compare("2G") != 0)) untracked_satellite = false;
                                }
                            if (untracked_satellite)
                                {
                                    Gnss_Signal gs = Gnss_Signal(result.get_satellite(), "1G");
                                    available_GLO_1G_signals_.remove(gs);
                                    available_GLO_1G_signals_.push_front(gs);
                                }
                        }
                }
            break;

        default:
            LOG(ERROR) << "This should not happen :-(";
            result = available_GPS_1C_signals_.front();
            if (pop)
                {
                    available_GPS_1C_signals_.pop_front();
                }
            break;
        }
    return result;
}

std::vector<std::string> GNSSFlowgraph::split_string(const std::string& s, char delim)
{
    std::vector<std::string> v;
    std::stringstream ss(s);
    std::string item;

    while (std::getline(ss, item, delim))
        {
            *(std::back_inserter(v)++) = item;
        }

    return v;
}
