/*!
 * \file gnss_flowgraph.cc
 * \brief Implementation of a GNSS receiver flow graph
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *         Carles Fernandez-Prades, 2014-2020. cfernandez(at)cttc.es
 *         Álvaro Cebrián Juan, 2018. acebrianjuan(at)gmail.com
 *         Javier Arribas, 2018. javiarribas(at)gmail.com
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

#include "gnss_flowgraph.h"
#include "GPS_L1_CA.h"
#include "GPS_L2C.h"
#include "GPS_L5.h"
#include "Galileo_E1.h"
#include "Galileo_E5a.h"
#include "Galileo_E5b.h"
#include "Galileo_E6.h"
#include "Galileo_OSNMA.h"
#include "channel.h"
#include "channel_fsm.h"
#include "channel_interface.h"
#include "configuration_interface.h"
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "gnss_satellite.h"
#include "gnss_sdr_make_unique.h"
#include "gnss_synchro_monitor.h"
#include "nav_message_monitor.h"
#include "signal_source_interface.h"
#include <boost/lexical_cast.hpp>    // for boost::lexical_cast
#include <boost/tokenizer.hpp>       // for boost::tokenizer
#include <gnuradio/basic_block.h>    // for basic_block
#include <gnuradio/filter/firdes.h>  // for gr::filter::firdes
#include <gnuradio/io_signature.h>   // for io_signature
#include <gnuradio/top_block.h>      // for top_block, make_top_block
#include <pmt/pmt_sugar.h>           // for mp
#include <algorithm>                 // for transform, sort, unique
#include <cmath>                     // for floor
#include <cstddef>                   // for size_t
#include <exception>                 // for exception
#include <iostream>                  // for operator<<
#include <iterator>                  // for insert_iterator, inserter
#include <memory>                    // for std::shared_ptr
#include <set>                       // for set
#include <sstream>                   // for std::stringstream
#include <stdexcept>                 // for invalid_argument
#include <thread>                    // for std::thread
#include <utility>                   // for std::move

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

#ifdef GR_GREATER_38
#include <gnuradio/filter/fir_filter_blk.h>
#else
#include <gnuradio/filter/fir_filter_ccf.h>
#endif


#define GNSS_SDR_ARRAY_SIGNAL_CONDITIONER_CHANNELS 8


GNSSFlowgraph::GNSSFlowgraph(std::shared_ptr<ConfigurationInterface> configuration,
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue)  // NOLINT(performance-unnecessary-value-param)
    : configuration_(std::move(configuration)),
      queue_(std::move(queue)),
      connected_(false),
      running_(false),
      multiband_(GNSSFlowgraph::is_multiband()),
      enable_osnma_rx_(false),
      enable_e6_has_rx_(false)
{
    enable_fpga_offloading_ = configuration_->property("GNSS-SDR.enable_FPGA", false);
    init();
}


GNSSFlowgraph::~GNSSFlowgraph()
{
    DLOG(INFO) << "GNSSFlowgraph destructor called";
    if (connected_)
        {
            GNSSFlowgraph::disconnect();
        }
}


void GNSSFlowgraph::init()
{
    /*
     * Instantiates the receiver blocks
     */
    auto block_factory = std::make_unique<GNSSBlockFactory>();

    channels_status_ = channel_status_msg_receiver_make();

    if (configuration_->property("Channels_E6.count", 0) > 0)
        {
            enable_e6_has_rx_ = true;
            gal_e6_has_rx_ = galileo_e6_has_msg_receiver_make();
            galileo_tow_map_ = galileo_tow_map_make();
        }
    else
        {
            gal_e6_has_rx_ = nullptr;
            galileo_tow_map_ = nullptr;
        }

    if (configuration_->property("Channels_1B.count", 0) > 0 && configuration_->property("GNSS-SDR.osnma_enable", true))
        {
            enable_osnma_rx_ = true;
            const auto certFilePath = configuration_->property("GNSS-SDR.osnma_public_key", CRTFILE_DEFAULT);
            const auto merKleTreePath = configuration_->property("GNSS-SDR.osnma_merkletree", MERKLEFILE_DEFAULT);
            std::string osnma_mode = configuration_->property("GNSS-SDR.osnma_mode", std::string(""));
            bool strict_mode = false;
            if (osnma_mode == "strict")
                {
                    strict_mode = true;
                }
            osnma_rx_ = osnma_msg_receiver_make(certFilePath, merKleTreePath, strict_mode);
        }
    else
        {
            osnma_rx_ = nullptr;
        }

    // 1. read the number of RF front-ends available (one file_source per RF front-end)
    int sources_count_deprecated = configuration_->property("Receiver.sources_count", 1);
    sources_count_ = configuration_->property("GNSS-SDR.num_sources", sources_count_deprecated);

    // Avoid segmentation fault caused by wrong configuration
    if (sources_count_ == 2 && block_factory->GetSignalSource(configuration_.get(), queue_.get(), 0)->implementation() == "Multichannel_File_Signal_Source")
        {
            std::cout << " * Please set GNSS-SDR.num_sources=1 in your configuraiion file\n";
            std::cout << "   if you are using the Multichannel_File_Signal_Source implementation.\n";
            sources_count_ = 1;
        }

    int signal_conditioner_ID = 0;

    for (int i = 0; i < sources_count_; i++)
        {
            DLOG(INFO) << "Creating source " << i;
            sig_source_.push_back(block_factory->GetSignalSource(configuration_.get(), queue_.get(), i));
            if (enable_fpga_offloading_ == false)
                {
                    auto& src = sig_source_.back();
                    auto RF_Channels = src->getRfChannels();
                    if (sources_count_ == 1)
                        {
                            std::cout << "RF Channels: " << RF_Channels << '\n';
                        }
                    for (auto j = 0U; j < RF_Channels; ++j)
                        {
                            sig_conditioner_.push_back(block_factory->GetSignalConditioner(configuration_.get(), signal_conditioner_ID));
                            signal_conditioner_ID++;
                        }
                }
        }
    if (sources_count_ != 1 && !enable_fpga_offloading_)
        {
            std::cout << "RF Channels: " << sources_count_ << '\n';
        }
    if (!sig_conditioner_.empty())
        {
            signal_conditioner_connected_ = std::vector<bool>(sig_conditioner_.size(), false);
        }

    observables_ = block_factory->GetObservables(configuration_.get());

    pvt_ = block_factory->GetPVT(configuration_.get());

    auto channels = block_factory->GetChannels(configuration_.get(), queue_.get());

    channels_count_ = static_cast<int>(channels->size());
    for (int i = 0; i < channels_count_; i++)
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
    mapStringValues_["7X"] = evGAL_7X;
    mapStringValues_["E6"] = evGAL_E6;
    mapStringValues_["1G"] = evGLO_1G;
    mapStringValues_["2G"] = evGLO_2G;
    mapStringValues_["B1"] = evBDS_B1;
    mapStringValues_["B3"] = evBDS_B3;

    // fill the signals queue with the satellites ID's to be searched by the acquisition
    set_signals_list();
    set_channels_state();
    DLOG(INFO) << "Blocks instantiated. " << channels_count_ << " channels.";

    /*
     * Instantiate the receiver monitor block, if required
     */
    enable_monitor_ = configuration_->property("Monitor.enable_monitor", false);
    if (enable_monitor_)
        {
            // Retrieve monitor properties
            bool enable_protobuf = configuration_->property("Monitor.enable_protobuf", true);
            if (configuration_->property("PVT.enable_protobuf", false) == true)
                {
                    enable_protobuf = true;
                }
            std::string address_string = configuration_->property("Monitor.client_addresses", std::string("127.0.0.1"));
            std::vector<std::string> udp_addr_vec = split_string(address_string, '_');
            std::sort(udp_addr_vec.begin(), udp_addr_vec.end());
            udp_addr_vec.erase(std::unique(udp_addr_vec.begin(), udp_addr_vec.end()), udp_addr_vec.end());

            // Instantiate monitor object
            GnssSynchroMonitor_ = gnss_synchro_make_monitor(channels_count_,
                configuration_->property("Monitor.decimation_factor", 1),
                configuration_->property("Monitor.udp_port", 1234),
                udp_addr_vec, enable_protobuf);
        }

    /*
     * Instantiate the receiver acquisition monitor block, if required
     */
    enable_acquisition_monitor_ = configuration_->property("AcquisitionMonitor.enable_monitor", false);
    if (enable_acquisition_monitor_)
        {
            // Retrieve monitor properties
            bool enable_protobuf = configuration_->property("AcquisitionMonitor.enable_protobuf", true);
            if (configuration_->property("PVT.enable_protobuf", false) == true)
                {
                    enable_protobuf = true;
                }
            std::string address_string = configuration_->property("AcquisitionMonitor.client_addresses", std::string("127.0.0.1"));
            std::vector<std::string> udp_addr_vec = split_string(address_string, '_');
            std::sort(udp_addr_vec.begin(), udp_addr_vec.end());
            udp_addr_vec.erase(std::unique(udp_addr_vec.begin(), udp_addr_vec.end()), udp_addr_vec.end());

            GnssSynchroAcquisitionMonitor_ = gnss_synchro_make_monitor(channels_count_,
                configuration_->property("AcquisitionMonitor.decimation_factor", 1),
                configuration_->property("AcquisitionMonitor.udp_port", 1235),
                udp_addr_vec, enable_protobuf);
        }

    /*
     * Instantiate the receiver tracking monitor block, if required
     */
    enable_tracking_monitor_ = configuration_->property("TrackingMonitor.enable_monitor", false);
    if (enable_tracking_monitor_)
        {
            // Retrieve monitor properties
            bool enable_protobuf = configuration_->property("TrackingMonitor.enable_protobuf", true);
            if (configuration_->property("PVT.enable_protobuf", false) == true)
                {
                    enable_protobuf = true;
                }
            std::string address_string = configuration_->property("TrackingMonitor.client_addresses", std::string("127.0.0.1"));
            std::vector<std::string> udp_addr_vec = split_string(address_string, '_');
            std::sort(udp_addr_vec.begin(), udp_addr_vec.end());
            udp_addr_vec.erase(std::unique(udp_addr_vec.begin(), udp_addr_vec.end()), udp_addr_vec.end());

            GnssSynchroTrackingMonitor_ = gnss_synchro_make_monitor(channels_count_,
                configuration_->property("TrackingMonitor.decimation_factor", 1),
                configuration_->property("TrackingMonitor.udp_port", 1236),
                udp_addr_vec, enable_protobuf);
        }

    /*
     * Instantiate the receiver nav message monitor block, if required
     */
    enable_navdata_monitor_ = configuration_->property("NavDataMonitor.enable_monitor", false);
    if (enable_navdata_monitor_)
        {
            // Retrieve monitor properties
            std::string address_string = configuration_->property("NavDataMonitor.client_addresses", std::string("127.0.0.1"));
            std::vector<std::string> udp_addr_vec = split_string(address_string, '_');
            std::sort(udp_addr_vec.begin(), udp_addr_vec.end());
            udp_addr_vec.erase(std::unique(udp_addr_vec.begin(), udp_addr_vec.end()), udp_addr_vec.end());
            NavDataMonitor_ = nav_message_monitor_make(udp_addr_vec, configuration_->property("NavDataMonitor.port", 1237));
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
            LOG(ERROR) << "Unable to start flowgraph: " << e.what();
            print_help();
            return;
        }

    if (enable_fpga_offloading_ == true)
        {
            // start the DMA if the receiver is in post-processing mode
            if (configuration_->property(sig_source_.at(0)->role() + ".switch_position", 0) == 0)
                {
                    sig_source_.at(0)->start();
                }
        }

    running_ = true;
}


void GNSSFlowgraph::stop()
{
    for (const auto& chan : channels_)
        {
            chan->stop_channel();  // stop the acquisition or tracking operation
        }
    top_block_->stop();

    if (enable_fpga_offloading_ == false)
        {
            top_block_->wait();
        }

    running_ = false;
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


void GNSSFlowgraph::connect()
{
    // Connects the blocks in the flow graph
    LOG(INFO) << "Connecting flowgraph";
    if (connected_)
        {
            LOG(WARNING) << "flowgraph already connected";
            return;
        }

#if ENABLE_FPGA
    if (enable_fpga_offloading_ == true)
        {
            if (connect_fpga_flowgraph() != 0)
                {
                    std::cerr << "Unable to connect flowgraph with FPGA off-loading\n";
                    print_help();
                    return;
                }
        }
    else
        {
            if (connect_desktop_flowgraph() != 0)
                {
                    std::cerr << "Unable to connect flowgraph\n";
                    print_help();
                    return;
                }
        }
#else
    if (connect_desktop_flowgraph() != 0)
        {
            std::cerr << "Unable to connect flowgraph\n";
            print_help();
            return;
        }
#endif

    connected_ = true;
    LOG(INFO) << "Flowgraph connected";
    top_block_->dump();
}


void GNSSFlowgraph::disconnect()
{
    LOG(INFO) << "Disconnecting flowgraph";

    if (!connected_)
        {
            LOG(INFO) << "Flowgraph was not connected";
            return;
        }
    connected_ = false;
    try
        {
            top_block_->disconnect_all();
        }
    catch (const std::exception& e)
        {
            LOG(INFO) << "Problem disconnecting the flowgraph: " << e.what();
        }

    LOG(INFO) << "Flowgraph disconnected";
}


int GNSSFlowgraph::connect_desktop_flowgraph()
{
    // Connect blocks to the top_block
    const int max_channels_in_acq = configuration_->property("Channels.in_acquisition", 0);
    if (max_channels_in_acq > channels_count_)
        {
            help_hint_ += " * The maximum number of channels with concurrent signal acquisition is set to Channels.in_acquisition=" + std::to_string(max_channels_in_acq) + ",\n";
            help_hint_ += " but the total number of channels is set to " + std::to_string(channels_count_) + ".\n";
            help_hint_ += " Please set Channels.in_acquisition to " + std::to_string(channels_count_) + " or lower, or increment the number of channels in your configuration file.\n";
            return 1;
        }

    if (connect_signal_sources() != 0)
        {
            return 1;
        }

    if (connect_signal_conditioners() != 0)
        {
            return 1;
        }

    if (connect_channels() != 0)
        {
            return 1;
        }

    if (connect_observables() != 0)
        {
            return 1;
        }

    if (connect_pvt() != 0)
        {
            return 1;
        }

    // Connect blocks between them to form the flow graph
    if (connect_signal_sources_to_signal_conditioners() != 0)
        {
            return 1;
        }

    if (connect_sample_counter() != 0)
        {
            return 1;
        }

    if (connect_signal_conditioners_to_channels() != 0)
        {
            return 1;
        }

    if (connect_channels_to_observables() != 0)
        {
            return 1;
        }

    check_signal_conditioners();

    if (assign_channels() != 0)
        {
            return 1;
        }

    if (connect_observables_to_pvt() != 0)
        {
            return 1;
        }

    if (connect_monitors() != 0)
        {
            return 1;
        }

    if (enable_e6_has_rx_)
        {
            if (connect_gal_e6_has() != 0)
                {
                    return 1;
                }
            if (connect_galileo_tow_map() != 0)
                {
                    return 1;
                }
        }

    if (enable_osnma_rx_)
        {
            if (connect_osnma() != 0)
                {
                    return 1;
                }
        }

    // Activate acquisition in enabled channels
    std::lock_guard<std::mutex> lock(signal_list_mutex_);
    for (int i = 0; i < channels_count_; i++)
        {
            LOG(INFO) << "Channel " << i << " assigned to " << channels_.at(i)->get_signal();
            if (channels_state_[i] == 1)
                {
#if ENABLE_FPGA
                    if (enable_fpga_offloading_)
                        {
                            // create a task for the FPGA such that it doesn't stop the flow
                            std::thread tmp_thread(&ChannelInterface::start_acquisition, channels_[i]);
                            tmp_thread.detach();
                        }
                    else
                        {
                            channels_.at(i)->start_acquisition();
                        }
#else
                    channels_.at(i)->start_acquisition();
#endif
                    LOG(INFO) << "Channel " << i << " connected to observables and ready for acquisition";
                }
            else
                {
                    LOG(INFO) << "Channel " << i << " connected to observables in standby mode";
                }
        }

    LOG(INFO) << "The GNU Radio flowgraph for the current GNSS-SDR configuration has been successfully connected";
    return 0;
}


#if ENABLE_FPGA
int GNSSFlowgraph::connect_fpga_flowgraph()
{
    const int max_channels_in_acq = configuration_->property("Channels.in_acquisition", 0);
    if (max_channels_in_acq != 1)
        {
            help_hint_ += " * The maximum number of channels with concurrent signal acquisition is set to Channels.in_acquisition=" + std::to_string(max_channels_in_acq) + ",\n";
            help_hint_ += " but it must be set to 1 in the FPGA flow graph.\n";
            help_hint_ += " Please set Channels.in_acquisition=1 in your configuration file.\n";
            return 1;
        }

    // Check that the Signal Source has been instantiated successfully
    for (auto& src : sig_source_)
        {
            if (src == nullptr)
                {
                    help_hint_ += " * Check implementation name for SignalSource block.\n";
                    help_hint_ += "   Signal Source block implementation for FPGA off-loading should be Ad9361_Fpga_Signal_Source\n";
                    return 1;
                }
            if (src->item_size() == 0)
                {
                    help_hint_ += " * The global configuration parameter GNSS-SDR.enable_FPGA is set to true,\n";
                    help_hint_ += "   but gnss-sdr does not appear to be executed in an FPGA-equipped platform,\n";
                    help_hint_ += "   or there are some required files that are missing.\n";
                    return 1;
                }
        }

    // Connect blocks to the top_block
    if (connect_channels() != 0)
        {
            return 1;
        }

    if (connect_observables() != 0)
        {
            return 1;
        }

    if (connect_pvt() != 0)
        {
            return 1;
        }

    DLOG(INFO) << "Blocks connected internally to the top_block";

    // Connect the counter
    if (connect_fpga_sample_counter() != 0)
        {
            return 1;
        }

    if (connect_channels_to_observables() != 0)
        {
            return 1;
        }

    if (assign_channels() != 0)
        {
            return 1;
        }

    if (connect_observables_to_pvt() != 0)
        {
            return 1;
        }

    if (connect_monitors() != 0)
        {
            return 1;
        }

    if (enable_e6_has_rx_)
        {
            if (connect_gal_e6_has() != 0)
                {
                    return 1;
                }
            if (connect_galileo_tow_map() != 0)
                {
                    return 1;
                }
        }

    if (enable_osnma_rx_)
        {
            if (connect_osnma() != 0)
                {
                    return 1;
                }
        }

    check_desktop_conf_in_fpga_env();

    LOG(INFO) << "The GNU Radio flowgraph for the current GNSS-SDR configuration with FPGA off-loading has been successfully connected";
    return 0;
}
#endif


int GNSSFlowgraph::connect_signal_sources()
{
    for (int i = 0; i < sources_count_; i++)
        {
            if (sig_source_.at(i) != nullptr)
                {
                    try
                        {
                            sig_source_.at(i)->connect(top_block_);
                        }
                    catch (const std::exception& e)
                        {
                            LOG(ERROR) << "Can't connect signal source block " << i << " internally: " << e.what();
                            top_block_->disconnect_all();
                            return 1;
                        }
                }
            else
                {
                    help_hint_ += " * Check implementation name for SignalSource" + (i == 0 ? " " : (std::to_string(i) + " ")) + "block\n";
                    help_hint_ += "   Signal Source blocks documentation at https://gnss-sdr.org/docs/sp-blocks/signal-source/\n";
                    top_block_->disconnect_all();
                    return 1;
                }
        }
    DLOG(INFO) << "Signal Source blocks successfully connected to the top_block";
    return 0;
}


int GNSSFlowgraph::connect_signal_conditioners()
{
    int error = 1;  // this should be bool (true)
    try
        {
            for (auto& sig : sig_conditioner_)
                {
                    if (sig == nullptr)
                        {
                            help_hint_ += " * The Signal_Conditioner implementation set in the configuration file does not exist.\n";
                            help_hint_ += "   Check the Signal Conditioner documentation at https://gnss-sdr.org/docs/sp-blocks/signal-conditioner/\n";
                            return error;
                        }
                    sig->connect(top_block_);
                }
            DLOG(INFO) << "Signal Conditioner blocks successfully connected to the top_block";
            error = 0;  // false
        }
    catch (const std::exception& e)
        {
            LOG(ERROR) << "Can't connect signal conditioner block internally: " << e.what();
            top_block_->disconnect_all();
            std::string reported_error(e.what());
            if (std::string::npos != reported_error.find(std::string("itemsize mismatch")))
                {
                    std::string replace_me("copy");
                    size_t pos = reported_error.find(replace_me);
                    while (pos != std::string::npos)
                        {
                            size_t len = replace_me.length();
                            reported_error.replace(pos, len, "Pass_Through");
                            pos = reported_error.find(replace_me, pos + 1);
                        }
                    help_hint_ += " * Blocks within the Signal Conditioner are connected with mismatched input/ouput item size\n";
                    help_hint_ += "   Reported error: " + reported_error + '\n';
                    help_hint_ += "   Check the Signal Conditioner documentation at https://gnss-sdr.org/docs/sp-blocks/signal-conditioner/\n";
                }
            if (std::string::npos != reported_error.find(std::string("DataTypeAdapter")))
                {
                    help_hint_ += " * The DataTypeAdapter implementation set in the configuration file does not exist\n";
                    help_hint_ += "   Check the DataTypeAdapter documentation at https://gnss-sdr.org/docs/sp-blocks/data-type-adapter/\n";
                }
            if (std::string::npos != reported_error.find(std::string("InputFilter")))
                {
                    if (std::string::npos != reported_error.find(std::string("itemsize mismatch")))
                        {
                            help_hint_ += " * The configured InputFilter input/output item types are not well defined.\n";
                        }
                    else
                        {
                            help_hint_ += " * The InputFilter implementation set in the configuration file does not exist\n";
                        }
                    help_hint_ += "   Check the InputFilter documentation at https://gnss-sdr.org/docs/sp-blocks/input-filter/\n";
                }
            if (std::string::npos != reported_error.find(std::string("Resampler")))
                {
                    if (std::string::npos != reported_error.find(std::string("itemsize mismatch")))
                        {
                            help_hint_ += " * The configured Resampler item type is not well defined.\n";
                        }
                    else
                        {
                            help_hint_ += " * The Resampler implementation set in the configuration file does not exist\n";
                        }
                    help_hint_ += "   Check the Resampler documentation at https://gnss-sdr.org/docs/sp-blocks/resampler/\n";
                }
        }
    return error;
}


int GNSSFlowgraph::connect_channels()
{
    if (channels_count_ <= 0)
        {
            LOG(ERROR) << "No channels have been assigned.";
            help_hint_ += " * No channels have been assigned, check your configuration file.\n";
            help_hint_ += "   At least one of the Channels_XX.count must be > 0.\n";
            help_hint_ += "   Channels documentation at https://gnss-sdr.org/docs/sp-blocks/channels/\n";
            top_block_->disconnect_all();
            return 1;
        }
    for (int i = 0; i < channels_count_; i++)
        {
            if (channels_.at(i) != nullptr)
                {
                    try
                        {
                            channels_.at(i)->connect(top_block_);
                        }
                    catch (const std::exception& e)
                        {
                            LOG(ERROR) << "Can't connect channel " << i << " internally: " << e.what();
                            top_block_->disconnect_all();
                            return 1;
                        }
                }
            else
                {
                    LOG(ERROR) << "Can't connect channel " << i << " internally";
                    help_hint_ += " * Check your configuration for Channel" + std::to_string(i) + " inner blocks.\n";
                    help_hint_ += "   Acquisition blocks documentation at https://gnss-sdr.org/docs/sp-blocks/acquisition/\n";
                    help_hint_ += "   Tracking blocks documentation at https://gnss-sdr.org/docs/sp-blocks/tracking/\n";
                    help_hint_ += "   Telemetry Decoder blocks documentation at https://gnss-sdr.org/docs/sp-blocks/telemetry-decoder/\n";
                    top_block_->disconnect_all();
                    return 1;
                }
        }
    DLOG(INFO) << "Channel blocks successfully connected to the top_block";
    return 0;
}


int GNSSFlowgraph::connect_observables()
{
    if (observables_ == nullptr)
        {
            help_hint_ += " * Check implementation name for the Observables block\n";
            help_hint_ += "   Observables block documentation at https://gnss-sdr.org/docs/sp-blocks/observables/\n";
            top_block_->disconnect_all();
            return 1;
        }
    try
        {
            observables_->connect(top_block_);
        }
    catch (const std::exception& e)
        {
            LOG(ERROR) << "Can't connect observables block internally: " << e.what();
            top_block_->disconnect_all();
            return 1;
        }
    DLOG(INFO) << "Observables block successfully connected to the top_block";
    return 0;
}


int GNSSFlowgraph::connect_pvt()
{
    if (pvt_ == nullptr)
        {
            help_hint_ += " * Check implementation name for the PVT block\n";
            help_hint_ += "   PVT block documentation at https://gnss-sdr.org/docs/sp-blocks/pvt/\n";
            top_block_->disconnect_all();
            return 1;
        }
    try
        {
            pvt_->connect(top_block_);
        }
    catch (const std::exception& e)
        {
            LOG(ERROR) << "Can't connect PVT block internally: " << e.what();
            top_block_->disconnect_all();
            return 1;
        }
    DLOG(INFO) << "PVT block successfully connected to the top_block";
    return 0;
}


int GNSSFlowgraph::connect_galileo_tow_map()
{
    try
        {
            for (int i = 0; i < channels_count_; i++)
                {
                    std::string sig = channels_.at(i)->get_signal().get_signal_str();
                    if (sig == "1B" || sig == "E6" || sig == "5X" || sig == "7X")
                        {
                            top_block_->msg_connect(channels_.at(i)->get_right_block(), pmt::mp("TOW_from_TLM"), galileo_tow_map_, pmt::mp("TOW_from_TLM"));
                            top_block_->msg_connect(galileo_tow_map_, pmt::mp("TOW_to_TLM"), channels_.at(i)->get_right_block(), pmt::mp("TOW_to_TLM"));
                        }
                }
        }
    catch (const std::exception& e)
        {
            LOG(ERROR) << "Can't connect The Galileo TOW map internally: " << e.what();
            top_block_->disconnect_all();
            return 1;
        }
    return 0;
}


int GNSSFlowgraph::connect_sample_counter()
{
    // connect the sample counter to the Signal Conditioner
    // connect the sample counter to Observables
    try
        {
            const double fs = static_cast<double>(configuration_->property("GNSS-SDR.internal_fs_sps", 0));
            if (fs == 0.0)
                {
                    LOG(WARNING) << "Set GNSS-SDR.internal_fs_sps in configuration file";
                    std::cout << "Set GNSS-SDR.internal_fs_sps in configuration file\n";
                    throw(std::invalid_argument("Set GNSS-SDR.internal_fs_sps in configuration"));
                }

            const int observable_interval_ms = configuration_->property("GNSS-SDR.observable_interval_ms", 20);
            ch_out_sample_counter_ = gnss_sdr_make_sample_counter(fs, observable_interval_ms, sig_conditioner_.at(0)->get_right_block()->output_signature()->sizeof_stream_item(0));
            top_block_->connect(sig_conditioner_.at(0)->get_right_block(), 0, ch_out_sample_counter_, 0);
            top_block_->connect(ch_out_sample_counter_, 0, observables_->get_left_block(), channels_count_);  // extra port for the sample counter pulse
        }
    catch (const std::exception& e)
        {
            LOG(ERROR) << "Can't connect sample counter: " << e.what();
            top_block_->disconnect_all();
            return 1;
        }
    DLOG(INFO) << "sample counter successfully connected to Signal Conditioner and Observables blocks";
    return 0;
}


#if ENABLE_FPGA
int GNSSFlowgraph::connect_fpga_sample_counter()
{
    // create a hardware-defined gnss_synchro pulse for the observables block
    try
        {
            const double fs = static_cast<double>(configuration_->property("GNSS-SDR.internal_fs_sps", 0));
            if (fs == 0.0)
                {
                    LOG(WARNING) << "Set GNSS-SDR.internal_fs_sps in configuration file";
                    std::cout << "Set GNSS-SDR.internal_fs_sps in configuration file\n";
                    throw(std::invalid_argument("Set GNSS-SDR.internal_fs_sps in configuration"));
                }
            const int observable_interval_ms = configuration_->property("GNSS-SDR.observable_interval_ms", 20);
            ch_out_fpga_sample_counter_ = gnss_sdr_make_fpga_sample_counter(fs, observable_interval_ms);
            top_block_->connect(ch_out_fpga_sample_counter_, 0, observables_->get_left_block(), channels_count_);  // extra port for the sample counter pulse
        }
    catch (const std::exception& e)
        {
            std::string reported_error(e.what());
            if (std::string::npos != reported_error.find(std::string("filesystem")))
                {
                    help_hint_ += " * The global configuration parameter GNSS-SDR.enable_FPGA is set to true,\n";
                    help_hint_ += "   but gnss-sdr does not appear to be executed in an FPGA-equipped platform.\n";
                }
            else
                {
                    LOG(ERROR) << reported_error;
                }
            top_block_->disconnect_all();
            return 1;
        }
    LOG(INFO) << "FPGA sample counter successfully connected";
    return 0;
}
#endif


int GNSSFlowgraph::connect_signal_sources_to_signal_conditioners()
{
    if (enable_fpga_offloading_)
        {
            help_hint_ += " * The global configuration parameter GNSS-SDR.enable_FPGA is set to true,\n";
            help_hint_ += "   but gnss-sdr was not compiled with the -DENABLE_FPGA=ON building option.\n";
            top_block_->disconnect_all();
            return 1;
        }
    unsigned int signal_conditioner_ID = 0;
    for (int i = 0; i < sources_count_; i++)
        {
            try
                {
                    auto& src = sig_source_.at(i);

                    // TODO: Remove this array implementation and create generic multistream connector
                    // (if a signal source has more than 1 stream, then connect it to the multistream signal conditioner)
                    if (src->implementation() == "Raw_Array_Signal_Source")
                        {
                            // Multichannel Array
                            std::cout << "ARRAY MODE\n";
                            for (int j = 0; j < GNSS_SDR_ARRAY_SIGNAL_CONDITIONER_CHANNELS; j++)
                                {
                                    std::cout << "connecting ch " << j << '\n';
                                    top_block_->connect(src->get_right_block(), j, sig_conditioner_.at(i)->get_left_block(), j);
                                }
                        }
                    else
                        {
                            auto RF_Channels = src->getRfChannels();

                            for (auto j = 0U; j < RF_Channels; ++j)
                                {
                                    // Connect the multichannel signal source to multiple signal conditioners
                                    // GNURADIO max_streams=-1 means infinite ports!
                                    size_t output_size = src->get_right_block()->output_signature()->sizeof_stream_item(0);
                                    size_t input_size = sig_conditioner_.at(signal_conditioner_ID)->get_left_block()->input_signature()->sizeof_stream_item(0);
                                    // Check configuration inconsistencies
                                    if (output_size != input_size)
                                        {
                                            help_hint_ += " * The Signal Source implementation " + src->implementation() + " has an output with a ";
                                            help_hint_ += src->role() + ".item_size of " + std::to_string(output_size);
                                            help_hint_ += " bytes, but it is connected to the Signal Conditioner implementation ";
                                            help_hint_ += sig_conditioner_.at(signal_conditioner_ID)->implementation() + " with input item size of " + std::to_string(input_size) + " bytes.\n";
                                            help_hint_ += "   Output ports must be connected to input ports with the same item size.\n";
                                            top_block_->disconnect_all();
                                            return 1;
                                        }

                                    if (src->get_right_block()->output_signature()->max_streams() > 1 or src->get_right_block()->output_signature()->max_streams() == -1)
                                        {
                                            if (sig_conditioner_.size() > signal_conditioner_ID)
                                                {
                                                    LOG(INFO) << "connecting sig_source_ " << i << " stream " << j << " to conditioner " << signal_conditioner_ID;
                                                    top_block_->connect(src->get_right_block(), j, sig_conditioner_.at(signal_conditioner_ID)->get_left_block(), 0);
                                                }
                                        }
                                    else
                                        {
                                            if (j == 0 || !src->get_right_block(j))
                                                {
                                                    // RF_channel 0 backward compatibility with single channel sources
                                                    LOG(INFO) << "connecting sig_source_ " << i << " stream " << 0 << " to conditioner " << signal_conditioner_ID;
                                                    top_block_->connect(src->get_right_block(), 0, sig_conditioner_.at(signal_conditioner_ID)->get_left_block(), 0);
                                                }
                                            else
                                                {
                                                    // Multiple channel sources using multiple output blocks of single channel (requires RF_channel selector in call)
                                                    LOG(INFO) << "connecting sig_source_ " << i << " stream " << j << " to conditioner " << signal_conditioner_ID;
                                                    top_block_->connect(src->get_right_block(j), 0, sig_conditioner_.at(signal_conditioner_ID)->get_left_block(), 0);
                                                }
                                        }
                                    signal_conditioner_ID++;
                                }
                        }
                }
            catch (const std::exception& e)
                {
                    LOG(ERROR) << "Can't connect SignalSource" << (i == 0 ? " " : (std::to_string(i) + " ")) << "to SignalConditioner" << (i == 0 ? " " : (std::to_string(i) + " ")) << ": " << e.what();
                    std::string reported_error(e.what());
                    if (std::string::npos != reported_error.find(std::string("itemsize mismatch")))
                        {
                            std::string replace_me("copy");
                            size_t pos = reported_error.find(replace_me);
                            while (pos != std::string::npos)
                                {
                                    size_t len = replace_me.length();
                                    reported_error.replace(pos, len, "Pass_Through");
                                    pos = reported_error.find(replace_me, pos + 1);
                                }
                            help_hint_ += " * The SignalSource output item size and the SignalConditioner input item size are mismatched\n";
                            help_hint_ += "   Reported error: " + reported_error + '\n';
                        }
                    top_block_->disconnect_all();
                    return 1;
                }
        }

    DLOG(INFO) << "Signal source(s) successfully connected to signal conditioner(s)";
    return 0;
}


int GNSSFlowgraph::connect_signal_conditioners_to_channels()
{
    for (int i = 0; i < channels_count_; i++)
        {
            int selected_signal_conditioner_ID = 0;
            const bool use_acq_resampler = configuration_->property("GNSS-SDR.use_acquisition_resampler", false);
            const uint32_t fs = configuration_->property("GNSS-SDR.internal_fs_sps", 0);

            try
                {
                    selected_signal_conditioner_ID = configuration_->property("Channels_" + channels_.at(i)->get_signal().get_signal_str() + ".RF_channel_ID", 0);
                    selected_signal_conditioner_ID = configuration_->property("Channel" + std::to_string(i) + ".RF_channel_ID", selected_signal_conditioner_ID);
                }
            catch (const std::exception& e)
                {
                    LOG(WARNING) << e.what();
                }
            try
                {
                    // Enable automatic resampler for the acquisition, if required
                    if (use_acq_resampler == true)
                        {
                            // create acquisition resamplers if required
                            double resampler_ratio = 1.0;
                            double acq_fs = fs;
                            // find the signal associated to this channel
                            switch (mapStringValues_[channels_.at(i)->get_signal().get_signal_str()])
                                {
                                case evGPS_1C:
                                    acq_fs = GPS_L1_CA_OPT_ACQ_FS_SPS;
                                    break;
                                case evGPS_2S:
                                    acq_fs = GPS_L2C_OPT_ACQ_FS_SPS;
                                    break;
                                case evGPS_L5:
                                    acq_fs = GPS_L5_OPT_ACQ_FS_SPS;
                                    break;
                                case evSBAS_1C:
                                    acq_fs = GPS_L1_CA_OPT_ACQ_FS_SPS;
                                    break;
                                case evGAL_1B:
                                    acq_fs = GALILEO_E1_OPT_ACQ_FS_SPS;
                                    break;
                                case evGAL_5X:
                                    acq_fs = GALILEO_E5A_OPT_ACQ_FS_SPS;
                                    break;
                                case evGAL_7X:
                                    acq_fs = GALILEO_E5B_OPT_ACQ_FS_SPS;
                                    break;
                                case evGAL_E6:
                                    acq_fs = GALILEO_E6_OPT_ACQ_FS_SPS;
                                    break;
                                case evGLO_1G:
                                case evGLO_2G:
                                case evBDS_B1:
                                case evBDS_B3:
                                    acq_fs = fs;
                                    break;
                                default:
                                    break;
                                }

                            if (acq_fs < fs)
                                {
                                    // check if the resampler is already created for the channel system/signal and for the specific RF Channel
                                    const std::string map_key = channels_.at(i)->get_signal().get_signal_str() + std::to_string(selected_signal_conditioner_ID);
                                    resampler_ratio = static_cast<double>(fs) / acq_fs;
                                    int decimation = floor(resampler_ratio);
                                    while (fs % decimation > 0)
                                        {
                                            decimation--;
                                        };
                                    const double acq_fs_decimated = static_cast<double>(fs) / static_cast<double>(decimation);

                                    if (decimation > 1)
                                        {
                                            // create a FIR low pass filter
                                            std::vector<float> taps = gr::filter::firdes::low_pass(1.0,
                                                fs,
                                                acq_fs_decimated / 2.1,
                                                acq_fs_decimated / 2);

                                            gr::basic_block_sptr fir_filter_ccf_ = gr::filter::fir_filter_ccf::make(decimation, taps);

                                            std::pair<std::map<std::string, gr::basic_block_sptr>::iterator, bool> ret;
                                            ret = acq_resamplers_.insert(std::pair<std::string, gr::basic_block_sptr>(map_key, fir_filter_ccf_));
                                            if (ret.second == true)
                                                {
                                                    top_block_->connect(sig_conditioner_.at(selected_signal_conditioner_ID)->get_right_block(), 0,
                                                        acq_resamplers_.at(map_key), 0);
                                                    LOG(INFO) << "Created "
                                                              << channels_.at(i)->get_signal().get_signal_str()
                                                              << " acquisition resampler for RF channel " << std::to_string(selected_signal_conditioner_ID) << " with " << taps.size() << " taps and decimation factor of " << decimation;
                                                }
                                            else
                                                {
                                                    LOG(INFO) << "Found existing "
                                                              << channels_.at(i)->get_signal().get_signal_str()
                                                              << " acquisition resampler for RF channel " << std::to_string(selected_signal_conditioner_ID) << " with " << taps.size() << " taps and decimation factor of " << decimation;
                                                }

                                            top_block_->connect(acq_resamplers_.at(map_key), 0,
                                                channels_.at(i)->get_left_block_acq(), 0);

                                            std::shared_ptr<Channel> channel_ptr = std::dynamic_pointer_cast<Channel>(channels_.at(i));
                                            channel_ptr->acquisition()->set_resampler_latency((taps.size() - 1) / 2);
                                        }
                                    else
                                        {
                                            LOG(INFO) << "Disabled acquisition resampler because the input sampling frequency is too low";
                                            // resampler not required!
                                            top_block_->connect(sig_conditioner_.at(selected_signal_conditioner_ID)->get_right_block(), 0,
                                                channels_.at(i)->get_left_block_acq(), 0);
                                        }
                                }
                            else
                                {
                                    LOG(INFO) << "Disabled acquisition resampler because the input sampling frequency is too low";
                                    top_block_->connect(sig_conditioner_.at(selected_signal_conditioner_ID)->get_right_block(), 0,
                                        channels_.at(i)->get_left_block_acq(), 0);
                                }
                        }
                    else
                        {
                            top_block_->connect(sig_conditioner_.at(selected_signal_conditioner_ID)->get_right_block(), 0,
                                channels_.at(i)->get_left_block_acq(), 0);
                        }
                    top_block_->connect(sig_conditioner_.at(selected_signal_conditioner_ID)->get_right_block(), 0,
                        channels_.at(i)->get_left_block_trk(), 0);
                }
            catch (const std::exception& e)
                {
                    LOG(ERROR) << "Can't connect signal conditioner " << selected_signal_conditioner_ID << " to channel " << i << ": " << e.what();
                    top_block_->disconnect_all();
                    return 1;
                }

            signal_conditioner_connected_.at(selected_signal_conditioner_ID) = true;  // annotate that this signal conditioner is connected
            DLOG(INFO) << "Signal conditioner " << selected_signal_conditioner_ID << " successfully connected to channel " << i;
        }
    return 0;
}


int GNSSFlowgraph::connect_channels_to_observables()
{
    for (int i = 0; i < channels_count_; i++)
        {
            try
                {
                    top_block_->connect(channels_.at(i)->get_right_block(), 0,
                        observables_->get_left_block(), i);
                }
            catch (const std::exception& e)
                {
                    LOG(ERROR) << "Can't connect channel " << i << " to observables: " << e.what();
                    top_block_->disconnect_all();
                    return 1;
                }
        }
    DLOG(INFO) << "Channel blocks successfully connected to the Observables block";
    return 0;
}


int GNSSFlowgraph::connect_observables_to_pvt()
{
    // Connect the observables output of each channel to the PVT block
    try
        {
            for (int i = 0; i < channels_count_; i++)
                {
                    top_block_->connect(observables_->get_right_block(), i, pvt_->get_left_block(), i);
                    top_block_->msg_connect(channels_.at(i)->get_right_block(), pmt::mp("telemetry"), pvt_->get_left_block(), pmt::mp("telemetry"));
                    // experimental Vector Tracking Loop (VTL) messages from PVT to Tracking blocks
                    // not supported by all tracking algorithms
                    pmt::pmt_t ports_in = channels_.at(i)->get_left_block_trk()->message_ports_in();
                    for (size_t n = 0; n < pmt::length(ports_in); n++)
                        {
                            // std::cout << "pmt: " << pmt::symbol_to_string(pmt::vector_ref(ports_in, n)) << "\n";
                            if (pmt::symbol_to_string(pmt::vector_ref(ports_in, n)) == "pvt_to_trk")
                                {
                                    top_block_->msg_connect(pvt_->get_left_block(), pmt::mp("pvt_to_trk"), channels_.at(i)->get_left_block_trk(), pmt::mp("pvt_to_trk"));
                                    LOG(INFO) << "pvt_to_trk message port connected in " << channels_.at(i)->implementation();
                                }
                        }
                }

            top_block_->msg_connect(observables_->get_right_block(), pmt::mp("status"), channels_status_, pmt::mp("status"));

            top_block_->msg_connect(pvt_->get_left_block(), pmt::mp("pvt_to_observables"), observables_->get_right_block(), pmt::mp("pvt_to_observables"));
            top_block_->msg_connect(pvt_->get_left_block(), pmt::mp("status"), channels_status_, pmt::mp("status"));
        }
    catch (const std::exception& e)
        {
            LOG(ERROR) << "Can't connect observables to PVT: " << e.what();
            top_block_->disconnect_all();
            return 1;
        }
    DLOG(INFO) << "Observables successfully connected to the PVT block";
    return 0;
}


int GNSSFlowgraph::connect_gnss_synchro_monitor()
{
    try
        {
            for (int i = 0; i < channels_count_; i++)
                {
                    top_block_->connect(observables_->get_right_block(), i, GnssSynchroMonitor_, i);
                }
        }
    catch (const std::exception& e)
        {
            LOG(ERROR) << "Can't connect observables to Monitor block: " << e.what();
            top_block_->disconnect_all();
            return 1;
        }
    DLOG(INFO) << "gnss_synchro_monitor successfully connected to Observables block";
    return 0;
}


int GNSSFlowgraph::connect_acquisition_monitor()
{
    try
        {
            for (int i = 0; i < channels_count_; i++)
                {
                    top_block_->connect(channels_.at(i)->get_right_block_acq(), 0, GnssSynchroAcquisitionMonitor_, i);
                }
        }
    catch (const std::exception& e)
        {
            LOG(ERROR) << "Can't connect acquisition intermediate outputs to Monitor block: " << e.what();
            top_block_->disconnect_all();
            return 1;
        }
    DLOG(INFO) << "acqusition_monitor successfully connected to Channel blocks";
    return 0;
}


int GNSSFlowgraph::connect_tracking_monitor()
{
    try
        {
            for (int i = 0; i < channels_count_; i++)
                {
                    top_block_->connect(channels_.at(i)->get_right_block_trk(), 0, GnssSynchroTrackingMonitor_, i);
                }
        }
    catch (const std::exception& e)
        {
            LOG(ERROR) << "Can't connect tracking outputs to Monitor block: " << e.what();
            top_block_->disconnect_all();
            return 1;
        }
    DLOG(INFO) << "tracking_monitor successfully connected to Channel blocks";
    return 0;
}


int GNSSFlowgraph::connect_navdata_monitor()
{
    try
        {
            for (int i = 0; i < channels_count_; i++)
                {
                    top_block_->msg_connect(channels_.at(i)->get_right_block(), pmt::mp("Nav_msg_from_TLM"), NavDataMonitor_, pmt::mp("Nav_msg_from_TLM"));
                }
            if (enable_e6_has_rx_)
                {
                    gal_e6_has_rx_->set_enable_navdata_monitor(true);
                    top_block_->msg_connect(gal_e6_has_rx_, pmt::mp("Nav_msg_from_TLM"), NavDataMonitor_, pmt::mp("Nav_msg_from_TLM"));
                }
        }
    catch (const std::exception& e)
        {
            LOG(ERROR) << "Can't connect TlM outputs to Monitor block: " << e.what();
            top_block_->disconnect_all();
            return 1;
        }
    DLOG(INFO) << "navdata monitor successfully connected to Channel blocks";
    return 0;
}


int GNSSFlowgraph::connect_monitors()
{
    // GNSS SYNCHRO MONITOR
    if (enable_monitor_)
        {
            if (connect_gnss_synchro_monitor() != 0)
                {
                    return 1;
                }
        }

    // GNSS SYNCHRO ACQUISITION MONITOR
    if (enable_acquisition_monitor_)
        {
            if (connect_acquisition_monitor() != 0)
                {
                    return 1;
                }
        }

    // GNSS SYNCHRO TRACKING MONITOR
    if (enable_tracking_monitor_)
        {
            if (connect_tracking_monitor() != 0)
                {
                    return 1;
                }
        }

    // NAVIGATION DATA MONITOR
    if (enable_navdata_monitor_)
        {
            if (connect_navdata_monitor() != 0)
                {
                    return 1;
                }
        }
    return 0;
}


int GNSSFlowgraph::connect_osnma()
{
    try
        {
            bool gal_e1_channels = false;
            for (int i = 0; i < channels_count_; i++)
                {
                    const std::string gnss_signal = channels_.at(i)->get_signal().get_signal_str();
                    switch (mapStringValues_[gnss_signal])
                        {
                        case evGAL_1B:
                            top_block_->msg_connect(channels_.at(i)->get_right_block(), pmt::mp("OSNMA_from_TLM"), osnma_rx_, pmt::mp("OSNMA_from_TLM"));
                            gal_e1_channels = true;
                            break;

                        default:
                            break;
                        }
                }

            if (gal_e1_channels == true)
                {
                    top_block_->msg_connect(osnma_rx_, pmt::mp("OSNMA_to_PVT"), pvt_->get_left_block(), pmt::mp("OSNMA_to_PVT"));
                }
        }
    catch (const std::exception& e)
        {
            LOG(ERROR) << "Can't connect Galileo OSNMA msg ports: " << e.what();
            top_block_->disconnect_all();
            return 1;
        }
    DLOG(INFO) << "Galileo OSNMA message ports connected";
    return 0;
}


int GNSSFlowgraph::connect_gal_e6_has()
{
    try
        {
            bool gal_e6_channels = false;
            for (int i = 0; i < channels_count_; i++)
                {
                    const std::string gnss_signal = channels_.at(i)->get_signal().get_signal_str();
                    switch (mapStringValues_[gnss_signal])
                        {
                        case evGAL_E6:
                            top_block_->msg_connect(channels_.at(i)->get_right_block(), pmt::mp("E6_HAS_from_TLM"), gal_e6_has_rx_, pmt::mp("E6_HAS_from_TLM"));
                            gal_e6_channels = true;
                            break;

                        default:
                            break;
                        }
                }

            if (gal_e6_channels == true)
                {
                    top_block_->msg_connect(gal_e6_has_rx_, pmt::mp("E6_HAS_to_PVT"), pvt_->get_left_block(), pmt::mp("E6_HAS_to_PVT"));
                }
        }
    catch (const std::exception& e)
        {
            LOG(ERROR) << "Can't connect Galileo E6 HAS msg ports: " << e.what();
            top_block_->disconnect_all();
            return 1;
        }
    DLOG(INFO) << "Galileo E6 HAS message ports connected";
    return 0;
}


void GNSSFlowgraph::check_signal_conditioners()
{
    // check for unconnected signal conditioners and connect null_sinks
    // in order to provide configuration flexibility to multiband files or signal sources
    for (size_t n = 0; n < sig_conditioner_.size(); n++)
        {
            if (signal_conditioner_connected_.at(n) == false)
                {
                    null_sinks_.push_back(gr::blocks::null_sink::make(sizeof(gr_complex)));
                    top_block_->connect(sig_conditioner_.at(n)->get_right_block(), 0,
                        null_sinks_.back(), 0);
                    LOG(INFO) << "Null sink connected to signal conditioner " << n << " due to lack of connection to any channel\n";
                }
        }
}


int GNSSFlowgraph::assign_channels()
{
    // Put channels fixed to a given satellite at the beginning of the vector, then the rest
    std::vector<unsigned int> vector_of_channels;
    for (int i = 0; i < channels_count_; i++)
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

    if (configuration_->property("Channels_1C.count", uint64_t(0ULL)) > available_GPS_1C_signals_.size() - 1)
        {
            help_hint_ += " * The number of GPS L1 channels is set to Channels_1C.count=" + std::to_string(configuration_->property("Channels_1C.count", 0));
            help_hint_ += " but the maximum number of available GPS satellites is " + std::to_string(available_GPS_1C_signals_.size()) + ".\n";
            help_hint_ += " Please set Channels_1C.count=" + std::to_string(available_GPS_1C_signals_.size() - 1) + " or lower in your configuration file.\n";
            top_block_->disconnect_all();
            return 1;
        }
    if (configuration_->property("Channels_2S.count", uint64_t(0ULL)) > available_GPS_2S_signals_.size() - 1)
        {
            help_hint_ += " * The number of GPS L2 channels is set to Channels_2S.count=" + std::to_string(configuration_->property("Channels_2S.count", 0));
            help_hint_ += " but the maximum number of available GPS satellites is " + std::to_string(available_GPS_2S_signals_.size()) + ".\n";
            help_hint_ += " Please set Channels_2S.count=" + std::to_string(available_GPS_2S_signals_.size() - 1) + " or lower in your configuration file.\n";
            top_block_->disconnect_all();
            return 1;
        }
    if (configuration_->property("Channels_L5.count", uint64_t(0ULL)) > available_GPS_L5_signals_.size() - 1)
        {
            help_hint_ += " * The number of GPS L5 channels is set to Channels_L5.count=" + std::to_string(configuration_->property("Channels_L5.count", 0));
            help_hint_ += " but the maximum number of available GPS satellites is " + std::to_string(available_GPS_L5_signals_.size()) + ".\n";
            help_hint_ += " Please set Channels_L5.count=" + std::to_string(available_GPS_L5_signals_.size() - 1) + " or lower in your configuration file.\n";
            top_block_->disconnect_all();
            return 1;
        }
    if (configuration_->property("Channels_1B.count", uint64_t(0ULL)) > available_GAL_1B_signals_.size() - 1)
        {
            help_hint_ += " * The number of Galileo E1 channels is set to Channels_1B.count=" + std::to_string(configuration_->property("Channels_1B.count", 0));
            help_hint_ += " but the maximum number of available Galileo satellites is " + std::to_string(available_GAL_1B_signals_.size()) + ".\n";
            help_hint_ += " Please set Channels_1B.count=" + std::to_string(available_GAL_1B_signals_.size()) + " or lower in your configuration file.\n";
            top_block_->disconnect_all();
            return 1;
        }
    if (configuration_->property("Channels_5X.count", uint64_t(0ULL)) > available_GAL_5X_signals_.size() - 1)
        {
            help_hint_ += " * The number of Galileo E5a channels is set to Channels_5X.count=" + std::to_string(configuration_->property("Channels_5X.count", 0));
            help_hint_ += " but the maximum number of available Galileo satellites is " + std::to_string(available_GAL_5X_signals_.size()) + ".\n";
            help_hint_ += " Please set Channels_5X.count=" + std::to_string(available_GAL_5X_signals_.size() - 1) + " or lower in your configuration file.\n";
            top_block_->disconnect_all();
            return 1;
        }
    if (configuration_->property("Channels_7X.count", uint64_t(0ULL)) > available_GAL_7X_signals_.size() - 1)
        {
            help_hint_ += " * The number of Galileo E5b channels is set to Channels_7X.count=" + std::to_string(configuration_->property("Channels_7X.count", 0));
            help_hint_ += " but the maximum number of available Galileo satellites is " + std::to_string(available_GAL_7X_signals_.size()) + ".\n";
            help_hint_ += " Please set Channels_7X.count=" + std::to_string(available_GAL_7X_signals_.size() - 1) + " or lower in your configuration file.\n";
            top_block_->disconnect_all();
            return 1;
        }
    if (configuration_->property("Channels_E6.count", uint64_t(0ULL)) > available_GAL_E6_signals_.size() - 1)
        {
            help_hint_ += " * The number of Galileo E6 channels is set to Channels_7X.count=" + std::to_string(configuration_->property("Channels_E6.count", 0));
            help_hint_ += " but the maximum number of available Galileo satellites is " + std::to_string(available_GAL_E6_signals_.size()) + ".\n";
            help_hint_ += " Please set Channels_E6.count=" + std::to_string(available_GAL_E6_signals_.size() - 1) + " or lower in your configuration file.\n";
            top_block_->disconnect_all();
            return 1;
        }
    if (configuration_->property("Channels_1G.count", uint64_t(0ULL)) > available_GLO_1G_signals_.size() + 7)  // satellites sharing same frequency number
        {
            help_hint_ += " * The number of Glonass L1 channels is set to Channels_1G.count=" + std::to_string(configuration_->property("Channels_1G.count", 0));
            help_hint_ += " but the maximum number of available Glonass satellites is " + std::to_string(available_GLO_1G_signals_.size() + 8) + ".\n";
            help_hint_ += " Please set Channels_1G.count=" + std::to_string(available_GLO_1G_signals_.size() + 7) + " or lower in your configuration file.\n";
            top_block_->disconnect_all();
            return 1;
        }
    if (configuration_->property("Channels_2G.count", uint64_t(0ULL)) > available_GLO_2G_signals_.size() + 7)  // satellites sharing same frequency number
        {
            help_hint_ += " * The number of Glonass L2 channels is set to Channels_2G.count=" + std::to_string(configuration_->property("Channels_2G.count", 0));
            help_hint_ += " but the maximum number of available Glonass satellites is " + std::to_string(available_GLO_2G_signals_.size() + 8) + ".\n";
            help_hint_ += " Please set Channels_2G.count=" + std::to_string(available_GLO_2G_signals_.size() + 7) + " or lower in your configuration file.\n";
            top_block_->disconnect_all();
            return 1;
        }
    if (configuration_->property("Channels_B1.count", uint64_t(0ULL)) > available_BDS_B1_signals_.size() - 1)
        {
            help_hint_ += " * The number of BeiDou B1 channels is set to Channels_B1.count=" + std::to_string(configuration_->property("Channels_B1.count", 0));
            help_hint_ += " but the maximum number of available BeiDou satellites is " + std::to_string(available_BDS_B1_signals_.size()) + ".\n";
            help_hint_ += " Please set Channels_B1.count=" + std::to_string(available_BDS_B1_signals_.size() - 1) + " or lower in your configuration file.\n";
            top_block_->disconnect_all();
            return 1;
        }
    if (configuration_->property("Channels_B3.count", uint64_t(0ULL)) > available_BDS_B3_signals_.size() - 1)
        {
            help_hint_ += " * The number of BeiDou B3 channels is set to Channels_B3.count=" + std::to_string(configuration_->property("Channels_B3.count", 0));
            help_hint_ += " but the maximum number of available BeiDou satellites is " + std::to_string(available_BDS_B3_signals_.size()) + ".\n";
            help_hint_ += " Please set Channels_B3.count=" + std::to_string(available_BDS_B3_signals_.size() - 1) + " or lower in your configuration file.\n";
            top_block_->disconnect_all();
            return 1;
        }

    // Assign satellites to channels in the initialization
    for (unsigned int& i : vector_of_channels)
        {
            const std::string gnss_signal_str = channels_.at(i)->get_signal().get_signal_str();  // use channel's implicit signal
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
                    bool assistance_available;
                    float estimated_doppler;
                    double RX_time;
                    bool is_primary_freq;
                    channels_.at(i)->set_signal(search_next_signal(gnss_signal_str, is_primary_freq, assistance_available, estimated_doppler, RX_time));
                }
            else
                {
                    std::string gnss_system_str;
                    Gnss_Signal gnss_signal;
                    switch (mapStringValues_[gnss_signal_str])
                        {
                        case evGPS_1C:
                            gnss_system_str = "GPS";
                            gnss_signal = Gnss_Signal(Gnss_Satellite(gnss_system_str, sat), gnss_signal_str);
                            available_GPS_1C_signals_.remove(gnss_signal);
                            break;

                        case evGPS_2S:
                            gnss_system_str = "GPS";
                            gnss_signal = Gnss_Signal(Gnss_Satellite(gnss_system_str, sat), gnss_signal_str);
                            available_GPS_2S_signals_.remove(gnss_signal);
                            break;

                        case evGPS_L5:
                            gnss_system_str = "GPS";
                            gnss_signal = Gnss_Signal(Gnss_Satellite(gnss_system_str, sat), gnss_signal_str);
                            available_GPS_L5_signals_.remove(gnss_signal);
                            break;

                        case evGAL_1B:
                            gnss_system_str = "Galileo";
                            gnss_signal = Gnss_Signal(Gnss_Satellite(gnss_system_str, sat), gnss_signal_str);
                            available_GAL_1B_signals_.remove(gnss_signal);
                            break;

                        case evGAL_5X:
                            gnss_system_str = "Galileo";
                            gnss_signal = Gnss_Signal(Gnss_Satellite(gnss_system_str, sat), gnss_signal_str);
                            available_GAL_5X_signals_.remove(gnss_signal);
                            break;

                        case evGAL_7X:
                            gnss_system_str = "Galileo";
                            gnss_signal = Gnss_Signal(Gnss_Satellite(gnss_system_str, sat), gnss_signal_str);
                            available_GAL_7X_signals_.remove(gnss_signal);
                            break;

                        case evGAL_E6:
                            gnss_system_str = "Galileo";
                            gnss_signal = Gnss_Signal(Gnss_Satellite(gnss_system_str, sat), gnss_signal_str);
                            available_GAL_E6_signals_.remove(gnss_signal);
                            break;

                        case evGLO_1G:
                            gnss_system_str = "Glonass";
                            gnss_signal = Gnss_Signal(Gnss_Satellite(gnss_system_str, sat), gnss_signal_str);
                            available_GLO_1G_signals_.remove(gnss_signal);
                            break;

                        case evGLO_2G:
                            gnss_system_str = "Glonass";
                            gnss_signal = Gnss_Signal(Gnss_Satellite(gnss_system_str, sat), gnss_signal_str);
                            available_GLO_2G_signals_.remove(gnss_signal);
                            break;

                        case evBDS_B1:
                            gnss_system_str = "Beidou";
                            gnss_signal = Gnss_Signal(Gnss_Satellite(gnss_system_str, sat), gnss_signal_str);
                            available_BDS_B1_signals_.remove(gnss_signal);
                            break;

                        case evBDS_B3:
                            gnss_system_str = "Beidou";
                            gnss_signal = Gnss_Signal(Gnss_Satellite(gnss_system_str, sat), gnss_signal_str);
                            available_BDS_B3_signals_.remove(gnss_signal);
                            break;

                        default:
                            LOG(ERROR) << "This should not happen :-(";
                            gnss_system_str = "GPS";
                            gnss_signal = Gnss_Signal(Gnss_Satellite(gnss_system_str, sat), gnss_signal_str);
                            available_GPS_1C_signals_.remove(gnss_signal);
                            break;
                        }

                    channels_.at(i)->set_signal(gnss_signal);
                }
        }
    return 0;
}


void GNSSFlowgraph::print_help()
{
    if (!help_hint_.empty())
        {
            std::cerr << "It seems that your configuration file is not well defined.\n";
            std::cerr << "A hint to fix your configuration file:\n";
            std::cerr << help_hint_;
        }
}


void GNSSFlowgraph::check_desktop_conf_in_fpga_env()
{
    int number_of_fpga_acq_channels = 0;
    for (int i = 0; i < channels_count_; i++)
        {
            if (channels_.at(i)->get_left_block_acq() == nullptr)
                {
                    number_of_fpga_acq_channels++;
                }
        }
    if (number_of_fpga_acq_channels != channels_count_)
        {
            help_hint_ += " * The Acquisition block implementation is not suitable for GNSS-SDR flowgraph with FPGA off-loading\n";
            help_hint_ += "   If you want to use this configuration in an environment without FPGA, please rebuild GNSS-SDR with CMake option '-DENABLE_FPGA=OFF'\n";
        }
}


bool GNSSFlowgraph::send_telemetry_msg(const pmt::pmt_t& msg)
{
    // Push ephemeris to PVT telemetry msg in port using a channel out port
    // it uses the first channel as a message producer (it is already connected to PVT)
    channels_.at(0)->get_right_block()->message_port_pub(pmt::mp("telemetry"), msg);
    return true;
}


void GNSSFlowgraph::push_back_signal(const Gnss_Signal& gs)
{
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

        case evGAL_7X:
            available_GAL_7X_signals_.remove(gs);
            available_GAL_7X_signals_.push_back(gs);
            break;

        case evGAL_E6:
            available_GAL_E6_signals_.remove(gs);
            available_GAL_E6_signals_.push_back(gs);
            break;

        case evGLO_1G:
            available_GLO_1G_signals_.remove(gs);
            available_GLO_1G_signals_.push_back(gs);
            break;

        case evGLO_2G:
            available_GLO_2G_signals_.remove(gs);
            available_GLO_2G_signals_.push_back(gs);
            break;

        case evBDS_B1:
            available_BDS_B1_signals_.remove(gs);
            available_BDS_B1_signals_.push_back(gs);
            break;

        case evBDS_B3:
            available_BDS_B3_signals_.remove(gs);
            available_BDS_B3_signals_.push_back(gs);
            break;

        default:
            LOG(ERROR) << "This should not happen :-(";
            break;
        }
}


void GNSSFlowgraph::remove_signal(const Gnss_Signal& gs)
{
    switch (mapStringValues_[gs.get_signal_str()])
        {
        case evGPS_1C:
            available_GPS_1C_signals_.remove(gs);
            break;

        case evGPS_2S:
            available_GPS_2S_signals_.remove(gs);
            break;

        case evGPS_L5:
            available_GPS_L5_signals_.remove(gs);
            break;

        case evGAL_1B:
            available_GAL_1B_signals_.remove(gs);
            break;

        case evGAL_5X:
            available_GAL_5X_signals_.remove(gs);
            break;

        case evGAL_7X:
            available_GAL_7X_signals_.remove(gs);
            break;

        case evGAL_E6:
            available_GAL_E6_signals_.remove(gs);
            break;

        case evGLO_1G:
            available_GLO_1G_signals_.remove(gs);
            break;

        case evGLO_2G:
            available_GLO_2G_signals_.remove(gs);
            break;

        case evBDS_B1:
            available_BDS_B1_signals_.remove(gs);
            break;

        case evBDS_B3:
            available_BDS_B3_signals_.remove(gs);
            break;

        default:
            LOG(ERROR) << "This should not happen :-(";
            break;
        }
}


// project Doppler from primary frequency to secondary frequency
double GNSSFlowgraph::project_doppler(const std::string& searched_signal, double primary_freq_doppler_hz)
{
    switch (mapStringValues_[searched_signal])
        {
        case evGPS_L5:
        case evGAL_5X:
            return (primary_freq_doppler_hz / FREQ1) * FREQ5;
            break;
        case evGAL_7X:
            return (primary_freq_doppler_hz / FREQ1) * FREQ7;
            break;
        case evGPS_2S:
            return (primary_freq_doppler_hz / FREQ1) * FREQ2;
            break;
        case evGAL_E6:
            return (primary_freq_doppler_hz / FREQ1) * FREQ6;
            break;
        default:
            return primary_freq_doppler_hz;
        }
}


void GNSSFlowgraph::acquisition_manager(unsigned int who)
{
    unsigned int current_channel;
    for (int i = 0; i < channels_count_; i++)
        {
            current_channel = (i + who + 1) % channels_count_;
            unsigned int sat_ = 0;
            try
                {
                    sat_ = configuration_->property("Channel" + std::to_string(current_channel) + ".satellite", 0);
                }
            catch (const std::exception& e)
                {
                    LOG(WARNING) << e.what();
                }
            if ((acq_channels_count_ < max_acq_channels_) && (channels_state_[current_channel] == 0))
                {
                    bool is_primary_freq = true;
                    bool assistance_available = false;
                    bool start_acquisition = false;
                    Gnss_Signal gnss_signal;
                    float estimated_doppler;
                    double RX_time;

                    if (sat_ == 0)
                        {
                            gnss_signal = search_next_signal(channels_[current_channel]->get_signal().get_signal_str(),
                                is_primary_freq,
                                assistance_available,
                                estimated_doppler,
                                RX_time);
                            channels_[current_channel]->set_signal(gnss_signal);
                            start_acquisition = is_primary_freq or assistance_available or !configuration_->property("GNSS-SDR.assist_dual_frequency_acq", multiband_);
                        }
                    else
                        {
                            channels_[current_channel]->set_signal(channels_[current_channel]->get_signal());
                            start_acquisition = true;
                        }

                    if (start_acquisition == true)
                        {
                            channels_state_[current_channel] = 1;
                            acq_channels_count_++;
                            DLOG(INFO) << "Channel " << current_channel
                                       << " Starting acquisition " << channels_[current_channel]->get_signal().get_satellite()
                                       << ", Signal " << channels_[current_channel]->get_signal().get_signal_str();
                            if (assistance_available == true and configuration_->property("GNSS-SDR.assist_dual_frequency_acq", multiband_))
                                {
                                    channels_[current_channel]->assist_acquisition_doppler(project_doppler(channels_[current_channel]->get_signal().get_signal_str(), estimated_doppler));
                                }
                            else
                                {
                                    // set Doppler center to 0 Hz
                                    channels_[current_channel]->assist_acquisition_doppler(0);
                                }
#if ENABLE_FPGA
                            if (enable_fpga_offloading_)
                                {
                                    // create a task for the FPGA such that it doesn't stop the flow
                                    std::thread tmp_thread(&ChannelInterface::start_acquisition, channels_[current_channel]);
                                    tmp_thread.detach();
                                }
                            else
                                {
                                    channels_[current_channel]->start_acquisition();
                                }
#else
                            channels_[current_channel]->start_acquisition();
#endif
                        }
                    else
                        {
                            push_back_signal(gnss_signal);
                            DLOG(INFO) << "Channel " << current_channel
                                       << " secondary frequency acquisition assistance not available in "
                                       << channels_[current_channel]->get_signal().get_satellite()
                                       << ", Signal " << channels_[current_channel]->get_signal().get_signal_str();
                        }
                }
            DLOG(INFO) << "Channel " << current_channel << " in state " << channels_state_[current_channel];
        }
}


/*
 * Applies an action to the flow graph
 *
 * \param[in] who   Who generated the action:
 *  -> 0-199 are the channels IDs
 *  -> 200 is the control_thread dispatched by the control_thread apply_action
 *  -> 300 is the telecommand system (TC) for receiver control
 *  -> 400 - 599 is the TC channel control for channels 0-199
 * \param[in] what  What is the action:
 * --- actions from channels ---
 * -> 0 acquisition failed
 * -> 1 acquisition successful
 * -> 2 tracking lost
 * --- actions from TC receiver control ---
 * -> 10 TC request standby mode
 * -> 11 TC request coldstart
 * -> 12 TC request hotstart
 * -> 13 TC request warmstart
 * --- actions from TC channel control ---
 * -> 20 stop channel
 * -> 21 start channel
 */
void GNSSFlowgraph::apply_action(unsigned int who, unsigned int what)
{
    // todo: the acquisition events are initiated from the acquisition success or failure queued msg. If the acquisition is disabled for non-assisted secondary freq channels, the engine stops..
    std::lock_guard<std::mutex> lock(signal_list_mutex_);
    DLOG(INFO) << "Received " << what << " from " << who;
    unsigned int sat = 0;
    Gnss_Signal gs;
    if (who < 200)
        {
            try
                {
                    sat = configuration_->property("Channel" + std::to_string(who) + ".satellite", 0);
                }
            catch (const std::exception& e)
                {
                    LOG(WARNING) << e.what();
                }
        }
    switch (what)
        {
        case 0:
            gs = channels_[who]->get_signal();
            DLOG(INFO) << "Channel " << who << " ACQ FAILED satellite " << gs.get_satellite() << ", Signal " << gs.get_signal_str();
            channels_state_[who] = 0;
            if (acq_channels_count_ > 0)
                {
                    acq_channels_count_--;
                }
            // call the acquisition manager to assign new satellite and start next acquisition (if required)
            acquisition_manager(who);
            // push back the old signal AFTER assigning a new one to avoid selecting the same signal
            if (sat == 0)
                {
                    push_back_signal(gs);
                }
            break;
        case 1:
            gs = channels_[who]->get_signal();
            DLOG(INFO) << "Channel " << who << " ACQ SUCCESS satellite " << gs.get_satellite();
            // If the satellite is in the list of available ones, remove it.
            remove_signal(gs);

            channels_state_[who] = 2;
            if (acq_channels_count_ > 0)
                {
                    acq_channels_count_--;
                }
            // call the acquisition manager to assign new satellite and start next acquisition (if required)
            acquisition_manager(who);
            break;

        case 2:
            gs = channels_[who]->get_signal();
            DLOG(INFO) << "Channel " << who << " TRK FAILED satellite " << gs.get_satellite();
            if (acq_channels_count_ < max_acq_channels_)
                {
                    // try to acquire the same satellite
                    channels_state_[who] = 1;
                    acq_channels_count_++;
                    DLOG(INFO) << "Channel " << who << " Starting acquisition " << gs.get_satellite() << ", Signal " << gs.get_signal_str();
                    channels_[who]->set_signal(channels_[who]->get_signal());

#if ENABLE_FPGA
                    if (enable_fpga_offloading_)
                        {
                            // create a task for the FPGA such that it doesn't stop the flow
                            std::thread tmp_thread(&ChannelInterface::start_acquisition, channels_[who]);
                            tmp_thread.detach();
                        }
                    else
                        {
                            channels_[who]->start_acquisition();
                        }
#else
                    channels_[who]->start_acquisition();
#endif
                }
            else
                {
                    channels_state_[who] = 0;
                    LOG(INFO) << "Channel " << who << " Idle state";
                    if (sat == 0)
                        {
                            push_back_signal(channels_[who]->get_signal());
                        }
                }
            break;
        case 10:  // request standby mode
            for (size_t n = 0; n < channels_.size(); n++)
                {
                    if (channels_state_[n] == 1 or channels_state_[n] == 2)  // channel in acquisition or in tracking
                        {
                            // recover the satellite assigned
                            Gnss_Signal gs_assigned = channels_[n]->get_signal();
                            push_back_signal(gs_assigned);

                            channels_[n]->stop_channel();  // stop the acquisition or tracking operation
                            channels_state_[n] = 0;
                        }
                }
            acq_channels_count_ = 0;  // all channels are in standby now and no new acquisition should be started
            break;
        default:
            break;
        }
}


void GNSSFlowgraph::priorize_satellites(const std::vector<std::pair<int, Gnss_Satellite>>& visible_satellites)
{
    size_t old_size;
    Gnss_Signal gs;
    for (const auto& visible_satellite : visible_satellites)
        {
            if (visible_satellite.second.get_system() == "GPS")
                {
                    gs = Gnss_Signal(visible_satellite.second, "1C");
                    old_size = available_GPS_1C_signals_.size();
                    available_GPS_1C_signals_.remove(gs);
                    if (old_size > available_GPS_1C_signals_.size())
                        {
                            available_GPS_1C_signals_.push_front(gs);
                        }

                    gs = Gnss_Signal(visible_satellite.second, "2S");
                    old_size = available_GPS_2S_signals_.size();
                    available_GPS_2S_signals_.remove(gs);
                    if (old_size > available_GPS_2S_signals_.size())
                        {
                            available_GPS_2S_signals_.push_front(gs);
                        }

                    gs = Gnss_Signal(visible_satellite.second, "L5");
                    old_size = available_GPS_L5_signals_.size();
                    available_GPS_L5_signals_.remove(gs);
                    if (old_size > available_GPS_L5_signals_.size())
                        {
                            available_GPS_L5_signals_.push_front(gs);
                        }
                }
            else if (visible_satellite.second.get_system() == "Galileo")
                {
                    gs = Gnss_Signal(visible_satellite.second, "1B");
                    old_size = available_GAL_1B_signals_.size();
                    available_GAL_1B_signals_.remove(gs);
                    if (old_size > available_GAL_1B_signals_.size())
                        {
                            available_GAL_1B_signals_.push_front(gs);
                        }

                    gs = Gnss_Signal(visible_satellite.second, "5X");
                    old_size = available_GAL_5X_signals_.size();
                    available_GAL_5X_signals_.remove(gs);
                    if (old_size > available_GAL_5X_signals_.size())
                        {
                            available_GAL_5X_signals_.push_front(gs);
                        }

                    gs = Gnss_Signal(visible_satellite.second, "7X");
                    old_size = available_GAL_7X_signals_.size();
                    available_GAL_7X_signals_.remove(gs);
                    if (old_size > available_GAL_7X_signals_.size())
                        {
                            available_GAL_7X_signals_.push_front(gs);
                        }

                    gs = Gnss_Signal(visible_satellite.second, "E6");
                    old_size = available_GAL_E6_signals_.size();
                    available_GAL_E6_signals_.remove(gs);
                    if (old_size > available_GAL_E6_signals_.size())
                        {
                            available_GAL_E6_signals_.push_front(gs);
                        }
                }
        }
}


void GNSSFlowgraph::set_configuration(const std::shared_ptr<ConfigurationInterface>& configuration)
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


#if ENABLE_FPGA
void GNSSFlowgraph::start_acquisition_helper()
{
    std::lock_guard<std::mutex> lock(signal_list_mutex_);
    for (int i = 0; i < channels_count_; i++)
        {
            if (channels_state_[i] == 1)
                {
#if ENABLE_FPGA
                    if (enable_fpga_offloading_)
                        {
                            // create a task for the FPGA such that it doesn't stop the flow
                            std::thread tmp_thread(&ChannelInterface::start_acquisition, channels_[i]);
                            tmp_thread.detach();
                        }
                    else
                        {
                            channels_.at(i)->start_acquisition();
                        }
#else
                    channels_.at(i)->start_acquisition();
#endif
                }
        }
}


void GNSSFlowgraph::perform_hw_reset()
{
    // a stop acquisition command causes the SW to reset the HW
    std::shared_ptr<Channel> channel_ptr;

    for (int i = 0; i < channels_count_; i++)
        {
            channel_ptr = std::dynamic_pointer_cast<Channel>(channels_.at(i));
            channel_ptr->tracking()->stop_tracking();
        }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    channel_ptr = std::dynamic_pointer_cast<Channel>(channels_.at(0));
    channel_ptr->acquisition()->stop_acquisition();
}
#endif


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

    std::set<unsigned int> available_beidou_prn = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
        11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
        30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
        50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63};

    std::string sv_list = configuration_->property("Galileo.prns", std::string(""));

    if (!sv_list.empty())
        {
            // Reset the available prns:
            std::set<unsigned int> tmp_set;
            boost::tokenizer<> tok(sv_list);
            std::transform(tok.begin(), tok.end(), std::inserter(tmp_set, tmp_set.begin()),
                boost::lexical_cast<unsigned int, std::string>);

            if (!tmp_set.empty())
                {
                    available_galileo_prn = std::move(tmp_set);
                }
        }

    std::string sv_banned = configuration_->property("GNSS-SDR.Galileo_banned_prns", std::string(""));
    if (!sv_banned.empty())
        {
            std::stringstream ss(sv_banned);
            while (ss.good())
                {
                    std::string substr;
                    std::getline(ss, substr, ',');
                    try
                        {
                            auto banned = static_cast<unsigned int>(std::stoi(substr));
                            available_galileo_prn.erase(banned);
                        }
                    catch (const std::invalid_argument& ia)
                        {
                            std::cerr << "Invalid argument at GNSS-SDR.Galileo_banned_prns configuration parameter: " << ia.what() << '\n';
                        }
                    catch (const std::out_of_range& oor)
                        {
                            std::cerr << "Out of range at GNSS-SDR.Galileo_banned_prns configuration parameter: " << oor.what() << '\n';
                        }
                }
        }

    sv_list = configuration_->property("GPS.prns", std::string(""));

    if (!sv_list.empty())
        {
            // Reset the available prns:
            std::set<unsigned int> tmp_set;
            boost::tokenizer<> tok(sv_list);
            std::transform(tok.begin(), tok.end(), std::inserter(tmp_set, tmp_set.begin()),
                boost::lexical_cast<unsigned int, std::string>);

            if (!tmp_set.empty())
                {
                    available_gps_prn = std::move(tmp_set);
                }
        }

    sv_banned = configuration_->property("GNSS-SDR.GPS_banned_prns", std::string(""));
    if (!sv_banned.empty())
        {
            std::stringstream ss(sv_banned);
            while (ss.good())
                {
                    std::string substr;
                    std::getline(ss, substr, ',');
                    try
                        {
                            auto banned = static_cast<unsigned int>(std::stoi(substr));
                            available_gps_prn.erase(banned);
                        }
                    catch (const std::invalid_argument& ia)
                        {
                            std::cerr << "Invalid argument at GNSS-SDR.GPS_banned_prns configuration parameter: " << ia.what() << '\n';
                        }
                    catch (const std::out_of_range& oor)
                        {
                            std::cerr << "Out of range at GNSS-SDR.GPS_banned_prns configuration parameter: " << oor.what() << '\n';
                        }
                }
        }

    sv_list = configuration_->property("SBAS.prns", std::string(""));

    if (!sv_list.empty())
        {
            // Reset the available prns:
            std::set<unsigned int> tmp_set;
            boost::tokenizer<> tok(sv_list);
            std::transform(tok.begin(), tok.end(), std::inserter(tmp_set, tmp_set.begin()),
                boost::lexical_cast<unsigned int, std::string>);

            if (!tmp_set.empty())
                {
                    available_sbas_prn = std::move(tmp_set);
                }
        }

    sv_banned = configuration_->property("GNSS-SDR.SBAS_banned_prns", std::string(""));
    if (!sv_banned.empty())
        {
            std::stringstream ss(sv_banned);
            while (ss.good())
                {
                    std::string substr;
                    std::getline(ss, substr, ',');
                    try
                        {
                            auto banned = static_cast<unsigned int>(std::stoi(substr));
                            available_sbas_prn.erase(banned);
                        }
                    catch (const std::invalid_argument& ia)
                        {
                            std::cerr << "Invalid argument at GNSS-SDR.SBAS_banned_prns configuration parameter: " << ia.what() << '\n';
                        }
                    catch (const std::out_of_range& oor)
                        {
                            std::cerr << "Out of range at GNSS-SDR.SBAS_banned_prns configuration parameter: " << oor.what() << '\n';
                        }
                }
        }

    sv_list = configuration_->property("Glonass.prns", std::string(""));

    if (!sv_list.empty())
        {
            // Reset the available prns:
            std::set<unsigned int> tmp_set;
            boost::tokenizer<> tok(sv_list);
            std::transform(tok.begin(), tok.end(), std::inserter(tmp_set, tmp_set.begin()),
                boost::lexical_cast<unsigned int, std::string>);

            if (!tmp_set.empty())
                {
                    available_glonass_prn = std::move(tmp_set);
                }
        }

    sv_banned = configuration_->property("GNSS-SDR.Glonass_banned_prns", std::string(""));
    if (!sv_banned.empty())
        {
            std::stringstream ss(sv_banned);
            while (ss.good())
                {
                    std::string substr;
                    std::getline(ss, substr, ',');
                    try
                        {
                            auto banned = static_cast<unsigned int>(std::stoi(substr));
                            available_glonass_prn.erase(banned);
                        }
                    catch (const std::invalid_argument& ia)
                        {
                            std::cerr << "Invalid argument at GNSS-SDR.Glonass_banned_prns configuration parameter: " << ia.what() << '\n';
                        }
                    catch (const std::out_of_range& oor)
                        {
                            std::cerr << "Out of range at GNSS-SDR.Glonass_banned_prns configuration parameter: " << oor.what() << '\n';
                        }
                }
        }

    sv_list = configuration_->property("Beidou.prns", std::string(""));

    if (!sv_list.empty())
        {
            // Reset the available prns:
            std::set<unsigned int> tmp_set;
            boost::tokenizer<> tok(sv_list);
            std::transform(tok.begin(), tok.end(), std::inserter(tmp_set, tmp_set.begin()),
                boost::lexical_cast<unsigned int, std::string>);

            if (!tmp_set.empty())
                {
                    available_beidou_prn = std::move(tmp_set);
                }
        }

    sv_banned = configuration_->property("GNSS-SDR.Beidou_banned_prns", std::string(""));
    if (!sv_banned.empty())
        {
            std::stringstream ss(sv_banned);
            while (ss.good())
                {
                    std::string substr;
                    std::getline(ss, substr, ',');
                    try
                        {
                            auto banned = static_cast<unsigned int>(std::stoi(substr));
                            available_beidou_prn.erase(banned);
                        }
                    catch (const std::invalid_argument& ia)
                        {
                            std::cerr << "Invalid argument at GNSS-SDR.Beidou_banned_prns configuration parameter: " << ia.what() << '\n';
                        }
                    catch (const std::out_of_range& oor)
                        {
                            std::cerr << "Out of range at GNSS-SDR.Beidou_banned_prns configuration parameter: " << oor.what() << '\n';
                        }
                }
        }

    if (configuration_->property("Channels_1C.count", 0) > 0)
        {
            // Loop to create GPS L1 C/A signals
            for (available_gnss_prn_iter = available_gps_prn.cbegin();
                 available_gnss_prn_iter != available_gps_prn.cend();
                 available_gnss_prn_iter++)
                {
                    available_GPS_1C_signals_.emplace_back(
                        Gnss_Satellite(std::string("GPS"), *available_gnss_prn_iter),
                        std::string("1C"));
                }
        }

    if (configuration_->property("Channels_2S.count", 0) > 0)
        {
            // Loop to create GPS L2C M signals
            for (available_gnss_prn_iter = available_gps_prn.cbegin();
                 available_gnss_prn_iter != available_gps_prn.cend();
                 available_gnss_prn_iter++)
                {
                    available_GPS_2S_signals_.emplace_back(
                        Gnss_Satellite(std::string("GPS"), *available_gnss_prn_iter),
                        std::string("2S"));
                }
        }

    if (configuration_->property("Channels_L5.count", 0) > 0)
        {
            // Loop to create GPS L5 signals
            for (available_gnss_prn_iter = available_gps_prn.cbegin();
                 available_gnss_prn_iter != available_gps_prn.cend();
                 available_gnss_prn_iter++)
                {
                    available_GPS_L5_signals_.emplace_back(
                        Gnss_Satellite(std::string("GPS"), *available_gnss_prn_iter),
                        std::string("L5"));
                }
        }

    if (configuration_->property("Channels_SBAS.count", 0) > 0)
        {
            // Loop to create SBAS L1 C/A signals
            for (available_gnss_prn_iter = available_sbas_prn.cbegin();
                 available_gnss_prn_iter != available_sbas_prn.cend();
                 available_gnss_prn_iter++)
                {
                    available_SBAS_1C_signals_.emplace_back(
                        Gnss_Satellite(std::string("SBAS"), *available_gnss_prn_iter),
                        std::string("1C"));
                }
        }

    if (configuration_->property("Channels_1B.count", 0) > 0)
        {
            // Loop to create the list of Galileo E1B signals
            for (available_gnss_prn_iter = available_galileo_prn.cbegin();
                 available_gnss_prn_iter != available_galileo_prn.cend();
                 available_gnss_prn_iter++)
                {
                    available_GAL_1B_signals_.emplace_back(
                        Gnss_Satellite(std::string("Galileo"), *available_gnss_prn_iter),
                        std::string("1B"));
                }
        }

    if (configuration_->property("Channels_5X.count", 0) > 0)
        {
            // Loop to create the list of Galileo E5a signals
            for (available_gnss_prn_iter = available_galileo_prn.cbegin();
                 available_gnss_prn_iter != available_galileo_prn.cend();
                 available_gnss_prn_iter++)
                {
                    available_GAL_5X_signals_.emplace_back(
                        Gnss_Satellite(std::string("Galileo"), *available_gnss_prn_iter),
                        std::string("5X"));
                }
        }

    if (configuration_->property("Channels_7X.count", 0) > 0)
        {
            // Loop to create the list of Galileo E5b signals
            for (available_gnss_prn_iter = available_galileo_prn.cbegin();
                 available_gnss_prn_iter != available_galileo_prn.cend();
                 available_gnss_prn_iter++)
                {
                    available_GAL_7X_signals_.emplace_back(
                        Gnss_Satellite(std::string("Galileo"), *available_gnss_prn_iter),
                        std::string("7X"));
                }
        }

    if (configuration_->property("Channels_E6.count", 0) > 0)
        {
            // Loop to create the list of Galileo E6 signals
            for (available_gnss_prn_iter = available_galileo_prn.cbegin();
                 available_gnss_prn_iter != available_galileo_prn.cend();
                 available_gnss_prn_iter++)
                {
                    available_GAL_E6_signals_.emplace_back(
                        Gnss_Satellite(std::string("Galileo"), *available_gnss_prn_iter),
                        std::string("E6"));
                }
        }

    if (configuration_->property("Channels_1G.count", 0) > 0)
        {
            // Loop to create the list of GLONASS L1 C/A signals
            for (available_gnss_prn_iter = available_glonass_prn.cbegin();
                 available_gnss_prn_iter != available_glonass_prn.cend();
                 available_gnss_prn_iter++)
                {
                    available_GLO_1G_signals_.emplace_back(
                        Gnss_Satellite(std::string("Glonass"), *available_gnss_prn_iter),
                        std::string("1G"));
                }
        }

    if (configuration_->property("Channels_2G.count", 0) > 0)
        {
            // Loop to create the list of GLONASS L2 C/A signals
            for (available_gnss_prn_iter = available_glonass_prn.cbegin();
                 available_gnss_prn_iter != available_glonass_prn.cend();
                 available_gnss_prn_iter++)
                {
                    available_GLO_2G_signals_.emplace_back(
                        Gnss_Satellite(std::string("Glonass"), *available_gnss_prn_iter),
                        std::string("2G"));
                }
        }

    if (configuration_->property("Channels_B1.count", 0) > 0)
        {
            // Loop to create the list of BeiDou B1C signals
            for (available_gnss_prn_iter = available_beidou_prn.cbegin();
                 available_gnss_prn_iter != available_beidou_prn.cend();
                 available_gnss_prn_iter++)
                {
                    available_BDS_B1_signals_.emplace_back(
                        Gnss_Satellite(std::string("Beidou"), *available_gnss_prn_iter),
                        std::string("B1"));
                }
        }

    if (configuration_->property("Channels_B3.count", 0) > 0)
        {
            // Loop to create the list of BeiDou B1C signals
            for (available_gnss_prn_iter = available_beidou_prn.cbegin();
                 available_gnss_prn_iter != available_beidou_prn.cend();
                 available_gnss_prn_iter++)
                {
                    available_BDS_B3_signals_.emplace_back(
                        Gnss_Satellite(std::string("Beidou"), *available_gnss_prn_iter),
                        std::string("B3"));
                }
        }
}


void GNSSFlowgraph::set_channels_state()
{
    std::lock_guard<std::mutex> lock(signal_list_mutex_);
    max_acq_channels_ = configuration_->property("Channels.in_acquisition", channels_count_);
    if (max_acq_channels_ > channels_count_)
        {
            max_acq_channels_ = channels_count_;
            LOG(WARNING) << "Channels_in_acquisition is bigger than number of channels. Variable acq_channels_count_ is set to " << channels_count_;
        }
    channels_state_.reserve(channels_count_);
    for (int i = 0; i < channels_count_; i++)
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


bool GNSSFlowgraph::is_multiband() const
{
    bool multiband = false;
    if (configuration_->property("Channels_1C.count", 0) > 0)
        {
            if (configuration_->property("Channels_2S.count", 0) > 0)
                {
                    multiband = true;
                }
            if (configuration_->property("Channels_L5.count", 0) > 0)
                {
                    multiband = true;
                }
        }
    if (configuration_->property("Channels_1B.count", 0) > 0)
        {
            if (configuration_->property("Channels_5X.count", 0) > 0)
                {
                    multiband = true;
                }
            if (configuration_->property("Channels_7X.count", 0) > 0)
                {
                    multiband = true;
                }
            if (configuration_->property("Channels_E6.count", 0) > 0)
                {
                    multiband = true;
                }
        }
    if (configuration_->property("Channels_1G.count", 0) > 0)
        {
            if (configuration_->property("Channels_2G.count", 0) > 0)
                {
                    multiband = true;
                }
        }
    if (configuration_->property("Channels_B1.count", 0) > 0)
        {
            if (configuration_->property("Channels_B3.count", 0) > 0)
                {
                    multiband = true;
                }
        }
    return multiband;
}


Gnss_Signal GNSSFlowgraph::search_next_signal(const std::string& searched_signal,
    bool& is_primary_frequency,
    bool& assistance_available,
    float& estimated_doppler,
    double& RX_time)
{
    is_primary_frequency = false;
    assistance_available = false;
    Gnss_Signal result{};
    bool found_signal = false;
    switch (mapStringValues_[searched_signal])
        {
        case evGPS_1C:
            // todo: assist the satellite selection with almanac and current PVT here (reuse priorize_satellite function used in control_thread)
            result = available_GPS_1C_signals_.front();
            available_GPS_1C_signals_.pop_front();
            available_GPS_1C_signals_.push_back(result);
            is_primary_frequency = true;  // indicate that the searched satellite signal belongs to "primary" link (L1, E1, B1, etc..)
            break;

        case evGPS_2S:
            if (configuration_->property("Channels_1C.count", 0) > 0)
                {
                    // 1. Get the current channel status map
                    std::map<int, std::shared_ptr<Gnss_Synchro>> current_channels_status = channels_status_->get_current_status_map();
                    // 2. search the currently tracked GPS L1 satellites and assist the GPS L2 acquisition if the satellite is not tracked on L2
                    for (auto& current_status : current_channels_status)
                        {
                            if (std::string(current_status.second->Signal) == "1C")
                                {
                                    std::list<Gnss_Signal>::iterator it2;
                                    it2 = std::find_if(std::begin(available_GPS_2S_signals_), std::end(available_GPS_2S_signals_),
                                        [&](Gnss_Signal const& sig) { return sig.get_satellite().get_PRN() == current_status.second->PRN; });

                                    if (it2 != available_GPS_2S_signals_.end())
                                        {
                                            estimated_doppler = static_cast<float>(current_status.second->Carrier_Doppler_hz);
                                            RX_time = current_status.second->RX_time;
                                            // 3. return the GPS L2 satellite and remove it from list
                                            result = *it2;
                                            available_GPS_2S_signals_.erase(it2);
                                            found_signal = true;
                                            assistance_available = true;
                                            break;
                                        }
                                }
                        }
                }
            // fallback: pick the front satellite because there is no tracked satellites in L1 to assist L2
            if (found_signal == false)
                {
                    result = available_GPS_2S_signals_.front();
                    available_GPS_2S_signals_.pop_front();
                    available_GPS_2S_signals_.push_back(result);
                }
            break;

        case evGPS_L5:
            if (configuration_->property("Channels_1C.count", 0) > 0)
                {
                    // 1. Get the current channel status map
                    std::map<int, std::shared_ptr<Gnss_Synchro>> current_channels_status = channels_status_->get_current_status_map();
                    // 2. search the currently tracked GPS L1 satellites and assist the GPS L5 acquisition if the satellite is not tracked on L5
                    for (auto& current_status : current_channels_status)
                        {
                            if (std::string(current_status.second->Signal) == "1C")
                                {
                                    std::list<Gnss_Signal>::iterator it2;
                                    it2 = std::find_if(std::begin(available_GPS_L5_signals_), std::end(available_GPS_L5_signals_),
                                        [&](Gnss_Signal const& sig) { return sig.get_satellite().get_PRN() == current_status.second->PRN; });

                                    if (it2 != available_GPS_L5_signals_.end())
                                        {
                                            estimated_doppler = static_cast<float>(current_status.second->Carrier_Doppler_hz);
                                            RX_time = current_status.second->RX_time;
                                            // std::cout << " Channel: " << it->first << " => Doppler: " << estimated_doppler << "[Hz] \n";
                                            // 3. return the GPS L5 satellite and remove it from list
                                            result = *it2;
                                            available_GPS_L5_signals_.erase(it2);
                                            found_signal = true;
                                            assistance_available = true;
                                            break;
                                        }
                                }
                        }
                }
            // fallback: pick the front satellite because there is no tracked satellites in L1 to assist L5
            if (found_signal == false)
                {
                    result = available_GPS_L5_signals_.front();
                    available_GPS_L5_signals_.pop_front();
                    available_GPS_L5_signals_.push_back(result);
                }
            break;

        case evGAL_1B:
            result = available_GAL_1B_signals_.front();
            available_GAL_1B_signals_.pop_front();
            available_GAL_1B_signals_.push_back(result);
            is_primary_frequency = true;  // indicate that the searched satellite signal belongs to "primary" link (L1, E1, B1, etc..)
            break;

        case evGAL_5X:
            if (configuration_->property("Channels_1B.count", 0) > 0)
                {
                    // 1. Get the current channel status map
                    std::map<int, std::shared_ptr<Gnss_Synchro>> current_channels_status = channels_status_->get_current_status_map();
                    // 2. search the currently tracked Galileo E1 satellites and assist the Galileo E5 acquisition if the satellite is not tracked on E5
                    for (auto& current_status : current_channels_status)
                        {
                            if (std::string(current_status.second->Signal) == "1B")
                                {
                                    std::list<Gnss_Signal>::iterator it2;
                                    it2 = std::find_if(std::begin(available_GAL_5X_signals_), std::end(available_GAL_5X_signals_),
                                        [&](Gnss_Signal const& sig) { return sig.get_satellite().get_PRN() == current_status.second->PRN; });

                                    if (it2 != available_GAL_5X_signals_.end())
                                        {
                                            estimated_doppler = static_cast<float>(current_status.second->Carrier_Doppler_hz);
                                            RX_time = current_status.second->RX_time;
                                            // std::cout << " Channel: " << it->first << " => Doppler: " << estimated_doppler << "[Hz] \n";
                                            // 3. return the Gal 5X satellite and remove it from list
                                            result = *it2;
                                            available_GAL_5X_signals_.erase(it2);
                                            found_signal = true;
                                            assistance_available = true;
                                            break;
                                        }
                                }
                        }
                }
            // fallback: pick the front satellite because there is no tracked satellites in E1 to assist E5
            if (found_signal == false)
                {
                    result = available_GAL_5X_signals_.front();
                    available_GAL_5X_signals_.pop_front();
                    available_GAL_5X_signals_.push_back(result);
                }
            break;

        case evGAL_7X:
            if (configuration_->property("Channels_1B.count", 0) > 0)
                {
                    // 1. Get the current channel status map
                    std::map<int, std::shared_ptr<Gnss_Synchro>> current_channels_status = channels_status_->get_current_status_map();
                    // 2. search the currently tracked Galileo E1 satellites and assist the Galileo E5 acquisition if the satellite is not tracked on E5
                    for (auto& current_status : current_channels_status)
                        {
                            if (std::string(current_status.second->Signal) == "1B")
                                {
                                    std::list<Gnss_Signal>::iterator it2;
                                    it2 = std::find_if(std::begin(available_GAL_7X_signals_), std::end(available_GAL_7X_signals_),
                                        [&](Gnss_Signal const& sig) { return sig.get_satellite().get_PRN() == current_status.second->PRN; });

                                    if (it2 != available_GAL_7X_signals_.end())
                                        {
                                            estimated_doppler = static_cast<float>(current_status.second->Carrier_Doppler_hz);
                                            RX_time = current_status.second->RX_time;
                                            // std::cout << " Channel: " << it->first << " => Doppler: " << estimated_doppler << "[Hz] \n";
                                            // 3. return the Gal 7X satellite and remove it from list
                                            result = *it2;
                                            available_GAL_7X_signals_.erase(it2);
                                            found_signal = true;
                                            assistance_available = true;
                                            break;
                                        }
                                }
                        }
                }
            // fallback: pick the front satellite because there is no tracked satellites in E1 to assist E5
            if (found_signal == false)
                {
                    result = available_GAL_7X_signals_.front();
                    available_GAL_7X_signals_.pop_front();
                    available_GAL_7X_signals_.push_back(result);
                }
            break;

        case evGAL_E6:
            if (configuration_->property("Channels_1B.count", 0) > 0)
                {
                    // 1. Get the current channel status map
                    std::map<int, std::shared_ptr<Gnss_Synchro>> current_channels_status = channels_status_->get_current_status_map();
                    // 2. search the currently tracked Galileo E1 satellites and assist the Galileo E5 acquisition if the satellite is not tracked on E5
                    for (auto& current_status : current_channels_status)
                        {
                            if (std::string(current_status.second->Signal) == "1B")
                                {
                                    std::list<Gnss_Signal>::iterator it2;
                                    it2 = std::find_if(std::begin(available_GAL_E6_signals_), std::end(available_GAL_E6_signals_),
                                        [&](Gnss_Signal const& sig) { return sig.get_satellite().get_PRN() == current_status.second->PRN; });

                                    if (it2 != available_GAL_E6_signals_.end())
                                        {
                                            estimated_doppler = static_cast<float>(current_status.second->Carrier_Doppler_hz);
                                            RX_time = current_status.second->RX_time;
                                            // std::cout << " Channel: " << it->first << " => Doppler: " << estimated_doppler << "[Hz] \n";
                                            // 3. return the Gal E6 satellite and remove it from list
                                            result = *it2;
                                            available_GAL_E6_signals_.erase(it2);
                                            found_signal = true;
                                            assistance_available = true;
                                            break;
                                        }
                                }
                        }
                }
            // fallback: pick the front satellite because there is no tracked satellites in E1 to assist E6
            if (found_signal == false)
                {
                    result = available_GAL_E6_signals_.front();
                    available_GAL_E6_signals_.pop_front();
                    available_GAL_E6_signals_.push_back(result);
                }
            break;

        case evGLO_1G:
            result = available_GLO_1G_signals_.front();
            available_GLO_1G_signals_.pop_front();
            available_GLO_1G_signals_.push_back(result);
            is_primary_frequency = true;  // indicate that the searched satellite signal belongs to "primary" link (L1, E1, B1, etc..)
            break;

        case evGLO_2G:
            result = available_GLO_2G_signals_.front();
            available_GLO_2G_signals_.pop_front();
            available_GLO_2G_signals_.push_back(result);
            break;

        case evBDS_B1:
            result = available_BDS_B1_signals_.front();
            available_BDS_B1_signals_.pop_front();
            available_BDS_B1_signals_.push_back(result);
            is_primary_frequency = true;  // indicate that the searched satellite signal belongs to "primary" link (L1, E1, B1, etc..)
            break;

        case evBDS_B3:
            result = available_BDS_B3_signals_.front();
            available_BDS_B3_signals_.pop_front();
            available_BDS_B3_signals_.push_back(result);
            break;

        default:
            LOG(ERROR) << "This should not happen :-(";
            if (!available_GPS_1C_signals_.empty())
                {
                    result = available_GPS_1C_signals_.front();
                    available_GPS_1C_signals_.pop_front();
                    available_GPS_1C_signals_.push_back(result);
                }
            break;
        }
    return result;
}
