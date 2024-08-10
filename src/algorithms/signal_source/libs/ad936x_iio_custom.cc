/*!
 * \file ad936x_iio_custom.cc
 * \brief A direct IIO custom front-end driver for the AD936x AD front-end family with special FPGA custom functionalities.
 * \author Javier Arribas, jarribas(at)cttc.es
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */
#include "ad936x_iio_custom.h"
#include "display.h"
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

ad936x_iio_custom::ad936x_iio_custom(int debug_level_, int log_level_)
{
    receive_samples = false;
    fpga_overflow = false;
    sample_rate_sps = 0;
    ctx = NULL;
    phy = NULL;
    dds_dev = NULL;
    stream_dev = NULL;
    debug_level = debug_level_;
    log_level = log_level_;
    PPS_mode = false;
    n_channels = 0;
}


ad936x_iio_custom::~ad936x_iio_custom()
{
    // disable TX
    if (phy != NULL) PlutoTxEnable(false);

    // Close device
    if (ctx != NULL) iio_context_destroy(ctx);
}


void ad936x_iio_custom::set_gnsstime_queue(std::shared_ptr<Concurrent_Queue<GnssTime>> queue)
{
    GnssTime_queue = std::move(queue);
}


void ad936x_iio_custom::set_pps_samplestamp_queue(std::shared_ptr<Concurrent_Queue<PpsSamplestamp>> queue)
{
    Pps_queue = std::move(queue);
}


bool ad936x_iio_custom::initialize_device(std::string pluto_device_uri, std::string board_type)
{
    // Find devices
    if (pluto_device_uri == "local")
        {
            struct iio_scan_context *tmp_ctx = iio_create_scan_context("usb", 0);
            if (!tmp_ctx)
                {
                    std::cout << "Unable to create scan context\n";
                    return false;
                }

            struct iio_context_info **info;
            int ret = iio_scan_context_get_info_list(tmp_ctx, &info);
            if (ret < 0)
                {
                    iio_scan_context_destroy(tmp_ctx);

                    std::cout << "Unable to scan for Pluto devices\n";
                    return false;
                }

            if (ret == 0)
                {
                    iio_context_info_list_free(info);
                    iio_scan_context_destroy(tmp_ctx);
                    std::cout << " No Pluto device found\n ";
                    return false;
                }

            if (ret > 1)
                {
                    std::cout << "More than one Pluto found:\n";

                    for (unsigned int i = 0; i < (size_t)ret; i++)
                        {
                            printf("\t%d: %s [%s]\n", i,
                                iio_context_info_get_description(info[i]),
                                iio_context_info_get_uri(info[i]));
                        }

                    std::cout << "We will use the first one.\n";
                }

            std::string uri(iio_context_info_get_uri(info[0]));
            iio_context_info_list_free(info);
            iio_scan_context_destroy(tmp_ctx);

            ctx = iio_create_context_from_uri(uri.c_str());
        }
    else
        {
            ctx = iio_create_context_from_uri(pluto_device_uri.c_str());
        }

    if (ctx == NULL)
        {
            std::cout << "Unable to create context from uri: " << pluto_device_uri << std::endl;
            return false;
        }

    phy = iio_context_find_device(ctx, "ad9361-phy");

    if (phy == NULL)
        {
            std::cout << "Unable to find ad9361-phy device from uri: " << pluto_device_uri << std::endl;
            return false;
        }

    if (board_type.compare("fmcomms5") == 0)
        {
            stream_dev = iio_context_find_device(ctx, "cf-ad9361-A");  // first ad9361 in FMCOMMS5
            if (stream_dev == NULL)
                {
                    std::cout << "Unable to find cf-ad9361-A device from uri: " << pluto_device_uri << std::endl;
                    return false;
                };
        }
    else
        {
            stream_dev = iio_context_find_device(ctx, "cf-ad9361-lpc");  // regular AD9361 stream device in single AD9361 boards
            if (stream_dev == NULL)
                {
                    std::cout << "Unable to find cf-ad9361-lpc device from uri: " << pluto_device_uri << std::endl;
                    return false;
                };
            dds_dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");  // DDS core for LO oscillator (external transverter operation)
            if (stream_dev == NULL)
                {
                    std::cout << "Warning: Unable to find cf-ad9361-dds-core-lpc device from uri: " << pluto_device_uri << std::endl;
                };
        }

    return true;
}


void ad936x_iio_custom::configure_params(struct iio_device *phy,
    const std::vector<std::string> &params)
{
    for (std::vector<std::string>::const_iterator it = params.begin();
         it != params.end(); ++it)
        {
            struct iio_channel *chn = NULL;
            const char *attr = NULL;
            size_t pos;
            int ret;

            pos = it->find('=');
            if (pos == std::string::npos)
                {
                    std::cerr << "Malformed line: " << *it << std::endl;
                    continue;
                }

            std::string key = it->substr(0, pos);
            std::string val = it->substr(pos + 1, std::string::npos);

            ret = iio_device_identify_filename(phy,
                key.c_str(), &chn, &attr);
            if (ret)
                {
                    std::cerr << "Parameter not recognized: "
                              << key << std::endl;
                    continue;
                }

            if (chn)
                ret = iio_channel_attr_write(chn,
                    attr, val.c_str());
            else if (iio_device_find_attr(phy, attr))
                ret = iio_device_attr_write(phy, attr, val.c_str());
            else
                ret = iio_device_debug_attr_write(phy,
                    attr, val.c_str());
            if (ret < 0)
                {
                    std::cerr << "Unable to write attribute " << key
                              << ": " << ret << std::endl;
                }
        }
}


void ad936x_iio_custom::set_params_rx(struct iio_device *phy_device,
    unsigned long long frequency,
    unsigned long samplerate, unsigned long bandwidth,
    bool quadrature, bool rfdc, bool bbdc,
    std::string gain1, double gain1_value,
    std::string gain2, double gain2_value,
    std::string port_select)
{
    std::vector<std::string> params;

    params.push_back("out_altvoltage0_RX_LO_frequency=" +
                     std::to_string(frequency));
    std::cout << params.back() << "\n";
    params.push_back("in_voltage_sampling_frequency=" +
                     std::to_string(samplerate));
    std::cout << params.back() << "\n";
    params.push_back("in_voltage_rf_bandwidth=" +
                     std::to_string(bandwidth));
    std::cout << params.back() << "\n";
    params.push_back("in_voltage_quadrature_tracking_en=" +
                     std::to_string(quadrature));
    params.push_back("in_voltage_rf_dc_offset_tracking_en=" +
                     std::to_string(rfdc));
    params.push_back("in_voltage_bb_dc_offset_tracking_en=" +
                     std::to_string(bbdc));
    params.push_back("in_voltage0_gain_control_mode=" +
                     gain1);
    params.push_back("in_voltage0_hardwaregain=" +
                     std::to_string(gain1_value));
    params.push_back("in_voltage1_gain_control_mode=" +
                     gain2);
    params.push_back("in_voltage1_hardwaregain=" +
                     std::to_string(gain2_value));
    params.push_back("in_voltage0_rf_port_select=" +
                     port_select);

    configure_params(phy_device, params);
}


bool ad936x_iio_custom::config_ad9361_dds(uint64_t freq_rf_tx_hz_,
    double tx_attenuation_db_,
    int64_t freq_dds_tx_hz_,
    double scale_dds_,
    double phase_dds_deg_,
    int channel)
{
    // TX stream config
    std::cout << "Start of AD9361 TX Oscillator DDS configuration\n";

    std::cout << "* Configuring AD9361 for streaming TX\n";

    // ENABLE DDS on TX1
    // Configure LO channel
    std::vector<std::string> params_phy;
    std::vector<std::string> params_dds;

    params_phy.push_back("out_altvoltage1_TX_LO_frequency=" +
                         std::to_string(freq_rf_tx_hz_));
    double disabled_tx_attenuation = 89.75;
    if (channel == 0)
        {
            params_phy.push_back("out_voltage0_hardwaregain=" +
                                 std::to_string(-tx_attenuation_db_));
            // disable the other TX
            params_phy.push_back("out_voltage1_hardwaregain=" +
                                 std::to_string(-disabled_tx_attenuation));

            configure_params(phy, params_phy);

            // DDS TX CH1 I (tone #1)
            params_dds.push_back("out_altvoltage0_TX1_I_F1_frequency=" +
                                 std::to_string(freq_dds_tx_hz_));
            params_dds.push_back("out_altvoltage0_TX1_I_F1_phase=" +
                                 std::to_string(phase_dds_deg_ * 1000.0));
            params_dds.push_back("out_altvoltage0_TX1_I_F1_scale=" +
                                 std::to_string(scale_dds_));
            params_dds.push_back("out_altvoltage0_TX1_I_F1_raw=1");
            // DDS TX CH1 Q (tone #1)
            params_dds.push_back("out_altvoltage2_TX1_Q_F1_frequency=" +
                                 std::to_string(freq_dds_tx_hz_));
            params_dds.push_back("out_altvoltage2_TX1_Q_F1_phase=" +
                                 std::to_string(phase_dds_deg_ * 1000.0 + 270000.0));
            params_dds.push_back("out_altvoltage2_TX1_Q_F1_scale=" +
                                 std::to_string(scale_dds_));
            params_dds.push_back("out_altvoltage2_TX1_Q_F1_raw=1");

            configure_params(dds_dev, params_dds);
        }
    else
        {
            params_phy.push_back("out_voltage1_hardwaregain=" +
                                 std::to_string(-tx_attenuation_db_));
            // disable the other TX
            params_phy.push_back("out_voltage0_hardwaregain=" +
                                 std::to_string(-disabled_tx_attenuation));

            configure_params(phy, params_phy);

            // DDS TX CH2 I (tone #1)
            params_dds.push_back("out_altvoltage4_TX2_I_F1_frequency=" +
                                 std::to_string(freq_dds_tx_hz_));
            params_dds.push_back("out_altvoltage4_TX2_I_F1_phase=" +
                                 std::to_string(phase_dds_deg_ * 1000.0));
            params_dds.push_back("out_altvoltage4_TX2_I_F1_scale=" +
                                 std::to_string(scale_dds_));
            params_dds.push_back("out_altvoltage4_TX2_I_F1_raw=1");
            // DDS TX CH2 Q (tone #1)
            params_dds.push_back("out_altvoltage6_TX2_Q_F1_frequency=" +
                                 std::to_string(freq_dds_tx_hz_));
            params_dds.push_back("out_altvoltage6_TX2_Q_F1_phase=" +
                                 std::to_string(phase_dds_deg_ * 1000.0 + 270000.0));
            params_dds.push_back("out_altvoltage6_TX2_Q_F1_scale=" +
                                 std::to_string(scale_dds_));
            params_dds.push_back("out_altvoltage6_TX2_Q_F1_raw=1");

            configure_params(dds_dev, params_dds);
        }

    return true;
}


bool ad936x_iio_custom::check_device()
{
    if (stream_dev != NULL)
        {
            return true;
        }
    else
        {
            return false;
        }
}


bool ad936x_iio_custom::get_iio_param(iio_device *dev, const std::string &param, std::string &value)
{
    struct iio_channel *chn = 0;
    const char *attr = 0;
    char valuestr[256];
    int ret;
    ssize_t nchars;

    ret = iio_device_identify_filename(dev, param.c_str(), &chn, &attr);

    if (ret)
        {
            std::cerr << "DevicePlutoSDR::get_param: Parameter not recognized: " << param << std::endl;
            return false;
        }

    if (chn)
        {
            nchars = iio_channel_attr_read(chn, attr, valuestr, 256);
        }
    else if (iio_device_find_attr(dev, attr))
        {
            nchars = iio_device_attr_read(dev, attr, valuestr, 256);
        }
    else
        {
            nchars = iio_device_debug_attr_read(dev, attr, valuestr, 256);
        }

    if (nchars < 0)
        {
            std::cerr << "DevicePlutoSDR::get_param: Unable to read attribute " << param << ": " << nchars << std::endl;
            return false;
        }
    else
        {
            value.assign(valuestr);
            return true;
        }
}


bool ad936x_iio_custom::read_die_temp(double &temp_c)
{
    std::string temp_mC_str;

    if (get_iio_param(phy, "in_temp0_input", temp_mC_str))
        {
            try
                {
                    uint32_t temp_mC = boost::lexical_cast<uint32_t>(temp_mC_str);
                    temp_c = static_cast<double>(temp_mC) / 1000.0;
                    if (temp_c > 120) temp_c = -1;
                    return true;
                }
            catch (const boost::bad_lexical_cast &e)
                {
                    std::cerr << "PlutoSDRDevice::getTemp: bad conversion to numeric" << std::endl;
                    return false;
                }
        }
    else
        {
            return false;
        }
}


bool ad936x_iio_custom::init_config_ad9361_rx(long long bandwidth_,
    long long sample_rate_,
    long long freq_,
    std::string rf_port_select_,
    std::string rf_filter,
    std::string gain_mode_rx0_,
    std::string gain_mode_rx1_,
    double rf_gain_rx0_,
    double rf_gain_rx1_,
    bool enable_ch0,
    bool enable_ch1,
    long long freq_2ch,
    double lo_attenuation_db_,
    bool high_side_lo_,
    int tx_lo_channel_)
{
    if (check_device() == false) return false;

    bool no_errors = true;
    std::cout << "Configuring phy device parameters...\n";
    int ret;
    if (rf_filter.compare("Disabled") == 0)
        {
            std::cout << "LNA Filter switch is disabled.\n";
        }
    else if (rf_filter.compare("Auto") == 0)
        {
            std::cout << "Selecting LNA RF filter based on the selected RF frequency... \n";
            if (freq_ == 1575420000)
                {
                    if (select_rf_filter("E1") == true)
                        {
                            std::cout << "LNA RF filter board switch set to E1\n";
                        }
                    else
                        {
                            std::cout << "Problem setting LNA RF filter switch value\n";
                        }
                }
            else
                {
                    if (select_rf_filter("E5E6") == true)
                        {
                            std::cout << "LNA RF filter board switch set to E5E6\n";
                        }
                    else
                        {
                            std::cout << "Problem setting LNA RF filter switch value\n";
                        }
                }
        }
    else
        {
            if (select_rf_filter(rf_filter) == true)
                {
                    std::cout << "LNA RF filter board switch set to " << rf_filter << "\n";
                }
            else
                {
                    std::cout << "Problem setting LNA RF filter switch value\n";
                }
        }

    std::vector<std::string> params;
    // Configure RX LO channel (NOTICE that altvoltage0 is the RX LO oscillator!, altvoltage1 is the TX oscillator)
    params.push_back("out_altvoltage0_RX_LO_frequency=" +
                     std::to_string(freq_));

    sample_rate_sps = sample_rate_;

    params.push_back("in_voltage_sampling_frequency=" +
                     std::to_string(sample_rate_));

    params.push_back("out_voltage_sampling_frequency=" +
                     std::to_string(sample_rate_));

    params.push_back("in_voltage_rf_bandwidth=" +
                     std::to_string(bandwidth_));

    params.push_back("out_voltage_rf_bandwidth=" +
                     std::to_string(bandwidth_));

    params.push_back("in_voltage_quadrature_tracking_en=1");
    params.push_back("in_voltage_rf_dc_offset_tracking_en=1");
    params.push_back("in_voltage_bb_dc_offset_tracking_en=1");

    configure_params(phy, params);

    ret = iio_device_attr_write(phy, "trx_rate_governor", "nominal");
    if (ret < 0)
        {
            std::cerr << "Failed to set trx_rate_governor: " << ret << std::endl;
            no_errors = false;
        }
    ret = iio_device_attr_write(phy, "ensm_mode", "fdd");
    if (ret < 0)
        {
            std::cerr << "Failed to set ensm_mode: " << ret << std::endl;
            no_errors = false;
        }
    ret = iio_device_attr_write(phy, "calib_mode", "auto");
    if (ret < 0)
        {
            std::cerr << "Failed to set calib_mode: " << ret << std::endl;
            no_errors = false;
        }

    if (enable_ch1 == true and enable_ch0 == true and freq_ != freq_2ch)
        {
            std::cout << "Two channels enabled with different frequencies, enabling the external RF transverter board:\n";
            long long int lo_freq_hz = 0;
            if (high_side_lo_ == false)
                {
                    std::cout << "Using LOW SIDE Local Oscillator (F_RF > F_LO)\n";
                    lo_freq_hz = freq_2ch - freq_;
                    if (lo_freq_hz < 0)
                        {
                            std::cout << "Configuration problem: 2nd channel frequency is " << freq_2ch << " [Hz], must be higher than main channel (" << freq_ << " [Hz])\n";
                            return false;
                        }
                }
            else
                {
                    std::cout << "Using HIGH SIDE Local Oscillator (F_RF < F_LO), consider baseband spectrum inversion.\n";
                    lo_freq_hz = freq_2ch + freq_;
                    if (lo_freq_hz < 0)
                        {
                            std::cout << "Configuration problem: 2nd channel frequency is " << freq_2ch << " [Hz], must be higher than main channel (" << freq_ << " [Hz])\n";
                            return false;
                        }
                }

            std::cout << "Configuring DDS Local Oscillator generation. LO Freq. is " << lo_freq_hz << " [Hz]\n";
            PlutoTxEnable(true);
            config_ad9361_dds(lo_freq_hz,
                lo_attenuation_db_,
                0,
                0.9,
                0,
                tx_lo_channel_);
            std::cout << "Configuring DDS Local Oscillator generation DONE\n";
        }
    else
        {
            PlutoTxEnable(false);  // power down the TX LO to reduce interferences
        }

    int set_filter_ret = ad9361_set_bb_rate(phy, sample_rate_sps);
    if (set_filter_ret != 0)
        {
            std::cout << "Warning: Unable to set AD936x ad9361_set_bb_rate parameters!\n";
        }

    //    int set_filter_ret = ad9361_set_bb_rate_custom_filter_auto(phy, sample_rate_sps);
    //    if (set_filter_ret != 0)
    //        {
    //            std::cout << "Warning: Unable to set AD936x RX filter parameters!\n";
    //        }

    // testing: set manual RX filter chain
    //    unsigned long RX_analog_bb_lpf_stop_hz = 1000000;
    //    unsigned long TX_analog_bb_lpf_stop_hz = 1000000;
    //
    //    unsigned long FIR_lpf_passband_hz = 1000000;
    //    unsigned long FIR_lpf_stopband_hz = 1200000;
    //    int set_filter_ret = ad9361_set_bb_rate_custom_filter_manual(phy,
    //        sample_rate_sps,
    //        FIR_lpf_passband_hz,
    //        FIR_lpf_stopband_hz,
    //        RX_analog_bb_lpf_stop_hz,
    //        TX_analog_bb_lpf_stop_hz);
    //    if (set_filter_ret != 0)
    //        {
    //            std::cout << "Warning: Unable to set AD936x RX filter parameters!\n";
    //        }
    n_channels = 0;
    if (enable_ch0 == true)
        {
            n_channels++;
            std::cout << "* Get AD9361 Phy RX channel 0...\n";
            std::stringstream name;
            name.str("");
            name << "voltage";
            name << 0;
            struct iio_channel *phy_ch;
            phy_ch = iio_device_find_channel(phy, name.str().c_str(), false);  // false means RX
            if (!phy_ch)
                {
                    std::cerr << "Could not find AD9361 phy channel: " << name.str() << "\n";
                    no_errors = false;
                }
            else
                {
                    ret = iio_channel_attr_write(phy_ch, "rf_port_select", rf_port_select_.c_str());
                    if (ret < 0)
                        {
                            std::cerr << "Warning: rf_port_select write returned: " << ret << "\n";
                            no_errors = false;
                        }
                    //            ret = iio_channel_attr_write_longlong(phy_ch, "rf_bandwidth", bandwidth_);
                    //            if (ret < 0)
                    //                {
                    //                    std::cerr << "Warning: rf_bandwidth write returned: " << ret << "\n";
                    //                    no_errors = false;
                    //                }
                    long long set_rf_bw;
                    ret = iio_channel_attr_read_longlong(phy_ch, "rf_bandwidth", &set_rf_bw);
                    if (ret < 0)
                        {
                            std::cerr << "Warning: rf_bandwidth read returned: " << ret << "\n";
                            no_errors = false;
                        }
                    else
                        {
                            std::cerr << "Info: rf_bandwidth read returned: " << set_rf_bw << " Hz \n";
                        }

                    if (setRXGain(0, gain_mode_rx0_, rf_gain_rx0_) == false)
                        {
                            std::cerr << "Info: setRXGain read returned false \n";
                            no_errors = false;
                        }
                }
        }

    if (enable_ch1 == true)
        {
            n_channels++;
            std::cout << "* Get AD9361 Phy RX channel 1...\n";
            std::stringstream name;
            name.str("");
            name << "voltage";
            name << 1;
            struct iio_channel *phy_ch;
            phy_ch = iio_device_find_channel(phy, name.str().c_str(), false);  // false means RX
            if (!phy_ch)
                {
                    std::cerr << "Could not find AD9361 phy channel: " << name.str() << "\n";
                    no_errors = false;
                }
            else
                {
                    ret = iio_channel_attr_write(phy_ch, "rf_port_select", rf_port_select_.c_str());
                    if (ret < 0)
                        {
                            std::cerr << "Warning: rf_port_select write returned: " << ret << "\n";
                            no_errors = false;
                        }
                    //            ret = iio_channel_attr_write_longlong(phy_ch, "rf_bandwidth", bandwidth_);
                    //            if (ret < 0)
                    //                {
                    //                    std::cerr << "Warning: rf_bandwidth write returned: " << ret << "\n";
                    //                    no_errors = false;
                    //                }
                    long long set_rf_bw;
                    ret = iio_channel_attr_read_longlong(phy_ch, "rf_bandwidth", &set_rf_bw);
                    if (ret < 0)
                        {
                            std::cerr << "Warning: rf_bandwidth read returned: " << ret << "\n";
                            no_errors = false;
                        }
                    else
                        {
                            std::cerr << "Info: rf_bandwidth read returned: " << set_rf_bw << " Hz \n";
                        }

                    if (setRXGain(1, gain_mode_rx1_, rf_gain_rx1_) == false)
                        {
                            std::cerr << "Info: setRXGain read returned false \n";
                            no_errors = false;
                        }
                }
        }

    std::cout << "AD936x Front-end configuration summary: \n";
    std::cout << "RF frequency tuned in AD936x: " << freq_ << " [Hz]\n";
    std::cout << "Baseband sampling frequency: " << sample_rate_sps << " [SPS]\n";
    std::cout << "RX chain gain: " << rf_gain_rx0_ << " [dB][only valid in manual mode]\n";
    std::cout << "RX chain gain mode: " << gain_mode_rx0_ << "\n";
    //    std::cout << "Analog baseband LPF stop frequency: " << RX_analog_bb_lpf_stop_hz << " [Hz]\n";
    //    std::cout << "Digital baseband LPF FIR passband frequency: " << FIR_lpf_passband_hz << " [Hz]\n";
    //    std::cout << "Digital baseband LPF FIR stopband frequency: " << FIR_lpf_stopband_hz << " [Hz]\n";
    std::cout << "End of AD9361 RX configuration.\n";
    return no_errors;
}


bool ad936x_iio_custom::set_rx_frequency(long long freq_hz)
{
    if (check_device() == false) return false;
    // Configure RX LO channel (NOTICE that altvoltage0 is the RX LO oscillator!, altvoltage1 is the TX oscillator)
    struct iio_channel *lo_ch;

    lo_ch = iio_device_find_channel(phy, "altvoltage0", true);
    if (!lo_ch)
        {
            std::cerr << "Could not find AD9361 RX LO channel altvoltage0\n";
            return false;
        }
    int ret;
    ret = iio_channel_attr_write_longlong(lo_ch, "frequency", freq_hz);
    if (ret < 0)
        {
            std::cerr << "Warning: RX LO frequency write returned: " << ret << "\n";
            return false;
        }
    return true;
}


bool ad936x_iio_custom::get_rx_frequency(long long &freq_hz)
{
    if (check_device() == false) return false;
    // Configure RX LO channel (NOTICE that altvoltage0 is the RX LO oscillator!, altvoltage1 is the TX oscillator)
    struct iio_channel *lo_ch;

    lo_ch = iio_device_find_channel(phy, "altvoltage0", true);
    if (!lo_ch)
        {
            std::cerr << "Could not find AD9361 RX LO channel altvoltage0\n";
            return false;
        }
    int ret;
    ret = iio_channel_attr_read_longlong(lo_ch, "frequency", &freq_hz);
    if (ret < 0)
        {
            std::cerr << "Warning: RX LO frequency read returned: " << ret << "\n";
            return false;
        }
    return true;
}


bool ad936x_iio_custom::setRXGain(int ch_num, std::string gain_mode, double gain_dB)
{
    if (check_device() == false) return false;
    std::vector<std::string> params;
    if (ch_num == 0)
        {
            params.clear();
            params.push_back("in_voltage0_gain_control_mode=" + gain_mode);
            if (gain_mode == "manual")
                {
                    params.push_back("in_voltage0_hardwaregain=" + std::to_string(gain_dB));
                }
            configure_params(phy, params);
            return true;
        }
    else if (ch_num == 1)
        {
            params.clear();
            params.push_back("in_voltage1_gain_control_mode=" + gain_mode);
            if (gain_mode == "manual")
                {
                    params.push_back("in_voltage1_hardwaregain=" + std::to_string(gain_dB));
                }
            configure_params(phy, params);
            return true;
        }
    else
        {
            return false;
        }
}


double ad936x_iio_custom::get_rx_gain(int ch_num)
{
    if (check_device() == false) return -1;
    double gain_dB;  // gain in dB
    int ret = 0;
    if (ch_num == 0)
        {
            ret = iio_device_attr_read_double(phy, "in_voltage0_hardwaregain", &gain_dB);
            if (ret < 0)
                {
                    std::cerr << "Failed to read in_voltage0_hardwaregain: " << ret << std::endl;
                    return -1.0;
                }
        }
    else if (ch_num == 1)
        {
            ret = iio_device_attr_read_double(phy, "in_voltage1_hardwaregain", &gain_dB);
            if (ret < 0)
                {
                    std::cerr << "Failed to read in_voltage1_hardwaregain: " << ret << std::endl;
                    return -1.0;
                }
        }
    else
        {
            return -1.0;
        }
    return gain_dB;
}


bool ad936x_iio_custom::calibrate([[maybe_unused]] int ch, [[maybe_unused]] double bw_hz)
{
    if (check_device() == false) return false;
    // todo
    return true;
}


void ad936x_iio_custom::monitor_thread_fn()
{
    uint32_t val;
    int ret;

    /* Give the main thread a moment to start the DMA */
    sleep(1);

    /* Clear all status bits */
    ret = iio_device_reg_write(stream_dev, 0x80000088, 0x6);
    if (ret)
        {
            fprintf(stderr, "Failed to clearn DMA status register: %s\n",
                strerror(-ret));
        }

    while (receive_samples)
        {
            ret = iio_device_reg_read(stream_dev, 0x80000088, &val);
            if (ret)
                {
                    fprintf(stderr, "Failed to read status register: %s\n",
                        strerror(-ret));
                    continue;
                }

            // if (device_is_tx) {
            // if (val & 1)
            // fprintf(stderr, "Underflow detected\n");
            // } else {
            if (val & 4)
                {
                    std::cout
                        << TEXT_BOLD_RED
                        << "WARNING: IIO status register reported overflow!\n";
                    LOG(WARNING) << "WARNING: IIO status register reported overflow!";
                }

            /* Clear bits */
            if (val)
                {
                    ret = iio_device_reg_write(stream_dev, 0x80000088, val);
                    if (ret)
                        fprintf(stderr, "Failed to clearn DMA status register: %s\n",
                            strerror(-ret));
                }
            sleep(1);
        }
    return;
}


void ad936x_iio_custom::stop_record()
{
    receive_samples = false;

    if (capture_samples_thread.joinable() == true)
        {
            std::cout << "Joining sample cature thread...\n";
            capture_samples_thread.join();
        }

    if (overflow_monitor_thread.joinable() == true)
        {
            std::cout << "Joining overflow monitor thread...\n";
            overflow_monitor_thread.join();
        }

    if (capture_time_thread.joinable() == true)
        {
            std::cout << "Joining time cature thread...\n";
            capture_time_thread.join();
        }
}


void ad936x_iio_custom::PlutoTxEnable(bool txon)
{
    if (check_device())
        {
            int ret;
            if (txon == false)
                {
                    ret = iio_channel_attr_write_bool(iio_device_find_channel(phy, "altvoltage1", true), "powerdown", true);  // turn off TX LO
                    if (ret < 0)
                        {
                            std::cerr << "Failed to write altvoltage1 powerdown: " << ret << std::endl;
                        }
                }
            else
                {
                    ret = iio_channel_attr_write_bool(iio_device_find_channel(phy, "altvoltage1", true), "powerdown", false);  // turn on TX LO
                    if (ret < 0)
                        {
                            std::cerr << "Failed to write altvoltage1 powerdown: " << ret << std::endl;
                        }
                }
        }
}


void ad936x_iio_custom::setPlutoGpo(int p)
{
    char pins[11];
    snprintf(pins, sizeof(pins), "0x27 0x%x0", p);  // direct access to AD9361 registers... WARNING!
    pins[9] = 0;
    int ret;
    // std::cout << "send: " << pins << " \n";
    if (check_device())
        {
            ret = iio_device_debug_attr_write(phy, "direct_reg_access", pins);
            if (ret < 0)
                {
                    std::cerr << "Failed to write direct_reg_access: " << ret << std::endl;
                }
        }
}


bool ad936x_iio_custom::select_rf_filter(std::string rf_filter)
{
    // adi,gpo-manual-mode-enable Enables GPO manual mode, this will conflict with automatic ENSM slave and eLNA mode
    // adi,gpo-manual-mode-enable-mask Enable bit mask, setting or clearing bits will change the level of the corresponding output. Bit0 → GPO, Bit1 → GPO1, Bit2 → GPO2, Bit3 → GP03
    // adi,gpo-manual-mode-enable
    // adi,gpo-manual-mode-enable-mask does not work...
    // some software use the direct_reg_access (see https://github.com/g4eml/Langstone/blob/master/LangstoneGUI.c)

    // since plutosdr fw 31:
    //    GPOs can be addressed individual 0..3 or altogether using 0xF as Identifier.
    //
    //    SYNTAX:
    //
    //    gpo_set <Identifier> <Value>
    //    Enable
    //    Value   Function
    //    0   Disable
    //    1   Enable
    //    X   Enable Mask if Identifier=0xF

    if (check_device() == false) return false;
    // int plutoGpo = 0;
    int ret;
    ret = iio_device_debug_attr_write(phy, "adi,gpo-manual-mode-enable", "1");

    if (ret < 0)
        {
            std::cerr << "Failed to write adi,gpo-manual-mode-enable: " << ret << std::endl;
            return false;
        }

    if (rf_filter.compare("E1") == 0)
        {
            //  set gpio0 to switch L1 filter
            //            setPlutoGpo(plutoGpo);
            ret = iio_device_debug_attr_write(phy, "gpo_set", "0 0");
            if (ret < 0)
                {
                    std::cerr << "Failed to write gpo_set: " << ret << std::endl;
                    return false;
                }
        }
    else if (rf_filter.compare("E5E6") == 0)
        {
            // set gpio0 to switch L5/L6 filter (GPO0)
            //            plutoGpo = plutoGpo | 0x10;
            //            setPlutoGpo(plutoGpo);  //set the Pluto GPO Pin
            ret = iio_device_debug_attr_write(phy, "gpo_set", "0 1");
            if (ret < 0)
                {
                    std::cerr << "Failed to write gpo_set: " << ret << std::endl;
                    return false;
                }
        }
    if (rf_filter.compare("none") == 0)
        {
            std::cout << "RF external filter not selected\n";
        }

    else
        {
            std::cout << "Unknown filter selected, switching to E1 filter...\n";
            ret = iio_device_debug_attr_write(phy, "gpo_set", "0 0");
            if (ret < 0)
                {
                    std::cerr << "Failed to write gpo_set: " << ret << std::endl;
                    return false;
                }
        }

    return true;
}


void ad936x_iio_custom::get_PPS_timestamp()
{
    GnssTime tow;
    PpsSamplestamp pps;
    GnssTime_queue->clear();
    Pps_queue->clear();

    std::cout << "Waiting for uBlox time message synchronization... (wait up to 10 seconds)\n";
    if (GnssTime_queue->timed_wait_and_pop(tow, 10000) == false)
        {
            std::cout << "uBlox time message synchronization error.\n";
            return;
        }

    std::cout << "Waiting for PPS Samplestamp message synchronization... (wait up to 10 seconds)\n";
    if (Pps_queue->timed_wait_and_pop(pps, 10000) == false)
        {
            std::cout << "PPS IP message synchronization error.\n";
            return;
        }

    // Get new PPS samplestamp and associate it to the corresponding uBlox TP message
    while (receive_samples == true)
        {
            std::cout << "[" << pps.samplestamp << "][o:" << pps.overflow_reg << "] uBlox time message received with TOW=" << tow.tow_ms << "\n";
            LOG(INFO) << "[" << pps.samplestamp << "][o:" << pps.overflow_reg << "] uBlox time message received with TOW=" << tow.tow_ms << "\n";
            // write timestamp information to timestamp metadata file:
            // uint64_t: absolute sample counter from the beginning of sample capture associated to the rising edge of the PPS signal
            //            ppstimefile.write(reinterpret_cast<char *>(&pps.samplestamp), sizeof(uint64_t));
            // int32_t: Galileo/GPS Week Number associated to the rising edge of PPS signal
            //            ppstimefile.write(reinterpret_cast<char *>(&tow.week), sizeof(int32_t));
            // int32_t: Galileo/GPS TOW associated to the rising edge of PPS signal
            //            ppstimefile.write(reinterpret_cast<char *>(&tow.tow_ms), sizeof(int32_t));
            // record pps rise samplestamp associated to the absolute sample counter
            // PPS rising edge must be associated with the corresponding uBlox time message (tx once a second)

            if (GnssTime_queue->timed_wait_and_pop(tow, 2000) == false)
                {
                    if (receive_samples == true)
                        {
                            std::cout << "ERROR: uBlox time message not received, check uBlox GNSS signal quality!\n";
                            LOG(INFO) << "ERROR: uBlox time message not received!";
                        }
                    break;
                }
            if (Pps_queue->timed_wait_and_pop(pps, 2000) == false)
                {
                    if (receive_samples == true)
                        {
                            std::cout << "ERROR: PPS time message not received, check uBlox GNSS signal quality!\n";
                            LOG(INFO) << "ERROR: PPS time message not received!";
                        }
                    break;
                }
            if (pps.overflow_reg > 0)
                {
                    if (receive_samples == true)
                        {
                            fpga_overflow = true;
                            std::cout << "ERROR: FPGA reported RX sample buffer overflow!\n";
                            LOG(INFO) << "ERROR: FPGA reported RX sample buffer overflow!\n";
                        }
                    break;
                }
        }
}


bool ad936x_iio_custom::start_sample_rx(bool ppsmode)
{
    // using queues of smart pointers to preallocated buffers
    free_buffers.clear();
    used_buffers.clear();
    // preallocate buffers and use queues
    std::cerr << "Allocating memory..\n";
    try
        {
            for (int n = 0; n < IIO_INPUTRAMFIFOSIZE; n++)
                {
                    free_buffers.push(std::make_shared<ad936x_iio_samples>());
                }
        }
    catch (const std::exception &ex)
        {
            std::cout << "ERROR: Problem allocating RAM buffer: " << ex.what() << "\n";
            return false;
        }

    // prepare capture channels
    std::vector<std::string> channels;
    switch (n_channels)
        {
        case 1:
            channels.push_back("voltage0");  // Channel 0 I
            channels.push_back("voltage1");  // Channel 0 Q
            break;
        case 2:
            channels.push_back("voltage0");  // Channel 0 I
            channels.push_back("voltage1");  // Channel 0 Q
            channels.push_back("voltage2");  // Channel 1 I
            channels.push_back("voltage3");  // Channel 1 Q
            break;
        default:
            channels.push_back("voltage0");  // Channel 0 I
            channels.push_back("voltage1");  // Channel 0 Q
        }

    receive_samples = true;
    // start sample capture thread
    capture_samples_thread = std::thread(&ad936x_iio_custom::capture, this, channels);
    // start sample overflow detector
    overflow_monitor_thread = std::thread(&ad936x_iio_custom::monitor_thread_fn, this);

    // start PPS and GNSS Time capture thread
    if (ppsmode == true)
        {
            capture_time_thread = std::thread(&ad936x_iio_custom::get_PPS_timestamp, this);
        }
    return true;
}


void ad936x_iio_custom::pop_sample_buffer(std::shared_ptr<ad936x_iio_samples> &current_buffer)
{
    used_buffers.wait_and_pop(current_buffer);
}


void ad936x_iio_custom::push_sample_buffer(std::shared_ptr<ad936x_iio_samples> &current_buffer)
{
    free_buffers.push(current_buffer);
}


void ad936x_iio_custom::capture(const std::vector<std::string> &channels)
{
    if (check_device() == false) return;

    struct iio_buffer *rxbuf;

    std::vector<struct iio_channel *> channel_list;

    /* First disable all channels */
    unsigned int nb_channels;
    nb_channels = iio_device_get_channels_count(stream_dev);
    for (unsigned int i = 0; i < nb_channels; i++)
        {
            iio_channel_disable(iio_device_get_channel(stream_dev, i));
        }

    // enable channels
    if (channels.empty())
        {
            for (unsigned int i = 0; i < nb_channels; i++)
                {
                    struct iio_channel *chn =
                        iio_device_get_channel(stream_dev, i);

                    iio_channel_enable(chn);
                    channel_list.push_back(chn);
                }
        }
    else
        {
            for (std::vector<std::string>::const_iterator it =
                     channels.begin();
                 it != channels.end(); ++it)
                {
                    struct iio_channel *chn =
                        iio_device_find_channel(stream_dev,
                            it->c_str(), false);
                    if (!chn)
                        {
                            std::cerr << "Channel " << it->c_str() << " not found\n";
                            return;
                        }
                    else
                        {
                            iio_channel_enable(chn);
                            channel_list.push_back(chn);
                        }
                }
        }

    const struct iio_data_format *format = iio_channel_get_data_format(channel_list[0]);

    std::cerr << "Format: length " << format->length
              << " bits " << format->bits
              << " shift " << format->shift
              << " is_signed " << format->is_signed
              << " is_fully_defined " << format->is_fully_defined
              << " is_be " << format->is_be
              << " with_scale " << format->with_scale
              << " scale " << format->scale
              << " repeat " << format->repeat << "\n";

    rxbuf = iio_device_create_buffer(stream_dev, IIO_DEFAULTAD936XAPIFIFOSIZE_SAMPLES, false);
    if (!rxbuf)
        {
            std::cout << "Could not create RX buffer. \n";
            return;
        }

    std::shared_ptr<ad936x_iio_samples> current_buffer;
    ad936x_iio_samples *current_samples;
    unsigned long items_in_buffer;
    std::cerr << "Enter capture loop...\n";
    int ret;
    int bytes_to_interleaved_iq_samples = n_channels * sizeof(int16_t);
    while (receive_samples == true)
        {
            free_buffers.wait_and_pop(current_buffer);
            current_samples = current_buffer.get();
            ret = iio_buffer_refill(rxbuf);
            if (ret < 0)
                {
                    /* -EBADF happens when the buffer is cancelled */
                    if (ret != -EBADF)
                        {
                            char err_buf[256];
                            iio_strerror(-ret, err_buf, sizeof(err_buf));
                            std::string error(err_buf);

                            std::cerr << "Unable to refill buffer: " << error << std::endl;
                            iio_buffer_destroy(rxbuf);
                            return;
                        }
                }
            memcpy(&current_samples->buffer[0], iio_buffer_start(rxbuf), ret);

            items_in_buffer = static_cast<unsigned long>(ret) / bytes_to_interleaved_iq_samples;

            if (items_in_buffer == 0) return;

            current_samples->n_channels = n_channels;
            current_samples->n_interleaved_iq_samples = items_in_buffer;
            current_samples->n_bytes = ret;
            current_samples->step_bytes = iio_buffer_step(rxbuf);
            used_buffers.push(current_buffer);
        }

    iio_buffer_destroy(rxbuf);
}
