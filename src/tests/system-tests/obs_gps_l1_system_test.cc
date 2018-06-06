/*!
 * \file obs_gps_l1_system_test.cc
 * \brief  This class implements a test for the validation of generated observables.
 * \author Carles Fernandez-Prades, 2016. cfernandez(at)cttc.es
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

#include "concurrent_map.h"
#include "concurrent_queue.h"
#include "control_thread.h"
#include "in_memory_configuration.h"
#include "signal_generator_flags.h"
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <gpstk/RinexUtilities.hpp>
#include <gpstk/Rinex3ObsBase.hpp>
#include <gpstk/Rinex3ObsData.hpp>
#include <gpstk/Rinex3ObsHeader.hpp>
#include <gpstk/Rinex3ObsStream.hpp>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>
#include <unistd.h>


// For GPS NAVIGATION (L1)
concurrent_queue<Gps_Acq_Assist> global_gps_acq_assist_queue;
concurrent_map<Gps_Acq_Assist> global_gps_acq_assist_map;


class ObsGpsL1SystemTest : public ::testing::Test
{
public:
    std::string generator_binary;
    std::string p1;
    std::string p2;
    std::string p3;
    std::string p4;
    std::string p5;

    const double baseband_sampling_freq = 2.6e6;

    std::string filename_rinex_obs = FLAGS_filename_rinex_obs;
    std::string filename_raw_data = FLAGS_filename_raw_data;
    std::string generated_rinex_obs;
    int configure_generator();
    int generate_signal();
    int configure_receiver();
    int run_receiver();
    void check_results();
    bool check_valid_rinex_nav(std::string filename);  // return true if the file is a valid Rinex navigation file.
    bool check_valid_rinex_obs(std::string filename);  // return true if the file is a valid Rinex observation file.
    double compute_stdev(const std::vector<double>& vec);

    std::shared_ptr<InMemoryConfiguration> config;
};


bool ObsGpsL1SystemTest::check_valid_rinex_nav(std::string filename)
{
    bool res = false;
    res = gpstk::isRinexNavFile(filename);
    return res;
}


double ObsGpsL1SystemTest::compute_stdev(const std::vector<double>& vec)
{
    double sum__ = std::accumulate(vec.begin(), vec.end(), 0.0);
    double mean__ = sum__ / vec.size();
    double accum__ = 0.0;
    std::for_each(std::begin(vec), std::end(vec), [&](const double d) {
        accum__ += (d - mean__) * (d - mean__);
    });
    double stdev__ = std::sqrt(accum__ / (vec.size() - 1));
    return stdev__;
}


bool ObsGpsL1SystemTest::check_valid_rinex_obs(std::string filename)
{
    bool res = false;
    res = gpstk::isRinexObsFile(filename);
    return res;
}


int ObsGpsL1SystemTest::configure_generator()
{
    // Configure signal generator
    generator_binary = FLAGS_generator_binary;

    p1 = std::string("-rinex_nav_file=") + FLAGS_rinex_nav_file;
    if (FLAGS_dynamic_position.empty())
        {
            p2 = std::string("-static_position=") + FLAGS_static_position + std::string(",") + std::to_string(std::min(FLAGS_duration * 10, 3000));
            if (FLAGS_duration > 300) std::cout << "WARNING: Duration has been set to its maximum value of 300 s" << std::endl;
        }
    else
        {
            p2 = std::string("-obs_pos_file=") + std::string(FLAGS_dynamic_position);
        }
    p3 = std::string("-rinex_obs_file=") + FLAGS_filename_rinex_obs;               // RINEX 2.10 observation file output
    p4 = std::string("-sig_out_file=") + FLAGS_filename_raw_data;                  // Baseband signal output file. Will be stored in int8_t IQ multiplexed samples
    p5 = std::string("-sampling_freq=") + std::to_string(baseband_sampling_freq);  //Baseband sampling frequency [MSps]
    return 0;
}


int ObsGpsL1SystemTest::generate_signal()
{
    pid_t wait_result;
    int child_status;

    char* const parmList[] = {&generator_binary[0], &generator_binary[0], &p1[0], &p2[0], &p3[0], &p4[0], &p5[0], NULL};

    int pid;
    if ((pid = fork()) == -1)
        perror("fork error");
    else if (pid == 0)
        {
            execv(&generator_binary[0], parmList);
            std::cout << "Return not expected. Must be an execv error." << std::endl;
            std::terminate();
        }

    wait_result = waitpid(pid, &child_status, 0);
    if (wait_result == -1) perror("waitpid error");
    EXPECT_EQ(true, check_valid_rinex_obs(filename_rinex_obs));
    std::cout << "Signal and Observables RINEX files created." << std::endl;
    return 0;
}


int ObsGpsL1SystemTest::configure_receiver()
{
    config = std::make_shared<InMemoryConfiguration>();

    const int sampling_rate_internal = baseband_sampling_freq;

    const int number_of_taps = 11;
    const int number_of_bands = 2;
    const float band1_begin = 0.0;
    const float band1_end = 0.48;
    const float band2_begin = 0.52;
    const float band2_end = 1.0;
    const float ampl1_begin = 1.0;
    const float ampl1_end = 1.0;
    const float ampl2_begin = 0.0;
    const float ampl2_end = 0.0;
    const float band1_error = 1.0;
    const float band2_error = 1.0;
    const int grid_density = 16;

    const float zero = 0.0;
    const int number_of_channels = 8;
    const int in_acquisition = 1;

    const float threshold = 0.01;
    const float doppler_max = 8000.0;
    const float doppler_step = 500.0;
    const int max_dwells = 1;
    const int tong_init_val = 2;
    const int tong_max_val = 10;
    const int tong_max_dwells = 30;
    const int coherent_integration_time_ms = 1;

    const float pll_bw_hz = 30.0;
    const float dll_bw_hz = 4.0;
    const float early_late_space_chips = 0.5;
    const float pll_bw_narrow_hz = 20.0;
    const float dll_bw_narrow_hz = 2.0;
    const int extend_correlation_ms = 1;

    const int display_rate_ms = 500;
    const int output_rate_ms = 10;

    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(sampling_rate_internal));

    // Set the assistance system parameters
    config->set_property("GNSS-SDR.SUPL_read_gps_assistance_xml", "false");
    config->set_property("GNSS-SDR.SUPL_gps_enabled", "false");
    config->set_property("GNSS-SDR.SUPL_gps_ephemeris_server", "supl.google.com");
    config->set_property("GNSS-SDR.SUPL_gps_ephemeris_port", std::to_string(7275));
    config->set_property("GNSS-SDR.SUPL_gps_acquisition_server", "supl.google.com");
    config->set_property("GNSS-SDR.SUPL_gps_acquisition_port", std::to_string(7275));
    config->set_property("GNSS-SDR.SUPL_MCC", std::to_string(244));
    config->set_property("GNSS-SDR.SUPL_MNS", std::to_string(5));
    config->set_property("GNSS-SDR.SUPL_LAC", "0x59e2");
    config->set_property("GNSS-SDR.SUPL_CI", "0x31b0");

    // Set the Signal Source
    config->set_property("SignalSource.implementation", "File_Signal_Source");
    config->set_property("SignalSource.filename", "./" + filename_raw_data);
    config->set_property("SignalSource.sampling_frequency", std::to_string(sampling_rate_internal));
    config->set_property("SignalSource.item_type", "ibyte");
    config->set_property("SignalSource.samples", std::to_string(zero));

    // Set the Signal Conditioner
    config->set_property("SignalConditioner.implementation", "Signal_Conditioner");
    config->set_property("DataTypeAdapter.implementation", "Ibyte_To_Complex");
    config->set_property("InputFilter.implementation", "Fir_Filter");
    config->set_property("InputFilter.dump", "false");
    config->set_property("InputFilter.input_item_type", "gr_complex");
    config->set_property("InputFilter.output_item_type", "gr_complex");
    config->set_property("InputFilter.taps_item_type", "float");
    config->set_property("InputFilter.number_of_taps", std::to_string(number_of_taps));
    config->set_property("InputFilter.number_of_bands", std::to_string(number_of_bands));
    config->set_property("InputFilter.band1_begin", std::to_string(band1_begin));
    config->set_property("InputFilter.band1_end", std::to_string(band1_end));
    config->set_property("InputFilter.band2_begin", std::to_string(band2_begin));
    config->set_property("InputFilter.band2_end", std::to_string(band2_end));
    config->set_property("InputFilter.ampl1_begin", std::to_string(ampl1_begin));
    config->set_property("InputFilter.ampl1_end", std::to_string(ampl1_end));
    config->set_property("InputFilter.ampl2_begin", std::to_string(ampl2_begin));
    config->set_property("InputFilter.ampl2_end", std::to_string(ampl2_end));
    config->set_property("InputFilter.band1_error", std::to_string(band1_error));
    config->set_property("InputFilter.band2_error", std::to_string(band2_error));
    config->set_property("InputFilter.filter_type", "bandpass");
    config->set_property("InputFilter.grid_density", std::to_string(grid_density));
    config->set_property("InputFilter.sampling_frequency", std::to_string(sampling_rate_internal));
    config->set_property("InputFilter.IF", std::to_string(zero));
    config->set_property("Resampler.implementation", "Pass_Through");
    config->set_property("Resampler.dump", "false");
    config->set_property("Resampler.item_type", "gr_complex");
    config->set_property("Resampler.sample_freq_in", std::to_string(sampling_rate_internal));
    config->set_property("Resampler.sample_freq_out", std::to_string(sampling_rate_internal));

    // Set the number of Channels
    config->set_property("Channels_1C.count", std::to_string(number_of_channels));
    config->set_property("Channels.in_acquisition", std::to_string(in_acquisition));
    config->set_property("Channel.signal", "1C");

    // Set Acquisition
    config->set_property("Acquisition_1C.implementation", "GPS_L1_CA_PCPS_Tong_Acquisition");
    config->set_property("Acquisition_1C.item_type", "gr_complex");
    config->set_property("Acquisition_1C.if", std::to_string(zero));
    config->set_property("Acquisition_1C.coherent_integration_time_ms", std::to_string(coherent_integration_time_ms));
    config->set_property("Acquisition_1C.threshold", std::to_string(threshold));
    config->set_property("Acquisition_1C.doppler_max", std::to_string(doppler_max));
    config->set_property("Acquisition_1C.doppler_step", std::to_string(doppler_step));
    config->set_property("Acquisition_1C.bit_transition_flag", "false");
    config->set_property("Acquisition_1C.max_dwells", std::to_string(max_dwells));
    config->set_property("Acquisition_1C.tong_init_val", std::to_string(tong_init_val));
    config->set_property("Acquisition_1C.tong_max_val", std::to_string(tong_max_val));
    config->set_property("Acquisition_1C.tong_max_dwells", std::to_string(tong_max_dwells));

    // Set Tracking
    config->set_property("Tracking_1C.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    //config->set_property("Tracking_1C.implementation", "GPS_L1_CA_DLL_PLL_C_Aid_Tracking");
    config->set_property("Tracking_1C.item_type", "gr_complex");
    config->set_property("Tracking_1C.if", std::to_string(zero));
    config->set_property("Tracking_1C.dump", "false");
    config->set_property("Tracking_1C.dump_filename", "./tracking_ch_");
    config->set_property("Tracking_1C.pll_bw_hz", std::to_string(pll_bw_hz));
    config->set_property("Tracking_1C.dll_bw_hz", std::to_string(dll_bw_hz));
    config->set_property("Tracking_1C.early_late_space_chips", std::to_string(early_late_space_chips));

    config->set_property("Tracking_1C.pll_bw_narrow_hz", std::to_string(pll_bw_narrow_hz));
    config->set_property("Tracking_1C.dll_bw_narrow_hz", std::to_string(dll_bw_narrow_hz));
    config->set_property("Tracking_1C.extend_correlation_ms", std::to_string(extend_correlation_ms));

    // Set Telemetry
    config->set_property("TelemetryDecoder_1C.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1C.dump", "false");

    // Set Observables
    config->set_property("Observables.implementation", "Hybrid_Observables");
    config->set_property("Observables.dump", "false");
    config->set_property("Observables.dump_filename", "./observables.dat");
    config->set_property("Observables.averaging_depth", std::to_string(100));

    // Set PVT
    config->set_property("PVT.implementation", "RTKLIB_PVT");
    config->set_property("PVT.positioning_mode", "Single");
    config->set_property("PVT.output_rate_ms", std::to_string(output_rate_ms));
    config->set_property("PVT.display_rate_ms", std::to_string(display_rate_ms));
    config->set_property("PVT.dump_filename", "./PVT");
    config->set_property("PVT.nmea_dump_filename", "./gnss_sdr_pvt.nmea");
    config->set_property("PVT.flag_nmea_tty_port", "false");
    config->set_property("PVT.nmea_dump_devname", "/dev/pts/4");
    config->set_property("PVT.flag_rtcm_server", "false");
    config->set_property("PVT.flag_rtcm_tty_port", "false");
    config->set_property("PVT.rtcm_dump_devname", "/dev/pts/1");
    config->set_property("PVT.dump", "false");
    config->set_property("PVT.rinex_version", std::to_string(2));

    return 0;
}


int ObsGpsL1SystemTest::run_receiver()
{
    std::shared_ptr<ControlThread> control_thread;
    control_thread = std::make_shared<ControlThread>(config);
    // start receiver
    try
        {
            control_thread->run();
        }
    catch (const boost::exception& e)
        {
            std::cout << "Boost exception: " << boost::diagnostic_information(e);
        }
    catch (const std::exception& ex)
        {
            std::cout << "STD exception: " << ex.what();
        }
    // Get the name of the RINEX obs file generated by the receiver
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    FILE* fp;
    std::string argum2 = std::string("/bin/ls *O | grep GSDR | tail -1");
    char buffer[1035];
    fp = popen(&argum2[0], "r");
    if (fp == NULL)
        {
            std::cout << "Failed to run command: " << argum2 << std::endl;
            return -1;
        }
    while (fgets(buffer, sizeof(buffer), fp) != NULL)
        {
            std::string aux = std::string(buffer);
            ObsGpsL1SystemTest::generated_rinex_obs = aux.erase(aux.length() - 1, 1);
        }
    pclose(fp);
    return 0;
}


void ObsGpsL1SystemTest::check_results()
{
    std::vector<std::vector<std::pair<double, double>>> pseudorange_ref(33);
    std::vector<std::vector<std::pair<double, double>>> carrierphase_ref(33);
    std::vector<std::vector<std::pair<double, double>>> doppler_ref(33);

    std::vector<std::vector<std::pair<double, double>>> pseudorange_meas(33);
    std::vector<std::vector<std::pair<double, double>>> carrierphase_meas(33);
    std::vector<std::vector<std::pair<double, double>>> doppler_meas(33);

    // Open and read reference RINEX observables file
    try
        {
            gpstk::Rinex3ObsStream r_ref(FLAGS_filename_rinex_obs);
            r_ref.exceptions(std::ios::failbit);
            gpstk::Rinex3ObsData r_ref_data;
            gpstk::Rinex3ObsHeader r_ref_header;

            gpstk::RinexDatum dataobj;

            r_ref >> r_ref_header;

            while (r_ref >> r_ref_data)
                {
                    for (int myprn = 1; myprn < 33; myprn++)
                        {
                            gpstk::SatID prn(myprn, gpstk::SatID::systemGPS);
                            gpstk::CommonTime time = r_ref_data.time;
                            double sow(static_cast<gpstk::GPSWeekSecond>(time).sow);

                            gpstk::Rinex3ObsData::DataMap::iterator pointer = r_ref_data.obs.find(prn);
                            if (pointer == r_ref_data.obs.end())
                                {
                                    // PRN not present; do nothing
                                }
                            else
                                {
                                    dataobj = r_ref_data.getObs(prn, "C1C", r_ref_header);
                                    double P1 = dataobj.data;
                                    std::pair<double, double> pseudo(sow, P1);
                                    pseudorange_ref.at(myprn).push_back(pseudo);

                                    dataobj = r_ref_data.getObs(prn, "L1C", r_ref_header);
                                    double L1 = dataobj.data;
                                    std::pair<double, double> carrier(sow, L1);
                                    carrierphase_ref.at(myprn).push_back(carrier);

                                    dataobj = r_ref_data.getObs(prn, "D1C", r_ref_header);
                                    double D1 = dataobj.data;
                                    std::pair<double, double> doppler(sow, D1);
                                    doppler_ref.at(myprn).push_back(doppler);
                                }  // End of 'if( pointer == roe.obs.end() )'
                        }          // end for
                }                  // end while
        }                          // End of 'try' block
    catch (const gpstk::FFStreamError& e)
        {
            std::cout << e;
            exit(1);
        }
    catch (const gpstk::Exception& e)
        {
            std::cout << e;
            exit(1);
        }
    catch (...)
        {
            std::cout << "unknown error.  I don't feel so well..." << std::endl;
            exit(1);
        }

    try
        {
            std::string arg2_gen = std::string("./") + ObsGpsL1SystemTest::generated_rinex_obs;
            gpstk::Rinex3ObsStream r_meas(arg2_gen);
            r_meas.exceptions(std::ios::failbit);
            gpstk::Rinex3ObsData r_meas_data;
            gpstk::Rinex3ObsHeader r_meas_header;
            gpstk::RinexDatum dataobj;

            r_meas >> r_meas_header;

            while (r_meas >> r_meas_data)
                {
                    for (int myprn = 1; myprn < 33; myprn++)
                        {
                            gpstk::SatID prn(myprn, gpstk::SatID::systemGPS);
                            gpstk::CommonTime time = r_meas_data.time;
                            double sow(static_cast<gpstk::GPSWeekSecond>(time).sow);

                            gpstk::Rinex3ObsData::DataMap::iterator pointer = r_meas_data.obs.find(prn);
                            if (pointer == r_meas_data.obs.end())
                                {
                                    // PRN not present; do nothing
                                }
                            else
                                {
                                    dataobj = r_meas_data.getObs(prn, "C1C", r_meas_header);
                                    double P1 = dataobj.data;
                                    std::pair<double, double> pseudo(sow, P1);
                                    pseudorange_meas.at(myprn).push_back(pseudo);

                                    dataobj = r_meas_data.getObs(prn, "L1C", r_meas_header);
                                    double L1 = dataobj.data;
                                    std::pair<double, double> carrier(sow, L1);
                                    carrierphase_meas.at(myprn).push_back(carrier);

                                    dataobj = r_meas_data.getObs(prn, "D1C", r_meas_header);
                                    double D1 = dataobj.data;
                                    std::pair<double, double> doppler(sow, D1);
                                    doppler_meas.at(myprn).push_back(doppler);
                                }  // End of 'if( pointer == roe.obs.end() )'
                        }          // end for
                }                  // end while
        }                          // End of 'try' block
    catch (const gpstk::FFStreamError& e)
        {
            std::cout << e;
            exit(1);
        }
    catch (const gpstk::Exception& e)
        {
            std::cout << e;
            exit(1);
        }
    catch (...)
        {
            std::cout << "unknown error.  I don't feel so well..." << std::endl;
            exit(1);
        }

    // Time alignment
    std::vector<std::vector<std::pair<double, double>>> pseudorange_ref_aligned(33);
    std::vector<std::vector<std::pair<double, double>>> carrierphase_ref_aligned(33);
    std::vector<std::vector<std::pair<double, double>>> doppler_ref_aligned(33);

    std::vector<std::vector<std::pair<double, double>>>::iterator iter;
    std::vector<std::pair<double, double>>::iterator it;
    std::vector<std::pair<double, double>>::iterator it2;

    std::vector<std::vector<double>> pr_diff(33);
    std::vector<std::vector<double>> cp_diff(33);
    std::vector<std::vector<double>> doppler_diff(33);

    std::vector<std::vector<double>>::iterator iter_diff;
    std::vector<double>::iterator iter_v;

    int prn_id = 0;
    for (iter = pseudorange_ref.begin(); iter != pseudorange_ref.end(); iter++)
        {
            for (it = iter->begin(); it != iter->end(); it++)
                {
                    // If a measure exists for this sow, store it
                    for (it2 = pseudorange_meas.at(prn_id).begin(); it2 != pseudorange_meas.at(prn_id).end(); it2++)
                        {
                            if (std::abs(it->first - it2->first) < 0.1)  // store measures closer than 10 ms.
                                {
                                    pseudorange_ref_aligned.at(prn_id).push_back(*it);
                                    pr_diff.at(prn_id).push_back(it->second - it2->second);
                                    //std::cout << "Sat " << prn_id << ": " << "PR_ref=" << it->second << "   PR_meas=" << it2->second << "    Diff:" << it->second - it2->second <<  std::endl;
                                }
                        }
                }
            prn_id++;
        }

    prn_id = 0;
    for (iter = carrierphase_ref.begin(); iter != carrierphase_ref.end(); iter++)
        {
            for (it = iter->begin(); it != iter->end(); it++)
                {
                    // If a measure exists for this sow, store it
                    for (it2 = carrierphase_meas.at(prn_id).begin(); it2 != carrierphase_meas.at(prn_id).end(); it2++)
                        {
                            if (std::abs(it->first - it2->first) < 0.1)  // store measures closer than 10 ms.
                                {
                                    carrierphase_ref_aligned.at(prn_id).push_back(*it);
                                    cp_diff.at(prn_id).push_back(it->second - it2->second);
                                    // std::cout << "Sat " << prn_id << ": " << "Carrier_ref=" << it->second << "   Carrier_meas=" << it2->second << "    Diff:" << it->second - it2->second <<  std::endl;
                                }
                        }
                }
            prn_id++;
        }
    prn_id = 0;
    for (iter = doppler_ref.begin(); iter != doppler_ref.end(); iter++)
        {
            for (it = iter->begin(); it != iter->end(); it++)
                {
                    // If a measure exists for this sow, store it
                    for (it2 = doppler_meas.at(prn_id).begin(); it2 != doppler_meas.at(prn_id).end(); it2++)
                        {
                            if (std::abs(it->first - it2->first) < 0.01)  // store measures closer than 10 ms.
                                {
                                    doppler_ref_aligned.at(prn_id).push_back(*it);
                                    doppler_diff.at(prn_id).push_back(it->second - it2->second);
                                }
                        }
                }
            prn_id++;
        }

    // Compute pseudorange error
    prn_id = 0;
    std::vector<double> mean_pr_diff_v;
    for (iter_diff = pr_diff.begin(); iter_diff != pr_diff.end(); iter_diff++)
        {
            // For each satellite with reference and measurements aligned in time
            int number_obs = 0;
            double mean_diff = 0.0;
            for (iter_v = iter_diff->begin(); iter_v != iter_diff->end(); iter_v++)
                {
                    mean_diff = mean_diff + *iter_v;
                    number_obs = number_obs + 1;
                }
            if (number_obs > 0)
                {
                    mean_diff = mean_diff / number_obs;
                    mean_pr_diff_v.push_back(mean_diff);
                    std::cout << "-- Mean pseudorange difference for sat " << prn_id << ": " << mean_diff;
                    double stdev_ = compute_stdev(*iter_diff);
                    std::cout << " +/- " << stdev_;
                    std::cout << " [m]" << std::endl;
                }
            else
                {
                    mean_diff = 0.0;
                }

            prn_id++;
        }
    double stdev_pr = compute_stdev(mean_pr_diff_v);
    std::cout << "Pseudorange diff error stdev = " << stdev_pr << " [m]" << std::endl;
    ASSERT_LT(stdev_pr, 10.0);

    // Compute carrier phase error
    prn_id = 0;
    std::vector<double> mean_cp_diff_v;
    for (iter_diff = cp_diff.begin(); iter_diff != cp_diff.end(); iter_diff++)
        {
            // For each satellite with reference and measurements aligned in time
            int number_obs = 0;
            double mean_diff = 0.0;
            for (iter_v = iter_diff->begin(); iter_v != iter_diff->end(); iter_v++)
                {
                    mean_diff = mean_diff + *iter_v;
                    number_obs = number_obs + 1;
                }
            if (number_obs > 0)
                {
                    mean_diff = mean_diff / number_obs;
                    mean_cp_diff_v.push_back(mean_diff);
                    std::cout << "-- Mean carrier phase difference for sat " << prn_id << ": " << mean_diff;
                    double stdev_pr_ = compute_stdev(*iter_diff);
                    std::cout << " +/- " << stdev_pr_ << " whole cycles (19 cm)" << std::endl;
                }
            else
                {
                    mean_diff = 0.0;
                }

            prn_id++;
        }

    // Compute Doppler error
    prn_id = 0;
    std::vector<double> mean_doppler_v;
    for (iter_diff = doppler_diff.begin(); iter_diff != doppler_diff.end(); iter_diff++)
        {
            // For each satellite with reference and measurements aligned in time
            int number_obs = 0;
            double mean_diff = 0.0;
            for (iter_v = iter_diff->begin(); iter_v != iter_diff->end(); iter_v++)
                {
                    //std::cout << *iter_v << std::endl;
                    mean_diff = mean_diff + *iter_v;
                    number_obs = number_obs + 1;
                }
            if (number_obs > 0)
                {
                    mean_diff = mean_diff / number_obs;
                    mean_doppler_v.push_back(mean_diff);
                    std::cout << "-- Mean Doppler difference for sat " << prn_id << ": " << mean_diff << " [Hz]" << std::endl;
                }
            else
                {
                    mean_diff = 0.0;
                }

            prn_id++;
        }

    double stdev_dp = compute_stdev(mean_doppler_v);
    std::cout << "Doppler error stdev = " << stdev_dp << " [Hz]" << std::endl;
    ASSERT_LT(stdev_dp, 10.0);
}


TEST_F(ObsGpsL1SystemTest, Observables_system_test)
{
    std::cout << "Validating input RINEX nav file: " << FLAGS_rinex_nav_file << " ..." << std::endl;
    bool is_rinex_nav_valid = check_valid_rinex_nav(FLAGS_rinex_nav_file);
    EXPECT_EQ(true, is_rinex_nav_valid) << "The RINEX navigation file " << FLAGS_rinex_nav_file << " is not well formed.";
    std::cout << "The file is valid." << std::endl;

    // Configure the signal generator
    configure_generator();

    // Generate signal raw signal samples and observations RINEX file
    if (!FLAGS_disable_generator)
        {
            generate_signal();
        }

    std::cout << "Validating generated reference RINEX obs file: " << FLAGS_filename_rinex_obs << " ..." << std::endl;
    bool is_gen_rinex_obs_valid = check_valid_rinex_obs("./" + FLAGS_filename_rinex_obs);
    EXPECT_EQ(true, is_gen_rinex_obs_valid) << "The RINEX observation file " << FLAGS_filename_rinex_obs << ", generated by gnss-sim, is not well formed.";
    std::cout << "The file is valid." << std::endl;

    // Configure receiver
    configure_receiver();

    // Run the receiver
    EXPECT_EQ(run_receiver(), 0) << "Problem executing the software-defined signal generator";

    std::cout << "Validating RINEX obs file obtained by GNSS-SDR: " << ObsGpsL1SystemTest::generated_rinex_obs << " ..." << std::endl;
    is_gen_rinex_obs_valid = check_valid_rinex_obs("./" + ObsGpsL1SystemTest::generated_rinex_obs);
    EXPECT_EQ(true, is_gen_rinex_obs_valid) << "The RINEX observation file " << ObsGpsL1SystemTest::generated_rinex_obs << ", generated by GNSS-SDR, is not well formed.";
    std::cout << "The file is valid." << std::endl;

    // Check results
    check_results();
}


int main(int argc, char** argv)
{
    std::cout << "Running Observables validation test..." << std::endl;
    int res = 0;
    try
        {
            testing::InitGoogleTest(&argc, argv);
        }
    catch (...)
        {
        }  // catch the "testing::internal::<unnamed>::ClassUniqueToAlwaysTrue" from gtest

    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    // Run the Tests
    try
        {
            res = RUN_ALL_TESTS();
        }
    catch (...)
        {
            LOG(WARNING) << "Unexpected catch";
        }
    google::ShutDownCommandLineFlags();
    return res;
}
