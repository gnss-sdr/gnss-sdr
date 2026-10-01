/*!
 * \file pcps_acquisition_fractional_dwell_test.cc
 * \brief  Regression test for pcps_acquisition's per-dwell sample-count
 * truncation fix: GNSS-SDR.internal_fs_sps not a multiple of 1 kHz, combined
 * with coherent_integration_time_ms spanning more than one code period and
 * max_dwells > 1, which is exactly the configuration the original bug
 * (floor() in d_samples_to_consume, and the resulting dwell-boundary drift
 * across non-coherent dwells) depended on.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "GPS_L1_CA.h"
#include "concurrent_queue.h"
#include "fir_filter.h"
#include "gen_signal_source.h"
#include "gnss_block_interface.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "in_memory_configuration.h"
#include "pass_through.h"
#include "pcps_acquisition_adapter.h"
#include "signal_generator.h"
#include "signal_generator_c.h"
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <pmt/pmt.h>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

#if HAS_GENERIC_LAMBDA
#else
#include <boost/bind/bind.hpp>
#endif

#if PMT_USES_BOOST_ANY
namespace wht = boost;
#else
namespace wht = std;
#endif

// ######## GNURADIO BLOCK MESSAGE RECEIVER #########
// Same pattern as the other acquisition test files in this directory,
// duplicated (not shared) to keep this file self-contained.
class PcpsAcquisitionFractionalDwellTest_msg_rx;

using PcpsAcquisitionFractionalDwellTest_msg_rx_sptr = gnss_shared_ptr<PcpsAcquisitionFractionalDwellTest_msg_rx>;

PcpsAcquisitionFractionalDwellTest_msg_rx_sptr PcpsAcquisitionFractionalDwellTest_msg_rx_make(Concurrent_Queue<int>& queue);

class PcpsAcquisitionFractionalDwellTest_msg_rx : public gr::block
{
private:
    friend PcpsAcquisitionFractionalDwellTest_msg_rx_sptr PcpsAcquisitionFractionalDwellTest_msg_rx_make(Concurrent_Queue<int>& queue);
    void msg_handler_channel_events(const pmt::pmt_t& msg);
    explicit PcpsAcquisitionFractionalDwellTest_msg_rx(Concurrent_Queue<int>& queue);
    Concurrent_Queue<int>& channel_internal_queue;

public:
    int rx_message{0};
};


PcpsAcquisitionFractionalDwellTest_msg_rx_sptr PcpsAcquisitionFractionalDwellTest_msg_rx_make(Concurrent_Queue<int>& queue)
{
    return PcpsAcquisitionFractionalDwellTest_msg_rx_sptr(new PcpsAcquisitionFractionalDwellTest_msg_rx(queue));
}


void PcpsAcquisitionFractionalDwellTest_msg_rx::msg_handler_channel_events(const pmt::pmt_t& msg)
{
    try
        {
            int64_t message = pmt::to_long(msg);
            rx_message = message;
            channel_internal_queue.push(rx_message);
        }
    catch (const wht::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_channel_events Bad any_cast: " << e.what();
            rx_message = 0;
        }
}


PcpsAcquisitionFractionalDwellTest_msg_rx::PcpsAcquisitionFractionalDwellTest_msg_rx(Concurrent_Queue<int>& queue)
    : gr::block("PcpsAcquisitionFractionalDwellTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0)), channel_internal_queue(queue)
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"),
#if HAS_GENERIC_LAMBDA
        [this](auto&& PH1) { msg_handler_channel_events(std::forward<decltype(PH1)>(PH1)); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
        boost::bind(&PcpsAcquisitionFractionalDwellTest_msg_rx::msg_handler_channel_events, this, boost::placeholders::_1));
#else
        boost::bind(&PcpsAcquisitionFractionalDwellTest_msg_rx::msg_handler_channel_events, this, _1));
#endif
#endif
}


// ###########################################################

class PcpsAcquisitionFractionalDwellTest : public ::testing::Test
{
protected:
    PcpsAcquisitionFractionalDwellTest()
    {
        gnss_synchro = Gnss_Synchro();
    }

    Concurrent_Queue<int> channel_internal_queue;
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue;
    gr::top_block_sptr top_block;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;

    // Deliberately NOT a multiple of 1 kHz: 4000123 / 1000 = 4000.123, so
    // d_samples_to_consume's floor() truncates a real, non-zero fraction of a
    // sample on every dwell -- exactly the condition the fix is for.
    static constexpr unsigned int kFsIn = 4000123;
    // GPS L1 C/A's own code period is 1 ms; 4 ms coherent integration means
    // sampled_ms (4) != ms_per_code (1), exercising the K > 1 case the
    // original per-dwell drift formula got wrong.
    static constexpr unsigned int kCoherentIntegrationMs = 4;
    static constexpr unsigned int kMaxDwells = 5;
    static constexpr double kExpectedDopplerHz = 750.0;
    static constexpr double kExpectedDelayChips = 600.0;
    // 2 / (3 * coherent_integration_time_s), the same bound every other test
    // in this directory uses.
    static constexpr double kMaxDopplerErrorHz = 2.0 / (3.0 * kCoherentIntegrationMs * 1e-3);
};


TEST_F(PcpsAcquisitionFractionalDwellTest, NonIntegerSamplesPerCodeMultiDwellAcquires)
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 10;

    config = std::make_shared<InMemoryConfiguration>();
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(kFsIn));

    config->set_property("SignalSource.fs_hz", std::to_string(kFsIn));
    config->set_property("SignalSource.item_type", "gr_complex");
    config->set_property("SignalSource.num_satellites", "1");
    config->set_property("SignalSource.system_0", "G");
    config->set_property("SignalSource.PRN_0", "10");
    config->set_property("SignalSource.CN0_dB_0", "44");
    config->set_property("SignalSource.doppler_Hz_0", std::to_string(kExpectedDopplerHz));
    config->set_property("SignalSource.delay_chips_0", std::to_string(kExpectedDelayChips));
    config->set_property("SignalSource.noise_flag", "false");
    config->set_property("SignalSource.data_flag", "false");
    config->set_property("SignalSource.BW_BB", "0.97");

    config->set_property("InputFilter.implementation", "Fir_Filter");
    config->set_property("InputFilter.input_item_type", "gr_complex");
    config->set_property("InputFilter.output_item_type", "gr_complex");
    config->set_property("InputFilter.taps_item_type", "float");
    config->set_property("InputFilter.number_of_taps", "11");
    config->set_property("InputFilter.number_of_bands", "2");
    config->set_property("InputFilter.band1_begin", "0.0");
    config->set_property("InputFilter.band1_end", "0.97");
    config->set_property("InputFilter.band2_begin", "0.98");
    config->set_property("InputFilter.band2_end", "1.0");
    config->set_property("InputFilter.ampl1_begin", "1.0");
    config->set_property("InputFilter.ampl1_end", "1.0");
    config->set_property("InputFilter.ampl2_begin", "0.0");
    config->set_property("InputFilter.ampl2_end", "0.0");
    config->set_property("InputFilter.band1_error", "1.0");
    config->set_property("InputFilter.band2_error", "1.0");
    config->set_property("InputFilter.filter_type", "bandpass");
    config->set_property("InputFilter.grid_density", "16");

    config->set_property("Acquisition_1C.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_1C.item_type", "gr_complex");
    config->set_property("Acquisition_1C.coherent_integration_time_ms", std::to_string(kCoherentIntegrationMs));
    config->set_property("Acquisition_1C.max_dwells", std::to_string(kMaxDwells));
    config->set_property("Acquisition_1C.pfa", "0.001");
    config->set_property("Acquisition_1C.doppler_max", "5000");
    config->set_property("Acquisition_1C.doppler_step", "50");
    config->set_property("Acquisition_1C.bit_transition_flag", "false");
    config->set_property("Acquisition_1C.dump", "false");

    queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    top_block = gr::make_top_block("Fractional dwell acquisition test");
    auto acquisition = std::make_shared<PcpsAcquisitionAdapter>(config.get(), "Acquisition_1C", "GPS_L1_CA_PCPS_Acquisition", 1, 0, GPS_1C);
    auto msg_rx = PcpsAcquisitionFractionalDwellTest_msg_rx_make(channel_internal_queue);

    acquisition->set_channel(1);
    acquisition->set_gnss_synchro(&gnss_synchro);
    acquisition->connect(top_block);
    top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));

    std::shared_ptr<GNSSBlockInterface> signal_generator = std::make_shared<SignalGenerator>(config.get(), "SignalSource", 0, 1, queue.get());
    std::shared_ptr<GNSSBlockInterface> filter = std::make_shared<FirFilter>(config.get(), "InputFilter", 1, 1);
    std::shared_ptr<GNSSBlockInterface> signal_source = std::make_shared<GenSignalSource>(signal_generator, filter, "SignalSource", queue.get());
    signal_source->connect(top_block);
    top_block->connect(signal_source->get_right_block(), 0, acquisition->get_left_block(), 0);

    acquisition->set_local_code();
    acquisition->reset();

    int message = 0;
    // SignalGenerator loops its generated vector indefinitely for a GPS-only
    // source, so top_block->run() never returns on its own; stop it as soon
    // as the acquisition message arrives, same as every other test in this
    // directory does via its own wait_message()/process_message() pair.
    std::thread wait_thread([&]() {
        channel_internal_queue.wait_and_pop(message);
        top_block->stop();
    });
    top_block->run();
    wait_thread.join();

    ASSERT_EQ(1, message) << "Acquisition failure with a non-1kHz-multiple sampling rate, "
                          << "coherent_integration_time_ms (" << kCoherentIntegrationMs
                          << ") > ms_per_code, and max_dwells (" << kMaxDwells << ") > 1.";

    const double doppler_error_hz = std::abs(kExpectedDopplerHz - gnss_synchro.Acq_doppler_hz);
    EXPECT_LE(doppler_error_hz, kMaxDopplerErrorHz)
        << "Doppler error " << doppler_error_hz << " Hz exceeds " << kMaxDopplerErrorHz
        << " Hz -- consistent with the dwell-boundary drifting across the " << kMaxDwells << " non-coherent dwells.";

    // Loose delay sanity check (generous +/-3 chip tolerance, no FIR-group-delay
    // correction): the drift this test guards against would put the reported
    // delay far outside this window, not just a fraction of a chip off.
    const double samples_per_chip = static_cast<double>(kFsIn) / GPS_L1_CA_CODE_RATE_CPS;
    const double delay_error_chips = std::abs(kExpectedDelayChips - gnss_synchro.Acq_delay_samples / samples_per_chip);
    EXPECT_LE(delay_error_chips, 3.0) << "Delay error " << delay_error_chips << " chips indicates the dwell-boundary tracking drifted.";
}
