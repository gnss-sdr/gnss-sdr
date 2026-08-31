/*!
 * \file pcps_acquisition_doppler_narrowing_test.cc
 * \brief  Tests for Acq_Conf::enable_doppler_narrowing (assisted-acquisition
 * Doppler search narrowing) in pcps_acquisition, using the same GPS L1 C/A
 * signal/fixture pattern as gps_l1_ca_pcps_acquisition_test.cc.
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
#include "gnss_block_interface.h"
#include "gnss_sdr_filesystem.h"
#include "gnss_synchro.h"
#include "in_memory_configuration.h"
#include "pcps_acquisition_adapter.h"
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <matio.h>
#include <pmt/pmt.h>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
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
// Same pattern as GpsL1CaPcpsAcquisitionTest_msg_rx in
// gps_l1_ca_pcps_acquisition_test.cc, duplicated (rather than shared) to keep
// this file self-contained -- both classes are file-local test doubles, not
// part of any public test-support surface.
class PcpsAcquisitionDopplerNarrowingTest_msg_rx;

using PcpsAcquisitionDopplerNarrowingTest_msg_rx_sptr = gnss_shared_ptr<PcpsAcquisitionDopplerNarrowingTest_msg_rx>;

PcpsAcquisitionDopplerNarrowingTest_msg_rx_sptr PcpsAcquisitionDopplerNarrowingTest_msg_rx_make();

class PcpsAcquisitionDopplerNarrowingTest_msg_rx : public gr::block
{
private:
    friend PcpsAcquisitionDopplerNarrowingTest_msg_rx_sptr PcpsAcquisitionDopplerNarrowingTest_msg_rx_make();
    void msg_handler_channel_events(const pmt::pmt_t &msg);
    PcpsAcquisitionDopplerNarrowingTest_msg_rx();

public:
    int rx_message{0};
};


PcpsAcquisitionDopplerNarrowingTest_msg_rx_sptr PcpsAcquisitionDopplerNarrowingTest_msg_rx_make()
{
    return PcpsAcquisitionDopplerNarrowingTest_msg_rx_sptr(new PcpsAcquisitionDopplerNarrowingTest_msg_rx());
}


void PcpsAcquisitionDopplerNarrowingTest_msg_rx::msg_handler_channel_events(const pmt::pmt_t &msg)
{
    try
        {
            int64_t message = pmt::to_long(msg);
            rx_message = message;
        }
    catch (const wht::bad_any_cast &e)
        {
            LOG(WARNING) << "msg_handler_channel_events Bad any_cast: " << e.what();
            rx_message = 0;
        }
}


PcpsAcquisitionDopplerNarrowingTest_msg_rx::PcpsAcquisitionDopplerNarrowingTest_msg_rx()
    : gr::block("PcpsAcquisitionDopplerNarrowingTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"),
#if HAS_GENERIC_LAMBDA
        [this](auto &&PH1) { msg_handler_channel_events(std::forward<decltype(PH1)>(PH1)); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
        boost::bind(&PcpsAcquisitionDopplerNarrowingTest_msg_rx::msg_handler_channel_events, this, boost::placeholders::_1));
#else
        boost::bind(&PcpsAcquisitionDopplerNarrowingTest_msg_rx::msg_handler_channel_events, this, _1));
#endif
#endif
}


// ###########################################################

// Ground truth for signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat, the same
// capture gps_l1_ca_pcps_acquisition_test.cc's ValidationOfResults uses.
constexpr double kTrueDopplerHz = 1680.0;
constexpr double kTrueDelaySamples = 524.0;
constexpr int kFsIn = 4000000;

class PcpsAcquisitionDopplerNarrowingTest : public ::testing::Test
{
protected:
    PcpsAcquisitionDopplerNarrowingTest()
    {
        config = std::make_shared<InMemoryConfiguration>();
        gnss_synchro = Gnss_Synchro();
    }

    // doppler_max/doppler_step are chosen by each test to control how many
    // full-grid Doppler bins pcps_acquisition computes
    // (d_num_doppler_bins == ceil(2*doppler_max/doppler_step)), since that bin
    // count is exactly what the reviewed bug depended on.
    void init(unsigned int doppler_max, unsigned int doppler_step, bool enable_doppler_narrowing, bool use_cfar);

    gr::top_block_sptr top_block;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
};


void PcpsAcquisitionDopplerNarrowingTest::init(unsigned int doppler_max, unsigned int doppler_step, bool enable_doppler_narrowing, bool use_cfar)
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 1;
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(kFsIn));
    config->set_property("Acquisition_1C.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_1C.item_type", "gr_complex");
    config->set_property("Acquisition_1C.coherent_integration_time_ms", "1");
    config->set_property("Acquisition_1C.dump", "false");
    config->set_property("Acquisition_1C.doppler_max", std::to_string(doppler_max));
    config->set_property("Acquisition_1C.doppler_step", std::to_string(doppler_step));
    config->set_property("Acquisition_1C.repeat_satellite", "false");
    config->set_property("Acquisition_1C.enable_doppler_narrowing", enable_doppler_narrowing ? "true" : "false");
    if (use_cfar)
        {
            // pfa > 0 both switches Acq_Conf::use_CFAR_algorithm_flag to true and
            // makes d_threshold/d_threshold_narrowed computed (not the fixed
            // .threshold value) -- needed to exercise the PFA-recalibration fix.
            // 0.001 is the same value used by several other acquisition tests in
            // this suite (e.g. Galileo E1, GLONASS) against comparably strong
            // captures.
            config->set_property("Acquisition_1C.pfa", "0.001");
        }
    else
        {
            // pfa left at its 0.0 default -> Acq_Conf::use_CFAR_algorithm_flag is
            // false (peak-ratio statistic) and the fixed .threshold below is used.
            config->set_property("Acquisition_1C.threshold", "0.001");
        }
}


namespace
{
// Runs one acquisition attempt against the known-truth capture and returns
// the completed adapter + received message code, so each TEST_F only has to
// state its own doppler_center/doppler_uncertainty/assertions.
struct RunResult
{
    int rx_message;
    double doppler_error_hz;
    double delay_error_chips;
};

RunResult run_acquisition(gr::top_block_sptr &top_block, InMemoryConfiguration *config, Gnss_Synchro &gnss_synchro,
    int doppler_center, unsigned int doppler_uncertainty)
{
    top_block = gr::make_top_block("Doppler narrowing acquisition test");
    auto acquisition = std::make_shared<PcpsAcquisitionAdapter>(config, "Acquisition_1C", "GPS_L1_CA_PCPS_Acquisition", 1, 0, GPS_1C);
    auto msg_rx = PcpsAcquisitionDopplerNarrowingTest_msg_rx_make();

    acquisition->set_channel(1);
    acquisition->set_gnss_synchro(&gnss_synchro);
    acquisition->connect(top_block);

    std::string path = std::string(TEST_PATH);
    std::string file = path + "signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat";
    gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file.c_str(), false);
    top_block->connect(file_source, 0, acquisition->get_left_block(), 0);
    top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));

    acquisition->set_local_code();
    // Simulates GNSSFlowgraph::acquisition_manager() calling
    // assist_acquisition_doppler(): the caller decides both the center and
    // whether it's exact (doppler_uncertainty == 0) or not (!= 0).
    acquisition->set_doppler_center(doppler_center);
    acquisition->set_doppler_uncertainty(doppler_uncertainty);
    acquisition->reset();

    top_block->run();

    const double doppler_error_hz = std::abs(kTrueDopplerHz - gnss_synchro.Acq_doppler_hz);
    const auto delay_error_samples = std::abs(kTrueDelaySamples - gnss_synchro.Acq_delay_samples);
    const auto delay_error_chips = delay_error_samples * GPS_L1_CA_CODE_LENGTH_CHIPS / kFsIn * (GPS_L1_CA_CODE_RATE_CPS / GPS_L1_CA_CODE_LENGTH_CHIPS);

    return {msg_rx->rx_message, doppler_error_hz, delay_error_chips};
}
}  // namespace


TEST_F(PcpsAcquisitionDopplerNarrowingTest /*unused*/, NarrowingDisabled /*unused*/)
{
    // enable_doppler_narrowing = false: even though the caller passes
    // doppler_uncertainty == 0 (as an assisted acquisition would), the full
    // configured Doppler grid must still be searched.
    init(/*doppler_max=*/5000, /*doppler_step=*/100, /*enable_doppler_narrowing=*/false, /*use_cfar=*/false);

    const RunResult result = run_acquisition(top_block, config.get(), gnss_synchro, static_cast<int>(kTrueDopplerHz), 0);

    ASSERT_EQ(1, result.rx_message) << "Acquisition failure with narrowing disabled.";
    EXPECT_LE(result.doppler_error_hz, 666) << "Doppler error exceeds the expected value: 666 Hz = 2/(3*integration period)";
    EXPECT_LT(result.delay_error_chips, 0.5) << "Delay error exceeds the expected value: 0.5 chips";
}


TEST_F(PcpsAcquisitionDopplerNarrowingTest /*unused*/, NarrowingEnabledOneBinGrid /*unused*/)
{
    // doppler_step >= 2*doppler_max -> d_num_doppler_bins == 1. Regression test
    // for the P1 fix: narrowing must NOT force a second active bin into a
    // 1-row grid allocation (that was an out-of-bounds write). With the
    // d_num_doppler_bins > 1 guard, narrowing silently stays off here and the
    // block just runs its (degenerate, single-hypothesis) full grid -- centered
    // so that its one bin lands exactly on the true Doppler, so a successful
    // acquisition here also confirms the fallback behaves correctly, not just
    // "didn't crash".
    init(/*doppler_max=*/5000, /*doppler_step=*/10000, /*enable_doppler_narrowing=*/true, /*use_cfar=*/false);

    // The single full-grid bin sits at doppler_center - doppler_max; choose
    // doppler_center so that bin lands exactly on the true Doppler.
    const int doppler_center = static_cast<int>(kTrueDopplerHz) + 5000;
    const RunResult result = run_acquisition(top_block, config.get(), gnss_synchro, doppler_center, 0);

    ASSERT_EQ(1, result.rx_message) << "Acquisition failure with a 1-bin full grid (narrowing should have stayed disabled, not crashed).";
    EXPECT_LE(result.doppler_error_hz, 666) << "Doppler error exceeds the expected value: 666 Hz = 2/(3*integration period)";
    EXPECT_LT(result.delay_error_chips, 0.5) << "Delay error exceeds the expected value: 0.5 chips";
}


TEST_F(PcpsAcquisitionDopplerNarrowingTest /*unused*/, NarrowingEnabledTwoBinGrid /*unused*/)
{
    // doppler_max=5000, doppler_step=6000 -> d_num_doppler_bins == 2, exactly
    // matching narrowed mode's own active-bin count. Regression test for the
    // other half of the P1 fix: the old code inferred "is this narrowed?" as
    // active_bins < full_bins, which is false here (2 < 2) even though this
    // *is* a narrowed request, so it silently fell back to a plain 2-bin full
    // grid instead of {known center, center + doppler_max}. That fallback's
    // nearest bin to the true Doppler is 1000 Hz off (a near-total phase
    // cancellation over the 1 ms coherent window), while the correct assisted
    // bin 0 lands exactly on it.
    init(/*doppler_max=*/5000, /*doppler_step=*/6000, /*enable_doppler_narrowing=*/true, /*use_cfar=*/false);

    const RunResult result = run_acquisition(top_block, config.get(), gnss_synchro, static_cast<int>(kTrueDopplerHz), 0);

    ASSERT_EQ(1, result.rx_message) << "Acquisition failure: narrowing likely did not activate for a 2-bin full grid (see d_doppler_search_narrowed).";
    EXPECT_LE(result.doppler_error_hz, 200) << "Doppler error indicates the un-narrowed 2-bin fallback ran instead of the assisted {center, center+doppler_max} pair.";
}


TEST_F(PcpsAcquisitionDopplerNarrowingTest /*unused*/, CfarExcludesReferenceBinFromCandidacy /*unused*/)
{
    // Deliberately mis-center the assist so the *true* signal falls on bin 1
    // (the CFAR noise-reference bin), not bin 0 (the only real candidate).
    // Regression test for the P2 fix: previously both statistic functions
    // scanned every computed bin, so bin 1's strong real correlation could win
    // and get reported as if it were the trusted assisted center. With the
    // fix, only bin 0 (pure noise here) is ever a candidate.
    init(/*doppler_max=*/5000, /*doppler_step=*/5000, /*enable_doppler_narrowing=*/true, /*use_cfar=*/true);

    const int doppler_center = static_cast<int>(kTrueDopplerHz) - 5000;  // bin 0 = center (noise); bin 1 = center + 5000 = true Doppler
    const RunResult result = run_acquisition(top_block, config.get(), gnss_synchro, doppler_center, 0);

    EXPECT_GT(result.doppler_error_hz, 500) << "The noise-reference bin (bin 1) was reported as the acquisition result.";
}


TEST_F(PcpsAcquisitionDopplerNarrowingTest /*unused*/, PeakRatioExcludesReferenceBinFromCandidacy /*unused*/)
{
    // Same construction as CfarExcludesReferenceBinFromCandidacy, exercising
    // the non-CFAR (first_vs_second_peak_statistic) path instead.
    init(/*doppler_max=*/5000, /*doppler_step=*/5000, /*enable_doppler_narrowing=*/true, /*use_cfar=*/false);

    const int doppler_center = static_cast<int>(kTrueDopplerHz) - 5000;
    const RunResult result = run_acquisition(top_block, config.get(), gnss_synchro, doppler_center, 0);

    EXPECT_GT(result.doppler_error_hz, 500) << "The noise-reference bin (bin 1) was reported as the acquisition result.";
}


TEST_F(PcpsAcquisitionDopplerNarrowingTest /*unused*/, FullToNarrowTransition /*unused*/)
{
    // Toggle doppler_uncertainty twice before running (full -> narrow -> full),
    // exercising set_doppler_uncertainty()'s resize/rebuild path
    // (update_grid_doppler_wipeoffs()) more than once on the same block
    // instance, then verify the final (full-grid) state still acquires
    // correctly -- regression coverage for the dump/grid-sizing fix (section 5)
    // interacting with d_doppler_search_narrowed toggling.
    init(/*doppler_max=*/5000, /*doppler_step=*/100, /*enable_doppler_narrowing=*/true, /*use_cfar=*/false);

    top_block = gr::make_top_block("Doppler narrowing transition test");
    auto acquisition = std::make_shared<PcpsAcquisitionAdapter>(config.get(), "Acquisition_1C", "GPS_L1_CA_PCPS_Acquisition", 1, 0, GPS_1C);
    auto msg_rx = PcpsAcquisitionDopplerNarrowingTest_msg_rx_make();

    acquisition->set_channel(1);
    acquisition->set_gnss_synchro(&gnss_synchro);
    acquisition->connect(top_block);

    std::string path = std::string(TEST_PATH);
    std::string file = path + "signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat";
    gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file.c_str(), false);
    top_block->connect(file_source, 0, acquisition->get_left_block(), 0);
    top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));

    acquisition->set_local_code();
    acquisition->set_doppler_center(static_cast<int>(kTrueDopplerHz));
    acquisition->set_doppler_uncertainty(0);  // narrow
    acquisition->set_doppler_uncertainty(1);  // back to full grid
    acquisition->reset();

    top_block->run();

    ASSERT_EQ(1, msg_rx->rx_message) << "Acquisition failure after a narrow->full transition.";
    const double doppler_error_hz = std::abs(kTrueDopplerHz - gnss_synchro.Acq_doppler_hz);
    EXPECT_LE(doppler_error_hz, 666) << "Doppler error exceeds the expected value: 666 Hz = 2/(3*integration period)";
}


namespace
{
// Runs a single-step, full-grid (unnarrowed) acquisition against
// GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat -- the same capture
// ValidationOfResultsMakeTwoStep in gps_l1_ca_pcps_acquisition_test.cc uses
// for two-step acquisition -- and returns the Doppler it finds. That capture's
// true Doppler/delay aren't asserted anywhere else in this test suite (the
// existing two-step test only checks for a successful acquisition), so this
// probes it directly instead of hard-coding an assumed ground truth.
double probe_doppler_hz_4ms_capture(std::shared_ptr<InMemoryConfiguration> &config, Gnss_Synchro &gnss_synchro)
{
    gr::top_block_sptr probe_top_block = gr::make_top_block("Doppler probe");
    // supersede_property, not set_property: init() already set these keys once,
    // and InMemoryConfiguration::set_property() is a std::map::insert, so a
    // second set_property() call for the same key is a silent no-op.
    config->supersede_property("Acquisition_1C.doppler_max", "10000");
    config->supersede_property("Acquisition_1C.doppler_step", "100");
    config->supersede_property("Acquisition_1C.enable_doppler_narrowing", "false");

    auto acquisition = std::make_shared<PcpsAcquisitionAdapter>(config.get(), "Acquisition_1C", "GPS_L1_CA_PCPS_Acquisition", 1, 0, GPS_1C);
    auto msg_rx = PcpsAcquisitionDopplerNarrowingTest_msg_rx_make();

    acquisition->set_channel(1);
    acquisition->set_gnss_synchro(&gnss_synchro);
    acquisition->connect(probe_top_block);

    std::string path = std::string(TEST_PATH);
    std::string file = path + "signal_samples/GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat";
    gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file.c_str(), false);
    probe_top_block->connect(file_source, 0, acquisition->get_left_block(), 0);
    probe_top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));

    acquisition->set_local_code();
    acquisition->reset();
    probe_top_block->run();

    EXPECT_EQ(1, msg_rx->rx_message) << "Probe acquisition (full grid, no narrowing) failed to find a Doppler to assist with.";
    return gnss_synchro.Acq_doppler_hz;
}
}  // namespace


TEST_F(PcpsAcquisitionDopplerNarrowingTest /*unused*/, TwoStepAcquisitionWithNarrowing /*unused*/)
{
    // Step 1 narrows (2-bin grid, assisted); step 2 (the fine local search
    // around step 1's result) never narrows -- assisted is unconditionally
    // false when d_step_two is true (compute_statistics()) -- so it should
    // still run its full second_nbins-wide fine search and improve on step 1's
    // coarse result. Uses the same 4 ms capture as
    // ValidationOfResultsMakeTwoStep in gps_l1_ca_pcps_acquisition_test.cc,
    // since two-step acquisition needs more samples than the 2 ms capture the
    // other tests in this file use provides.
    init(/*doppler_max=*/5000, /*doppler_step=*/6000, /*enable_doppler_narrowing=*/true, /*use_cfar=*/false);
    const auto assist_doppler_hz = probe_doppler_hz_4ms_capture(config, gnss_synchro);

    gnss_synchro = Gnss_Synchro();
    // Restore the narrowed 2-bin step-1 settings the probe above overrode, and
    // add the two-step-specific keys (not touched by init(), so a plain
    // set_property() is fine for these).
    config->supersede_property("Acquisition_1C.doppler_max", "5000");
    config->supersede_property("Acquisition_1C.doppler_step", "6000");
    config->supersede_property("Acquisition_1C.enable_doppler_narrowing", "true");
    config->set_property("Acquisition_1C.make_two_steps", "true");
    config->set_property("Acquisition_1C.second_nbins", "5");
    config->set_property("Acquisition_1C.second_doppler_step", "20");

    top_block = gr::make_top_block("Doppler narrowing two-step test");
    auto acquisition = std::make_shared<PcpsAcquisitionAdapter>(config.get(), "Acquisition_1C", "GPS_L1_CA_PCPS_Acquisition", 1, 0, GPS_1C);
    auto msg_rx = PcpsAcquisitionDopplerNarrowingTest_msg_rx_make();

    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 1;

    acquisition->set_channel(1);
    acquisition->set_gnss_synchro(&gnss_synchro);
    acquisition->connect(top_block);

    std::string path = std::string(TEST_PATH);
    std::string file = path + "signal_samples/GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat";
    gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file.c_str(), false);
    top_block->connect(file_source, 0, acquisition->get_left_block(), 0);
    top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));

    acquisition->set_local_code();
    acquisition->set_doppler_center(static_cast<int>(std::lround(assist_doppler_hz)));
    acquisition->set_doppler_uncertainty(0);
    acquisition->reset();

    top_block->run();

    ASSERT_EQ(1, msg_rx->rx_message) << "Acquisition failure with two-step acquisition + step-1 narrowing.";
    const double doppler_error_hz = std::abs(assist_doppler_hz - gnss_synchro.Acq_doppler_hz);
    // Tolerance covers both the wide probe's own 100 Hz step quantization and
    // step 2's 20 Hz fine-search quantization, not just step 2's alone.
    EXPECT_LE(doppler_error_hz, 150) << "Step 2's fine search should have converged close to the (narrowed) step-1 estimate.";
}


TEST_F(PcpsAcquisitionDopplerNarrowingTest /*unused*/, NarrowedDumpMetadata /*unused*/)
{
    // Regression test for the section 5 fix: a narrowed dump must describe
    // itself accurately (2 columns, {0, doppler_max} encoding, an explicit
    // doppler_narrowed flag) instead of claiming the full grid's width/step
    // while actually only having written 2 live columns.
    init(/*doppler_max=*/5000, /*doppler_step=*/5000, /*enable_doppler_narrowing=*/true, /*use_cfar=*/false);
    // supersede_property, not set_property: init() already called set_property
    // for "Acquisition_1C.dump" (InMemoryConfiguration::set_property() is a
    // std::map::insert, so a second set_property() with the same key is a
    // silent no-op -- supersede_property() actually overwrites it).
    config->supersede_property("Acquisition_1C.dump", "true");
    config->supersede_property("Acquisition_1C.dump_filename", "./tmp-acq-narrow-dump/acquisition");
    config->supersede_property("Acquisition_1C.dump_channel", "1");

    std::string data_str = "./tmp-acq-narrow-dump";
    if (fs::exists(data_str))
        {
            fs::remove_all(data_str);
        }
    fs::create_directory(data_str);

    const int doppler_center = static_cast<int>(kTrueDopplerHz);
    const RunResult result = run_acquisition(top_block, config.get(), gnss_synchro, doppler_center, 0);
    ASSERT_EQ(1, result.rx_message) << "Acquisition failure while producing the narrowed dump.";

    const std::string dump_filename = "./tmp-acq-narrow-dump/acquisition_G_1C_ch_1_1_sat_1.mat";
    mat_t *matfp = Mat_Open(dump_filename.c_str(), MAT_ACC_RDONLY);
    ASSERT_NE(matfp, nullptr) << "Could not open narrowed acquisition dump " << dump_filename;

    matvar_t *grid_var = Mat_VarRead(matfp, "acq_grid");
    ASSERT_NE(grid_var, nullptr) << "acq_grid missing from narrowed dump";
    EXPECT_EQ(2U, grid_var->dims[1]) << "Narrowed acq_grid should be 2 columns wide (known bin + reference bin), not the full configured grid.";
    Mat_VarFree(grid_var);

    matvar_t *narrowed_var = Mat_VarRead(matfp, "doppler_narrowed");
    ASSERT_NE(narrowed_var, nullptr) << "doppler_narrowed missing from narrowed dump";
    EXPECT_EQ(1, *static_cast<int32_t *>(narrowed_var->data));
    Mat_VarFree(narrowed_var);

    matvar_t *doppler_max_var = Mat_VarRead(matfp, "doppler_max");
    ASSERT_NE(doppler_max_var, nullptr) << "doppler_max missing from narrowed dump";
    EXPECT_EQ(0, *static_cast<int32_t *>(doppler_max_var->data)) << "Narrowed dumps encode doppler_max=0 (bin 0 == the known center).";
    Mat_VarFree(doppler_max_var);

    matvar_t *doppler_step_var = Mat_VarRead(matfp, "doppler_step");
    ASSERT_NE(doppler_step_var, nullptr) << "doppler_step missing from narrowed dump";
    EXPECT_EQ(5000, *static_cast<int32_t *>(doppler_step_var->data)) << "Narrowed dumps encode doppler_step=doppler_max (bin 1 == center + doppler_max).";
    Mat_VarFree(doppler_step_var);

    matvar_t *doppler_center_var = Mat_VarRead(matfp, "doppler_center");
    ASSERT_NE(doppler_center_var, nullptr) << "doppler_center missing from narrowed dump";
    EXPECT_EQ(doppler_center, *static_cast<int32_t *>(doppler_center_var->data));
    Mat_VarFree(doppler_center_var);

    Mat_Close(matfp);
    fs::remove_all(data_str);
}
