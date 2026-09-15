/*!
 * \file satellite_visibility_test.cc
 * \brief Regression tests for satellite visibility classification and timing
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
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

#include "channel_status_msg_receiver.h"
#include "geofunctions.h"
#include "gnss_sdr_sample_counter.h"
#include "hybrid_observables_gs.h"
#include "in_memory_configuration.h"
#include "monitor_pvt.h"
#include "pvt_conf.h"
#include "pvt_interface.h"
#include "rtklib_conversions.h"
#include "rtklib_ephemeris.h"
#include "rtklib_pvt_gs.h"
#include "rtklib_rtkcmn.h"
#include "rtklib_rtksvr.h"
#include "satellite_visibility.h"
#include "sensor_data/sensor_data_source_configuration.h"
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/null_source.h>
#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

class VisibilityTestPvt : public PvtInterface
{
public:
    std::map<int, Gps_Ephemeris> gps_eph;
    std::map<int, Galileo_Ephemeris> gal_eph;
    std::map<int, Beidou_Dnav_Ephemeris> bds_eph;
    std::map<int, Gps_Almanac> gps_alm;
    std::map<int, Galileo_Almanac> gal_alm;
    std::map<int, Beidou_Dnav_Almanac> bds_alm;
    std::map<int, Gps_CNAV_Ephemeris> gps_cnav_ephemeris;
    std::map<int, Glonass_Gnav_Ephemeris> glonass_ephemeris;
    std::map<int, Glonass_Gnav_Almanac> glonass_almanac;
    Glonass_Gnav_Utc_Model glo_utc;
    mutable unsigned int gps_reads = 0;

    std::string role() override { return "PVT"; }
    std::string implementation() override { return "VisibilityTestPvt"; }
    size_t item_size() override { return 0; }
    void connect(gr::top_block_sptr /*top_block*/) override {}
    void disconnect(gr::top_block_sptr /*top_block*/) override {}
    gr::basic_block_sptr get_left_block() override { return nullptr; }
    gr::basic_block_sptr get_right_block() override { return nullptr; }
    void reset() override {}
    void clear_ephemeris() override {}
    void clear_ephemeris_keep_almanac() override {}
    std::map<int, Gps_Ephemeris> get_gps_ephemeris() const override
    {
        ++gps_reads;
        return gps_eph;
    }
    std::map<int, Galileo_Ephemeris> get_galileo_ephemeris() const override { return gal_eph; }
    std::map<int, Beidou_Dnav_Ephemeris> get_beidou_dnav_ephemeris() const override { return bds_eph; }
    std::map<int, Gps_Almanac> get_gps_almanac() const override { return gps_alm; }
    std::map<int, Galileo_Almanac> get_galileo_almanac() const override { return gal_alm; }
    std::map<int, Beidou_Dnav_Almanac> get_beidou_dnav_almanac() const override { return bds_alm; }
    std::map<int, Gps_CNAV_Ephemeris> get_gps_cnav_ephemeris() const override { return gps_cnav_ephemeris; }
    std::map<int, Glonass_Gnav_Ephemeris> get_glonass_ephemeris() const override { return glonass_ephemeris; }
    std::map<int, Glonass_Gnav_Almanac> get_glonass_almanac() const override { return glonass_almanac; }
    Glonass_Gnav_Utc_Model get_glonass_utc_model() const override { return glo_utc; }
    bool get_latest_PVT(double* /*longitude_deg*/, double* /*latitude_deg*/, double* /*height_m*/, double* /*ground_speed_kmh*/, double* /*course_over_ground_deg*/, time_t* /*UTC_time*/) override { return false; }
};


class SatelliteVisibilityTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        configuration = std::make_shared<InMemoryConfiguration>();
        configuration->set_property("GNSS-SDR.enable_visibility_aware_search", "true");
        pvt = std::make_shared<VisibilityTestPvt>();
        fix = Monitor_Pvt{};
        fix.week = 2400;
        fix.RX_time = 100000.0;
        fix.latitude = 0.0;
        fix.longitude = 0.0;
        fix.height = 0.0;
    }

    void SetGpsAlmanac(int toa)
    {
        Gps_Almanac almanac;
        almanac.PRN = 1;
        almanac.WNa = fix.week % 256;
        almanac.toa = toa;
        almanac.sqrtA = 5153.795;
        almanac.ecc = 0.01;
        pvt->gps_alm[1] = almanac;
    }

    void SetConstellationData(int tow)
    {
        Gps_Ephemeris gps;
        gps.PRN = 1;
        gps.WN = fix.week % 1024;
        gps.toe = gps.toc = gps.tow = tow;
        gps.sqrtA = 5153.795;
        pvt->gps_eph[1] = gps;
        SetGpsAlmanac(tow);
        Gps_Almanac gps_almanac = pvt->gps_alm.at(1);
        gps_almanac.PRN = 2;
        pvt->gps_alm.clear();
        pvt->gps_alm[2] = gps_almanac;

        Galileo_Ephemeris galileo;
        galileo.PRN = 1;
        galileo.WN = fix.week - 1024;
        galileo.toe = galileo.toc = galileo.tow = tow;
        galileo.sqrtA = 5440.0;
        pvt->gal_eph[1] = galileo;
        Galileo_Almanac galileo_almanac;
        galileo_almanac.PRN = 2;
        galileo_almanac.WNa = (fix.week - 1024) % 4;
        galileo_almanac.toa = tow;
        galileo_almanac.sqrtA = 5440.0;
        pvt->gal_alm[2] = galileo_almanac;

        Beidou_Dnav_Ephemeris beidou;
        beidou.PRN = 10;
        beidou.WN = fix.week - 1356;
        beidou.toe = beidou.toc = beidou.tow = tow;
        beidou.sqrtA = 5282.0;
        pvt->bds_eph[10] = beidou;
        Beidou_Dnav_Almanac beidou_almanac;
        beidou_almanac.PRN = 11;
        beidou_almanac.WNa = fix.week - 1356;
        beidou_almanac.toa = tow;
        beidou_almanac.sqrtA = 5282.0;
        pvt->bds_alm[11] = beidou_almanac;
    }

    size_t ClassifiedCount(const gtime_t& time)
    {
        std::vector<std::pair<int, Gnss_Satellite>> excluded;
        const auto visible = compute_visible_satellites(pvt, time, arma::vec{6378137.0, 0.0, 0.0},
            -90.0, &excluded, 259200.0);
        return visible.size() + excluded.size();
    }

    double GpsElevation(double height_m = 0.0) const
    {
        const auto almanac = alm_to_rtklib(pvt->gps_alm.at(1), fix.week);
        double position[3];
        double clock_bias;
        alm2pos(gpst2time(fix.week, fix.RX_time), &almanac, position, &clock_bias);
        const arma::vec receiver{6378137.0 + height_m, 0.0, 0.0};
        const arma::vec satellite{position[0], position[1], position[2]};
        double azimuth;
        double elevation;
        double distance;
        topocent(&azimuth, &elevation, &distance, receiver, satellite - receiver);
        return elevation;
    }

    std::shared_ptr<InMemoryConfiguration> configuration;
    std::shared_ptr<VisibilityTestPvt> pvt;
    Monitor_Pvt fix{};
};


TEST_F(SatelliteVisibilityTest, RejectsWeekOldDataAcrossConstellations)
{
    SetConstellationData(100000);
    EXPECT_EQ(6U, ClassifiedCount(gpst2time(fix.week, fix.RX_time)));
    EXPECT_EQ(0U, ClassifiedCount(gpst2time(fix.week + 1, fix.RX_time)));
}


TEST_F(SatelliteVisibilityTest, AcceptsFreshDataAcrossWeekRollover)
{
    SetConstellationData(604700);
    EXPECT_EQ(6U, ClassifiedCount(gpst2time(fix.week + 1, 100.0)));
}


TEST_F(SatelliteVisibilityTest, PreservesFractionalMaskClassification)
{
    SetGpsAlmanac(100000);
    const double elevation = GpsElevation();
    const double mask = std::floor(elevation) + 0.1;
    ASSERT_GT(elevation, mask);
    configuration->set_property("GNSS-SDR.search_elevation_mask", std::to_string(mask));
    SatelliteVisibility visibility(configuration);
    EXPECT_TRUE(visibility.Tick(pvt, fix));
    EXPECT_TRUE(visibility.IsVisible(Gnss_Satellite("GPS", 1)));
    EXPECT_FALSE(visibility.IsExcluded(Gnss_Satellite("GPS", 1)));
}


TEST_F(SatelliteVisibilityTest, ExcludesUnhealthySatelliteAboveFractionalMask)
{
    SetGpsAlmanac(100000);
    const double mask = std::floor(GpsElevation()) + 0.1;
    configuration->set_property("GNSS-SDR.search_elevation_mask", std::to_string(mask));
    pvt->gps_alm.at(1).SV_health = 1;
    SatelliteVisibility visibility(configuration);
    EXPECT_TRUE(visibility.Tick(pvt, fix));
    EXPECT_FALSE(visibility.IsVisible(Gnss_Satellite("GPS", 1)));
    EXPECT_TRUE(visibility.IsExcluded(Gnss_Satellite("GPS", 1)));
}


TEST_F(SatelliteVisibilityTest, ExcludesHealthySatelliteBelowMask)
{
    SetGpsAlmanac(100000);
    configuration->set_property("GNSS-SDR.search_elevation_mask", std::to_string(GpsElevation() + 0.1));
    SatelliteVisibility visibility(configuration);
    EXPECT_TRUE(visibility.Tick(pvt, fix));
    EXPECT_FALSE(visibility.IsVisible(Gnss_Satellite("GPS", 1)));
    EXPECT_TRUE(visibility.IsExcluded(Gnss_Satellite("GPS", 1)));
}


TEST_F(SatelliteVisibilityTest, PeriodicSweepCrossesWeekRollover)
{
    SetGpsAlmanac(604700);
    fix.RX_time = 604700.0;
    SatelliteVisibility visibility(configuration);
    visibility.Tick(pvt, fix);
    const unsigned int reads = pvt->gps_reads;
    fix.week++;
    fix.RX_time = 19.0;
    visibility.Tick(pvt, fix);
    EXPECT_EQ(reads, pvt->gps_reads);
    fix.RX_time = 20.0;
    visibility.Tick(pvt, fix);
    EXPECT_GT(pvt->gps_reads, reads);
}


TEST_F(SatelliteVisibilityTest, ExpiryCrossesWeekRolloverWithoutPeriodicSweep)
{
    configuration->set_property("GNSS-SDR.visibility_recompute_interval_s", "1000000");
    configuration->set_property("GNSS-SDR.visibility_almanac_max_age_s", "200");
    SetGpsAlmanac(604700);
    fix.RX_time = 604790.0;
    SatelliteVisibility visibility(configuration);
    visibility.Tick(pvt, fix);
    const Gnss_Satellite satellite("GPS", 1);
    ASSERT_TRUE(visibility.IsVisible(satellite) || visibility.IsExcluded(satellite));
    fix.week++;
    fix.RX_time = 101.0;
    EXPECT_TRUE(visibility.Tick(pvt, fix));
    EXPECT_FALSE(visibility.IsVisible(satellite));
    EXPECT_FALSE(visibility.IsExcluded(satellite));
}


TEST_F(SatelliteVisibilityTest, StaleFixReclassifiesRisingSatellite)
{
    SetGpsAlmanac(100000);
    pvt->gps_alm.at(1).M_0 = -0.5;
    const double initial_elevation = GpsElevation();
    fix.RX_time += 21600.0;
    const double later_elevation = GpsElevation();
    fix.RX_time -= 21600.0;
    ASSERT_LT(initial_elevation, 0.0);
    ASSERT_GT(later_elevation, 0.0);
    SatelliteVisibility visibility(configuration);
    const Gnss_Satellite satellite("GPS", 1);
    ASSERT_TRUE(visibility.Tick(pvt, fix, 0.0));
    ASSERT_TRUE(visibility.IsExcluded(satellite));
    // No new fix or navigation message arrives, only more samples.
    EXPECT_TRUE(visibility.Tick(pvt, fix, 21600.0));
    EXPECT_TRUE(visibility.IsVisible(satellite));
    EXPECT_FALSE(visibility.IsExcluded(satellite));
}


TEST_F(SatelliteVisibilityTest, StaleFixExpiresAcrossWeekRollover)
{
    configuration->set_property("GNSS-SDR.visibility_recompute_interval_s", "1000000");
    configuration->set_property("GNSS-SDR.visibility_almanac_max_age_s", "200");
    SetGpsAlmanac(604700);
    fix.RX_time = 604790.0;
    SatelliteVisibility visibility(configuration);
    const Gnss_Satellite satellite("GPS", 1);
    ASSERT_TRUE(visibility.Tick(pvt, fix, 0.0));
    EXPECT_TRUE(visibility.Tick(pvt, fix, 111.0));
    EXPECT_FALSE(visibility.IsVisible(satellite));
    EXPECT_FALSE(visibility.IsExcluded(satellite));
}


TEST_F(SatelliteVisibilityTest, FreshFixReanchorsSampleClock)
{
    configuration->set_property("GNSS-SDR.visibility_recompute_interval_s", "1000000");
    configuration->set_property("GNSS-SDR.visibility_almanac_max_age_s", "200");
    SetGpsAlmanac(100000);
    SatelliteVisibility visibility(configuration);
    const Gnss_Satellite satellite("GPS", 1);
    ASSERT_TRUE(visibility.Tick(pvt, fix, 0.0));
    fix.RX_time += 100.0;
    visibility.Tick(pvt, fix, 100.0);
    visibility.Tick(pvt, fix, 150.0);
    EXPECT_TRUE(visibility.IsVisible(satellite) || visibility.IsExcluded(satellite));
    EXPECT_TRUE(visibility.Tick(pvt, fix, 201.0));
    EXPECT_FALSE(visibility.IsVisible(satellite));
    EXPECT_FALSE(visibility.IsExcluded(satellite));
}


TEST_F(SatelliteVisibilityTest, AgnssReferenceExpiresWithoutFirstFix)
{
    configuration->set_property("GNSS-SDR.AGNSS_ref_location", "0,0");
    configuration->set_property("GNSS-SDR.AGNSS_ref_utc_time", "01/01/2024 00:00:00");
    configuration->set_property("GNSS-SDR.visibility_recompute_interval_s", "1000000");
    configuration->set_property("GNSS-SDR.visibility_almanac_max_age_s", "200");
    const double utc_epoch[6] = {2024, 1, 1, 0, 0, 0};
    int week = 0;
    const double tow = time2gpst(utc2gpst(epoch2time(utc_epoch)), &week);
    fix.week = week;
    SetGpsAlmanac(static_cast<int>(tow));
    fix.RX_time = -1.0;
    SatelliteVisibility visibility(configuration);
    const Gnss_Satellite satellite("GPS", 1);
    ASSERT_TRUE(visibility.Tick(pvt, fix, 0.0));
    EXPECT_TRUE(visibility.Tick(pvt, fix, 201.0));
    EXPECT_FALSE(visibility.IsVisible(satellite));
    EXPECT_FALSE(visibility.IsExcluded(satellite));
}


TEST_F(SatelliteVisibilityTest, CommandReferenceWorksWithoutFixAndAnchorsSampleClock)
{
    configuration->set_property("GNSS-SDR.visibility_recompute_interval_s", "1000000");
    configuration->set_property("GNSS-SDR.visibility_almanac_max_age_s", "200");
    SetGpsAlmanac(100000);
    const time_t utc = gpst2utc(gpst2time(fix.week, fix.RX_time)).time;
    fix.RX_time = -1.0;
    SatelliteVisibility visibility(configuration);
    const Gnss_Satellite satellite("GPS", 1);
    EXPECT_FALSE(visibility.Tick(pvt, fix, 900000.0));
    visibility.SetCommandReference(utc, {0.0F, 0.0F, 0.0F}, fix, 900000.0);
    ASSERT_TRUE(visibility.Tick(pvt, fix, 900000.0));
    EXPECT_TRUE(visibility.IsVisible(satellite) || visibility.IsExcluded(satellite));
    visibility.Tick(pvt, fix, 900150.0);
    EXPECT_TRUE(visibility.IsVisible(satellite) || visibility.IsExcluded(satellite));
    EXPECT_TRUE(visibility.Tick(pvt, fix, 900201.0));
    EXPECT_FALSE(visibility.IsVisible(satellite));
    EXPECT_FALSE(visibility.IsExcluded(satellite));
}


TEST_F(SatelliteVisibilityTest, CommandReferenceOverridesConfiguredAssistance)
{
    configuration->set_property("GNSS-SDR.AGNSS_ref_location", "0,0");
    configuration->set_property("GNSS-SDR.AGNSS_ref_utc_time", "01/01/2024 00:00:00");
    SetGpsAlmanac(100000);
    const time_t utc = gpst2utc(gpst2time(fix.week, fix.RX_time)).time;
    fix.RX_time = -1.0;
    SatelliteVisibility visibility(configuration);
    const Gnss_Satellite satellite("GPS", 1);
    visibility.Tick(pvt, fix, 100.0);
    ASSERT_FALSE(visibility.IsVisible(satellite));
    ASSERT_FALSE(visibility.IsExcluded(satellite));
    visibility.SetCommandReference(utc, {0.0F, 0.0F, 0.0F}, fix, 100.0);
    EXPECT_TRUE(visibility.Tick(pvt, fix, 100.0));
    EXPECT_TRUE(visibility.IsVisible(satellite) || visibility.IsExcluded(satellite));
}


TEST_F(SatelliteVisibilityTest, CommandReferenceOverridesRetainedFixUntilNewEpoch)
{
    SetGpsAlmanac(100000);
    pvt->gps_alm.at(1).M_0 = -0.5;
    SatelliteVisibility visibility(configuration);
    const Gnss_Satellite satellite("GPS", 1);
    ASSERT_TRUE(visibility.Tick(pvt, fix, 100.0));
    ASSERT_TRUE(visibility.IsExcluded(satellite));
    const time_t utc = gpst2utc(gpst2time(fix.week, fix.RX_time + 21600.0)).time;
    visibility.SetCommandReference(utc, {0.0F, 0.0F, 0.0F}, fix, 100.0);
    ASSERT_TRUE(visibility.Tick(pvt, fix, 100.0));
    ASSERT_TRUE(visibility.IsVisible(satellite));
    // Even after another full sweep, the unchanged pre-command fix is ignored.
    visibility.Tick(pvt, fix, 221.0);
    EXPECT_TRUE(visibility.IsVisible(satellite));
    fix.RX_time += 1.0;
    EXPECT_TRUE(visibility.Tick(pvt, fix, 222.0));
    EXPECT_TRUE(visibility.IsExcluded(satellite));
    EXPECT_FALSE(visibility.IsVisible(satellite));
}


TEST_F(SatelliteVisibilityTest, CommandReferenceForcesPositionRefreshAndIncludesHeight)
{
    SetGpsAlmanac(100000);
    const double ground_elevation = GpsElevation();
    const double raised_elevation = GpsElevation(2000.0);
    ASSERT_GT(ground_elevation, raised_elevation);
    configuration->set_property("GNSS-SDR.search_elevation_mask", std::to_string((ground_elevation + raised_elevation) / 2.0));
    configuration->set_property("GNSS-SDR.visibility_recompute_position_threshold_m", "1000000000");
    configuration->set_property("GNSS-SDR.visibility_recompute_interval_s", "1000000");
    SatelliteVisibility visibility(configuration);
    const Gnss_Satellite satellite("GPS", 1);
    ASSERT_TRUE(visibility.Tick(pvt, fix, 100.0));
    ASSERT_TRUE(visibility.IsVisible(satellite));
    const time_t utc = gpst2utc(gpst2time(fix.week, fix.RX_time)).time;
    visibility.SetCommandReference(utc, {0.0F, 0.0F, 2000.0F}, fix, 100.0);
    EXPECT_TRUE(visibility.Tick(pvt, fix, 100.0));
    EXPECT_TRUE(visibility.IsExcluded(satellite));
    // Repeated commands also force a refresh below the movement/time thresholds.
    visibility.SetCommandReference(utc, {0.0F, 0.0F, 0.0F}, fix, 100.0);
    EXPECT_TRUE(visibility.Tick(pvt, fix, 100.0));
    EXPECT_TRUE(visibility.IsVisible(satellite));
}


TEST_F(SatelliteVisibilityTest, ObservablesReportSampleTimeWithoutTracking)
{
    Obs_Conf conf;
    conf.nchannels_in = 2;
    conf.nchannels_out = 1;
    const auto observables = hybrid_observables_gs_make(conf);
    const auto samples = gr::blocks::null_source::make(sizeof(gr_complex));
    const auto tracking = gr::blocks::null_source::make(sizeof(Gnss_Synchro));
    const auto counter = gnss_sdr_make_sample_counter(4000.0, 20, sizeof(gr_complex));
    const auto sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
    const auto status = channel_status_msg_receiver_make();
    const auto flowgraph = gr::make_top_block("visibility_sample_clock_test");
    flowgraph->connect(samples, 0, counter, 0);
    flowgraph->connect(counter, 0, observables, 1);
    flowgraph->connect(tracking, 0, observables, 0);
    flowgraph->connect(observables, 0, sink, 0);
    flowgraph->msg_connect(observables, pmt::mp("status"), status, pmt::mp("status"));
    flowgraph->start();
    double receiver_time_s = 0.0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (receiver_time_s < 2.0 && std::chrono::steady_clock::now() < deadline)
        {
            status->get_current_status_pvt(&receiver_time_s);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    flowgraph->stop();
    flowgraph->wait();
    EXPECT_GE(receiver_time_s, 2.0);
    EXPECT_LT(status->get_current_status_pvt().RX_time, 0.0);
    EXPECT_TRUE(status->get_current_tracking_map().empty());
}


TEST_F(SatelliteVisibilityTest, DisabledSearchDoesNotReadNavigationData)
{
    auto disabled_configuration = std::make_shared<InMemoryConfiguration>();
    SatelliteVisibility visibility(disabled_configuration);
    SetGpsAlmanac(100000);
    EXPECT_FALSE(visibility.Tick(pvt, fix));
    EXPECT_EQ(0U, pvt->gps_reads);
}


TEST_F(SatelliteVisibilityTest, ConcurrentNavigationSnapshotsAndClears)
{
    Pvt_Conf conf;
    conf.output_enabled = false;
    conf.rinex_output_enabled = false;
    conf.gpx_output_enabled = false;
    conf.geojson_output_enabled = false;
    conf.nmea_output_file_enabled = false;
    conf.kml_output_enabled = false;
    conf.xml_output_enabled = false;
    conf.rtcm_output_file_enabled = false;
    conf.enable_rx_clock_correction = false;
    conf.output_rate_ms = 1000;
    conf.display_rate_ms = 1000;
    conf.signal_enabled_flags = 1;
    rtk_t rtk{};
    rtk.opt = PRCOPT_DEFAULT;
    const SensorDataSourceConfiguration sensor_configuration(configuration.get());
    const auto block = rtklib_make_pvt_gs(1, conf, rtk, sensor_configuration);
    const auto source = gr::blocks::null_source::make(sizeof(Gnss_Synchro));
    const auto flowgraph = gr::make_top_block("visibility_navigation_snapshot_test");
    flowgraph->connect(source, 0, block, 0);
    flowgraph->start();

    // Telemetry runs on the GNU Radio worker while snapshots and clears run
    // on this thread, just as they do during visibility-aware acquisition.
    for (int update = 1; update <= 256; ++update)
        {
            auto almanac = std::make_shared<Gps_Almanac>();
            almanac->PRN = 1 + update % 32;
            almanac->toa = update;
            almanac->af0 = update;
            almanac->af1 = -update;
            block->_post(pmt::mp("telemetry"), pmt::make_any(almanac));
            for (const auto& entry : block->get_gps_almanac_map())
                {
                    EXPECT_EQ(entry.first, static_cast<int>(entry.second.PRN));
                    EXPECT_DOUBLE_EQ(entry.second.toa, entry.second.af0);
                    EXPECT_DOUBLE_EQ(-entry.second.toa, entry.second.af1);
                }
            if (update % 32 == 0)
                {
                    block->clear_ephemeris();
                }
        }

    // Wait for a final record after the last clear, proving that the worker
    // consumed telemetry and did not deadlock against the snapshot getter.
    auto sentinel = std::make_shared<Gps_Almanac>();
    sentinel->PRN = 1;
    sentinel->toa = 10000;
    block->_post(pmt::mp("telemetry"), pmt::make_any(sentinel));
    bool received = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline)
        {
            const auto snapshot = block->get_gps_almanac_map();
            const auto it = snapshot.find(1);
            if (it != snapshot.end() && it->second.toa == sentinel->toa)
                {
                    received = true;
                    break;
                }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    EXPECT_TRUE(received);
    const auto retained_almanac = block->get_gps_almanac_map();
    auto gps = std::make_shared<Gps_Ephemeris>();
    gps->PRN = 1;
    auto galileo = std::make_shared<Galileo_Ephemeris>();
    galileo->PRN = 1;
    galileo->nav_message_type = Galileo_Nav_Message_Type::INAV;
    auto galileo_almanac = std::make_shared<Galileo_Almanac>();
    galileo_almanac->PRN = 1;
    auto beidou = std::make_shared<Beidou_Dnav_Ephemeris>();
    beidou->PRN = 10;
    auto beidou_almanac = std::make_shared<Beidou_Dnav_Almanac>();
    beidou_almanac->PRN = 10;
    block->_post(pmt::mp("telemetry"), pmt::make_any(gps));
    block->_post(pmt::mp("telemetry"), pmt::make_any(galileo));
    block->_post(pmt::mp("telemetry"), pmt::make_any(galileo_almanac));
    block->_post(pmt::mp("telemetry"), pmt::make_any(beidou));
    block->_post(pmt::mp("telemetry"), pmt::make_any(beidou_almanac));
    const auto navigation_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (block->get_beidou_dnav_almanac_map().empty() && std::chrono::steady_clock::now() < navigation_deadline)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    EXPECT_EQ(1U, block->get_gps_ephemeris_map().size());
    EXPECT_EQ(1U, block->get_galileo_ephemeris_map().size());
    EXPECT_EQ(1U, block->get_galileo_almanac_map().size());
    EXPECT_EQ(1U, block->get_beidou_dnav_ephemeris_map().size());
    EXPECT_EQ(1U, block->get_beidou_dnav_almanac_map().size());

    // Clear must be immediately visible to readers. The first subsequent
    // message must also clear the worker's maps before publishing new data.
    block->clear_ephemeris();
    EXPECT_TRUE(block->get_gps_almanac_map().empty());
    EXPECT_TRUE(block->get_gps_ephemeris_map().empty());
    EXPECT_TRUE(block->get_galileo_ephemeris_map().empty());
    EXPECT_TRUE(block->get_galileo_almanac_map().empty());
    EXPECT_TRUE(block->get_beidou_dnav_ephemeris_map().empty());
    EXPECT_TRUE(block->get_beidou_dnav_almanac_map().empty());
    auto replacement = std::make_shared<Gps_Almanac>(*sentinel);
    replacement->PRN = 2;
    block->_post(pmt::mp("telemetry"), pmt::make_any(replacement));
    const auto replacement_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (block->get_gps_almanac_map().count(2) == 0 && std::chrono::steady_clock::now() < replacement_deadline)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    const auto replaced = block->get_gps_almanac_map();
    EXPECT_EQ(1U, replaced.size());
    EXPECT_EQ(1U, replaced.count(2));
    EXPECT_EQ(0U, replaced.count(1));
    EXPECT_EQ(sentinel->toa, retained_almanac.at(1).toa);
    EXPECT_TRUE(block->get_gps_ephemeris_map().empty());
    EXPECT_TRUE(block->get_galileo_ephemeris_map().empty());
    EXPECT_TRUE(block->get_galileo_almanac_map().empty());
    EXPECT_TRUE(block->get_beidou_dnav_ephemeris_map().empty());
    EXPECT_TRUE(block->get_beidou_dnav_almanac_map().empty());

    double longitude;
    double latitude;
    double height;
    double speed;
    double course;
    time_t utc;
    EXPECT_FALSE(block->get_latest_PVT(&longitude, &latitude, &height, &speed, &course, &utc));
    flowgraph->stop();
    flowgraph->wait();
    block->clear_ephemeris();
    EXPECT_TRUE(block->get_gps_almanac_map().empty());
}


TEST_F(SatelliteVisibilityTest, GlonassAlmanacMatchesIcdExample)
{
    // GLONASS ICD 5.1, Appendix A.3.2.3. Published coordinates are inertial
    // kilometres; rotate them to ECEF using the example's sidereal angle.
    const double pi = std::acos(-1.0);
    Glonass_Gnav_Almanac almanac;
    almanac.d_N_4 = 2;
    almanac.d_N_A = 615;
    almanac.d_lambda_n_A = -0.189986229 * pi;
    almanac.d_t_lambda_n_A = 27122.09375;
    almanac.d_Delta_i_n_A = 0.011929512 * pi;
    almanac.d_Delta_T_n_A = -2655.76171875;
    almanac.d_Delta_T_n_A_dot = 0.000549316;
    almanac.d_epsilon_n_A = 0.001482010;
    almanac.d_omega_n_A = 0.440277100 * pi;
    std::array<double, 3> position{};
    ASSERT_TRUE(almanac.satellite_position(33300.0 - almanac.d_t_lambda_n_A, position));
    const double sidereal = 6.02401539573 + 7.292115e-5 * (33300.0 - 10800.0);
    EXPECT_NEAR(10947021.572 * std::cos(sidereal) + 13078978.287 * std::sin(sidereal), position[0], 0.2);
    EXPECT_NEAR(-10947021.572 * std::sin(sidereal) + 13078978.287 * std::cos(sidereal), position[1], 0.2);
    EXPECT_NEAR(18922063.362, position[2], 0.2);
    const double utc_epoch[6] = {2001, 9, 6, 0, 0, 0};
    EXPECT_NEAR(almanac.d_t_lambda_n_A - 10800.0,
        timediff(gpst2utc(glonass_almanac_epoch(almanac)), epoch2time(utc_epoch)), 1.0e-9);
}


TEST_F(SatelliteVisibilityTest, GlonassAlmanacHealthAgeAndMissingDate)
{
    Glonass_Gnav_Almanac almanac;
    almanac.PRN = 1;
    almanac.d_N_4 = 8;
    almanac.d_N_A = 1;
    almanac.d_t_lambda_n_A = 10000.0;
    almanac.d_C_n = true;
    pvt->glonass_almanac[1] = almanac;
    const auto epoch = glonass_almanac_epoch(almanac);
    std::vector<std::pair<int, Gnss_Satellite>> excluded;
    const arma::vec receiver{6378137.0, 0.0, 0.0};
    auto visible = compute_visible_satellites(pvt, epoch, receiver, 0.0, &excluded, 259200.0);
    ASSERT_EQ(1U, visible.size());
    EXPECT_EQ("Glonass", visible.front().second.get_system());
    EXPECT_TRUE(excluded.empty());
    pvt->glonass_almanac[1].d_C_n = false;
    EXPECT_TRUE(compute_visible_satellites(pvt, epoch, receiver, 0.0, &excluded, 259200.0).empty());
    EXPECT_EQ(1U, excluded.size());
    pvt->glonass_almanac[1].d_C_n = true;
    pvt->glonass_almanac[1].d_l_n = true;
    excluded.clear();
    EXPECT_TRUE(compute_visible_satellites(pvt, epoch, receiver, 0.0, &excluded, 259200.0).empty());
    EXPECT_EQ(1U, excluded.size());
    EXPECT_EQ(0U, ClassifiedCount(timeadd(epoch, 604800.0)));
    EXPECT_EQ(1U, ClassifiedCount(timeadd(epoch, 86400.0)));  // across the four-year boundary
    pvt->glonass_almanac[1].d_N_A = 0;
    EXPECT_EQ(0U, ClassifiedCount(epoch));
    pvt->glonass_almanac[1].d_N_A = 1;
    pvt->glonass_almanac[1].d_epsilon_n_A = 1.0;
    EXPECT_EQ(0U, ClassifiedCount(epoch));
}


TEST_F(SatelliteVisibilityTest, GlonassSearchCombinesSlotsOnTheSameFrequency)
{
    Glonass_Gnav_Ephemeris ephemeris;
    ephemeris.PRN = ephemeris.i_satellite_slot_number = 1;
    ephemeris.d_yr = 2024;
    ephemeris.d_N_T = 1;
    ephemeris.d_t_b = ephemeris.d_t_k = 10800.0;
    ephemeris.d_Xn = -25500.0;
    ephemeris.d_VZn = 3.9;
    pvt->glonass_ephemeris[1] = ephemeris;
    const auto epoch = eph_to_rtklib(ephemeris, pvt->glo_utc).toe;
    int week = 0;
    fix.RX_time = time2gpst(epoch, &week);
    fix.week = week;
    SatelliteVisibility visibility(configuration);
    const Gnss_Satellite representative("Glonass", 1);
    const Gnss_Satellite partner("Glonass", 5);
    auto update = [&]() {
        for (int tick = 0; tick < 20; ++tick)
            {
                visibility.Tick(pvt, fix);
            }
    };

    // Slot 1 is below the horizon, but missing data for slot 5 must leave
    // their shared frequency searchable as unknown.
    update();
    EXPECT_TRUE(visibility.IsExcluded(representative));
    EXPECT_FALSE(visibility.IsSearchExcluded(representative));
    EXPECT_FALSE(visibility.IsSearchVisible(representative));

    ephemeris.PRN = ephemeris.i_satellite_slot_number = 5;
    ephemeris.d_Xn = 25500.0;
    pvt->glonass_ephemeris[5] = ephemeris;
    update();
    EXPECT_TRUE(visibility.IsExcluded(representative));
    EXPECT_TRUE(visibility.IsVisible(partner));
    for (const auto& satellite : {representative, partner})
        {
            EXPECT_TRUE(visibility.IsSearchVisible(satellite));
            EXPECT_FALSE(visibility.IsSearchExcluded(satellite));
        }

    // A visible slot on another frequency cannot keep this one eligible.
    ephemeris.PRN = ephemeris.i_satellite_slot_number = 2;
    pvt->glonass_ephemeris[2] = ephemeris;
    pvt->glonass_ephemeris[5].d_B_n = 4;
    update();
    EXPECT_TRUE(visibility.IsVisible(Gnss_Satellite("Glonass", 2)));
    for (const auto& satellite : {representative, partner})
        {
            EXPECT_FALSE(visibility.IsSearchVisible(satellite));
            EXPECT_TRUE(visibility.IsSearchExcluded(satellite));
        }

    pvt->glonass_ephemeris.erase(5);
    update();
    EXPECT_FALSE(visibility.IsSearchExcluded(representative));
    EXPECT_FALSE(visibility.IsSearchVisible(representative));

    fix.RX_time += MAXDTOE_GLO + 1.0;
    update();
    EXPECT_FALSE(visibility.IsExcluded(representative));
    EXPECT_FALSE(visibility.IsSearchExcluded(representative));
    EXPECT_FALSE(visibility.IsSearchVisible(representative));
}


TEST_F(SatelliteVisibilityTest, GlonassEphemerisPrecedesAlmanacAndExpires)
{
    Glonass_Gnav_Ephemeris ephemeris;
    ephemeris.PRN = ephemeris.i_satellite_slot_number = 1;
    ephemeris.d_yr = 2024;
    ephemeris.d_N_T = 1;
    ephemeris.d_t_b = ephemeris.d_t_k = 10800.0;
    ephemeris.d_Xn = -25500.0;  // below the horizon
    ephemeris.d_VZn = 3.9;
    pvt->glonass_ephemeris[1] = ephemeris;
    Glonass_Gnav_Almanac almanac;
    almanac.PRN = 1;
    almanac.d_N_4 = 8;
    almanac.d_N_A = 1;
    almanac.d_t_lambda_n_A = 10800.0;
    almanac.d_C_n = true;  // directly above the receiver at this ascending node
    pvt->glonass_almanac[1] = almanac;
    const auto epoch = glonass_almanac_epoch(almanac);
    const arma::vec receiver{6378137.0, 0.0, 0.0};
    std::vector<std::pair<int, Gnss_Satellite>> excluded;
    double expiry = 0.0;
    EXPECT_TRUE(compute_visible_satellites(pvt, epoch, receiver, 0.0, &excluded, 259200.0, &expiry).empty());
    ASSERT_EQ(1U, excluded.size());
    EXPECT_NEAR(MAXDTOE_GLO, expiry, 1.0e-6);
    excluded.clear();
    EXPECT_EQ(1U, compute_visible_satellites(pvt, timeadd(epoch, MAXDTOE_GLO + 1.0), receiver, 0.0, &excluded, 259200.0).size());
    EXPECT_TRUE(excluded.empty());
    pvt->glonass_almanac.clear();
    pvt->glonass_ephemeris[1].d_Xn = 25500.0;
    pvt->glonass_ephemeris[1].d_B_n = 4;
    EXPECT_TRUE(compute_visible_satellites(pvt, epoch, receiver, 0.0).empty());
    EXPECT_EQ(1U, compute_visible_satellites(pvt, epoch, receiver, 0.0, nullptr, 259200.0, nullptr, nullptr, false).size());
    pvt->glonass_ephemeris[1].d_l3rd_n = true;
    EXPECT_TRUE(compute_visible_satellites(pvt, epoch, receiver, 0.0, nullptr, 259200.0, nullptr, nullptr, false).empty());
    EXPECT_EQ(0U, ClassifiedCount(timeadd(epoch, 86400.0)));
}


TEST_F(SatelliteVisibilityTest, QzssLnavCnavAlmanacAndAlias)
{
    SetConstellationData(static_cast<int>(fix.RX_time));
    auto lnav = pvt->gps_eph.at(1);
    auto almanac = pvt->gps_alm.at(2);
    pvt->gps_eph.clear();
    pvt->gps_alm.clear();
    pvt->gal_eph.clear();
    pvt->gal_alm.clear();
    pvt->bds_eph.clear();
    pvt->bds_alm.clear();
    lnav.PRN = 196;
    lnav.sqrtA = std::sqrt(42164000.0);
    pvt->gps_eph[196] = lnav;
    almanac.PRN = 194;
    almanac.sqrtA = lnav.sqrtA;
    almanac.set_system('J');
    pvt->gps_alm[194] = almanac;
    Gps_CNAV_Ephemeris cnav;
    cnav.PRN = 195;
    cnav.WN = fix.week;
    cnav.toe1 = cnav.toe2 = cnav.toc = cnav.tow = static_cast<int>(fix.RX_time);
    cnav.sqrtA = lnav.sqrtA;
    pvt->gps_cnav_ephemeris[195] = cnav;
    configuration->set_property("GNSS-SDR.search_elevation_mask", "-90");
    SatelliteVisibility visibility(configuration);
    EXPECT_TRUE(visibility.Tick(pvt, fix));
    EXPECT_TRUE(visibility.IsVisible(Gnss_Satellite("QZSS", 194)));
    EXPECT_TRUE(visibility.IsVisible(Gnss_Satellite("QZSS", 195)));
    EXPECT_TRUE(visibility.IsVisible(Gnss_Satellite("QZSS", 196)));
    EXPECT_TRUE(visibility.IsVisible(Gnss_Satellite("QZSS", 203)));  // alias of 196
    EXPECT_TRUE(visibility.IsSearchVisible(Gnss_Satellite("QZSS", 203)));
    EXPECT_FALSE(visibility.IsSearchExcluded(Gnss_Satellite("QZSS", 203)));
    EXPECT_EQ(3U, ClassifiedCount(gpst2time(fix.week, fix.RX_time)));
    EXPECT_EQ(0U, ClassifiedCount(gpst2time(fix.week + 1, fix.RX_time)));
    pvt->gps_eph[196].SV_health = 1;
    for (int tick = 0; tick < 20; ++tick)
        {
            visibility.Tick(pvt, fix);
        }
    EXPECT_TRUE(visibility.IsExcluded(Gnss_Satellite("QZSS", 203)));
    EXPECT_TRUE(visibility.IsSearchExcluded(Gnss_Satellite("QZSS", 203)));
    EXPECT_FALSE(visibility.IsSearchVisible(Gnss_Satellite("QZSS", 203)));
}
