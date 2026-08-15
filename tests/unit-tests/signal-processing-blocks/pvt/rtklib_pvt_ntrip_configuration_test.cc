/*!
 * \file rtklib_pvt_ntrip_configuration_test.cc
 * \brief Unit tests for RTKLIB PVT NTRIP client configuration validation
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * SPDX-FileCopyrightText: 2026 Carles Fernandez-Prades <carles.fernandez@cttc.es>
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "in_memory_configuration.h"
#include "rtklib_pvt.h"
#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <string>

namespace rtklib_pvt_ntrip_configuration_test_detail
{
const char* const ROLE = "PVT";


std::unique_ptr<InMemoryConfiguration> make_base_configuration()
{
    std::unique_ptr<InMemoryConfiguration> configuration(new InMemoryConfiguration);
    configuration->set_property("Channels_1C.count", "1");
    configuration->set_property("Channels_2S.count", "1");
    configuration->set_property("PVT.output_enabled", "false");
    configuration->set_property("PVT.dump", "false");
    return configuration;
}


std::unique_ptr<InMemoryConfiguration> make_valid_ntrip_configuration()
{
    std::unique_ptr<InMemoryConfiguration> configuration = make_base_configuration();
    configuration->set_property("PVT.ntrip_client_enabled", "true");
    configuration->set_property("PVT.ntrip_caster_address", "127.0.0.1");
    configuration->set_property("PVT.ntrip_mountpoint", "TEST");
    configuration->set_property("PVT.positioning_mode", "Static");
    configuration->set_property("PVT.num_bands", "2");
    configuration->set_property("PVT.navigation_system", "1");
    return configuration;
}
}  // namespace rtklib_pvt_ntrip_configuration_test_detail


class RtklibPvtNtripConfigurationTest : public ::testing::Test
{
protected:
    static void expect_invalid_configuration(
        const InMemoryConfiguration& configuration,
        const std::string& expected_message_fragment)
    {
        try
            {
                Rtklib_Pvt adapter(&configuration,
                    rtklib_pvt_ntrip_configuration_test_detail::ROLE,
                    2,
                    0);
                (void)adapter;
                FAIL() << "Expected the NTRIP configuration to be rejected";
            }
        catch (const std::invalid_argument& error)
            {
                EXPECT_NE(std::string::npos, std::string(error.what()).find(expected_message_fragment))
                    << "Unexpected validation error: " << error.what();
            }
        catch (const std::exception& error)
            {
                FAIL() << "Expected std::invalid_argument, got: " << error.what();
            }
        catch (...)
            {
                FAIL() << "Expected std::invalid_argument, got a non-standard exception";
            }
    }
};


TEST_F(RtklibPvtNtripConfigurationTest, ClientDefaultsToDisabledAndIgnoresNtripOnlyProperties)
{
    using namespace rtklib_pvt_ntrip_configuration_test_detail;
    std::unique_ptr<InMemoryConfiguration> configuration = make_base_configuration();
    configuration->set_property("PVT.ntrip_caster_address", "not a valid endpoint://");
    configuration->set_property("PVT.ntrip_caster_port", "0");
    configuration->set_property("PVT.ntrip_mountpoint", "");
    configuration->set_property("PVT.ntrip_max_correction_age_s", "0");

    std::unique_ptr<Rtklib_Pvt> adapter;
    EXPECT_NO_THROW(adapter.reset(new Rtklib_Pvt(configuration.get(), ROLE, 2, 0)));
    ASSERT_NE(nullptr, adapter);
    EXPECT_EQ("RTKLIB_PVT", adapter->implementation());
}


TEST_F(RtklibPvtNtripConfigurationTest, ExplicitlyDisabledClientIgnoresNtripOnlyProperties)
{
    using namespace rtklib_pvt_ntrip_configuration_test_detail;
    std::unique_ptr<InMemoryConfiguration> configuration = make_base_configuration();
    configuration->set_property("PVT.ntrip_client_enabled", "false");
    configuration->set_property("PVT.ntrip_caster_address", "https://user:secret@example.invalid:0");
    configuration->set_property("PVT.ntrip_mountpoint", "bad/path");
    configuration->set_property("PVT.ntrip_inactivity_timeout_ms", "1");

    std::unique_ptr<Rtklib_Pvt> adapter;
    EXPECT_NO_THROW(adapter.reset(new Rtklib_Pvt(configuration.get(), ROLE, 2, 0)));
    ASSERT_NE(nullptr, adapter);
    EXPECT_EQ("PVT", adapter->role());
}


TEST_F(RtklibPvtNtripConfigurationTest, RejectsUnsupportedPositioningModeBeforeConnecting)
{
    using namespace rtklib_pvt_ntrip_configuration_test_detail;
    std::unique_ptr<InMemoryConfiguration> configuration = make_valid_ntrip_configuration();
    configuration->supersede_property("PVT.positioning_mode", "Single");

    expect_invalid_configuration(*configuration, "positioning_mode=Static or positioning_mode=Kinematic");
}


TEST_F(RtklibPvtNtripConfigurationTest, RejectsMissingCasterOrMountpointBeforeConnecting)
{
    using namespace rtklib_pvt_ntrip_configuration_test_detail;
    std::unique_ptr<InMemoryConfiguration> missing_caster = make_valid_ntrip_configuration();
    missing_caster->supersede_property("PVT.ntrip_caster_address", "");
    expect_invalid_configuration(*missing_caster, "ntrip_caster_address");

    std::unique_ptr<InMemoryConfiguration> missing_mountpoint = make_valid_ntrip_configuration();
    missing_mountpoint->supersede_property("PVT.ntrip_mountpoint", "");
    expect_invalid_configuration(*missing_mountpoint, "ntrip_mountpoint");
}


TEST_F(RtklibPvtNtripConfigurationTest, RejectsOutOfRangePortOrStationBeforeConnecting)
{
    using namespace rtklib_pvt_ntrip_configuration_test_detail;
    std::unique_ptr<InMemoryConfiguration> bad_port = make_valid_ntrip_configuration();
    bad_port->supersede_property("PVT.ntrip_caster_port", "65536");
    expect_invalid_configuration(*bad_port, "ntrip_caster_port");

    std::unique_ptr<InMemoryConfiguration> bad_station = make_valid_ntrip_configuration();
    bad_station->supersede_property("PVT.ntrip_station_id", "4096");
    expect_invalid_configuration(*bad_station, "ntrip_station_id");
}


TEST_F(RtklibPvtNtripConfigurationTest, RejectsUnsupportedNtripVersionBeforeConnecting)
{
    using namespace rtklib_pvt_ntrip_configuration_test_detail;
    std::unique_ptr<InMemoryConfiguration> bad_version = make_valid_ntrip_configuration();
    bad_version->supersede_property("PVT.ntrip_version", "3");
    expect_invalid_configuration(*bad_version, "ntrip_version");

    std::unique_ptr<InMemoryConfiguration> forced_v1 = make_valid_ntrip_configuration();
    forced_v1->supersede_property("PVT.ntrip_version", "1");
    std::unique_ptr<Rtklib_Pvt> adapter;
    EXPECT_NO_THROW(adapter.reset(new Rtklib_Pvt(forced_v1.get(), ROLE, 2, 0)));
    ASSERT_NE(nullptr, adapter);

    std::unique_ptr<InMemoryConfiguration> with_tls = make_valid_ntrip_configuration();
    with_tls->supersede_property("PVT.ntrip_tls_enabled", "true");
    EXPECT_NO_THROW(adapter.reset(new Rtklib_Pvt(with_tls.get(), ROLE, 2, 0)));
    ASSERT_NE(nullptr, adapter);
}


TEST_F(RtklibPvtNtripConfigurationTest, RejectsInvalidAgeOrTimeoutBeforeConnecting)
{
    using namespace rtklib_pvt_ntrip_configuration_test_detail;
    std::unique_ptr<InMemoryConfiguration> bad_age = make_valid_ntrip_configuration();
    bad_age->supersede_property("PVT.ntrip_max_correction_age_s", "0");
    expect_invalid_configuration(*bad_age, "ntrip_max_correction_age_s");

    std::unique_ptr<InMemoryConfiguration> bad_timeout = make_valid_ntrip_configuration();
    bad_timeout->supersede_property("PVT.ntrip_inactivity_timeout_ms", "999");
    expect_invalid_configuration(*bad_timeout, "timeout and reconnect intervals");

    std::unique_ptr<InMemoryConfiguration> bad_reconnect = make_valid_ntrip_configuration();
    bad_reconnect->supersede_property("PVT.ntrip_reconnect_interval_ms", "999");
    expect_invalid_configuration(*bad_reconnect, "timeout and reconnect intervals");
}


TEST_F(RtklibPvtNtripConfigurationTest, RejectsSignalsOutsideGpsL1L2BeforeConnecting)
{
    using namespace rtklib_pvt_ntrip_configuration_test_detail;
    std::unique_ptr<InMemoryConfiguration> missing_l2 = make_valid_ntrip_configuration();
    missing_l2->supersede_property("Channels_2S.count", "0");
    expect_invalid_configuration(*missing_l2, "exactly GPS L1 C/A and GPS L2C channels");

    std::unique_ptr<InMemoryConfiguration> additional_signal = make_valid_ntrip_configuration();
    additional_signal->set_property("Channels_1B.count", "1");
    expect_invalid_configuration(*additional_signal, "exactly GPS L1 C/A and GPS L2C channels");
}


TEST_F(RtklibPvtNtripConfigurationTest, RejectsWrongBandCountOrNavigationSystemBeforeConnecting)
{
    using namespace rtklib_pvt_ntrip_configuration_test_detail;
    std::unique_ptr<InMemoryConfiguration> bad_band_count = make_valid_ntrip_configuration();
    bad_band_count->supersede_property("PVT.num_bands", "1");
    expect_invalid_configuration(*bad_band_count, "num_bands must be 2");

    std::unique_ptr<InMemoryConfiguration> bad_navigation_system = make_valid_ntrip_configuration();
    bad_navigation_system->supersede_property("PVT.navigation_system", "9");
    expect_invalid_configuration(*bad_navigation_system, "navigation_system must select GPS only");
}


TEST_F(RtklibPvtNtripConfigurationTest, RejectsInvalidPasswordConfigurationBeforeConnecting)
{
    using namespace rtklib_pvt_ntrip_configuration_test_detail;
    std::unique_ptr<InMemoryConfiguration> missing_username = make_valid_ntrip_configuration();
    missing_username->set_property("PVT.ntrip_password", "secret");
    expect_invalid_configuration(*missing_username, "ntrip_username is required");

    std::unique_ptr<InMemoryConfiguration> duplicate_password_source = make_valid_ntrip_configuration();
    duplicate_password_source->set_property("PVT.ntrip_username", "user");
    duplicate_password_source->set_property("PVT.ntrip_password", "secret");
    duplicate_password_source->set_property("PVT.ntrip_password_env", "UNUSED_NTRIP_PASSWORD");
    expect_invalid_configuration(*duplicate_password_source, "mutually exclusive");
}
