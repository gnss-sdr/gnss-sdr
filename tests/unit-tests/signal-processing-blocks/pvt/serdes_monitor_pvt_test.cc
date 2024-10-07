/*!
 * \file serdes_monitor_pvt_test.cc
 * \brief Implements Unit Test for the serdes_monitor_pvt class.
 * \author Carles Fernandez-Prades, 2019. cfernandez(at)cttc.es
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

#include "serdes_galileo_eph.h"
#include "serdes_monitor_pvt.h"
#include <memory>
#include <utility>

TEST(Serdes_Monitor_Pvt_Test, Simpletest)
{
    auto monitor = std::make_shared<Monitor_Pvt>(Monitor_Pvt());
    double true_latitude = 23.4;
    monitor->latitude = true_latitude;

    Serdes_Monitor_Pvt serdes = Serdes_Monitor_Pvt();
    std::string serialized_data = serdes.createProtobuffer(monitor.get());

    gnss_sdr::MonitorPvt mon;
    mon.ParseFromString(serialized_data);
    double read_latitude = mon.latitude();
    EXPECT_NEAR(true_latitude, read_latitude, 0.000001);
}


TEST(Serdes_Monitor_Pvt_Test, GalileoEphemerisSerdes)
{
    auto eph = std::make_shared<Galileo_Ephemeris>();
    int true_tow = 12345;
    eph->tow = true_tow;

    Serdes_Galileo_Eph gal_serdes = Serdes_Galileo_Eph();
    std::string serialized_data = gal_serdes.createProtobuffer(std::move(eph));

    gnss_sdr::GalileoEphemeris ephgal;
    ephgal.ParseFromString(serialized_data);

    double true_delta_n = 0.33;
    ephgal.set_delta_n(true_delta_n);

    int read_tow = ephgal.tow();
    EXPECT_EQ(true_tow, read_tow);

    double read_delta_n = ephgal.delta_n();
    EXPECT_NEAR(true_delta_n, read_delta_n, 0.000001);

    auto eph2 = gal_serdes.readProtobuffer(ephgal);
    double read2_delta_n = eph2.delta_n;
    int read2_tow = eph2.tow;
    EXPECT_EQ(true_tow, read2_tow);
    EXPECT_NEAR(true_delta_n, read2_delta_n, 0.000001);
}
