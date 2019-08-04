/*!
 * \file serdes_monitor_pvt_test.cc
 * \brief Implements Unit Test for the serdes_monitor_pvt class.
 * \author Carles Fernandez_prades, 2019. cfernandez(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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

#include "serdes_monitor_pvt.h"
#include <memory>

TEST(Serdes_Monitor_Pvt_Test, Simpletest)
{
    std::shared_ptr<Monitor_Pvt> monitor = std::make_shared<Monitor_Pvt>(Monitor_Pvt());
    double true_latitude = 23.4;
    monitor->latitude = true_latitude;

    Serdes_Monitor_Pvt serdes = Serdes_Monitor_Pvt();
    std::string serialized_data = serdes.createProtobuffer(monitor);

    gnss_sdr::MonitorPvt mon;
    mon.ParseFromString(serialized_data);

    double read_latitude = mon.latitude();
    EXPECT_NEAR(true_latitude, read_latitude, 0.000001);
}
