/*!
 * \file gps_l1c_pcps_acquisition_test.cc
 * \brief Tests the GPS L1C PCPS acquisition with a synthetic L1CP + L1CD signal
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

#include "GPS_L1C.h"
#include "MATH_CONSTANTS.h"
#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include "gnss_synchro.h"
#include "gps_l1c_signal_replica.h"
#include "in_memory_configuration.h"
#include "pcps_acquisition_adapter.h"
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <pmt/pmt.h>
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#ifdef GR_GREATER_38
#include <gnuradio/blocks/vector_source.h>
#else
#include <gnuradio/blocks/vector_source_c.h>
#endif

#if HAS_GENERIC_LAMBDA
#else
#include <boost/bind.hpp>
#endif

namespace
{
class GpsL1cAcqTestMsgRx;
using GpsL1cAcqTestMsgRx_sptr = gnss_shared_ptr<GpsL1cAcqTestMsgRx>;

// Forwards the acquisition events (1: positive, 2: negative) to a queue
class GpsL1cAcqTestMsgRx : public gr::block
{
public:
    explicit GpsL1cAcqTestMsgRx(Concurrent_Queue<int>& queue)
        : gr::block("GpsL1cAcqTestMsgRx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0)),
          d_queue(queue)
    {
        this->message_port_register_in(pmt::mp("events"));
        this->set_msg_handler(pmt::mp("events"),
#if HAS_GENERIC_LAMBDA
            [this](const pmt::pmt_t& msg) { d_queue.push(static_cast<int>(pmt::to_long(msg))); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
            boost::bind(&GpsL1cAcqTestMsgRx::handler, this, boost::placeholders::_1));
#else
            boost::bind(&GpsL1cAcqTestMsgRx::handler, this, _1));
#endif
#endif
    }

private:
    void handler(const pmt::pmt_t& msg) { d_queue.push(static_cast<int>(pmt::to_long(msg))); }
    Concurrent_Queue<int>& d_queue;
};


// One code period of an L1C signal with 75 % of the power in the pilot,
// delayed by delay_samples and shifted by doppler_hz, which must be a
// multiple of 100 Hz so that the period can be repeated without phase jumps
std::vector<gr_complex> l1c_period(uint32_t prn, int32_t fs, int32_t delay_samples, double doppler_hz)
{
    const auto n = static_cast<size_t>(static_cast<double>(fs) * GPS_L1C_CODE_PERIOD_S);
    std::vector<std::complex<float>> pilot(n);
    std::vector<std::complex<float>> data(n);
    gps_l1c_code_gen_complex_sampled(pilot, true, false, prn, fs, 0);
    gps_l1c_code_gen_complex_sampled(data, false, false, prn, fs, 0);
    std::vector<gr_complex> signal(n);
    for (size_t i = 0; i < n; i++)
        {
            const size_t k = (i + n - static_cast<size_t>(delay_samples)) % n;
            const double phase = TWO_PI * doppler_hz * static_cast<double>(i) / static_cast<double>(fs);
            const auto chip = std::sqrt(0.75F) * pilot[k].real() - std::sqrt(0.25F) * data[k].real();
            signal[i] = chip * gr_complex(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
        }
    return signal;
}


struct AcquisitionResult
{
    int event{0};
    double delay_samples{0.0};
    uint64_t sample_stamp{0};
    double doppler_hz{0.0};
};


AcquisitionResult acquire(uint32_t searched_prn, const std::vector<gr_complex>& period, int32_t fs, bool tmboc)
{
    auto config = std::make_shared<InMemoryConfiguration>();
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(fs));
    config->set_property("Acquisition_1S.item_type", "gr_complex");
    config->set_property("Acquisition_1S.doppler_max", "3000");
    config->set_property("Acquisition_1S.doppler_step", "50");
    config->set_property("Acquisition_1S.pfa", "0.001");
    config->set_property("Acquisition_1S.tmboc", tmboc ? "true" : "false");
    config->set_property("Acquisition_1S.repeat_satellite", "false");

    Gnss_Synchro gnss_synchro{};
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    gnss_synchro.Signal[0] = '1';
    gnss_synchro.Signal[1] = 'S';
    gnss_synchro.PRN = searched_prn;

    Concurrent_Queue<int> events;
    auto top_block = gr::make_top_block("GPS L1C acquisition test");
    auto acquisition = std::make_shared<PcpsAcquisitionAdapter>(config.get(), "Acquisition_1S", "GPS_L1C_PCPS_Ambiguous_Acquisition", 1, 0, GPS_L1C);
    auto msg_rx = gnss_make_shared<GpsL1cAcqTestMsgRx>(events);
    auto source = gr::blocks::vector_source_c::make(period, true);
    acquisition->set_channel(0);
    acquisition->set_gnss_synchro(&gnss_synchro);
    acquisition->connect(top_block);
    top_block->connect(source, 0, acquisition->get_left_block(), 0);
    top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    acquisition->set_local_code();
    acquisition->reset();

    top_block->start();
    AcquisitionResult result;
    events.wait_and_pop(result.event);
    top_block->stop();
    top_block->wait();
    result.delay_samples = gnss_synchro.Acq_delay_samples;
    result.doppler_hz = gnss_synchro.Acq_doppler_hz;
    result.sample_stamp = gnss_synchro.Acq_samplestamp_samples;
    return result;
}
}  // namespace


TEST(GpsL1cPcpsAcquisitionTest, AcquiresPilotDelayAndDoppler)
{
    constexpr int32_t fs = 4000000;
    constexpr int32_t delay_samples = 12345;
    constexpr double doppler_hz = 1200.0;
    const auto period = l1c_period(11, fs, delay_samples, doppler_hz);

    for (const bool tmboc : {false, true})
        {
            const AcquisitionResult present = acquire(11, period, fs, tmboc);
            ASSERT_EQ(present.event, 1) << "tmboc " << tmboc;
            const double samples_per_code = static_cast<double>(fs) * GPS_L1C_CODE_PERIOD_S;
            // The code phase is measured from the start of the searched window,
            // and the stamp marks a window boundary, a whole number of periods away
            const double stamp_phase = std::fmod(static_cast<double>(present.sample_stamp), samples_per_code);
            double delay_error = std::fmod(present.delay_samples + stamp_phase - delay_samples + 2.0 * samples_per_code, samples_per_code);
            delay_error = std::min(delay_error, samples_per_code - delay_error);
            // BOC(1,1): two samples per subcarrier half-cycle at 4 Msps
            EXPECT_LE(delay_error, 1.0) << "tmboc " << tmboc;
            EXPECT_NEAR(present.doppler_hz, doppler_hz, 50.0) << "tmboc " << tmboc;
        }

    const AcquisitionResult absent = acquire(12, period, fs, false);
    EXPECT_EQ(absent.event, 2);
}
