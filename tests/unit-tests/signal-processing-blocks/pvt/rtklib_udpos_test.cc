/*!
 * \file rtklib_udpos_test.cc
 * \brief This file implements tests for the temporal update of the position,
 * velocity and acceleration states of the RTK filter.
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

#include "rtklib_rtkpos.h"
#include "rtklib_rtksvr.h"
#include <gtest/gtest.h>
#include <cmath>
#include <memory>

namespace rtklib_udpos_test_detail
{
// a plausible ECEF position, so that the filter is not reinitialized
constexpr double POSITION_ECEF_M[3] = {4600000.0, 700000.0, 4370000.0};
constexpr double VELOCITY_ECEF_MPS[3] = {3.0, -2.0, 1.0};
constexpr double ACCELERATION_ECEF_MPS2[3] = {0.4, -0.3, 0.2};


prcopt_t dynamic_options()
{
    prcopt_t options = PRCOPT_DEFAULT;
    options.mode = PMODE_KINEMA;
    options.nf = 1;
    options.navsys = SYS_GPS;
    options.dynamics = 1;
    options.armaxposvar = 0.25;
    return options;
}


class Rtk_State
{
public:
    explicit Rtk_State(const prcopt_t& options)
    {
        rtkinit(&d_rtk, &options);
    }

    ~Rtk_State()
    {
        rtkfree(&d_rtk);
    }

    Rtk_State(const Rtk_State&) = delete;
    Rtk_State& operator=(const Rtk_State&) = delete;

    //! Puts the filter in a converged state moving with a constant acceleration
    void set_dynamic_state(double position_variance_m2)
    {
        for (int i = 0; i < 3; i++)
            {
                d_rtk.x[i] = POSITION_ECEF_M[i];
                d_rtk.x[i + 3] = VELOCITY_ECEF_MPS[i];
                d_rtk.x[i + 6] = ACCELERATION_ECEF_MPS2[i];
                d_rtk.sol.rr[i] = POSITION_ECEF_M[i];
                d_rtk.sol.rr[i + 3] = VELOCITY_ECEF_MPS[i];
            }
        for (int i = 0; i < 9; i++)
            {
                d_rtk.P[i + i * d_rtk.nx] = position_variance_m2;
            }
    }

    rtk_t& rtk() { return d_rtk; }

private:
    rtk_t d_rtk{};
};
}  // namespace rtklib_udpos_test_detail


TEST(RtklibUdposTest, AccelerationDrivesPositionOnceTheFilterHasConverged)
{
    using namespace rtklib_udpos_test_detail;
    Rtk_State state(dynamic_options());
    state.set_dynamic_state(0.01);  // well below armaxposvar

    const double tt = 0.5;
    udpos(&state.rtk(), tt);

    for (int i = 0; i < 3; i++)
        {
            const double expected_position = POSITION_ECEF_M[i] +
                                             VELOCITY_ECEF_MPS[i] * tt +
                                             0.5 * ACCELERATION_ECEF_MPS2[i] * tt * tt;
            const double expected_velocity = VELOCITY_ECEF_MPS[i] + ACCELERATION_ECEF_MPS2[i] * tt;
            EXPECT_NEAR(expected_position, state.rtk().x[i], 1e-9) << "axis " << i;
            EXPECT_NEAR(expected_velocity, state.rtk().x[i + 3], 1e-9) << "axis " << i;
        }
}


TEST(RtklibUdposTest, AccelerationIsKeptOutOfPositionWhileTheVarianceIsLarge)
{
    using namespace rtklib_udpos_test_detail;
    prcopt_t options = dynamic_options();
    Rtk_State state(options);
    state.set_dynamic_state(10.0 * options.armaxposvar);

    const double tt = 0.5;
    udpos(&state.rtk(), tt);

    for (int i = 0; i < 3; i++)
        {
            // velocity still propagates, the acceleration term does not
            const double expected_position = POSITION_ECEF_M[i] + VELOCITY_ECEF_MPS[i] * tt;
            EXPECT_NEAR(expected_position, state.rtk().x[i], 1e-9) << "axis " << i;
        }
}


TEST(RtklibUdposTest, TheAccelerationGateCanBeDisabled)
{
    using namespace rtklib_udpos_test_detail;
    prcopt_t options = dynamic_options();
    options.armaxposvar = 0.0;  // no gate
    Rtk_State state(options);
    // large, but below the variance that makes the filter reset the position
    state.set_dynamic_state(100.0);

    const double tt = 0.5;
    udpos(&state.rtk(), tt);

    for (int i = 0; i < 3; i++)
        {
            const double expected_position = POSITION_ECEF_M[i] +
                                             VELOCITY_ECEF_MPS[i] * tt +
                                             0.5 * ACCELERATION_ECEF_MPS2[i] * tt * tt;
            EXPECT_NEAR(expected_position, state.rtk().x[i], 1e-9) << "axis " << i;
        }
}


TEST(RtklibUdposTest, AccelerationProcessNoiseGrowsWithTheElapsedTime)
{
    using namespace rtklib_udpos_test_detail;
    const prcopt_t options = dynamic_options();

    const auto acceleration_variance_after = [&options](double tt) {
        Rtk_State state(options);
        state.set_dynamic_state(0.01);
        const double before = state.rtk().P[6 + 6 * state.rtk().nx];
        udpos(&state.rtk(), tt);
        return state.rtk().P[6 + 6 * state.rtk().nx] - before;
    };

    const double growth_half_second = acceleration_variance_after(0.5);
    const double growth_one_second = acceleration_variance_after(1.0);

    EXPECT_GT(growth_half_second, 0.0);
    EXPECT_NEAR(2.0, growth_one_second / growth_half_second, 1e-6);
}


TEST(RtklibUdposTest, ProcessNoiseStaysPositiveWhenTimeGoesBackwards)
{
    using namespace rtklib_udpos_test_detail;
    const prcopt_t options = dynamic_options();

    Rtk_State forward(options);
    forward.set_dynamic_state(0.01);
    const double before = forward.rtk().P[6 + 6 * forward.rtk().nx];
    udpos(&forward.rtk(), 1.0);
    const double forward_growth = forward.rtk().P[6 + 6 * forward.rtk().nx] - before;

    Rtk_State backward(options);
    backward.set_dynamic_state(0.01);
    udpos(&backward.rtk(), -1.0);
    const double backward_growth = backward.rtk().P[6 + 6 * backward.rtk().nx] - before;

    // a negative time step must not remove process noise from the filter. The
    // two are not bit-identical because the noise is rotated to ECEF at the
    // propagated position, which differs between the two directions
    EXPECT_GT(backward_growth, 0.0);
    EXPECT_NEAR(1.0, backward_growth / forward_growth, 1e-5);
}


TEST(RtklibUdposTest, IonosphericProcessNoiseStaysPositiveWhenTimeGoesBackwards)
{
    using namespace rtklib_udpos_test_detail;
    prcopt_t options = dynamic_options();
    options.ionoopt = IONOOPT_EST;
    Rtk_State state(options);

    // an already initialized ionospheric state, so that udion() adds process
    // noise to it instead of initializing it
    const int satellite = 1;
    const int index = II_RTK(satellite, &state.rtk().opt);
    state.rtk().x[index] = 1.0;
    state.rtk().P[index + index * state.rtk().nx] = 0.1;
    state.rtk().ssat[satellite - 1].azel[1] = 45.0 * D2R;

    const double baseline_m = 10000.0;
    const double variance_before = state.rtk().P[index + index * state.rtk().nx];
    udion(&state.rtk(), -1.0, baseline_m, &satellite, 1);

    // a negative time step must not remove process noise from the filter
    EXPECT_GT(state.rtk().P[index + index * state.rtk().nx], variance_before);
}


TEST(RtklibUdposTest, AnImplausiblePositionStateIsReinitialized)
{
    using namespace rtklib_udpos_test_detail;
    Rtk_State state(dynamic_options());
    state.set_dynamic_state(0.01);

    // a leftover state that is not a position on the Earth: the filter would
    // never converge from here
    for (int i = 0; i < 3; i++)
        {
            state.rtk().x[i] = 1000.0;
        }

    udpos(&state.rtk(), 1.0);

    for (int i = 0; i < 3; i++)
        {
            // reinitialized from the single-point solution, then propagated
            EXPECT_GT(std::fabs(state.rtk().x[i]), 1000.0) << "axis " << i;
            EXPECT_NEAR(POSITION_ECEF_M[i], state.rtk().x[i], 10.0) << "axis " << i;
        }
}
