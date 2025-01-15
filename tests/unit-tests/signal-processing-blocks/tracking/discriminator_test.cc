/*!
 * \file discriminator_test.cc
 * \brief  This file implements tests for the tracking discriminators
 * \author Cillian O'Driscoll, 2019. cillian.odriscoll(at)gmail.com
 *
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

#include "tracking_discriminators.h"
#include <gtest/gtest.h>
#include <cmath>
#include <vector>

double BpskCorrelationFunction(double offset_in_chips)
{
    double abs_tau = std::abs(offset_in_chips);

    if (abs_tau > 1.0)
        {
            return 0.0;
        }
    return 1.0 - abs_tau;
}


TEST(DllNcEMinusLNormalizedTest, Bpsk)
{
    std::vector<gr_complex> complex_amplitude_vector = {{1.0, 0.0}, {-1.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}};
    std::vector<double> spacing_vector = {0.5, 0.25, 0.1, 0.01};
    std::vector<double> error_vector = {0.0, 0.01, 0.1, 0.25, -0.25, -0.1, -0.01};

    for (auto A : complex_amplitude_vector)
        {
            for (auto spacing : spacing_vector)
                {
                    for (auto err : error_vector)
                        {
                            gr_complex E = A * static_cast<float>(BpskCorrelationFunction(err - spacing));
                            gr_complex L = A * static_cast<float>(BpskCorrelationFunction(err + spacing));

                            double disc_out = dll_nc_e_minus_l_normalized(E, L, spacing);

                            if (std::abs(err) < 2.0 * spacing)
                                {
                                    EXPECT_NEAR(disc_out, err, 1e-4) << " Spacing: " << spacing;
                                }
                            else
                                {
                                    EXPECT_TRUE(err * disc_out >= 0.0);
                                }

                            if (spacing != 0.5 and err != 0.0)
                                {
                                    double disc_out_old = dll_nc_e_minus_l_normalized(E, L);

                                    EXPECT_NE(disc_out_old, err);
                                }
                        }
                }
        }
}


TEST(DllNcEMinusLNormalizedTest, SinBoc11)
{
    std::vector<gr_complex> complex_amplitude_vector = {{1.0, 0.0}, {-1.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}};
    std::vector<double> spacing_vector = {0.75, 0.6666, 5.0 / 12.0, 0.25, 1.0 / 6.0, 0.01};
    std::vector<double> error_vector = {0.0, 0.01, 0.1, 0.25, -0.25, -0.1, -0.01};

    for (auto A : complex_amplitude_vector)
        {
            for (auto spacing : spacing_vector)
                {
                    double corr_slope = -CalculateSlopeAbs(&SinBocCorrelationFunction<1, 1>, spacing);
                    double y_intercept = GetYInterceptAbs(&SinBocCorrelationFunction<1, 1>, spacing);

                    for (auto err : error_vector)
                        {
                            gr_complex E = A * static_cast<float>(SinBocCorrelationFunction<1, 1>(err - spacing));
                            gr_complex L = A * static_cast<float>(SinBocCorrelationFunction<1, 1>(err + spacing));

                            double disc_out = dll_nc_e_minus_l_normalized(E, L, spacing, corr_slope, y_intercept);
                            double corr_slope_at_err = -CalculateSlopeAbs(&SinBocCorrelationFunction<1, 1>, spacing + err);
                            double corr_slope_at_neg_err = -CalculateSlopeAbs(&SinBocCorrelationFunction<1, 1>, spacing - err);

                            bool in_linear_region = (std::abs(err) < spacing) and (std::abs(corr_slope_at_err - corr_slope_at_neg_err) < 0.01);
                            double norm_factor = (y_intercept - corr_slope * spacing) / spacing;

                            if (in_linear_region)
                                {
                                    EXPECT_NEAR(disc_out, err, 1e-4) << " Spacing: " << spacing << ", slope : " << corr_slope << ", y_intercept: " << y_intercept << ", norm: " << norm_factor << " E: " << E << ", L: " << L;
                                    if (norm_factor != 0.5 and err != 0.0)
                                        {
                                            double disc_out_old = dll_nc_e_minus_l_normalized(E, L);
                                            EXPECT_NE(disc_out_old, err) << " Spacing: " << spacing << ", slope : " << corr_slope << ", y_intercept: " << y_intercept << ", norm: " << norm_factor << " E: " << E << ", L: " << L;
                                        }
                                }
                        }
                }
        }
}


TEST(CosBocCorrelationFunction, FixedPoints)
{
    double res = CosBocCorrelationFunction<1, 1>(0.0);
    EXPECT_NEAR(res, 1.0, 1e-4);
    res = CosBocCorrelationFunction<1, 1>(0.2);
    EXPECT_NEAR(res, 0.0, 1e-4);
    res = CosBocCorrelationFunction<1, 1>(0.25);
    EXPECT_NEAR(res, -0.25, 1e-4);
    res = CosBocCorrelationFunction<1, 1>(0.5);
    EXPECT_NEAR(res, -0.5, 1e-4);
    res = CosBocCorrelationFunction<1, 1>(0.75);
    EXPECT_NEAR(res, 0.25, 1e-4);
    res = CosBocCorrelationFunction<1, 1>(1.0);
    EXPECT_NEAR(res, 0.0, 1e-4);
    res = CosBocCorrelationFunction<1, 1>(-0.2);
    EXPECT_NEAR(res, 0.0, 1e-4);
    res = CosBocCorrelationFunction<1, 1>(-0.5);
    EXPECT_NEAR(res, -0.5, 1e-4);
    res = CosBocCorrelationFunction<1, 1>(-0.75);
    EXPECT_NEAR(res, 0.25, 1e-4);
    res = CosBocCorrelationFunction<1, 1>(-1.0);
    EXPECT_NEAR(res, 0.0, 1e-4);
}


TEST(DllNcEMinusLNormalizedTest, CosBoc11)
{
    std::vector<gr_complex> complex_amplitude_vector = {{1.0, 0.0}, {-1.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}};
    std::vector<double> spacing_vector = {0.875, 0.588, 0.1, 0.01};
    std::vector<double> error_vector = {0.0, 0.01, 0.1, 0.25, -0.25, -0.1, -0.01};

    for (auto A : complex_amplitude_vector)
        {
            for (auto spacing : spacing_vector)
                {
                    double corr_slope = -CalculateSlopeAbs(&CosBocCorrelationFunction<1, 1>, spacing);
                    double y_intercept = GetYInterceptAbs(&CosBocCorrelationFunction<1, 1>, spacing);
                    for (auto err : error_vector)
                        {
                            gr_complex E = A * static_cast<float>(CosBocCorrelationFunction<1, 1>(err - spacing));
                            gr_complex L = A * static_cast<float>(CosBocCorrelationFunction<1, 1>(err + spacing));

                            double disc_out = dll_nc_e_minus_l_normalized(E, L, spacing, corr_slope, y_intercept);
                            double corr_slope_at_err = -CalculateSlopeAbs(&CosBocCorrelationFunction<1, 1>, spacing + err);
                            double corr_slope_at_neg_err = -CalculateSlopeAbs(&CosBocCorrelationFunction<1, 1>, spacing - err);

                            bool in_linear_region = (std::abs(err) < spacing) and (std::abs(corr_slope_at_err - corr_slope_at_neg_err) < 0.01);
                            double norm_factor = (y_intercept - corr_slope * spacing) / spacing;

                            if (in_linear_region)
                                {
                                    EXPECT_NEAR(disc_out, err, 1e-4) << " Spacing: " << spacing << ", slope : " << corr_slope << ", y_intercept: " << y_intercept << ", norm: " << norm_factor << " E: " << E << ", L: " << L;
                                    if (norm_factor != 0.5 and err != 0.0)
                                        {
                                            double disc_out_old = dll_nc_e_minus_l_normalized(E, L);
                                            EXPECT_NE(disc_out_old, err) << " Spacing: " << spacing << ", slope : " << corr_slope << ", y_intercept: " << y_intercept << ", norm: " << norm_factor << " E: " << E << ", L: " << L;
                                        }
                                }
                        }
                }
        }
}
