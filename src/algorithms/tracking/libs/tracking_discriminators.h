/*!
 * \file tracking_discriminators.h
 * \brief Interface of a library with a set of code tracking and carrier
 * tracking discriminators.
 * \authors <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          </ul>
 *
 * Library with a set of code tracking and carrier tracking discriminators
 * that is used by the tracking algorithms.
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

#ifndef GNSS_SDR_TRACKING_DISCRIMINATORS_H
#define GNSS_SDR_TRACKING_DISCRIMINATORS_H

#include <gnuradio/gr_complex.h>
#include <cmath>

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_libs
 * \{ */


/*! brief FLL four quadrant arctan discriminator
 *
 * FLL four quadrant arctan discriminator:
 * \f{equation}
 *     \frac{\phi_2-\phi_1}{t_2-t1}=\frac{ATAN2(cross,dot)}{t_1-t_2},
 * \f}
 * where \f$cross=I_{PS1}Q_{PS2}-I_{PS2}Q_{PS1}\f$ and \f$dot=I_{PS1}I_{PS2}+Q_{PS1}Q_{PS2}\f$,
 * \f$I_{PS1},Q_{PS1}\f$ are the inphase and quadrature prompt correlator outputs respectively at sample time \f$t_1\f$, and
 * \f$I_{PS2},Q_{PS2}\f$ are the inphase and quadrature prompt correlator outputs respectively at sample time \f$t_2\f$. The output is in [radians/second].
 */
double fll_four_quadrant_atan(gr_complex prompt_s1, gr_complex prompt_s2, double t1, double t2);


/*
 * FLL differential arctan discriminator:
 * \f{equation}
 *     e_{atan}(k)=\frac{1}{t_1-t_2}\text{phase_unwrap}(\tan^-1(\frac{Q(k)}{I(k)})-\tan^-1(\frac{Q(k-1)}{I(k-1)}))
 * \f}
 * The output is in [radians/second].
 */
double fll_diff_atan(gr_complex prompt_s1, gr_complex prompt_s2, double t1, double t2);


/*! \brief Phase unwrapping function, input is [rad]
 */
double phase_unwrap(double phase_rad);


/*! \brief PLL four quadrant arctan discriminator
 *
 * PLL four quadrant arctan discriminator:
 * \f{equation}
 *     \phi=ATAN2(Q_{PS},I_{PS}),
 * \f}
 * where \f$I_{PS1},Q_{PS1}\f$ are the inphase and quadrature prompt correlator outputs respectively. The output is in [radians].
 */
double pll_four_quadrant_atan(gr_complex prompt_s1);


/*! \brief PLL Costas loop two quadrant arctan discriminator
 *
 * PLL Costas loop two quadrant arctan discriminator:
 * \f{equation}
 *     \phi=ATAN\left(\frac{Q_{PS}}{I_{PS}}\right),
 * \f}
 * where \f$I_{PS1},Q_{PS1}\f$ are the inphase and quadrature prompt correlator outputs respectively. The output is in [radians].
 */
double pll_cloop_two_quadrant_atan(gr_complex prompt_s1);


/*! \brief DLL Noncoherent Early minus Late envelope normalized discriminator
 *
 * DLL Noncoherent Early minus Late envelope normalized discriminator:
 * \f{equation}
 *     error = \frac{y_{intercept} - \text{slope} * \epsilon}{\text{slope}} \frac{E-L}{E+L},
 * \f}
 * where \f$E=\sqrt{I_{ES}^2+Q_{ES}^2}\f$ is the Early correlator output absolute value and
 * \f$L=\sqrt{I_{LS}^2+Q_{LS}^2}\f$ is the Late correlator output absolute value. The output is in [chips].
 */
double dll_nc_e_minus_l_normalized(gr_complex early_s1, gr_complex late_s1, float spc = 0.5, float slope = 1.0, float y_intercept = 1.0);


/*! \brief DLL Noncoherent Very Early Minus Late Power (VEMLP) normalized discriminator
 *
 * DLL Noncoherent Very Early Minus Late Power (VEMLP) normalized discriminator, using the outputs
 * of four correlators, Very Early (VE), Early (E), Late (L) and Very Late (VL):
 * \f{equation}
 *  error=\frac{E-L}{E+L},
 * \f}
 * where \f$E=\sqrt{I_{VE}^2+Q_{VE}^2+I_{E}^2+Q_{E}^2}\f$ and
 * \f$L=\sqrt{I_{VL}^2+Q_{VL}^2+I_{L}^2+Q_{L}^2}\f$ . The output is in [chips].
 */
double dll_nc_vemlp_normalized(gr_complex very_early_s1, gr_complex early_s1, gr_complex late_s1, gr_complex very_late_s1);


template <typename Fun>
double CalculateSlope(Fun &&f, double x)
{
    static constexpr double dx = 1e-6;

    return (f(x + dx / 2.0) - f(x - dx / 2.0)) / dx;
}

template <typename Fun>
double CalculateSlopeAbs(Fun &&f, double x)
{
    static constexpr double dx = 1e-6;

    return (std::abs(f(x + dx / 2.0)) - std::abs(f(x - dx / 2.0))) / dx;
}

template <typename Fun>
double GetYIntercept(Fun &&f, double x)
{
    double slope = CalculateSlope(f, x);
    double y1 = f(x);

    return y1 - slope * x;
}

template <typename Fun>
double GetYInterceptAbs(Fun &&f, double x)
{
    double slope = CalculateSlopeAbs(f, x);
    double y1 = std::abs(f(x));
    return y1 - slope * x;
}

// SinBocCorrelationFunction and CosBocCorrelationFunction from
// Sousa, F. and Nunes, F., "New Expressions for the Autocorrelation
// Function of BOC GNSS Signals", NAVIGATION - Journal of the Institute
// of Navigation, March 2013.
//
template <int M = 1, int N = M>
double SinBocCorrelationFunction(double offset_in_chips)
{
    static constexpr int TWO_P = 2 * M / N;

    double abs_tau = std::abs(offset_in_chips);

    if (abs_tau > 1.0)
        {
            return 0.0;
        }

    int k = static_cast<int>(std::ceil(TWO_P * abs_tau));

    double sgn = ((k & 0x01) == 0 ? 1.0 : -1.0);  // (-1)^k

    return sgn * (2.0 * (k * k - k * TWO_P - k) / TWO_P + 1.0 +
                     (2 * TWO_P - 2 * k + 1) * abs_tau);
}


template <int M = 1, int N = M>
double CosBocCorrelationFunction(double offset_in_chips)
{
    static constexpr int TWO_P = 2 * M / N;

    double abs_tau = std::abs(offset_in_chips);

    if (abs_tau > 1.0)
        {
            return 0.0;
        }

    int k = static_cast<int>(std::floor(2.0 * TWO_P * abs_tau));

    if ((k & 0x01) == 0)  // k is even
        {
            double sgn = ((k >> 1) & 0x01 ? -1.0 : 1.0);  // (-1)^(k/2)

            return sgn * ((2 * k * TWO_P + 2 * TWO_P - k * k) / (2.0 * TWO_P) + (-2 * TWO_P + k - 1) * abs_tau);
        }
    else
        {
            double sgn = (((k + 1) >> 1) & 0x01 ? -1.0 : 1.0);  // (-1)^((k+1)/2)

            return sgn * ((k * k + 2 * k - 2 * k * TWO_P + 1) / (2.0 * TWO_P) + (2 * TWO_P - k - 2) * abs_tau);
        }
}


/** \} */
/** \} */
#endif  // GNSS_SDR_TRACKING_DISCRIMINATORS_H
