/*!
 * \file tracking_discriminators.cc
 * \brief Implementation of a library with a set of code tracking
 * and carrier tracking discriminators that is used by the tracking algorithms.
 * \authors <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          </ul>
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
#include "MATH_CONSTANTS.h"
#include <gnuradio/math.h>

//  All the outputs are in RADIANS

double phase_unwrap(double phase_rad)
{
    if (phase_rad >= HALF_PI)
        {
            return phase_rad - GNSS_PI;
        }
    if (phase_rad <= -HALF_PI)
        {
            return phase_rad + GNSS_PI;
        }
    else
        {
            return phase_rad;
        }
}


/*
 * FLL four quadrant arctan discriminator:
 * \f{equation}
 *     \frac{\phi_2-\phi_1}{t_2-t1}=\frac{ATAN2(cross,dot)}{t_1-t_2},
 * \f}
 * where \f$cross=I_{PS1}Q_{PS2}-I_{PS2}Q_{PS1}\f$ and \f$dot=I_{PS1}I_{PS2}+Q_{PS1}Q_{PS2}\f$,
 * \f$I_{PS1},Q_{PS1}\f$ are the inphase and quadrature prompt correlator outputs respectively at sample time \f$t_1\f$, and
 * \f$I_{PS2},Q_{PS2}\f$ are the inphase and quadrature prompt correlator outputs respectively at sample time \f$t_2\f$. The output is in [radians/second].
 */
double fll_four_quadrant_atan(gr_complex prompt_s1, gr_complex prompt_s2, double t1, double t2)
{
    const float dot = prompt_s1.real() * prompt_s2.real() + prompt_s1.imag() * prompt_s2.imag();
    const float cross = prompt_s1.real() * prompt_s2.imag() - prompt_s2.real() * prompt_s1.imag();
    return gr::fast_atan2f(cross, dot) / (t2 - t1);
}


/*
 * FLL differential arctan discriminator:
 * \f{equation}
 *     e_{atan}(k)=\frac{1}{t_1-t_2}\text{phase_unwrap}(\tan^-1(\frac{Q(k)}{I(k)})-\tan^-1(\frac{Q(k-1)}{I(k-1)}))
 * \f}
 * The output is in [radians/second].
 */
double fll_diff_atan(gr_complex prompt_s1, gr_complex prompt_s2, double t1, double t2)
{
    double diff_atan = std::atan(prompt_s2.imag() / prompt_s2.real()) - std::atan(prompt_s1.imag() / prompt_s1.real());
    if (std::isnan(diff_atan))
        {
            diff_atan = 0;
        }
    return phase_unwrap(diff_atan) / (t2 - t1);
}


/*
 * PLL four quadrant arctan discriminator:
 * \f{equation}
 *     \phi=ATAN2(Q_{PS},I_{PS}),
 * \f}
 * where \f$I_{PS1},Q_{PS1}\f$ are the inphase and quadrature prompt correlator outputs respectively. The output is in [radians].
 */
double pll_four_quadrant_atan(gr_complex prompt_s1)
{
    return gr::fast_atan2f(prompt_s1.imag(), prompt_s1.real());
}


/*
 * PLL Costas loop two quadrant arctan discriminator:
 * \f{equation}
 *     \phi=ATAN\left(\frac{Q_{PS}}{I_{PS}}\right),
 * \f}
 * where \f$I_{PS1},Q_{PS1}\f$ are the inphase and quadrature prompt correlator outputs respectively. The output is in [radians].
 */
double pll_cloop_two_quadrant_atan(gr_complex prompt_s1)
{
    if (prompt_s1.real() != 0.0)
        {
            return static_cast<double>(std::atan(prompt_s1.imag() / prompt_s1.real()));
        }
    return 0.0;
}


/*
 * DLL Noncoherent Early minus Late envelope normalized discriminator:
 * \f{equation}
 *     error = \frac{y_{intercept} - \text{slope} * \epsilon}{\text{slope}} \frac{E-L}{E+L},
 * \f}
 * where \f$E=\sqrt{I_{ES}^2+Q_{ES}^2}\f$ is the Early correlator output absolute value and
 * \f$L=\sqrt{I_{LS}^2+Q_{LS}^2}\f$ is the Late correlator output absolute value. The output is in [chips].
 */
double dll_nc_e_minus_l_normalized(gr_complex early_s1, gr_complex late_s1, float spc, float slope, float y_intercept)
{
    const double P_early = std::abs(early_s1);
    const double P_late = std::abs(late_s1);
    const double E_plus_L = P_early + P_late;
    if (E_plus_L == 0.0)
        {
            return 0.0;
        }
    return ((y_intercept - slope * spc) / slope) * (P_early - P_late) / E_plus_L;
}


/*
 * DLL Noncoherent Very Early Minus Late Power (VEMLP) normalized discriminator, using the outputs
 * of four correlators, Very Early (VE), Early (E), Late (L) and Very Late (VL):
 * \f{equation}
 *  error=\frac{E-L}{E+L},
 * \f}
 * where \f$E=\sqrt{I_{VE}^2+Q_{VE}^2+I_{E}^2+Q_{E}^2}\f$ and
 * \f$L=\sqrt{I_{VL}^2+Q_{VL}^2+I_{L}^2+Q_{L}^2}\f$ . The output is in [chips].
 */
double dll_nc_vemlp_normalized(gr_complex very_early_s1, gr_complex early_s1, gr_complex late_s1, gr_complex very_late_s1)
{
    const double Early = std::sqrt(very_early_s1.real() * very_early_s1.real() + very_early_s1.imag() * very_early_s1.imag() + early_s1.real() * early_s1.real() + early_s1.imag() * early_s1.imag());
    const double Late = std::sqrt(late_s1.real() * late_s1.real() + late_s1.imag() * late_s1.imag() + very_late_s1.real() * very_late_s1.real() + very_late_s1.imag() * very_late_s1.imag());
    const double E_plus_L = Early + Late;
    if (E_plus_L == 0.0)
        {
            return 0.0;
        }
    return (Early - Late) / E_plus_L;
}


namespace
{
double boc11_correlation(double tau)
{
    return SinBocCorrelationFunction<1, 1>(tau);
}


double tmboc_correlation(double tau)
{
    return (29.0 * SinBocCorrelationFunction<1, 1>(tau) + 4.0 * SinBocCorrelationFunction<6, 1>(tau)) / 33.0;
}
}  // namespace


bool tmboc_vemlp_spacing_is_valid(double early_late_space_chips, double very_early_late_space_chips)
{
    // Discriminator output for a code delay tau [chips]. The loop is stable
    // where the output crosses zero from negative to positive.
    const auto discriminator = [&](double (*corr)(double), double tau) {
        return dll_nc_vemlp_normalized(gr_complex(static_cast<float>(corr(tau - very_early_late_space_chips)), 0.0F),
            gr_complex(static_cast<float>(corr(tau - early_late_space_chips)), 0.0F),
            gr_complex(static_cast<float>(corr(tau + early_late_space_chips)), 0.0F),
            gr_complex(static_cast<float>(corr(tau + very_early_late_space_chips)), 0.0F));
    };
    const auto tmboc = &tmboc_correlation;
    const auto boc11 = &boc11_correlation;

    // Delays at (i + 0.5) * step, so that the scan never evaluates zero delay
    constexpr int steps = 400;
    constexpr double step = 1.0 / (3.0 * steps);
    int stable_points = 0;
    bool zero_is_stable = false;
    double previous = discriminator(tmboc, (0.5 - steps) * step);
    for (int i = 1 - steps; i < steps; i++)
        {
            const double current = discriminator(tmboc, (i + 0.5) * step);
            if (previous < 0.0 && current >= 0.0)
                {
                    stable_points++;
                    zero_is_stable = zero_is_stable || (i == 0);
                }
            previous = current;
        }
    if (!zero_is_stable || stable_points != 1)
        {
            return false;
        }

    const double tmboc_gain = discriminator(tmboc, 0.5 * step) - discriminator(tmboc, -0.5 * step);
    const double boc11_gain = discriminator(boc11, 0.5 * step) - discriminator(boc11, -0.5 * step);
    return tmboc_gain >= boc11_gain;
}
