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
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_TRACKING_DISCRIMINATORS_H_
#define GNSS_SDR_TRACKING_DISCRIMINATORS_H_

#include <gnuradio/gr_complex.h>

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
 *     error=\frac{E-L}{E+L},
 * \f}
 * where \f$E=\sqrt{I_{ES}^2+Q_{ES}^2}\f$ is the Early correlator output absolute value and
 * \f$L=\sqrt{I_{LS}^2+Q_{LS}^2}\f$ is the Late correlator output absolute value. The output is in [chips].
 */
double dll_nc_e_minus_l_normalized(gr_complex early_s1, gr_complex late_s1);


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


#endif
