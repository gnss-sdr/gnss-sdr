/*!
 * \file gnss_sdr_source_b_impl.cc
 * \brief Source Block Driver Implementation for the GNSS-SDR Hacker's Edition.
 * \author Ajith Peter, Google Summer of Code 2014-15, ajith.peter(at)gmail.com
 *         Javier Arribas, 2014-15 jarribas(at)cttc.es
 *         Carles Fernandez Prades, 2014 carles.fernandez (at) cttc.cat
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <gnuradio/io_signature.h>
#include "gnss_sdr_source_b_impl.h"
#include "capture.hpp"

namespace gr {
  namespace gnss_sdr {

  gnss_sdr_source_b::sptr
  gnss_sdr_source_b::make()
  {
    return gnuradio::get_initial_sptr
      (new gnss_sdr_source_b_impl());
  }

  gnss_sdr_source_b_impl::gnss_sdr_source_b_impl()
    : gr::sync_block("gnss_sdr_source_b",
            gr::io_signature::make(0, 0, 0),
            gr::io_signature::make(1, 1, sizeof(signed char)))
  {
  }

  gnss_sdr_source_b_impl::~gnss_sdr_source_b_impl()
  {
  }

  bool gnss_sdr_source_b_impl::start()
  {
    if (cap_gnss_sdr_he.program_registers() == -1) return false;
    
    cap_gnss_sdr_he.start_capture();
    fprintf(stdout, "GNSS Sample Capture started.\n");
    fflush(stdout);
    return true;
  }

  bool gnss_sdr_source_b_impl::stop()
  {
    cap_gnss_sdr_he.stop_capture();
    fprintf(stdout, "GNSS Sample Capture stopped.\n");
    fflush(stdout);
    return true;
  }

  int
  gnss_sdr_source_b_impl::work(int noutput_items,
		  gr_vector_const_void_star &input_items,
		  gr_vector_void_star &output_items)
  {
    signed char *out = (signed char *) output_items[0];
    return cap_gnss_sdr_he.pop_samples(&out, noutput_items);
  }

  /* Setter routines for CONF1 */
  void gnss_sdr_source_b_impl::set_frontend_register_CHIPEN(unsigned int value)
  {
    cap_gnss_sdr_he.set_CHIPEN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_IDLE(unsigned int value)
  {
    cap_gnss_sdr_he.set_IDLE(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_ILNA1(unsigned int value)
  {
    cap_gnss_sdr_he.set_ILNA1(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_ILNA2(unsigned int value)
  {
    cap_gnss_sdr_he.set_ILNA2(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_ILO(unsigned int value)
  {
    cap_gnss_sdr_he.set_ILO(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_IMIX(unsigned int value)
  {
    cap_gnss_sdr_he.set_IMIX(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_MIXPOLE(unsigned int value)
  {
    cap_gnss_sdr_he.set_MIXPOLE(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_LNAMODE(unsigned int value)
  {
    cap_gnss_sdr_he.set_LNAMODE(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_MIXEN(unsigned int value)
  {
    cap_gnss_sdr_he.set_MIXEN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_ANTEN(unsigned int value)
  {
    cap_gnss_sdr_he.set_ANTEN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_FCEN(unsigned int value)
  {
    cap_gnss_sdr_he.set_FCEN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_FBW(unsigned int value)
  {
    cap_gnss_sdr_he.set_FBW(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_F3OR5(unsigned int value)
  {
    cap_gnss_sdr_he.set_F3OR5(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_FCENX(unsigned int value)
  {
    cap_gnss_sdr_he.set_FCENX(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_FGAIN(unsigned int value)
  {
    cap_gnss_sdr_he.set_FGAIN(value);
  }

  /* Setter routines for CONF2 */
  void gnss_sdr_source_b_impl::set_frontend_register_IQEN(unsigned int value)
  {
    cap_gnss_sdr_he.set_IQEN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_GAINREF(unsigned int value)
  {
    cap_gnss_sdr_he.set_GAINREF(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_AGCMODE(unsigned int value)
  {
    cap_gnss_sdr_he.set_AGCMODE(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_FORMAT(unsigned int value)
  {
    cap_gnss_sdr_he.set_FORMAT(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_BITS(unsigned int value)
  {
    cap_gnss_sdr_he.set_BITS(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_DRVCFG(unsigned int value)
  {
    cap_gnss_sdr_he.set_DRVCFG(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_LOEN(unsigned int value)
  {
    cap_gnss_sdr_he.set_LOEN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_DIEID(unsigned int value)
  {
    cap_gnss_sdr_he.set_DIEID(value);
  }

/* Setter routines for CONF3 */
  void gnss_sdr_source_b_impl::set_frontend_register_GAININ(unsigned int value)
  {
    cap_gnss_sdr_he.set_GAININ(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_FSLOWEN(unsigned int value)
  {
    cap_gnss_sdr_he.set_FSLOWEN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_HILOADEN(unsigned int value)
  {
    cap_gnss_sdr_he.set_HILOADEN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_ADCEN(unsigned int value)
  {
    cap_gnss_sdr_he.set_ADCEN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_DRVEN(unsigned int value)
  {
    cap_gnss_sdr_he.set_DRVEN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_FOFSTEN(unsigned int value)
  {
    cap_gnss_sdr_he.set_FOFSTEN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_FILTEN(unsigned int value)
  {
    cap_gnss_sdr_he.set_FILTEN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_FHIPEN(unsigned int value)
  {
    cap_gnss_sdr_he.set_FHIPEN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_PGAIEN(unsigned int value)
  {
    cap_gnss_sdr_he.set_PGAIEN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_PGAQEN(unsigned int value)
  {
    cap_gnss_sdr_he.set_PGAQEN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_STRMEN(unsigned int value)
  {
    cap_gnss_sdr_he.set_STRMEN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_STRMSTART(unsigned int value)
  {
    cap_gnss_sdr_he.set_STRMSTART(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_STRMSTOP(unsigned int value)
  {
    cap_gnss_sdr_he.set_STRMSTOP(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_STRMCOUNT(unsigned int value)
  {
    cap_gnss_sdr_he.set_STRMCOUNT(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_STRMBITS(unsigned int value)
  {
    cap_gnss_sdr_he.set_STRMBITS(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_STAMPEN(unsigned int value)
  {
    cap_gnss_sdr_he.set_STAMPEN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_TIMESYNCEN(unsigned int value)
  {
    cap_gnss_sdr_he.set_TIMESYNCEN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_DATSYNCEN(unsigned int value)
  {
    cap_gnss_sdr_he.set_DATSYNCEN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_STRMRST(unsigned int value)
  {
    cap_gnss_sdr_he.set_STRMRST(value);
  }

  /* Setter routines for PLLCONF */

  void gnss_sdr_source_b_impl::set_frontend_register_VCOEN(unsigned int value)
  {
    cap_gnss_sdr_he.set_VCOEN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_IVCO(unsigned int value)
  {
    cap_gnss_sdr_he.set_IVCO(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_REFOUTEN(unsigned int value)
  {
    cap_gnss_sdr_he.set_REFOUTEN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_REFDIV(unsigned int value)
  {
    cap_gnss_sdr_he.set_REFDIV(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_IXTAL(unsigned int value)
  {
    cap_gnss_sdr_he.set_IXTAL(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_XTALCAP(unsigned int value)
  {
    cap_gnss_sdr_he.set_XTALCAP(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_LDMUX(unsigned int value)
  {
    cap_gnss_sdr_he.set_LDMUX(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_ICP(unsigned int value)
  {
    cap_gnss_sdr_he.set_ICP(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_PFDEN(unsigned int value)
  {
    cap_gnss_sdr_he.set_PFDEN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_CPTEST(unsigned int value)
  {
    cap_gnss_sdr_he.set_CPTEST(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_INT_PLL(unsigned int value)
  {
    cap_gnss_sdr_he.set_INT_PLL(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_PWRSAV(unsigned int value)
  {
    cap_gnss_sdr_he.set_PWRSAV(value);
  }

  /* Setter routines for DIV */
  void gnss_sdr_source_b_impl::set_frontend_register_NDIV(unsigned int value)
  {
    cap_gnss_sdr_he.set_NDIV(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_RDIV(unsigned int value)
  {
    cap_gnss_sdr_he.set_RDIV(value);
  }

  /* Setter routines for FDIV */
  void gnss_sdr_source_b_impl::set_frontend_register_FDIV(unsigned int value)
  {
    cap_gnss_sdr_he.set_FDIV(value);
  }

  /* Setter routines for STRM */
  void gnss_sdr_source_b_impl::set_frontend_register_FRAMECOUNT(unsigned int value)
  {
    cap_gnss_sdr_he.set_FRAMECOUNT(value);
  }

  /* Setter routines for CLK */
  void gnss_sdr_source_b_impl::set_frontend_register_L_CNT(unsigned int value)
  {
    cap_gnss_sdr_he.set_L_CNT(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_M_CNT(unsigned int value)
  {
    cap_gnss_sdr_he.set_M_CNT(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_FCLKIN(unsigned int value)
  {
    cap_gnss_sdr_he.set_FCLKIN(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_ADCCLK(unsigned int value)
  {
    cap_gnss_sdr_he.set_ADCCLK(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_SERCLK(unsigned int value)
  {
    cap_gnss_sdr_he.set_SERCLK(value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_MODE(unsigned int value)
  {
    cap_gnss_sdr_he.set_MODE(value);
  }


  } /* namespace gnss_sdr */
} /* namespace gr */

