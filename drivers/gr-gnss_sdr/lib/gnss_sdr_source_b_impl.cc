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
extern "C"
{
  #include "gnss_stream.h"
}

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
    init_gnss_device_context(&(this->dev));
  }

  gnss_sdr_source_b_impl::~gnss_sdr_source_b_impl()
  {
  }

  bool gnss_sdr_source_b_impl::start()
  {
    if (program_registers(&(this->dev)) == -1) return false;
    if (open_GNSS_SDR(&(this->dev)) == -1) return false;
    start_capture(&(this->dev));
    fprintf(stdout, "GNSS Sample Capture started.\n");
    fflush(stdout);
    return true;
  }

  bool gnss_sdr_source_b_impl::stop()
  {
    stop_capture(&(this->dev));
    close_GNSS_SDR(&(this->dev));
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
    return get_next_packet_data(&(this->dev), &out, noutput_items);
  }

  /* Setter routines for CONF1 */
  void gnss_sdr_source_b_impl::set_frontend_register_CHIPEN(unsigned int value)
  {
    set_CHIPEN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_IDLE(unsigned int value)
  {
    set_IDLE(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_ILNA1(unsigned int value)
  {
    set_ILNA1(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_ILNA2(unsigned int value)
  {
    set_ILNA2(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_ILO(unsigned int value)
  {
    set_ILO(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_IMIX(unsigned int value)
  {
    set_IMIX(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_MIXPOLE(unsigned int value)
  {
    set_MIXPOLE(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_LNAMODE(unsigned int value)
  {
    set_LNAMODE(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_MIXEN(unsigned int value)
  {
    set_MIXEN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_ANTEN(unsigned int value)
  {
    set_ANTEN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_FCEN(unsigned int value)
  {
    set_FCEN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_FBW(unsigned int value)
  {
    set_FBW(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_F3OR5(unsigned int value)
  {
    set_F3OR5(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_FCENX(unsigned int value)
  {
    set_FCENX(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_FGAIN(unsigned int value)
  {
    set_FGAIN(&(this->dev), value);
  }

  /* Setter routines for CONF2 */
  void gnss_sdr_source_b_impl::set_frontend_register_IQEN(unsigned int value)
  {
    set_IQEN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_GAINREF(unsigned int value)
  {
    set_GAINREF(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_AGCMODE(unsigned int value)
  {
    set_AGCMODE(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_FORMAT(unsigned int value)
  {
    set_FORMAT(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_BITS(unsigned int value)
  {
    set_BITS(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_DRVCFG(unsigned int value)
  {
    set_DRVCFG(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_LOEN(unsigned int value)
  {
    set_LOEN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_DIEID(unsigned int value)
  {
    set_DIEID(&(this->dev), value);
  }

/* Setter routines for CONF3 */
  void gnss_sdr_source_b_impl::set_frontend_register_GAININ(unsigned int value)
  {
    set_GAININ(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_FSLOWEN(unsigned int value)
  {
    set_FSLOWEN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_HILOADEN(unsigned int value)
  {
    set_HILOADEN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_ADCEN(unsigned int value)
  {
    set_ADCEN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_DRVEN(unsigned int value)
  {
    set_DRVEN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_FOFSTEN(unsigned int value)
  {
    set_FOFSTEN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_FILTEN(unsigned int value)
  {
    set_FILTEN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_FHIPEN(unsigned int value)
  {
    set_FHIPEN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_PGAIEN(unsigned int value)
  {
    set_PGAIEN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_PGAQEN(unsigned int value)
  {
    set_PGAQEN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_STRMEN(unsigned int value)
  {
    set_STRMEN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_STRMSTART(unsigned int value)
  {
    set_STRMSTART(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_STRMSTOP(unsigned int value)
  {
    set_STRMSTOP(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_STRMCOUNT(unsigned int value)
  {
    set_STRMCOUNT(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_STRMBITS(unsigned int value)
  {
    set_STRMBITS(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_STAMPEN(unsigned int value)
  {
    set_STAMPEN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_TIMESYNCEN(unsigned int value)
  {
    set_TIMESYNCEN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_DATSYNCEN(unsigned int value)
  {
    set_DATSYNCEN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_STRMRST(unsigned int value)
  {
    set_STRMRST(&(this->dev), value);
  }

  /* Setter routines for PLLCONF */

  void gnss_sdr_source_b_impl::set_frontend_register_VCOEN(unsigned int value)
  {
    set_VCOEN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_IVCO(unsigned int value)
  {
    set_IVCO(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_REFOUTEN(unsigned int value)
  {
    set_REFOUTEN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_REFDIV(unsigned int value)
  {
    set_REFDIV(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_IXTAL(unsigned int value)
  {
    set_IXTAL(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_XTALCAP(unsigned int value)
  {
    set_XTALCAP(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_LDMUX(unsigned int value)
  {
    set_LDMUX(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_ICP(unsigned int value)
  {
    set_ICP(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_PFDEN(unsigned int value)
  {
    set_PFDEN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_CPTEST(unsigned int value)
  {
    set_CPTEST(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_INT_PLL(unsigned int value)
  {
    set_INT_PLL(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_PWRSAV(unsigned int value)
  {
    set_PWRSAV(&(this->dev), value);
  }

  /* Setter routines for DIV */
  void gnss_sdr_source_b_impl::set_frontend_register_NDIV(unsigned int value)
  {
    set_NDIV(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_RDIV(unsigned int value)
  {
    set_RDIV(&(this->dev), value);
  }

  /* Setter routines for FDIV */
  void gnss_sdr_source_b_impl::set_frontend_register_FDIV(unsigned int value)
  {
    set_FDIV(&(this->dev), value);
  }

  /* Setter routines for STRM */
  void gnss_sdr_source_b_impl::set_frontend_register_FRAMECOUNT(unsigned int value)
  {
    set_FRAMECOUNT(&(this->dev), value);
  }

  /* Setter routines for CLK */
  void gnss_sdr_source_b_impl::set_frontend_register_L_CNT(unsigned int value)
  {
    set_L_CNT(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_M_CNT(unsigned int value)
  {
    set_M_CNT(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_FCLKIN(unsigned int value)
  {
    set_FCLKIN(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_ADCCLK(unsigned int value)
  {
    set_ADCCLK(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_SERCLK(unsigned int value)
  {
    set_SERCLK(&(this->dev), value);
  }
  void gnss_sdr_source_b_impl::set_frontend_register_MODE(unsigned int value)
  {
    set_MODE(&(this->dev), value);
  }


  } /* namespace gnss_sdr */
} /* namespace gr */

