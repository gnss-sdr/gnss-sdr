/*!
 * \file gnss_sdr_source_b_impl.h
 * \brief Source Driver Implementation Header for the GNSS-SDR Hacker's Edition
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
#ifndef INCLUDED_GNSS_SDR_GNSS_SDR_SOURCE_B_IMPL_H
#define INCLUDED_GNSS_SDR_GNSS_SDR_SOURCE_B_IMPL_H

#include <gnss_sdr/gnss_sdr_source_b.h>
extern "C"
{
  #include "gnss_stream.h"
}
namespace gr {
  namespace gnss_sdr {

    class gnss_sdr_source_b_impl : public gnss_sdr_source_b
    {
     private:
      struct gnss_device_context dev;

     public:
      gnss_sdr_source_b_impl();
      ~gnss_sdr_source_b_impl();
      bool start();
      bool stop();
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);

      /* Routines to program configuration registers*/
      /* Setter routines for CONF1 */

      void set_frontend_register_CHIPEN(unsigned int);
      void set_frontend_register_IDLE(unsigned int);
      void set_frontend_register_ILNA1(unsigned int);
      void set_frontend_register_ILNA2(unsigned int);
      void set_frontend_register_ILO(unsigned int);
      void set_frontend_register_IMIX(unsigned int);
      void set_frontend_register_MIXPOLE(unsigned int);
      void set_frontend_register_LNAMODE(unsigned int);
      void set_frontend_register_MIXEN(unsigned int);
      void set_frontend_register_ANTEN(unsigned int);
      void set_frontend_register_FCEN(unsigned int);
      void set_frontend_register_FBW(unsigned int);
      void set_frontend_register_F3OR5(unsigned int);
      void set_frontend_register_FCENX(unsigned int);
      void set_frontend_register_FGAIN(unsigned int);

      /* Setter routines for CONF2 */

      void set_frontend_register_IQEN(unsigned int);
      void set_frontend_register_GAINREF(unsigned int);
      void set_frontend_register_AGCMODE(unsigned int);
      void set_frontend_register_FORMAT(unsigned int);
      void set_frontend_register_BITS(unsigned int);
      void set_frontend_register_DRVCFG(unsigned int);
      void set_frontend_register_LOEN(unsigned int);
      void set_frontend_register_DIEID(unsigned int);

      /* Setter routines for CONF3 */

      void set_frontend_register_GAININ(unsigned int);
      void set_frontend_register_FSLOWEN(unsigned int);
      void set_frontend_register_HILOADEN(unsigned int);
      void set_frontend_register_ADCEN(unsigned int);
      void set_frontend_register_DRVEN(unsigned int);
      void set_frontend_register_FOFSTEN(unsigned int);
      void set_frontend_register_FILTEN(unsigned int);
      void set_frontend_register_FHIPEN(unsigned int);
      void set_frontend_register_PGAIEN(unsigned int);
      void set_frontend_register_PGAQEN(unsigned int);
      void set_frontend_register_STRMEN(unsigned int);
      void set_frontend_register_STRMSTART(unsigned int);
      void set_frontend_register_STRMSTOP(unsigned int);
      void set_frontend_register_STRMCOUNT(unsigned int);
      void set_frontend_register_STRMBITS(unsigned int);
      void set_frontend_register_STAMPEN(unsigned int);
      void set_frontend_register_TIMESYNCEN(unsigned int);
      void set_frontend_register_DATSYNCEN(unsigned int);
      void set_frontend_register_STRMRST(unsigned int);

      /* Setter routines for PLLCONF */

      void set_frontend_register_VCOEN(unsigned int);
      void set_frontend_register_IVCO(unsigned int);
      void set_frontend_register_REFOUTEN(unsigned int);
      void set_frontend_register_REFDIV(unsigned int);
      void set_frontend_register_IXTAL(unsigned int);
      void set_frontend_register_XTALCAP(unsigned int);
      void set_frontend_register_LDMUX(unsigned int);
      void set_frontend_register_ICP(unsigned int);
      void set_frontend_register_PFDEN(unsigned int);
      void set_frontend_register_CPTEST(unsigned int);
      void set_frontend_register_INT_PLL(unsigned int);
      void set_frontend_register_PWRSAV(unsigned int);

      /* Setter routines for DIV */

      void set_frontend_register_NDIV(unsigned int);
      void set_frontend_register_RDIV(unsigned int);

      /* Setter routines for FDIV */

      void set_frontend_register_FDIV(unsigned int);

      /* Setter routines for STRM */

      void set_frontend_register_FRAMECOUNT(unsigned int);

      /* Setter routines for CLK */

      void set_frontend_register_L_CNT(unsigned int);
      void set_frontend_register_M_CNT(unsigned int);
      void set_frontend_register_FCLKIN(unsigned int);
      void set_frontend_register_ADCCLK(unsigned int);
      void set_frontend_register_SERCLK(unsigned int);
      void set_frontend_register_MODE(unsigned int);
    };


  } // namespace gnss_sdr
} // namespace gr

#endif /* INCLUDED_GNSS_SDR_GNSS_SDR_SOURCE_B_IMPL_H */

