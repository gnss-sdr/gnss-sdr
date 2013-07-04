/*!
 * \file gn3s_source_cc.h
 * \brief GNU Radio source block to acces to SiGe GN3S USB sampler v2.
 * \author Javier Arribas, 2012. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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
#ifndef INCLUDED_GN3S_SOURCE_CC_H
#define INCLUDED_GN3S_SOURCE_CC_H

#include "gn3s_api.h"
#include <gnuradio/block.h>
#include "gn3s_source.h"
#include "gn3s_defines.h"

class gn3s_source_cc;

/*
 * We use boost::shared_ptr's instead of raw pointers for all access
 * to gr_blocks (and many other data structures).  The shared_ptr gets
 * us transparent reference counting, which greatly simplifies storage
 * management issues.  This is especially helpful in our hybrid
 * C++ / Python system.
 *
 * See http://www.boost.org/libs/smart_ptr/smart_ptr.htm
 *
 * As a convention, the _sptr suffix indicates a boost::shared_ptr
 */
typedef boost::shared_ptr<gn3s_source_cc> gn3s_source_cc_sptr;

/*!
 * \brief Return a shared_ptr to a new instance of howto_square_ff.
 *
 * To avoid accidental use of raw pointers, gn3s_source's
 * constructor is private.  gn3s_source is the public
 * interface for creating new instances.
 */
GN3S_API gn3s_source_cc_sptr gn3s_make_source_cc ();

/*!
 * \brief SiGe GN3S V2 sampler USB driver.
 * \ingroup block
 *
 * \sa gn3s_source for a version that subclasses gr_block.
 */
class GN3S_API gn3s_source_cc : public gr::block
{
private:
  // The friend declaration allows gn3s_source to
  // access the private constructor.

  /* Create the GN3S object*/
  gn3s_Source *gn3s_drv;
  gn3s_ms_packet packet;

  friend GN3S_API gn3s_source_cc_sptr gn3s_make_source_cc ();

  /*!
   * \brief
   */
  gn3s_source_cc ();  	// private constructor

 public:
  ~gn3s_source_cc ();	// public destructor

  // Where all the action really happens

  int general_work (int noutput_items,
		    gr_vector_int &ninput_items,
		    gr_vector_const_void_star &input_items,
		    gr_vector_void_star &output_items);
};

#endif /* INCLUDED_GN3S_SOURCE_CC_H */
