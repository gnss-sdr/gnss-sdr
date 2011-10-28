/*!
 * \file gps_telemetry.cc
 * \brief GPS L1 C/A telemetry processing
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
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

#include "gps_telemetry.h"

#include <iostream>
#include <bitset>
#include <cstring>

bool gps_word_parityCheck(unsigned int gpsword)
{
  unsigned int d1,d2,d3,d4,d5,d6,d7,t,parity;

  /* XOR as many bits in parallel as possible.  The magic constants pick
       up bits which are to be XOR'ed together to implement the GPS parity
       check algorithm described in ICD-GPS-200.  This avoids lengthy shift-
       and-xor loops. */

  d1 = gpsword & 0xFBFFBF00;
  d2 = _lrotl(gpsword,1) & 0x07FFBF01;
  d3 = _lrotl(gpsword,2) & 0xFC0F8100;
  d4 = _lrotl(gpsword,3) & 0xF81FFE02;
  d5 = _lrotl(gpsword,4) & 0xFC00000E;
  d6 = _lrotl(gpsword,5) & 0x07F00001;
  d7 = _lrotl(gpsword,6) & 0x00003000;

  t = d1 ^ d2 ^ d3 ^ d4 ^ d5 ^ d6 ^ d7;

  // Now XOR the 5 6-bit fields together to produce the 6-bit final result.

  parity = t ^ _lrotl(t,6) ^ _lrotl(t,12) ^ _lrotl(t,18) ^ _lrotl(t,24);
  parity = parity & 0x3F;
  if (parity == (gpsword&0x3F))
    return(true);
  else
    return(false);

}

char bit_mask(int num_bits)
{
  char mask;
  mask=1;
  for (int i=0;i<(num_bits-1);i++)
    {
    mask<<=1;
    mask++;
    }
  return mask;
}

char bit_select(int num_bit)
{
  char mask;
  mask=1;
  for (int i=0;i<(num_bit-1);i++)
    {
    mask<<=1;
    }
  return mask;
}

void print_gps_word_bytes(unsigned int GPS_word)
{
  std::cout << " Word =";
  std::cout<<std::bitset<32>(GPS_word);
  std::cout<<std::endl;
}
/*----------------------------------------------------------------------------------------------*/



