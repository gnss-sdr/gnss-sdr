/*!
 * \file fpga_switch.h
 * \brief Switch that connects the HW accelerator queues to the analog front end or the DMA.
 * \authors <ul>
 * 			<li> Marc Majoral, 2017. mmajoral(at)cttc.cat
 *          <li> Javier Arribas, 2016. jarribas(at)cttc.es
 *          </ul>
 *
 * Class that controls a switch in the FPGA
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_FPGA_SWITCH_H_
#define GNSS_SDR_FPGA_SWITCH_H_

#include <gnuradio/block.h>

#define MAX_LENGTH_DEVICEIO_NAME 50

class fpga_switch
{
public:
    fpga_switch(std::string device_name);
    ~fpga_switch();
    void set_switch_position(int switch_position);
	
private:
    int d_device_descriptor; // driver descriptor
    volatile unsigned *d_map_base; // driver memory map

    // private functions
    unsigned fpga_switch_test_register(unsigned writeval);
	void close_device(void);
	
};

#endif /* GNSS_SDR_FPGA_SWITCH_H_ */
