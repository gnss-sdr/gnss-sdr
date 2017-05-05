/*!
 * \file gps_fpga_acquisition_8sc.cc
 * \brief High optimized FPGA vector correlator class
 * \authors <ul>
 *    <li> Marc Majoral, 2017. mmajoral(at)cttc.cat
 * </ul>
 *
 * Class that controls and executes a high optimized vector correlator
 * class in the FPGA
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

#include "gps_fpga_acquisition_8sc.h"
#include <cmath>

// allocate memory dynamically
#include <new>

// libraries used by DMA test code and GIPO test code
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

// libraries used by DMA test code
#include <sys/stat.h>
#include <stdint.h>
#include <unistd.h>
#include <assert.h>

// libraries used by GPIO test code
#include <stdlib.h>
#include <signal.h>
#include <sys/mman.h>

// logging
#include <glog/logging.h>

#include "GPS_L1_CA.h"

#define PAGE_SIZE 0x10000
#define CODE_RESAMPLER_NUM_BITS_PRECISION 20
#define CODE_PHASE_STEP_CHIPS_NUM_NBITS CODE_RESAMPLER_NUM_BITS_PRECISION
#define pwrtwo(x) (1 << (x))
#define MAX_CODE_RESAMPLER_COUNTER pwrtwo(CODE_PHASE_STEP_CHIPS_NUM_NBITS) // 2^CODE_PHASE_STEP_CHIPS_NUM_NBITS
#define PHASE_CARR_NBITS 32
#define PHASE_CARR_NBITS_INT 1
#define PHASE_CARR_NBITS_FRAC PHASE_CARR_NBITS - PHASE_CARR_NBITS_INT

#define MAX_PHASE_STEP_RAD 0.999999999534339 // 1 - pow(2,-31);


	bool gps_fpga_acquisition_8sc::init(unsigned int fft_size, unsigned int nsamples_total, long freq, unsigned int doppler_max, unsigned int doppler_step, int num_doppler_bins, long fs_in, unsigned select_queue)
	{
		float phase_step_rad_fpga;
		float phase_step_rad_fpga_real;

		d_phase_step_rad_vector = new float[num_doppler_bins];

		for (unsigned int doppler_index = 0; doppler_index < num_doppler_bins; doppler_index++)
		{
			int doppler = -static_cast<int>(doppler_max) + doppler_step * doppler_index;
			float phase_step_rad = GPS_TWO_PI * (freq + doppler) / static_cast<float>(fs_in);
			// The doppler step can never be outside the range -pi to +pi, otherwise there would be aliasing
			// The FPGA expects phase_step_rad between -1 (-pi) to +1 (+pi)
			// The FPGA also expects the phase to be negative since it produces cos(x) -j*sin(x)
			// while the gnss-sdr software (volk_gnsssdr_s32f_sincos_32fc) generates cos(x) + j*sin(x)
			phase_step_rad_fpga = phase_step_rad/(GPS_TWO_PI/2);
			// avoid saturation of the fixed point representation in the fpga
			// (only the positive value can saturate due to the 2's complement representation)
			if (phase_step_rad_fpga == 1.0)
			{
					phase_step_rad_fpga = MAX_PHASE_STEP_RAD;
			}
			d_phase_step_rad_vector[doppler_index] = phase_step_rad_fpga;

		}

	    // sanity check : check test register
	    unsigned writeval = 0x55AA;
	    unsigned readval;
	    readval = gps_fpga_acquisition_8sc::fpga_acquisition_test_register(writeval);
	    if (writeval != readval)
	        {
	    		printf("test register fail\n");
	            LOG(WARNING) << "Acquisition test register sanity check failed";
	        }
	    else
	        {
	    		printf("test register success\n");
	            LOG(INFO) << "Acquisition test register sanity check success !";
	        }

		d_nsamples = fft_size;
		d_nsamples_total = nsamples_total;

	    gps_fpga_acquisition_8sc::configure_acquisition();

    return true;
}



bool gps_fpga_acquisition_8sc::set_local_code(gr_complex* fft_codes)
{
	int i;
	float val;
	float max = 0;
	d_fft_codes = new lv_16sc_t[d_nsamples_total];

	for (i=0;i<d_nsamples_total;i++)
	{
		if(abs(fft_codes[i].real()) > max)
		{
			max = abs(fft_codes[i].real());
		}
		if(abs(fft_codes[i].imag()) > max)
		{
			max = abs(fft_codes[i].imag());
		}
	}

	for (i=0;i<d_nsamples_total;i++)
	{
		d_fft_codes[i] = lv_16sc_t((int) (fft_codes[i].real()*(pow(2,7) - 1)/max), (int) (fft_codes[i].imag()*(pow(2,7) - 1)/max));
	}

	gps_fpga_acquisition_8sc::fpga_configure_acquisition_local_code(d_fft_codes);

    return true;
}



gps_fpga_acquisition_8sc::gps_fpga_acquisition_8sc()
{


    if ((d_fd = open(d_device_io_name, O_RDWR | O_SYNC )) == -1)
        {
            LOG(WARNING) << "Cannot open deviceio" << d_device_io_name;
        }
    d_map_base = (volatile unsigned *)mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, d_fd,0);

    if (d_map_base == (void *) -1)
        {
            LOG(WARNING) << "Cannot map the FPGA acquisition module into user memory";
        }

}


gps_fpga_acquisition_8sc::~gps_fpga_acquisition_8sc()
{

	if (munmap((void*)d_map_base, PAGE_SIZE) == -1)
	{
		printf("Failed to unmap memory uio\n");
	}

    close(d_fd);

}


bool gps_fpga_acquisition_8sc::free()
{
	if (d_fft_codes != nullptr)
	{
		delete [] d_fft_codes;
		d_fft_codes = nullptr;
	}
	if (d_phase_step_rad_vector != nullptr)
	{
		delete [] d_phase_step_rad_vector;
		d_phase_step_rad_vector = nullptr;
	}

    return true;
}


unsigned gps_fpga_acquisition_8sc::fpga_acquisition_test_register(unsigned writeval)
{

	unsigned readval;
	// write value to test register
	d_map_base[15] = writeval;
	// read value from test register
	readval = d_map_base[15];
	// return read value
	return readval;
}

void gps_fpga_acquisition_8sc::fpga_configure_acquisition_local_code(lv_16sc_t fft_local_code[])
{
	short int local_code;
	unsigned int k, tmp, tmp2;

	// clear memory address counter
	d_map_base[4] = 0x10000000;
	for (k = 0; k < d_nsamples_total; k++)
	{
		tmp = fft_local_code[k].real();
		tmp2 = fft_local_code[k].imag();
		local_code = (tmp & 0xFF) | ((tmp2*256) & 0xFF00); // put together the real part and the imaginary part
		d_map_base[4] = 0x0C000000 | (local_code & 0xFFFF);
	}

}


void gps_fpga_acquisition_8sc::run_acquisition(void)
{
	// enable interrupts
	int reenable = 1;
	write(d_fd, (void *)&reenable, sizeof(int));

	d_map_base[5] = 0;	// writing anything to reg 4 launches the acquisition process

	int irq_count;
    ssize_t nb;
    // wait for interrupt
    nb=read(d_fd, &irq_count, sizeof(irq_count));
    if (nb != sizeof(irq_count))
        {
            printf("Tracking_module Read failed to retrive 4 bytes!\n");
            printf("Tracking_module Interrupt number %d\n", irq_count);
        }



}




void gps_fpga_acquisition_8sc::configure_acquisition()
{

	d_map_base[0] = d_select_queue;
	d_map_base[1] = d_nsamples_total;
	d_map_base[2] = d_nsamples;


}

void gps_fpga_acquisition_8sc::set_phase_step(unsigned int doppler_index)
{
	float phase_step_rad_real;
	float phase_step_rad_int_temp;
	int32_t phase_step_rad_int;

	phase_step_rad_real = d_phase_step_rad_vector[doppler_index];

	phase_step_rad_int_temp = phase_step_rad_real*4;				// * 2^2
	phase_step_rad_int = (int32_t) (phase_step_rad_int_temp*(536870912));		// * 2^29 (in total it makes x2^31 in two steps to avoid the warnings

	d_map_base[3] = phase_step_rad_int;

}

void gps_fpga_acquisition_8sc::read_acquisition_results(uint32_t* max_index, float* max_magnitude, unsigned *initial_sample, float *power_sum)
{
	unsigned readval = 0;
	readval = d_map_base[0];
	readval = d_map_base[1];
	*initial_sample = readval;
	readval = d_map_base[2];
	*max_magnitude = (float) readval;
	readval = d_map_base[4];
	*power_sum = (float) readval;
	readval = d_map_base[3];
	*max_index = readval;

}

void gps_fpga_acquisition_8sc::block_samples()
{
	d_map_base[14] = 1; // block the samples
}

void gps_fpga_acquisition_8sc::unblock_samples()
{
	d_map_base[14] = 0; // unblock the samples
}


