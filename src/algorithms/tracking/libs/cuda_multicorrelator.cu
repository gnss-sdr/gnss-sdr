/*!
 * \file cuda_multicorrelator.cu
 * \brief High optimized CUDA GPU vector multiTAP correlator class
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
 *          </ul>
 *
 * Class that implements a high optimized vector multiTAP correlator class for NVIDIA CUDA GPUs
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

#include "cuda_multicorrelator.h"

#include <stdio.h>
#include <iostream>
// For the CUDA runtime routines (prefixed with "cuda_")
#include <cuda_runtime.h>

#define ACCUM_N 128

__global__ void Doppler_wippe_scalarProdGPUCPXxN_shifts_chips(
    GPU_Complex *d_corr_out,
    GPU_Complex *d_sig_in,
    GPU_Complex *d_sig_wiped,
    GPU_Complex *d_local_code_in,
    float *d_shifts_chips,
    int code_length_chips,
    float code_phase_step_chips,
    float rem_code_phase_chips,
    int vectorN,
    int elementN,
    float rem_carrier_phase_in_rad,
    float phase_step_rad
)
{
    //Accumulators cache
    __shared__ GPU_Complex accumResult[ACCUM_N];

	// CUDA version of floating point NCO and vector dot product integrated
    float sin;
    float cos;
    for (int i = blockIdx.x * blockDim.x + threadIdx.x;
         i < elementN;
         i += blockDim.x * gridDim.x)
    {
    	__sincosf(rem_carrier_phase_in_rad + i*phase_step_rad, &sin, &cos);
    	d_sig_wiped[i] =  d_sig_in[i] * GPU_Complex(cos,-sin);
    }

    __syncthreads();
    ////////////////////////////////////////////////////////////////////////////
    // Cycle through every pair of vectors,
    // taking into account that vector counts can be different
    // from total number of thread blocks
    ////////////////////////////////////////////////////////////////////////////
    for (int vec = blockIdx.x; vec < vectorN; vec += gridDim.x)
    {
        //int vectorBase = IMUL(elementN, vec);
        //int vectorEnd  = elementN;

        ////////////////////////////////////////////////////////////////////////
        // Each accumulator cycles through vectors with
        // stride equal to number of total number of accumulators ACCUM_N
        // At this stage ACCUM_N is only preferred be a multiple of warp size
        // to meet memory coalescing alignment constraints.
        ////////////////////////////////////////////////////////////////////////
        for (int iAccum = threadIdx.x; iAccum < ACCUM_N; iAccum += blockDim.x)
        {
        	GPU_Complex sum = GPU_Complex(0,0);
            float local_code_chip_index=0.0;;
            //float code_phase;
            for (int pos = iAccum; pos < elementN; pos += ACCUM_N)
            {
            	//original sample code
                //sum = sum + d_sig_in[pos-vectorBase] * d_nco_in[pos-vectorBase] * d_local_codes_in[pos];
            	//sum = sum + d_sig_in[pos-vectorBase] * d_local_codes_in[pos];
            	//sum.multiply_acc(d_sig_in[pos],d_local_codes_in[pos+d_shifts_samples[vec]]);

            	//custom code for multitap correlator
            	// 1.resample local code for the current shift

            	local_code_chip_index= fmodf(code_phase_step_chips*__int2float_rd(pos)+ d_shifts_chips[vec] - rem_code_phase_chips, code_length_chips);

            	//Take into account that in multitap correlators, the shifts can be negative!
            	if (local_code_chip_index<0.0) local_code_chip_index+=(code_length_chips-1);
            	//printf("vec= %i, pos %i, chip_idx=%i chip_shift=%f \r\n",vec, pos,__float2int_rd(local_code_chip_index),local_code_chip_index);
            	// 2.correlate
            	sum.multiply_acc(d_sig_wiped[pos],d_local_code_in[__float2int_rd(local_code_chip_index)]);

            }
            accumResult[iAccum] = sum;
        }

        ////////////////////////////////////////////////////////////////////////
        // Perform tree-like reduction of accumulators' results.
        // ACCUM_N has to be power of two at this stage
        ////////////////////////////////////////////////////////////////////////
        for (int stride = ACCUM_N / 2; stride > 0; stride >>= 1)
        {
            __syncthreads();

            for (int iAccum = threadIdx.x; iAccum < stride; iAccum += blockDim.x)
            {
                accumResult[iAccum] += accumResult[stride + iAccum];
            }
        }

        if (threadIdx.x == 0)
        	{
        		d_corr_out[vec] = accumResult[0];
        	}
    }
}

bool cuda_multicorrelator::init_cuda_integrated_resampler(
		int signal_length_samples,
		int code_length_chips,
		int n_correlators
		)
{
	// use command-line specified CUDA device, otherwise use device with highest Gflops/s
//	findCudaDevice(argc, (const char **)argv);
      cudaDeviceProp  prop;
    int num_devices, device;
    cudaGetDeviceCount(&num_devices);
    if (num_devices > 1) {
          int max_multiprocessors = 0, max_device = 0;
          for (device = 0; device < num_devices; device++) {
                  cudaDeviceProp properties;
                  cudaGetDeviceProperties(&properties, device);
                  if (max_multiprocessors < properties.multiProcessorCount) {
                          max_multiprocessors = properties.multiProcessorCount;
                          max_device = device;
                  }
                  printf("Found GPU device # %i\n",device);
          }
          //cudaSetDevice(max_device);

          //set random device!
	  selected_gps_device=rand() % num_devices;//generates a random number between 0 and num_devices to split the threads between GPUs
          cudaSetDevice(selected_gps_device); 

          cudaGetDeviceProperties( &prop, max_device );
          //debug code
          if (prop.canMapHostMemory != 1) {
              printf( "Device can not map memory.\n" );
          }
          printf("L2 Cache size= %u \n",prop.l2CacheSize);
          printf("maxThreadsPerBlock= %u \n",prop.maxThreadsPerBlock);
          printf("maxGridSize= %i \n",prop.maxGridSize[0]);
          printf("sharedMemPerBlock= %lu \n",prop.sharedMemPerBlock);
          printf("deviceOverlap= %i \n",prop.deviceOverlap);
  	    printf("multiProcessorCount= %i \n",prop.multiProcessorCount);
    }else{
    	    cudaGetDevice( &selected_gps_device);
    	    cudaGetDeviceProperties( &prop, selected_gps_device );
    	    //debug code
    	    if (prop.canMapHostMemory != 1) {
    	        printf( "Device can not map memory.\n" );
    	    }

    	    printf("L2 Cache size= %u \n",prop.l2CacheSize);
    	    printf("maxThreadsPerBlock= %u \n",prop.maxThreadsPerBlock);
    	    printf("maxGridSize= %i \n",prop.maxGridSize[0]);
    	    printf("sharedMemPerBlock= %lu \n",prop.sharedMemPerBlock);
    	    printf("deviceOverlap= %i \n",prop.deviceOverlap);
    	    printf("multiProcessorCount= %i \n",prop.multiProcessorCount);
    }

	// (cudaFuncSetCacheConfig(CUDA_32fc_x2_multiply_x2_dot_prod_32fc_, cudaFuncCachePreferShared));

    // ALLOCATE GPU MEMORY FOR INPUT/OUTPUT and INTERNAL vectors
    size_t size = signal_length_samples * sizeof(GPU_Complex);

	//********* ZERO COPY VERSION ************
	// Set flag to enable zero copy access
    // Optimal in shared memory devices (like Jetson K1)
	//cudaSetDeviceFlags(cudaDeviceMapHost);

	//******** CudaMalloc version ***********

	// input signal GPU memory (can be mapped to CPU memory in shared memory devices!)
	//	cudaMalloc((void **)&d_sig_in, size);
	//	cudaMemset(d_sig_in,0,size);

	// Doppler-free signal (internal GPU memory)
	cudaMalloc((void **)&d_sig_doppler_wiped, size);
	cudaMemset(d_sig_doppler_wiped,0,size);

	// Local code GPU memory (can be mapped to CPU memory in shared memory devices!)
	cudaMalloc((void **)&d_local_codes_in, sizeof(std::complex<float>)*code_length_chips);
	cudaMemset(d_local_codes_in,0,sizeof(std::complex<float>)*code_length_chips);

    d_code_length_chips=code_length_chips;

	// Vector with the chip shifts for each correlator tap
    //GPU memory (can be mapped to CPU memory in shared memory devices!)
	cudaMalloc((void **)&d_shifts_chips, sizeof(float)*n_correlators);
	cudaMemset(d_shifts_chips,0,sizeof(float)*n_correlators);

	//scalars
	//cudaMalloc((void **)&d_corr_out, sizeof(std::complex<float>)*n_correlators);
	//cudaMemset(d_corr_out,0,sizeof(std::complex<float>)*n_correlators);

    // Launch the Vector Add CUDA Kernel
    // TODO: write a smart load balance using device info!
	threadsPerBlock = 64;
    blocksPerGrid = 128;//(int)(signal_length_samples+threadsPerBlock-1)/threadsPerBlock;

	cudaStreamCreate (&stream1) ;
	//cudaStreamCreate (&stream2) ;
	return true;
}

bool cuda_multicorrelator::set_local_code_and_taps(
		int code_length_chips,
		const std::complex<float>* local_codes_in,
		float *shifts_chips,
		int n_correlators
		)
{

          cudaSetDevice(selected_gps_device);
	//********* ZERO COPY VERSION ************
//	// Get device pointer from host memory. No allocation or memcpy
//	cudaError_t code;
//	// local code CPU -> GPU copy memory
//	code=cudaHostGetDevicePointer((void **)&d_local_codes_in,  (void *) local_codes_in, 0);
//	if (code!=cudaSuccess)
//	{
//		printf("cuda cudaHostGetDevicePointer error in set_local_code_and_taps \r\n");
//	}
//	// Correlator shifts vector CPU -> GPU copy memory (fractional chip shifts are allowed!)
//	code=cudaHostGetDevicePointer((void **)&d_shifts_chips,  (void *) shifts_chips, 0);
//	if (code!=cudaSuccess)
//	{
//		printf("cuda cudaHostGetDevicePointer error in set_local_code_and_taps \r\n");
//	}

	//******** CudaMalloc version ***********
    //local code CPU -> GPU copy memory
    cudaMemcpyAsync(d_local_codes_in, local_codes_in, sizeof(GPU_Complex)*code_length_chips, cudaMemcpyHostToDevice,stream1);
    d_code_length_chips=code_length_chips;

    //Correlator shifts vector CPU -> GPU copy memory (fractional chip shifts are allowed!)
    cudaMemcpyAsync(d_shifts_chips, shifts_chips, sizeof(float)*n_correlators,
                                    cudaMemcpyHostToDevice,stream1);

	return true;
}

bool cuda_multicorrelator::set_input_output_vectors(
		std::complex<float>* corr_out,
		std::complex<float>* sig_in
		)
{

         cudaSetDevice(selected_gps_device);
	// Save CPU pointers
	d_sig_in_cpu =sig_in;
	d_corr_out_cpu = corr_out;

	// Zero Copy version
	// Get device pointer from host memory. No allocation or memcpy
	cudaError_t code;
	code=cudaHostGetDevicePointer((void **)&d_sig_in,  (void *) sig_in, 0);
	code=cudaHostGetDevicePointer((void **)&d_corr_out,  (void *) corr_out, 0);
	if (code!=cudaSuccess)
	{
		printf("cuda cudaHostGetDevicePointer error \r\n");
	}
	return true;

}

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess)
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}

bool cuda_multicorrelator::Carrier_wipeoff_multicorrelator_resampler_cuda(
		float rem_carrier_phase_in_rad,
		float phase_step_rad,
        float code_phase_step_chips,
        float rem_code_phase_chips,
		int signal_length_samples,
		int n_correlators)
	{

    cudaSetDevice(selected_gps_device); 
	// cudaMemCpy version
	//size_t memSize = signal_length_samples * sizeof(std::complex<float>);
	// input signal CPU -> GPU copy memory
    //cudaMemcpyAsync(d_sig_in, d_sig_in_cpu, memSize,
    //                               cudaMemcpyHostToDevice, stream2);
    //***** NOTICE: NCO is computed on-the-fly, not need to copy NCO into GPU! ****

    //launch the multitap correlator with integrated local code resampler!

    Doppler_wippe_scalarProdGPUCPXxN_shifts_chips<<<blocksPerGrid, threadsPerBlock,0 ,stream1>>>(
			d_corr_out,
			d_sig_in,
			d_sig_doppler_wiped,
			d_local_codes_in,
			d_shifts_chips,
			d_code_length_chips,
	        code_phase_step_chips,
	        rem_code_phase_chips,
			n_correlators,
			signal_length_samples,
			rem_carrier_phase_in_rad,
			phase_step_rad
			);

    gpuErrchk( cudaPeekAtLastError() );
    gpuErrchk( cudaStreamSynchronize(stream1));

	// cudaMemCpy version
    // Copy the device result vector in device memory to the host result vector
    // in host memory.
    //scalar products (correlators outputs)
    //cudaMemcpyAsync(d_corr_out_cpu, d_corr_out, sizeof(std::complex<float>)*n_correlators,
    //        cudaMemcpyDeviceToHost,stream1);
    return true;
}

cuda_multicorrelator::cuda_multicorrelator()
{
	d_sig_in=NULL;
	d_nco_in=NULL;
	d_sig_doppler_wiped=NULL;
	d_local_codes_in=NULL;
	d_shifts_samples=NULL;
	d_shifts_chips=NULL;
	d_corr_out=NULL;
	threadsPerBlock=0;
	blocksPerGrid=0;
	d_code_length_chips=0;
}

bool cuda_multicorrelator::free_cuda()
{
	// Free device global memory
	if (d_sig_in!=NULL) cudaFree(d_sig_in);
	if (d_nco_in!=NULL) cudaFree(d_nco_in);
	if (d_sig_doppler_wiped!=NULL) cudaFree(d_sig_doppler_wiped);
	if (d_local_codes_in!=NULL) cudaFree(d_local_codes_in);
	if (d_corr_out!=NULL) cudaFree(d_corr_out);
	if (d_shifts_samples!=NULL) cudaFree(d_shifts_samples);
	if (d_shifts_chips!=NULL) cudaFree(d_shifts_chips);
    // Reset the device and exit
    // cudaDeviceReset causes the driver to clean up all state. While
    // not mandatory in normal operation, it is good practice.  It is also
    // needed to ensure correct operation when the application is being
    // profiled. Calling cudaDeviceReset causes all profile data to be
    // flushed before the application exits
	cudaDeviceReset();
	return true;
}

