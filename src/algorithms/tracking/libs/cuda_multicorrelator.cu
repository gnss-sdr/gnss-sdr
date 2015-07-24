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

///////////////////////////////////////////////////////////////////////////////
// On G80-class hardware 24-bit multiplication takes 4 clocks per warp
// (the same as for floating point  multiplication and addition),
// whereas full 32-bit multiplication takes 16 clocks per warp.
// So if integer multiplication operands are  guaranteed to fit into 24 bits
// (always lie withtin [-8M, 8M - 1] range in signed case),
// explicit 24-bit multiplication is preferred for performance.
///////////////////////////////////////////////////////////////////////////////
#define IMUL(a, b) __mul24(a, b)

#include "cuda_multicorrelator.h"

#include <stdio.h>

// For the CUDA runtime routines (prefixed with "cuda_")
#include <cuda_runtime.h>

// helper functions and utilities to work with CUDA
#include <helper_cuda.h>
#include <helper_functions.h>

#define ACCUM_N 1024

///////////////////////////////////////////////////////////////////////////////
// Calculate scalar products of VectorN vectors of ElementN elements on GPU
// Parameters restrictions:
// 1) ElementN is strongly preferred to be a multiple of warp size to
//    meet alignment constraints of memory coalescing.
// 2) ACCUM_N must be a power of two.
///////////////////////////////////////////////////////////////////////////////


__global__ void scalarProdGPUCPXxN_shifts(
    GPU_Complex *d_corr_out,
    GPU_Complex *d_sig_in,
    GPU_Complex *d_local_codes_in,
    int *d_shifts_samples,
    int vectorN,
    int elementN
)
{
    //Accumulators cache
    __shared__ GPU_Complex accumResult[ACCUM_N];

    ////////////////////////////////////////////////////////////////////////////
    // Cycle through every pair of vectors,
    // taking into account that vector counts can be different
    // from total number of thread blocks
    ////////////////////////////////////////////////////////////////////////////
    for (int vec = blockIdx.x; vec < vectorN; vec += gridDim.x)
    {
        int vectorBase = IMUL(elementN, vec);
        int vectorEnd  = vectorBase + elementN;

        ////////////////////////////////////////////////////////////////////////
        // Each accumulator cycles through vectors with
        // stride equal to number of total number of accumulators ACCUM_N
        // At this stage ACCUM_N is only preferred be a multiple of warp size
        // to meet memory coalescing alignment constraints.
        ////////////////////////////////////////////////////////////////////////
        for (int iAccum = threadIdx.x; iAccum < ACCUM_N; iAccum += blockDim.x)
        {
        	GPU_Complex sum = GPU_Complex(0,0);

            for (int pos = vectorBase + iAccum; pos < vectorEnd; pos += ACCUM_N)
            {
                //sum = sum + d_sig_in[pos-vectorBase] * d_nco_in[pos-vectorBase] * d_local_codes_in[pos];
            	//sum = sum + d_sig_in[pos-vectorBase] * d_local_codes_in[pos];
            	sum.multiply_acc(d_sig_in[pos-vectorBase],d_local_codes_in[pos-vectorBase+d_shifts_samples[vec]]);
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


__global__ void scalarProdGPUCPXxN(
    GPU_Complex *d_corr_out,
    GPU_Complex *d_sig_in,
    GPU_Complex *d_local_codes_in,
    int vectorN,
    int elementN
)
{
    //Accumulators cache
    __shared__ GPU_Complex accumResult[ACCUM_N];

    ////////////////////////////////////////////////////////////////////////////
    // Cycle through every pair of vectors,
    // taking into account that vector counts can be different
    // from total number of thread blocks
    ////////////////////////////////////////////////////////////////////////////
    for (int vec = blockIdx.x; vec < vectorN; vec += gridDim.x)
    {
        int vectorBase = IMUL(elementN, vec);
        int vectorEnd  = vectorBase + elementN;

        ////////////////////////////////////////////////////////////////////////
        // Each accumulator cycles through vectors with
        // stride equal to number of total number of accumulators ACCUM_N
        // At this stage ACCUM_N is only preferred be a multiple of warp size
        // to meet memory coalescing alignment constraints.
        ////////////////////////////////////////////////////////////////////////
        for (int iAccum = threadIdx.x; iAccum < ACCUM_N; iAccum += blockDim.x)
        {
        	GPU_Complex sum = GPU_Complex(0,0);

            for (int pos = vectorBase + iAccum; pos < vectorEnd; pos += ACCUM_N)
            {
                //sum = sum + d_sig_in[pos-vectorBase] * d_nco_in[pos-vectorBase] * d_local_codes_in[pos];
            	//sum = sum + d_sig_in[pos-vectorBase] * d_local_codes_in[pos];
            	sum.multiply_acc(d_sig_in[pos-vectorBase],d_local_codes_in[pos]);
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


//*********** CUDA processing **************
// Treads: a minimal parallel execution code on GPU
// Blocks: a set of N threads
/**
 * CUDA Kernel Device code
 *
 * Computes the vectorial product of A and B into C. The 3 vectors have the same
 * number of elements numElements.
 */
__global__ void CUDA_32fc_x2_multiply_32fc(  GPU_Complex *A,   GPU_Complex  *B, GPU_Complex  *C, int numElements)
{
    int i = blockDim.x * blockIdx.x + threadIdx.x;

    if (i < numElements)
    {
        C[i] =  A[i] * B[i];
    }
}


/**
 * CUDA Kernel Device code
 *
 * Computes the carrier Doppler wipe-off by integrating the NCO in the CUDA kernel
 */
__global__ void
CUDA_32fc_Doppler_wipeoff(  GPU_Complex *sig_out, GPU_Complex *sig_in, float rem_carrier_phase_in_rad, float phase_step_rad, int numElements)
{
	//*** NCO CPU code (GNURadio FXP NCO)
	//float sin_f, cos_f;
	//float phase_step_rad = static_cast<float>(2 * GALILEO_PI) * d_carrier_doppler_hz / static_cast<float>(d_fs_in);
	//int phase_step_rad_i = gr::fxpt::float_to_fixed(phase_step_rad);
	//int phase_rad_i = gr::fxpt::float_to_fixed(d_rem_carr_phase_rad);
	//
	//for(int i = 0; i < d_current_prn_length_samples; i++)
	//    {
	//        gr::fxpt::sincos(phase_rad_i, &sin_f, &cos_f);
	//        d_carr_sign[i] = std::complex<float>(cos_f, -sin_f);
	//        phase_rad_i += phase_step_rad_i;
	//    }

	// CUDA version of floating point NCO and vector dot product integrated

    int i = blockDim.x * blockIdx.x + threadIdx.x;
    float sin;
    float cos;
    if (i < numElements)
    {
    	__sincosf(rem_carrier_phase_in_rad + i*phase_step_rad, &sin, &cos);
    	sig_out[i] =  sig_in[i] * GPU_Complex(cos,-sin);
    }
}


/**
 * CUDA Kernel Device code
 *
 * Computes the vectorial product of A and B into C. The 3 vectors have the same
 * number of elements numElements.
 */
__global__ void
CUDA_32fc_x2_add_32fc(  GPU_Complex *A,   GPU_Complex  *B, GPU_Complex  *C, int numElements)
{
    int i = blockDim.x * blockIdx.x + threadIdx.x;

    if (i < numElements)
    {
        C[i] =  A[i] * B[i];
    }
}


bool cuda_multicorrelator::init_cuda(const int argc, const char **argv, int signal_length_samples, int local_codes_length_samples, int n_correlators)
{
	// use command-line specified CUDA device, otherwise use device with highest Gflops/s
//	findCudaDevice(argc, (const char **)argv);
      cudaDeviceProp  prop;
    int num_devices, device;
    cudaGetDeviceCount(&num_devices);
    num_gpu_devices=num_devices;
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
		    //set random device!
		    selected_device=(rand() % num_devices);
		    printf("selected_device=%i\n",selected_device);
          cudaGetDeviceProperties( &prop, selected_device );
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
    		selected_device=0;
    	    int whichDevice;
    	    cudaGetDevice( &whichDevice );
    	    cudaGetDeviceProperties( &prop, whichDevice );
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


	//checkCudaErrors(cudaFuncSetCacheConfig(CUDA_32fc_x2_multiply_x2_dot_prod_32fc_, cudaFuncCachePreferShared));


    // ALLOCATE GPU MEMORY FOR INPUT/OUTPUT and INTERNAL vectors

    size_t size = signal_length_samples * sizeof(GPU_Complex);
    cudaSetDevice(selected_device); //generates a random number between 0 and num_devices to split the threads between GPUs
    
	checkCudaErrors(cudaMalloc((void **)&d_sig_in, size));
	//checkCudaErrors(cudaMalloc((void **)&d_nco_in, size));
	checkCudaErrors(cudaMalloc((void **)&d_sig_doppler_wiped, size));

	// old version: all local codes are independent vectors
	//checkCudaErrors(cudaMalloc((void **)&d_local_codes_in, size*n_correlators));

	// new version: only one vector with extra samples to shift the local code for the correlator set
	// Required: The last correlator tap in d_shifts_samples has the largest sample shift
    size_t size_local_code_bytes = local_codes_length_samples * sizeof(GPU_Complex);
	checkCudaErrors(cudaMalloc((void **)&d_local_codes_in, size_local_code_bytes));
	checkCudaErrors(cudaMalloc((void **)&d_shifts_samples, size+sizeof(int)*n_correlators));

	//scalars
	checkCudaErrors(cudaMalloc((void **)&d_corr_out, sizeof(std::complex<float>)*n_correlators));

    // Launch the Vector Add CUDA Kernel
	threadsPerBlock = 256;
    blocksPerGrid =(int)(signal_length_samples+threadsPerBlock-1)/threadsPerBlock;

	cudaStreamCreate (&stream1) ;
	cudaStreamCreate (&stream2) ;
	return true;
}


bool cuda_multicorrelator::Carrier_wipeoff_multicorrelator_cuda(
		std::complex<float>* corr_out,
		const std::complex<float>* sig_in,
		const std::complex<float>* local_codes_in,
		float rem_carrier_phase_in_rad,
		float phase_step_rad,
		const int *shifts_samples,
		int signal_length_samples,
		int n_correlators)
	{

	size_t memSize = signal_length_samples * sizeof(std::complex<float>);
    cudaSetDevice(selected_device); //generates a random number between 0 and num_devices to split the threads between GPUs
	// input signal CPU -> GPU copy memory

    checkCudaErrors(cudaMemcpyAsync(d_sig_in, sig_in, memSize,
                                    cudaMemcpyHostToDevice, stream1));

    //***** NOTICE: NCO is computed on-the-fly, not need to copy NCO into GPU! ****
    //checkCudaErrors(cudaMemcpyAsync(d_nco_in, nco_in, memSize,
    //                                cudaMemcpyHostToDevice, stream1));


	// old version: all local codes are independent vectors
    //checkCudaErrors(cudaMemcpyAsync(d_local_codes_in, local_codes_in, memSize*n_correlators,
    //                                cudaMemcpyHostToDevice, stream2));

	// new version: only one vector with extra samples to shift the local code for the correlator set
	// Required: The last correlator tap in d_shifts_samples has the largest sample shift

    // local code CPU -> GPU copy memory
    checkCudaErrors(cudaMemcpyAsync(d_local_codes_in, local_codes_in, memSize+sizeof(std::complex<float>)*shifts_samples[n_correlators-1],
                                    cudaMemcpyHostToDevice, stream2));
    // Correlator shifts vector CPU -> GPU copy memory
    checkCudaErrors(cudaMemcpyAsync(d_shifts_samples, shifts_samples, sizeof(int)*n_correlators,
                                    cudaMemcpyHostToDevice, stream2));


    //Launch carrier wipe-off kernel here, while local codes are being copied to GPU!
    checkCudaErrors(cudaStreamSynchronize(stream1));
    CUDA_32fc_Doppler_wipeoff<<<blocksPerGrid, threadsPerBlock,0, stream1>>>(d_sig_doppler_wiped, d_sig_in,rem_carrier_phase_in_rad,phase_step_rad, signal_length_samples);


    //printf("CUDA kernel launch with %d blocks of %d threads\n", blocksPerGrid, threadsPerBlock);

    //wait for Doppler wipeoff end...
    checkCudaErrors(cudaStreamSynchronize(stream1));
    checkCudaErrors(cudaStreamSynchronize(stream2));
    //checkCudaErrors(cudaDeviceSynchronize());

    //old
//    scalarProdGPUCPXxN<<<blocksPerGrid, threadsPerBlock,0 ,stream2>>>(
//    		d_corr_out,
//    		d_sig_doppler_wiped,
//    		d_local_codes_in,
//            3,
//            signal_length_samples
//        );

    //new
    //launch the multitap correlator
    scalarProdGPUCPXxN_shifts<<<blocksPerGrid, threadsPerBlock,0 ,stream2>>>(
			d_corr_out,
			d_sig_doppler_wiped,
			d_local_codes_in,
			d_shifts_samples,
			n_correlators,
			signal_length_samples
		);
    checkCudaErrors(cudaGetLastError());
    //wait for correlators end...
    checkCudaErrors(cudaStreamSynchronize(stream2));
    // Copy the device result vector in device memory to the host result vector
    // in host memory.

    //scalar products (correlators outputs)
    checkCudaErrors(cudaMemcpy(corr_out, d_corr_out, sizeof(std::complex<float>)*n_correlators,
            cudaMemcpyDeviceToHost));
    return true;
}

bool cuda_multicorrelator::free_cuda()
{
    cudaSetDevice(selected_device); //generates a random number between 0 and num_devices to split the threads between GPUs
	// Free device global memory
	cudaFree(d_sig_in);
	//cudaFree(d_nco_in);
	cudaFree(d_local_codes_in);
	cudaFree(d_corr_out);

	cudaStreamDestroy(stream1) ;
	cudaStreamDestroy(stream2) ;

    // Reset the device and exit
    // cudaDeviceReset causes the driver to clean up all state. While
    // not mandatory in normal operation, it is good practice.  It is also
    // needed to ensure correct operation when the application is being
    // profiled. Calling cudaDeviceReset causes all profile data to be
    // flushed before the application exits
	//checkCudaErrors(cudaDeviceReset());
	return true;
}

