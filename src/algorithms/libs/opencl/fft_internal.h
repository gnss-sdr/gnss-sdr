
//
// File:       fft_internal.h
//
// Version:    <1.0>
//
// Disclaimer: IMPORTANT:  This Apple software is supplied to you by Apple Inc. ("Apple")
//             in consideration of your agreement to the following terms, and your use,
//             installation, modification or redistribution of this Apple software
//             constitutes acceptance of these terms.  If you do not agree with these
//             terms, please do not use, install, modify or redistribute this Apple
//             software.
//
//             In consideration of your agreement to abide by the following terms, and
//             subject to these terms, Apple grants you a personal, non - exclusive
//             license, under Apple's copyrights in this original Apple software ( the
//             "Apple Software" ), to use, reproduce, modify and redistribute the Apple
//             Software, with or without modifications, in source and / or binary forms;
//             provided that if you redistribute the Apple Software in its entirety and
//             without modifications, you must retain this notice and the following text
//             and disclaimers in all such redistributions of the Apple Software. Neither
//             the name, trademarks, service marks or logos of Apple Inc. may be used to
//             endorse or promote products derived from the Apple Software without specific
//             prior written permission from Apple.  Except as expressly stated in this
//             notice, no other rights or licenses, express or implied, are granted by
//             Apple herein, including but not limited to any patent rights that may be
//             infringed by your derivative works or by other works in which the Apple
//             Software may be incorporated.
//
//             The Apple Software is provided by Apple on an "AS IS" basis.  APPLE MAKES NO
//             WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION THE IMPLIED
//             WARRANTIES OF NON - INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
//             PARTICULAR PURPOSE, REGARDING THE APPLE SOFTWARE OR ITS USE AND OPERATION
//             ALONE OR IN COMBINATION WITH YOUR PRODUCTS.
//
//             IN NO EVENT SHALL APPLE BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL OR
//             CONSEQUENTIAL DAMAGES ( INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//             SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//             INTERRUPTION ) ARISING IN ANY WAY OUT OF THE USE, REPRODUCTION, MODIFICATION
//             AND / OR DISTRIBUTION OF THE APPLE SOFTWARE, HOWEVER CAUSED AND WHETHER
//             UNDER THEORY OF CONTRACT, TORT ( INCLUDING NEGLIGENCE ), STRICT LIABILITY OR
//             OTHERWISE, EVEN IF APPLE HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright ( C ) 2008 Apple Inc. All Rights Reserved.
//
////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef __CLFFT_INTERNAL_H
#define __CLFFT_INTERNAL_H

#include "clFFT.h"
#include <iostream>
#include <string>
#include <sstream>

using namespace std;

typedef enum kernel_dir_t
{
    cl_fft_kernel_x,
    cl_fft_kernel_y,
    cl_fft_kernel_z
} cl_fft_kernel_dir;

typedef struct kernel_info_t
{
    cl_kernel kernel;
    char *kernel_name;
    unsigned lmem_size;
    unsigned num_workgroups;
    unsigned num_xforms_per_workgroup;
    unsigned num_workitems_per_workgroup;
    cl_fft_kernel_dir dir;
    int in_place_possible;
    kernel_info_t *next;
} cl_fft_kernel_info;

typedef struct
{
    // context in which fft resources are created and kernels are executed
    cl_context context;

    // size of signal
    clFFT_Dim3 n;

    // dimension of transform ... must be either 1D, 2D or 3D
    clFFT_Dimension dim;

    // data format ... must be either interleaved or plannar
    clFFT_DataFormat format;

    // string containing kernel source. Generated at runtime based on
    // n, dim, format and other parameters
    string *kernel_string;

    // CL program containing source and kernel this particular
    // n, dim, data format
    cl_program program;

    // linked list of kernels which needs to be executed for this fft
    cl_fft_kernel_info *kernel_info;

    // number of kernels
    int num_kernels;

    // twist kernel for virtualizing fft of very large sizes that do not
    // fit in GPU global memory
    cl_kernel twist_kernel;

    // flag indicating if temporary intermediate buffer is needed or not.
    // this depends on fft kernels being executed and if transform is
    // in-place or out-of-place. e.g. Local memory fft (say 1D 1024 ...
    // one that does not require global transpose do not need temporary buffer)
    // 2D 1024x1024 out-of-place fft however do require intermediate buffer.
    // If temp buffer is needed, its allocation is lazy i.e. its not allocated
    // until its needed
    cl_int temp_buffer_needed;

    // Batch size is runtime parameter and size of temporary buffer (if needed)
    // depends on batch size. Allocation of temporary buffer is lazy i.e. its
    // only created when needed. Once its created at first call of clFFT_Executexxx
    // it is not allocated next time if next time clFFT_Executexxx is called with
    // batch size different than the first call. last_batch_size caches the last
    // batch size with which this plan is used so that we dont keep allocating/deallocating
    // temp buffer if same batch size is used again and again.
    unsigned last_batch_size;

    // temporary buffer for interleaved plan
    cl_mem tempmemobj;

    // temporary buffer for planner plan. Only one of tempmemobj or
    // (tempmemobj_real, tempmemobj_imag) pair is valid (allocated) depending
    // data format of plan (plannar or interleaved)
    cl_mem tempmemobj_real, tempmemobj_imag;

    // Maximum size of signal for which local memory transposed based
    // fft is sufficient i.e. no global mem transpose (communication)
    // is needed
    unsigned max_localmem_fft_size;

    // Maximum work items per work group allowed. This, along with max_radix below controls
    // maximum local memory being used by fft kernels of this plan. Set to 256 by default
    unsigned max_work_item_per_workgroup;

    // Maximum base radix for local memory fft ... this controls the maximum register
    // space used by work items. Currently defaults to 16
    unsigned max_radix;

    // Device depended parameter that tells how many work-items need to be read consecutive
    // values to make sure global memory access by work-items of a work-group result in
    // coalesced memory access to utilize full bandwidth e.g. on NVidia tesla, this is 16
    unsigned min_mem_coalesce_width;

    // Number of local memory banks. This is used to geneate kernel with local memory
    // transposes with appropriate padding to avoid bank conflicts to local memory
    // e.g. on NVidia it is 16.
    unsigned num_local_mem_banks;
} cl_fft_plan;

void FFT1D(cl_fft_plan *plan, cl_fft_kernel_dir dir);

#endif
