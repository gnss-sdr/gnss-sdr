
//
// File:       fft_setup.cpp
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


#include "fft_internal.h"
#include "fft_base_kernels.h"
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <iostream>
#include <string>
#include <sstream>
#include <limits>

using namespace std;

extern void getKernelWorkDimensions(cl_fft_plan *plan, cl_fft_kernel_info *kernelInfo, cl_int *batchSize, size_t *gWorkItems, size_t *lWorkItems);

static void 
getBlockConfigAndKernelString(cl_fft_plan *plan)
{
	plan->temp_buffer_needed = 0;
	*plan->kernel_string += baseKernels;
	
	if(plan->format == clFFT_SplitComplexFormat)
		*plan->kernel_string += twistKernelPlannar;
	else
		*plan->kernel_string += twistKernelInterleaved;
	
	switch(plan->dim) 
	{
		case clFFT_1D:
			FFT1D(plan, cl_fft_kernel_x);
			break;
			
		case clFFT_2D:
			FFT1D(plan, cl_fft_kernel_x); 
			FFT1D(plan, cl_fft_kernel_y);  
			break;
			
		case clFFT_3D:
			FFT1D(plan, cl_fft_kernel_x); 
			FFT1D(plan, cl_fft_kernel_y); 
			FFT1D(plan, cl_fft_kernel_z); 
			break;
			
		default:
			return;
	}
	
	plan->temp_buffer_needed = 0;
	cl_fft_kernel_info *kInfo = plan->kernel_info;
	while(kInfo)
	{
		plan->temp_buffer_needed |= !kInfo->in_place_possible;
		kInfo = kInfo->next;
	}
}

 
static void
deleteKernelInfo(cl_fft_kernel_info *kInfo)
{
	if(kInfo)
	{
	    if(kInfo->kernel_name)
		    free(kInfo->kernel_name);
	    if(kInfo->kernel)
		    clReleaseKernel(kInfo->kernel);
		free(kInfo);
	}	
}

static void
destroy_plan(cl_fft_plan *Plan)
{
    cl_fft_kernel_info *kernel_info = Plan->kernel_info;

	while(kernel_info)
	{
		cl_fft_kernel_info *tmp = kernel_info->next;
		deleteKernelInfo(kernel_info);
		kernel_info = tmp;
	}
	
	Plan->kernel_info = NULL;
		
	if(Plan->kernel_string)
	{
		delete Plan->kernel_string;
		Plan->kernel_string = NULL;
	}			
	if(Plan->twist_kernel)
	{
		clReleaseKernel(Plan->twist_kernel);
		Plan->twist_kernel = NULL;
	}
	if(Plan->program)
	{
		clReleaseProgram(Plan->program);
		Plan->program = NULL;
	}
	if(Plan->tempmemobj) 
	{
		clReleaseMemObject(Plan->tempmemobj);
		Plan->tempmemobj = NULL;
	}
	if(Plan->tempmemobj_real)
	{
		clReleaseMemObject(Plan->tempmemobj_real);
		Plan->tempmemobj_real = NULL;
	}
	if(Plan->tempmemobj_imag)
	{
		clReleaseMemObject(Plan->tempmemobj_imag);
		Plan->tempmemobj_imag = NULL;
	}
}

static int
createKernelList(cl_fft_plan *plan) 
{
	cl_program program = plan->program;
	cl_fft_kernel_info *kernel_info = plan->kernel_info;
	
	cl_int err;
	while(kernel_info)
	{
		kernel_info->kernel = clCreateKernel(program, kernel_info->kernel_name, &err);
		if(!kernel_info->kernel || err != CL_SUCCESS)
			return err;
		kernel_info = kernel_info->next;		
	}
	
	if(plan->format == clFFT_SplitComplexFormat)
		plan->twist_kernel = clCreateKernel(program, "clFFT_1DTwistSplit", &err);
	else
		plan->twist_kernel = clCreateKernel(program, "clFFT_1DTwistInterleaved", &err);
	
	if(!plan->twist_kernel || err)
		return err;

	return CL_SUCCESS;
}

int getMaxKernelWorkGroupSize(cl_fft_plan *plan, unsigned int *max_wg_size, unsigned int num_devices, cl_device_id *devices)
{	
    int reg_needed = 0;
    *max_wg_size = std::numeric_limits<int>::max();
    int err;
    unsigned wg_size;
    
    unsigned int i;
    for(i = 0; i < num_devices; i++)
    {
	    cl_fft_kernel_info *kInfo = plan->kernel_info;
	    while(kInfo)
	    {
		    err = clGetKernelWorkGroupInfo(kInfo->kernel, devices[i], CL_KERNEL_WORK_GROUP_SIZE, sizeof(size_t), &wg_size, NULL);
		    if(err != CL_SUCCESS)
		        return -1;
		        
		    if(wg_size < kInfo->num_workitems_per_workgroup)
		        reg_needed |= 1;
		    
		    if(*max_wg_size > wg_size)
		        *max_wg_size = wg_size;
		        
		    kInfo = kInfo->next;
	    }
	}
	
	return reg_needed;
}	

#define ERR_MACRO(err) { \
                         if( err != CL_SUCCESS) \
                         { \
                           if(error_code) \
                               *error_code = err; \
                           clFFT_DestroyPlan((clFFT_Plan) plan); \
						   return (clFFT_Plan) NULL; \
                         } \
					   }

clFFT_Plan
clFFT_CreatePlan(cl_context context, clFFT_Dim3 n, clFFT_Dimension dim, clFFT_DataFormat dataFormat, cl_int *error_code )
{
	int i;
	cl_int err;
	int isPow2 = 1;
	cl_fft_plan *plan = NULL;
	ostringstream kString;
	int num_devices;
	int gpu_found = 0;
	cl_device_id devices[16];
	size_t ret_size;
	cl_device_type device_type;
	
    if(!context)
		ERR_MACRO(CL_INVALID_VALUE);
	
	isPow2 |= n.x && !( (n.x - 1) & n.x );
	isPow2 |= n.y && !( (n.y - 1) & n.y );
    isPow2 |= n.z && !( (n.z - 1) & n.z );

	if(!isPow2)
		ERR_MACRO(CL_INVALID_VALUE);
	
	if( (dim == clFFT_1D && (n.y != 1 || n.z != 1)) || (dim == clFFT_2D && n.z != 1) )
		ERR_MACRO(CL_INVALID_VALUE);

	plan = (cl_fft_plan *) malloc(sizeof(cl_fft_plan));
	if(!plan)
		ERR_MACRO(CL_OUT_OF_RESOURCES);
	
	plan->context = context;
	clRetainContext(context);
	plan->n = n;
	plan->dim = dim;
	plan->format = dataFormat;
	plan->kernel_info = 0;
	plan->num_kernels = 0;
	plan->twist_kernel = 0;
	plan->program = 0;
	plan->temp_buffer_needed = 0;
	plan->last_batch_size = 0;
	plan->tempmemobj = 0;
	plan->tempmemobj_real = 0;
	plan->tempmemobj_imag = 0;
	plan->max_localmem_fft_size = 2048;
	plan->max_work_item_per_workgroup = 256;
	plan->max_radix = 16;
	plan->min_mem_coalesce_width = 16;
	plan->num_local_mem_banks = 16;	
	
patch_kernel_source:

	plan->kernel_string = new string("");
	if(!plan->kernel_string)
        ERR_MACRO(CL_OUT_OF_RESOURCES);

	getBlockConfigAndKernelString(plan);
	
	const char *source_str = plan->kernel_string->c_str();
	plan->program = clCreateProgramWithSource(context, 1, (const char**) &source_str, NULL, &err);
    ERR_MACRO(err);

	err = clGetContextInfo(context, CL_CONTEXT_DEVICES, sizeof(devices), devices, &ret_size);
	ERR_MACRO(err);
	
	num_devices = (int)(ret_size / sizeof(cl_device_id));
	
	for(i = 0; i < num_devices; i++)
	{
		err = clGetDeviceInfo(devices[i], CL_DEVICE_TYPE, sizeof(device_type), &device_type, NULL);
		ERR_MACRO(err);
		
		if(device_type == CL_DEVICE_TYPE_GPU)
		{	
			gpu_found = 1;
	        err = clBuildProgram(plan->program, 1, &devices[i], "-cl-mad-enable", NULL, NULL);
	        if (err != CL_SUCCESS)
	        {
		        char *build_log;				
				char devicename[200];
		        size_t log_size;
				
		        err = clGetProgramBuildInfo(plan->program, devices[i], CL_PROGRAM_BUILD_LOG, 0, NULL, &log_size);
				ERR_MACRO(err);
				
		        build_log = (char *) malloc(log_size + 1);
				
			    err = clGetProgramBuildInfo(plan->program, devices[i], CL_PROGRAM_BUILD_LOG, log_size, build_log, NULL);
				ERR_MACRO(err);
				
				err = clGetDeviceInfo(devices[i], CL_DEVICE_NAME, sizeof(devicename), devicename, NULL);
				ERR_MACRO(err);
				
				fprintf(stdout, "FFT program build log on device %s\n", devicename);
		        fprintf(stdout, "%s\n", build_log);
		        free(build_log);
				
				ERR_MACRO(err);
			}	
		}	
	}
	
	if(!gpu_found)
		ERR_MACRO(CL_INVALID_CONTEXT);
	
	err = createKernelList(plan); 
    ERR_MACRO(err);
    
    // we created program and kernels based on "some max work group size (default 256)" ... this work group size
    // may be larger than what kernel may execute with ... if thats the case we need to regenerate the kernel source 
    // setting this as limit i.e max group size and rebuild. 
	unsigned int max_kernel_wg_size; 
	int patching_req = getMaxKernelWorkGroupSize(plan, &max_kernel_wg_size, num_devices, devices);
	if(patching_req == -1)
	{
	    ERR_MACRO(err);
	}
	
	if(patching_req)
	{
	    destroy_plan(plan);
	    plan->max_work_item_per_workgroup = max_kernel_wg_size;
	    goto patch_kernel_source;
	}
	
	cl_fft_kernel_info *kInfo = plan->kernel_info;
	while(kInfo)
	{
		plan->num_kernels++;
		kInfo = kInfo->next;
	}
	
	if(error_code)
		*error_code = CL_SUCCESS;
			
	return (clFFT_Plan) plan;
}

void		 
clFFT_DestroyPlan(clFFT_Plan plan)
{
    cl_fft_plan *Plan = (cl_fft_plan *) plan;
	if(Plan) 
	{	
		destroy_plan(Plan);	
		clReleaseContext(Plan->context);
		free(Plan);
	}		
}

void clFFT_DumpPlan( clFFT_Plan Plan, FILE *file)
{
	size_t gDim, lDim;
	FILE *out;
	if(!file)
		out = stdout;
	else 
		out = file;
	
	cl_fft_plan *plan = (cl_fft_plan *) Plan;
	cl_fft_kernel_info *kInfo = plan->kernel_info;
	
	while(kInfo)
	{
		cl_int s = 1;
		getKernelWorkDimensions(plan, kInfo, &s, &gDim, &lDim);
		fprintf(out, "Run kernel %s with global dim = {%zd*BatchSize}, local dim={%zd}\n", kInfo->kernel_name, gDim, lDim);
		kInfo = kInfo->next;
	}
	fprintf(out, "%s\n", plan->kernel_string->c_str());
}
