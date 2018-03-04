
//
// File:       clFFT.h
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


#ifndef __CLFFT_H
#define __CLFFT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif

    // XForm type
    typedef enum
    {
        clFFT_Forward = -1,
        clFFT_Inverse = 1

    } clFFT_Direction;

    // XForm dimension
    typedef enum
    {
        clFFT_1D = 0,
        clFFT_2D = 1,
        clFFT_3D = 3

    } clFFT_Dimension;

    // XForm Data type
    typedef enum
    {
        clFFT_SplitComplexFormat = 0,
        clFFT_InterleavedComplexFormat = 1
    } clFFT_DataFormat;

    typedef struct
    {
        unsigned int x;
        unsigned int y;
        unsigned int z;
    } clFFT_Dim3;

    typedef struct
    {
        float *real;
        float *imag;
    } clFFT_SplitComplex;

    typedef struct
    {
        float real;
        float imag;
    } clFFT_Complex;

    typedef void *clFFT_Plan;

    clFFT_Plan clFFT_CreatePlan(cl_context context, clFFT_Dim3 n, clFFT_Dimension dim, clFFT_DataFormat dataFormat, cl_int *error_code);

    void clFFT_DestroyPlan(clFFT_Plan plan);

    cl_int clFFT_ExecuteInterleaved(cl_command_queue queue, clFFT_Plan plan, cl_int batchSize, clFFT_Direction dir,
        cl_mem data_in, cl_mem data_out,
        cl_int num_events, cl_event *event_list, cl_event *event);

    cl_int clFFT_ExecutePlannar(cl_command_queue queue, clFFT_Plan plan, cl_int batchSize, clFFT_Direction dir,
        cl_mem data_in_real, cl_mem data_in_imag, cl_mem data_out_real, cl_mem data_out_imag,
        cl_int num_events, cl_event *event_list, cl_event *event);

    cl_int clFFT_1DTwistInterleaved(clFFT_Plan Plan, cl_command_queue queue, cl_mem array,
        size_t numRows, size_t numCols, size_t startRow, size_t rowsToProcess, clFFT_Direction dir);


    cl_int clFFT_1DTwistPlannar(clFFT_Plan Plan, cl_command_queue queue, cl_mem array_real, cl_mem array_imag,
        size_t numRows, size_t numCols, size_t startRow, size_t rowsToProcess, clFFT_Direction dir);

    void clFFT_DumpPlan(clFFT_Plan plan, FILE *file);

#ifdef __cplusplus
}
#endif

#endif
