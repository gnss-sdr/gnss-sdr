/*!
 * \file clFFT.h
 * \brief FFT in OpenCL
 *
 *
 * Version:    <1.0>
 *
 * Copyright ( C ) 2008 Apple Inc. All Rights Reserved.
 * SPDX-License-Identifier: LicenseRef-Apple-Permissive
 *
 */

#ifndef __CLFFT_H
#define __CLFFT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>

#ifdef __APPLE__
#define CL_SILENCE_DEPRECATION
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
