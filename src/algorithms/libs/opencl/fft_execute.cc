/*!
 * \file fft_execute.cc
 *
 * Version:    <1.0>
 *
 * Copyright ( C ) 2008 Apple Inc. All Rights Reserved.
 * SPDX-License-Identifier: LicenseRef-Apple-Permissive
 *
 */


#include "clFFT.h"
#include "fft_internal.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>

#define max(a, b) (((a) > (b)) ? (a) : (b))
#define min(a, b) (((a) < (b)) ? (a) : (b))

static cl_int
allocateTemporaryBufferInterleaved(cl_fft_plan *plan, cl_uint batchSize)
{
    cl_int err = CL_SUCCESS;
    if (plan->temp_buffer_needed && plan->last_batch_size != batchSize)
        {
            plan->last_batch_size = batchSize;
            size_t tmpLength = plan->n.x * plan->n.y * plan->n.z * batchSize * 2 * sizeof(cl_float);

            if (plan->tempmemobj)
                clReleaseMemObject(plan->tempmemobj);

            plan->tempmemobj = clCreateBuffer(plan->context, CL_MEM_READ_WRITE, tmpLength, nullptr, &err);
        }
    return err;
}

static cl_int
allocateTemporaryBufferPlannar(cl_fft_plan *plan, cl_uint batchSize)
{
    cl_int err = CL_SUCCESS;
    cl_int terr;
    if (plan->temp_buffer_needed && plan->last_batch_size != batchSize)
        {
            plan->last_batch_size = batchSize;
            size_t tmpLength = plan->n.x * plan->n.y * plan->n.z * batchSize * sizeof(cl_float);

            if (plan->tempmemobj_real)
                clReleaseMemObject(plan->tempmemobj_real);

            if (plan->tempmemobj_imag)
                clReleaseMemObject(plan->tempmemobj_imag);

            plan->tempmemobj_real = clCreateBuffer(plan->context, CL_MEM_READ_WRITE, tmpLength, nullptr, &err);
            plan->tempmemobj_imag = clCreateBuffer(plan->context, CL_MEM_READ_WRITE, tmpLength, nullptr, &terr);
            err |= terr;
        }
    return err;
}

void getKernelWorkDimensions(cl_fft_plan *plan, cl_fft_kernel_info *kernelInfo, cl_int *batchSize, size_t *gWorkItems, size_t *lWorkItems)
{
    *lWorkItems = kernelInfo->num_workitems_per_workgroup;
    int numWorkGroups = kernelInfo->num_workgroups;
    int numXFormsPerWG = kernelInfo->num_xforms_per_workgroup;

    switch (kernelInfo->dir)
        {
        case cl_fft_kernel_x:
            *batchSize *= (plan->n.y * plan->n.z);
            numWorkGroups = (*batchSize % numXFormsPerWG) ? (*batchSize / numXFormsPerWG + 1) : (*batchSize / numXFormsPerWG);
            numWorkGroups *= kernelInfo->num_workgroups;
            break;
        case cl_fft_kernel_y:
            *batchSize *= plan->n.z;
            numWorkGroups *= *batchSize;
            break;
        case cl_fft_kernel_z:
            numWorkGroups *= *batchSize;
            break;
        }

    *gWorkItems = numWorkGroups * *lWorkItems;
}

cl_int
clFFT_ExecuteInterleaved(cl_command_queue queue, clFFT_Plan Plan, cl_int batchSize, clFFT_Direction dir,
    cl_mem data_in, cl_mem data_out,
    cl_int num_events, cl_event *event_list, cl_event *event)
{
    int s;
    auto *plan = (cl_fft_plan *)Plan;
    if (plan->format != clFFT_InterleavedComplexFormat)
        return CL_INVALID_VALUE;

    cl_int err;
    size_t gWorkItems;
    size_t lWorkItems;
    int inPlaceDone;

    cl_int isInPlace = data_in == data_out ? 1 : 0;

    if ((err = allocateTemporaryBufferInterleaved(plan, batchSize)) != CL_SUCCESS)
        return err;

    cl_mem memObj[3];
    memObj[0] = data_in;
    memObj[1] = data_out;
    memObj[2] = plan->tempmemobj;
    cl_fft_kernel_info *kernelInfo = plan->kernel_info;
    int numKernels = plan->num_kernels;

    int numKernelsOdd = numKernels & 1;
    int currRead = 0;
    int currWrite = 1;

    // at least one external dram shuffle (transpose) required
    if (plan->temp_buffer_needed)
        {
            // in-place transform
            if (isInPlace)
                {
                    inPlaceDone = 0;
                    currRead = 1;
                    currWrite = 2;
                }
            else
                {
                    currWrite = (numKernels & 1) ? 1 : 2;
                }

            while (kernelInfo)
                {
                    if (isInPlace && numKernelsOdd && !inPlaceDone && kernelInfo->in_place_possible)
                        {
                            currWrite = currRead;
                            inPlaceDone = 1;
                        }

                    s = batchSize;
                    getKernelWorkDimensions(plan, kernelInfo, &s, &gWorkItems, &lWorkItems);
                    err |= clSetKernelArg(kernelInfo->kernel, 0, sizeof(cl_mem), &memObj[currRead]);
                    err |= clSetKernelArg(kernelInfo->kernel, 1, sizeof(cl_mem), &memObj[currWrite]);
                    err |= clSetKernelArg(kernelInfo->kernel, 2, sizeof(cl_int), &dir);
                    err |= clSetKernelArg(kernelInfo->kernel, 3, sizeof(cl_int), &s);

                    err |= clEnqueueNDRangeKernel(queue, kernelInfo->kernel, 1, nullptr, &gWorkItems, &lWorkItems, 0, nullptr, nullptr);
                    if (err)
                        return err;

                    currRead = (currWrite == 1) ? 1 : 2;
                    currWrite = (currWrite == 1) ? 2 : 1;

                    kernelInfo = kernelInfo->next;
                }
        }
    // no dram shuffle (transpose required) transform
    // all kernels can execute in-place.
    else
        {
            while (kernelInfo)
                {
                    s = batchSize;
                    getKernelWorkDimensions(plan, kernelInfo, &s, &gWorkItems, &lWorkItems);
                    err |= clSetKernelArg(kernelInfo->kernel, 0, sizeof(cl_mem), &memObj[currRead]);
                    err |= clSetKernelArg(kernelInfo->kernel, 1, sizeof(cl_mem), &memObj[currWrite]);
                    err |= clSetKernelArg(kernelInfo->kernel, 2, sizeof(cl_int), &dir);
                    err |= clSetKernelArg(kernelInfo->kernel, 3, sizeof(cl_int), &s);

                    err |= clEnqueueNDRangeKernel(queue, kernelInfo->kernel, 1, nullptr, &gWorkItems, &lWorkItems, 0, nullptr, nullptr);
                    if (err)
                        return err;

                    currRead = 1;
                    currWrite = 1;

                    kernelInfo = kernelInfo->next;
                }
        }

    return err;
}

cl_int
clFFT_ExecutePlannar(cl_command_queue queue, clFFT_Plan Plan, cl_int batchSize, clFFT_Direction dir,
    cl_mem data_in_real, cl_mem data_in_imag, cl_mem data_out_real, cl_mem data_out_imag,
    cl_int num_events, cl_event *event_list, cl_event *event)
{
    int s;
    auto *plan = (cl_fft_plan *)Plan;

    if (plan->format != clFFT_SplitComplexFormat)
        return CL_INVALID_VALUE;

    cl_int err;
    size_t gWorkItems;
    size_t lWorkItems;
    int inPlaceDone;

    cl_int isInPlace = ((data_in_real == data_out_real) && (data_in_imag == data_out_imag)) ? 1 : 0;

    if ((err = allocateTemporaryBufferPlannar(plan, batchSize)) != CL_SUCCESS)
        return err;

    cl_mem memObj_real[3];
    cl_mem memObj_imag[3];
    memObj_real[0] = data_in_real;
    memObj_real[1] = data_out_real;
    memObj_real[2] = plan->tempmemobj_real;
    memObj_imag[0] = data_in_imag;
    memObj_imag[1] = data_out_imag;
    memObj_imag[2] = plan->tempmemobj_imag;

    cl_fft_kernel_info *kernelInfo = plan->kernel_info;
    int numKernels = plan->num_kernels;

    int numKernelsOdd = numKernels & 1;
    int currRead = 0;
    int currWrite = 1;

    // at least one external dram shuffle (transpose) required
    if (plan->temp_buffer_needed)
        {
            // in-place transform
            if (isInPlace)
                {
                    inPlaceDone = 0;
                    currRead = 1;
                    currWrite = 2;
                }
            else
                {
                    currWrite = (numKernels & 1) ? 1 : 2;
                }

            while (kernelInfo)
                {
                    if (isInPlace && numKernelsOdd && !inPlaceDone && kernelInfo->in_place_possible)
                        {
                            currWrite = currRead;
                            inPlaceDone = 1;
                        }

                    s = batchSize;
                    getKernelWorkDimensions(plan, kernelInfo, &s, &gWorkItems, &lWorkItems);
                    err |= clSetKernelArg(kernelInfo->kernel, 0, sizeof(cl_mem), &memObj_real[currRead]);
                    err |= clSetKernelArg(kernelInfo->kernel, 1, sizeof(cl_mem), &memObj_imag[currRead]);
                    err |= clSetKernelArg(kernelInfo->kernel, 2, sizeof(cl_mem), &memObj_real[currWrite]);
                    err |= clSetKernelArg(kernelInfo->kernel, 3, sizeof(cl_mem), &memObj_imag[currWrite]);
                    err |= clSetKernelArg(kernelInfo->kernel, 4, sizeof(cl_int), &dir);
                    err |= clSetKernelArg(kernelInfo->kernel, 5, sizeof(cl_int), &s);

                    err |= clEnqueueNDRangeKernel(queue, kernelInfo->kernel, 1, nullptr, &gWorkItems, &lWorkItems, 0, nullptr, nullptr);
                    if (err)
                        return err;

                    currRead = (currWrite == 1) ? 1 : 2;
                    currWrite = (currWrite == 1) ? 2 : 1;

                    kernelInfo = kernelInfo->next;
                }
        }
    // no dram shuffle (transpose required) transform
    else
        {
            while (kernelInfo)
                {
                    s = batchSize;
                    getKernelWorkDimensions(plan, kernelInfo, &s, &gWorkItems, &lWorkItems);
                    err |= clSetKernelArg(kernelInfo->kernel, 0, sizeof(cl_mem), &memObj_real[currRead]);
                    err |= clSetKernelArg(kernelInfo->kernel, 1, sizeof(cl_mem), &memObj_imag[currRead]);
                    err |= clSetKernelArg(kernelInfo->kernel, 2, sizeof(cl_mem), &memObj_real[currWrite]);
                    err |= clSetKernelArg(kernelInfo->kernel, 3, sizeof(cl_mem), &memObj_imag[currWrite]);
                    err |= clSetKernelArg(kernelInfo->kernel, 4, sizeof(cl_int), &dir);
                    err |= clSetKernelArg(kernelInfo->kernel, 5, sizeof(cl_int), &s);

                    err |= clEnqueueNDRangeKernel(queue, kernelInfo->kernel, 1, nullptr, &gWorkItems, &lWorkItems, 0, nullptr, nullptr);
                    if (err)
                        return err;

                    currRead = 1;
                    currWrite = 1;

                    kernelInfo = kernelInfo->next;
                }
        }

    return err;
}

cl_int
clFFT_1DTwistInterleaved(clFFT_Plan Plan, cl_command_queue queue, cl_mem array,
    unsigned numRows, unsigned numCols, unsigned startRow, unsigned rowsToProcess, clFFT_Direction dir)
{
    auto *plan = (cl_fft_plan *)Plan;

    unsigned int N = numRows * numCols;
    unsigned int nCols = numCols;
    unsigned int sRow = startRow;
    unsigned int rToProcess = rowsToProcess;
    int d = dir;
    int err = 0;

    cl_device_id device_id;
    err = clGetCommandQueueInfo(queue, CL_QUEUE_DEVICE, sizeof(cl_device_id), &device_id, nullptr);
    if (err)
        return err;

    size_t gSize;
    err = clGetKernelWorkGroupInfo(plan->twist_kernel, device_id, CL_KERNEL_WORK_GROUP_SIZE, sizeof(size_t), &gSize, nullptr);
    if (err)
        return err;

    gSize = min(128, gSize);
    size_t numGlobalThreads[1] = {max(numCols / gSize, 1) * gSize};
    size_t numLocalThreads[1] = {gSize};

    err |= clSetKernelArg(plan->twist_kernel, 0, sizeof(cl_mem), &array);
    err |= clSetKernelArg(plan->twist_kernel, 1, sizeof(unsigned int), &sRow);
    err |= clSetKernelArg(plan->twist_kernel, 2, sizeof(unsigned int), &nCols);
    err |= clSetKernelArg(plan->twist_kernel, 3, sizeof(unsigned int), &N);
    err |= clSetKernelArg(plan->twist_kernel, 4, sizeof(unsigned int), &rToProcess);
    err |= clSetKernelArg(plan->twist_kernel, 5, sizeof(int), &d);

    err |= clEnqueueNDRangeKernel(queue, plan->twist_kernel, 1, nullptr, numGlobalThreads, numLocalThreads, 0, nullptr, nullptr);

    return err;
}

cl_int
clFFT_1DTwistPlannar(clFFT_Plan Plan, cl_command_queue queue, cl_mem array_real, cl_mem array_imag,
    unsigned numRows, unsigned numCols, unsigned startRow, unsigned rowsToProcess, clFFT_Direction dir)
{
    auto *plan = (cl_fft_plan *)Plan;

    unsigned int N = numRows * numCols;
    unsigned int nCols = numCols;
    unsigned int sRow = startRow;
    unsigned int rToProcess = rowsToProcess;
    int d = dir;
    int err = 0;

    cl_device_id device_id;
    err = clGetCommandQueueInfo(queue, CL_QUEUE_DEVICE, sizeof(cl_device_id), &device_id, nullptr);
    if (err)
        return err;

    size_t gSize;
    err = clGetKernelWorkGroupInfo(plan->twist_kernel, device_id, CL_KERNEL_WORK_GROUP_SIZE, sizeof(size_t), &gSize, nullptr);
    if (err)
        return err;

    gSize = min(128, gSize);
    size_t numGlobalThreads[1] = {max(numCols / gSize, 1) * gSize};
    size_t numLocalThreads[1] = {gSize};

    err |= clSetKernelArg(plan->twist_kernel, 0, sizeof(cl_mem), &array_real);
    err |= clSetKernelArg(plan->twist_kernel, 1, sizeof(cl_mem), &array_imag);
    err |= clSetKernelArg(plan->twist_kernel, 2, sizeof(unsigned int), &sRow);
    err |= clSetKernelArg(plan->twist_kernel, 3, sizeof(unsigned int), &nCols);
    err |= clSetKernelArg(plan->twist_kernel, 4, sizeof(unsigned int), &N);
    err |= clSetKernelArg(plan->twist_kernel, 5, sizeof(unsigned int), &rToProcess);
    err |= clSetKernelArg(plan->twist_kernel, 6, sizeof(int), &d);

    err |= clEnqueueNDRangeKernel(queue, plan->twist_kernel, 1, nullptr, numGlobalThreads, numLocalThreads, 0, nullptr, nullptr);

    return err;
}
