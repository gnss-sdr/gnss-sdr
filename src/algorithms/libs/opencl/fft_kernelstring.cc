
//
// File:       fft_kernelstring.cpp
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


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <string.h>
#include <assert.h>
#include "fft_internal.h"
#include "clFFT.h"

using namespace std;

#define max(A, B) ((A) > (B) ? (A) : (B))
#define min(A, B) ((A) < (B) ? (A) : (B))

static string
num2str(int num)
{
    char temp[200];
    sprintf(temp, "%d", num);
    return string(temp);
}

// For any n, this function decomposes n into factors for loacal memory tranpose
// based fft. Factors (radices) are sorted such that the first one (radixArray[0])
// is the largest. This base radix determines the number of registers used by each
// work item and product of remaining radices determine the size of work group needed.
// To make things concrete with and example, suppose n = 1024. It is decomposed into
// 1024 = 16 x 16 x 4. Hence kernel uses float2 a[16], for local in-register fft and
// needs 16 x 4 = 64 work items per work group. So kernel first performance 64 length
// 16 ffts (64 work items working in parallel) following by transpose using local
// memory followed by again 64 length 16 ffts followed by transpose using local memory
// followed by 256 length 4 ffts. For the last step since with size of work group is
// 64 and each work item can array for 16 values, 64 work items can compute 256 length
// 4 ffts by each work item computing 4 length 4 ffts.
// Similarly for n = 2048 = 8 x 8 x 8 x 4, each work group has 8 x 8 x 4 = 256 work
// iterms which each computes 256 (in-parallel) length 8 ffts in-register, followed
// by transpose using local memory, followed by 256 length 8 in-register ffts, followed
// by transpose using local memory, followed by 256 length 8 in-register ffts, followed
// by transpose using local memory, followed by 512 length 4 in-register ffts. Again,
// for the last step, each work item computes two length 4 in-register ffts and thus
// 256 work items are needed to compute all 512 ffts.
// For n = 32 = 8 x 4, 4 work items first compute 4 in-register
// lenth 8 ffts, followed by transpose using local memory followed by 8 in-register
// length 4 ffts, where each work item computes two length 4 ffts thus 4 work items
// can compute 8 length 4 ffts. However if work group size of say 64 is choosen,
// each work group can compute 64/ 4 = 16 size 32 ffts (batched transform).
// Users can play with these parameters to figure what gives best performance on
// their particular device i.e. some device have less register space thus using
// smaller base radix can avoid spilling ... some has small local memory thus
// using smaller work group size may be required etc

static void
getRadixArray(unsigned int n, unsigned int *radixArray, unsigned int *numRadices, unsigned int maxRadix)
{
    if (maxRadix > 1)
        {
            maxRadix = min(n, maxRadix);
            unsigned int cnt = 0;
            while (n > maxRadix)
                {
                    radixArray[cnt++] = maxRadix;
                    n /= maxRadix;
                }
            radixArray[cnt++] = n;
            *numRadices = cnt;
            return;
        }

    switch (n)
        {
        case 2:
            *numRadices = 1;
            radixArray[0] = 2;
            break;

        case 4:
            *numRadices = 1;
            radixArray[0] = 4;
            break;

        case 8:
            *numRadices = 1;
            radixArray[0] = 8;
            break;

        case 16:
            *numRadices = 2;
            radixArray[0] = 8;
            radixArray[1] = 2;
            break;

        case 32:
            *numRadices = 2;
            radixArray[0] = 8;
            radixArray[1] = 4;
            break;

        case 64:
            *numRadices = 2;
            radixArray[0] = 8;
            radixArray[1] = 8;
            break;

        case 128:
            *numRadices = 3;
            radixArray[0] = 8;
            radixArray[1] = 4;
            radixArray[2] = 4;
            break;

        case 256:
            *numRadices = 4;
            radixArray[0] = 4;
            radixArray[1] = 4;
            radixArray[2] = 4;
            radixArray[3] = 4;
            break;

        case 512:
            *numRadices = 3;
            radixArray[0] = 8;
            radixArray[1] = 8;
            radixArray[2] = 8;
            break;

        case 1024:
            *numRadices = 3;
            radixArray[0] = 16;
            radixArray[1] = 16;
            radixArray[2] = 4;
            break;
        case 2048:
            *numRadices = 4;
            radixArray[0] = 8;
            radixArray[1] = 8;
            radixArray[2] = 8;
            radixArray[3] = 4;
            break;
        default:
            *numRadices = 0;
            return;
        }
}

static void
insertHeader(string &kernelString, string &kernelName, clFFT_DataFormat dataFormat)
{
    if (dataFormat == clFFT_SplitComplexFormat)
        kernelString += string("__kernel void ") + kernelName + string("(__global float *in_real, __global float *in_imag, __global float *out_real, __global float *out_imag, int dir, int S)\n");
    else
        kernelString += string("__kernel void ") + kernelName + string("(__global float2 *in, __global float2 *out, int dir, int S)\n");
}

static void
insertVariables(string &kStream, int maxRadix)
{
    kStream += string("    int i, j, r, indexIn, indexOut, index, tid, bNum, xNum, k, l;\n");
    kStream += string("    int s, ii, jj, offset;\n");
    kStream += string("    float2 w;\n");
    kStream += string("    float ang, angf, ang1;\n");
    kStream += string("    __local float *lMemStore, *lMemLoad;\n");
    kStream += string("    float2 a[") + num2str(maxRadix) + string("];\n");
    kStream += string("    int lId = get_local_id( 0 );\n");
    kStream += string("    int groupId = get_group_id( 0 );\n");
}

static void
formattedLoad(string &kernelString, int aIndex, int gIndex, clFFT_DataFormat dataFormat)
{
    if (dataFormat == clFFT_InterleavedComplexFormat)
        kernelString += string("        a[") + num2str(aIndex) + string("] = in[") + num2str(gIndex) + string("];\n");
    else
        {
            kernelString += string("        a[") + num2str(aIndex) + string("].x = in_real[") + num2str(gIndex) + string("];\n");
            kernelString += string("        a[") + num2str(aIndex) + string("].y = in_imag[") + num2str(gIndex) + string("];\n");
        }
}

static void
formattedStore(string &kernelString, int aIndex, int gIndex, clFFT_DataFormat dataFormat)
{
    if (dataFormat == clFFT_InterleavedComplexFormat)
        kernelString += string("        out[") + num2str(gIndex) + string("] = a[") + num2str(aIndex) + string("];\n");
    else
        {
            kernelString += string("        out_real[") + num2str(gIndex) + string("] = a[") + num2str(aIndex) + string("].x;\n");
            kernelString += string("        out_imag[") + num2str(gIndex) + string("] = a[") + num2str(aIndex) + string("].y;\n");
        }
}

static int
insertGlobalLoadsAndTranspose(string &kernelString, int N, int numWorkItemsPerXForm, int numXFormsPerWG, int R0, int mem_coalesce_width, clFFT_DataFormat dataFormat)
{
    int log2NumWorkItemsPerXForm = (int)log2(numWorkItemsPerXForm);
    int groupSize = numWorkItemsPerXForm * numXFormsPerWG;
    int i, j;
    int lMemSize = 0;

    if (numXFormsPerWG > 1)
        kernelString += string("        s = S & ") + num2str(numXFormsPerWG - 1) + string(";\n");

    if (numWorkItemsPerXForm >= mem_coalesce_width)
        {
            if (numXFormsPerWG > 1)
                {
                    kernelString += string("    ii = lId & ") + num2str(numWorkItemsPerXForm - 1) + string(";\n");
                    kernelString += string("    jj = lId >> ") + num2str(log2NumWorkItemsPerXForm) + string(";\n");
                    kernelString += string("    if( !s || (groupId < get_num_groups(0)-1) || (jj < s) ) {\n");
                    kernelString += string("        offset = mad24( mad24(groupId, ") + num2str(numXFormsPerWG) + string(", jj), ") + num2str(N) + string(", ii );\n");
                    if (dataFormat == clFFT_InterleavedComplexFormat)
                        {
                            kernelString += string("        in += offset;\n");
                            kernelString += string("        out += offset;\n");
                        }
                    else
                        {
                            kernelString += string("        in_real += offset;\n");
                            kernelString += string("        in_imag += offset;\n");
                            kernelString += string("        out_real += offset;\n");
                            kernelString += string("        out_imag += offset;\n");
                        }
                    for (i = 0; i < R0; i++)
                        formattedLoad(kernelString, i, i * numWorkItemsPerXForm, dataFormat);
                    kernelString += string("    }\n");
                }
            else
                {
                    kernelString += string("    ii = lId;\n");
                    kernelString += string("    jj = 0;\n");
                    kernelString += string("    offset =  mad24(groupId, ") + num2str(N) + string(", ii);\n");
                    if (dataFormat == clFFT_InterleavedComplexFormat)
                        {
                            kernelString += string("        in += offset;\n");
                            kernelString += string("        out += offset;\n");
                        }
                    else
                        {
                            kernelString += string("        in_real += offset;\n");
                            kernelString += string("        in_imag += offset;\n");
                            kernelString += string("        out_real += offset;\n");
                            kernelString += string("        out_imag += offset;\n");
                        }
                    for (i = 0; i < R0; i++)
                        formattedLoad(kernelString, i, i * numWorkItemsPerXForm, dataFormat);
                }
        }
    else if (N >= mem_coalesce_width)
        {
            int numInnerIter = N / mem_coalesce_width;
            int numOuterIter = numXFormsPerWG / (groupSize / mem_coalesce_width);

            kernelString += string("    ii = lId & ") + num2str(mem_coalesce_width - 1) + string(";\n");
            kernelString += string("    jj = lId >> ") + num2str((int)log2(mem_coalesce_width)) + string(";\n");
            kernelString += string("    lMemStore = sMem + mad24( jj, ") + num2str(N + numWorkItemsPerXForm) + string(", ii );\n");
            kernelString += string("    offset = mad24( groupId, ") + num2str(numXFormsPerWG) + string(", jj);\n");
            kernelString += string("    offset = mad24( offset, ") + num2str(N) + string(", ii );\n");
            if (dataFormat == clFFT_InterleavedComplexFormat)
                {
                    kernelString += string("        in += offset;\n");
                    kernelString += string("        out += offset;\n");
                }
            else
                {
                    kernelString += string("        in_real += offset;\n");
                    kernelString += string("        in_imag += offset;\n");
                    kernelString += string("        out_real += offset;\n");
                    kernelString += string("        out_imag += offset;\n");
                }

            kernelString += string("if((groupId == get_num_groups(0)-1) && s) {\n");
            for (i = 0; i < numOuterIter; i++)
                {
                    kernelString += string("    if( jj < s ) {\n");
                    for (j = 0; j < numInnerIter; j++)
                        formattedLoad(kernelString, i * numInnerIter + j, j * mem_coalesce_width + i * (groupSize / mem_coalesce_width) * N, dataFormat);
                    kernelString += string("    }\n");
                    if (i != numOuterIter - 1)
                        kernelString += string("    jj += ") + num2str(groupSize / mem_coalesce_width) + string(";\n");
                }
            kernelString += string("}\n ");
            kernelString += string("else {\n");
            for (i = 0; i < numOuterIter; i++)
                {
                    for (j = 0; j < numInnerIter; j++)
                        formattedLoad(kernelString, i * numInnerIter + j, j * mem_coalesce_width + i * (groupSize / mem_coalesce_width) * N, dataFormat);
                }
            kernelString += string("}\n");

            kernelString += string("    ii = lId & ") + num2str(numWorkItemsPerXForm - 1) + string(";\n");
            kernelString += string("    jj = lId >> ") + num2str(log2NumWorkItemsPerXForm) + string(";\n");
            kernelString += string("    lMemLoad  = sMem + mad24( jj, ") + num2str(N + numWorkItemsPerXForm) + string(", ii);\n");

            for (i = 0; i < numOuterIter; i++)
                {
                    for (j = 0; j < numInnerIter; j++)
                        {
                            kernelString += string("    lMemStore[") + num2str(j * mem_coalesce_width + i * (groupSize / mem_coalesce_width) * (N + numWorkItemsPerXForm)) + string("] = a[") +
                                            num2str(i * numInnerIter + j) + string("].x;\n");
                        }
                }
            kernelString += string("    barrier( CLK_LOCAL_MEM_FENCE );\n");

            for (i = 0; i < R0; i++)
                kernelString += string("    a[") + num2str(i) + string("].x = lMemLoad[") + num2str(i * numWorkItemsPerXForm) + string("];\n");
            kernelString += string("    barrier( CLK_LOCAL_MEM_FENCE );\n");

            for (i = 0; i < numOuterIter; i++)
                {
                    for (j = 0; j < numInnerIter; j++)
                        {
                            kernelString += string("    lMemStore[") + num2str(j * mem_coalesce_width + i * (groupSize / mem_coalesce_width) * (N + numWorkItemsPerXForm)) + string("] = a[") +
                                            num2str(i * numInnerIter + j) + string("].y;\n");
                        }
                }
            kernelString += string("    barrier( CLK_LOCAL_MEM_FENCE );\n");

            for (i = 0; i < R0; i++)
                kernelString += string("    a[") + num2str(i) + string("].y = lMemLoad[") + num2str(i * numWorkItemsPerXForm) + string("];\n");
            kernelString += string("    barrier( CLK_LOCAL_MEM_FENCE );\n");

            lMemSize = (N + numWorkItemsPerXForm) * numXFormsPerWG;
        }
    else
        {
            kernelString += string("    offset = mad24( groupId,  ") + num2str(N * numXFormsPerWG) + string(", lId );\n");
            if (dataFormat == clFFT_InterleavedComplexFormat)
                {
                    kernelString += string("        in += offset;\n");
                    kernelString += string("        out += offset;\n");
                }
            else
                {
                    kernelString += string("        in_real += offset;\n");
                    kernelString += string("        in_imag += offset;\n");
                    kernelString += string("        out_real += offset;\n");
                    kernelString += string("        out_imag += offset;\n");
                }

            kernelString += string("    ii = lId & ") + num2str(N - 1) + string(";\n");
            kernelString += string("    jj = lId >> ") + num2str((int)log2(N)) + string(";\n");
            kernelString += string("    lMemStore = sMem + mad24( jj, ") + num2str(N + numWorkItemsPerXForm) + string(", ii );\n");

            kernelString += string("if((groupId == get_num_groups(0)-1) && s) {\n");
            for (i = 0; i < R0; i++)
                {
                    kernelString += string("    if(jj < s )\n");
                    formattedLoad(kernelString, i, i * groupSize, dataFormat);
                    if (i != R0 - 1)
                        kernelString += string("    jj += ") + num2str(groupSize / N) + string(";\n");
                }
            kernelString += string("}\n");
            kernelString += string("else {\n");
            for (i = 0; i < R0; i++)
                {
                    formattedLoad(kernelString, i, i * groupSize, dataFormat);
                }
            kernelString += string("}\n");

            if (numWorkItemsPerXForm > 1)
                {
                    kernelString += string("    ii = lId & ") + num2str(numWorkItemsPerXForm - 1) + string(";\n");
                    kernelString += string("    jj = lId >> ") + num2str(log2NumWorkItemsPerXForm) + string(";\n");
                    kernelString += string("    lMemLoad = sMem + mad24( jj, ") + num2str(N + numWorkItemsPerXForm) + string(", ii );\n");
                }
            else
                {
                    kernelString += string("    ii = 0;\n");
                    kernelString += string("    jj = lId;\n");
                    kernelString += string("    lMemLoad = sMem + mul24( jj, ") + num2str(N + numWorkItemsPerXForm) + string(");\n");
                }


            for (i = 0; i < R0; i++)
                kernelString += string("    lMemStore[") + num2str(i * (groupSize / N) * (N + numWorkItemsPerXForm)) + string("] = a[") + num2str(i) + string("].x;\n");
            kernelString += string("    barrier( CLK_LOCAL_MEM_FENCE );\n");

            for (i = 0; i < R0; i++)
                kernelString += string("    a[") + num2str(i) + string("].x = lMemLoad[") + num2str(i * numWorkItemsPerXForm) + string("];\n");
            kernelString += string("    barrier( CLK_LOCAL_MEM_FENCE );\n");

            for (i = 0; i < R0; i++)
                kernelString += string("    lMemStore[") + num2str(i * (groupSize / N) * (N + numWorkItemsPerXForm)) + string("] = a[") + num2str(i) + string("].y;\n");
            kernelString += string("    barrier( CLK_LOCAL_MEM_FENCE );\n");

            for (i = 0; i < R0; i++)
                kernelString += string("    a[") + num2str(i) + string("].y = lMemLoad[") + num2str(i * numWorkItemsPerXForm) + string("];\n");
            kernelString += string("    barrier( CLK_LOCAL_MEM_FENCE );\n");

            lMemSize = (N + numWorkItemsPerXForm) * numXFormsPerWG;
        }

    return lMemSize;
}

static int
insertGlobalStoresAndTranspose(string &kernelString, int N, int maxRadix, int Nr, int numWorkItemsPerXForm, int numXFormsPerWG, int mem_coalesce_width, clFFT_DataFormat dataFormat)
{
    int groupSize = numWorkItemsPerXForm * numXFormsPerWG;
    int i, j, k, ind;
    int lMemSize = 0;
    int numIter = maxRadix / Nr;
    string indent = string("");

    if (numWorkItemsPerXForm >= mem_coalesce_width)
        {
            if (numXFormsPerWG > 1)
                {
                    kernelString += string("    if( !s || (groupId < get_num_groups(0)-1) || (jj < s) ) {\n");
                    indent = string("    ");
                }
            for (i = 0; i < maxRadix; i++)
                {
                    j = i % numIter;
                    k = i / numIter;
                    ind = j * Nr + k;
                    formattedStore(kernelString, ind, i * numWorkItemsPerXForm, dataFormat);
                }
            if (numXFormsPerWG > 1)
                kernelString += string("    }\n");
        }
    else if (N >= mem_coalesce_width)
        {
            int numInnerIter = N / mem_coalesce_width;
            int numOuterIter = numXFormsPerWG / (groupSize / mem_coalesce_width);

            kernelString += string("    lMemLoad  = sMem + mad24( jj, ") + num2str(N + numWorkItemsPerXForm) + string(", ii );\n");
            kernelString += string("    ii = lId & ") + num2str(mem_coalesce_width - 1) + string(";\n");
            kernelString += string("    jj = lId >> ") + num2str((int)log2(mem_coalesce_width)) + string(";\n");
            kernelString += string("    lMemStore = sMem + mad24( jj,") + num2str(N + numWorkItemsPerXForm) + string(", ii );\n");

            for (i = 0; i < maxRadix; i++)
                {
                    j = i % numIter;
                    k = i / numIter;
                    ind = j * Nr + k;
                    kernelString += string("    lMemLoad[") + num2str(i * numWorkItemsPerXForm) + string("] = a[") + num2str(ind) + string("].x;\n");
                }
            kernelString += string("    barrier( CLK_LOCAL_MEM_FENCE );\n");

            for (i = 0; i < numOuterIter; i++)
                for (j = 0; j < numInnerIter; j++)
                    kernelString += string("    a[") + num2str(i * numInnerIter + j) + string("].x = lMemStore[") + num2str(j * mem_coalesce_width + i * (groupSize / mem_coalesce_width) * (N + numWorkItemsPerXForm)) + string("];\n");
            kernelString += string("    barrier( CLK_LOCAL_MEM_FENCE );\n");

            for (i = 0; i < maxRadix; i++)
                {
                    j = i % numIter;
                    k = i / numIter;
                    ind = j * Nr + k;
                    kernelString += string("    lMemLoad[") + num2str(i * numWorkItemsPerXForm) + string("] = a[") + num2str(ind) + string("].y;\n");
                }
            kernelString += string("    barrier( CLK_LOCAL_MEM_FENCE );\n");

            for (i = 0; i < numOuterIter; i++)
                for (j = 0; j < numInnerIter; j++)
                    kernelString += string("    a[") + num2str(i * numInnerIter + j) + string("].y = lMemStore[") + num2str(j * mem_coalesce_width + i * (groupSize / mem_coalesce_width) * (N + numWorkItemsPerXForm)) + string("];\n");
            kernelString += string("    barrier( CLK_LOCAL_MEM_FENCE );\n");

            kernelString += string("if((groupId == get_num_groups(0)-1) && s) {\n");
            for (i = 0; i < numOuterIter; i++)
                {
                    kernelString += string("    if( jj < s ) {\n");
                    for (j = 0; j < numInnerIter; j++)
                        formattedStore(kernelString, i * numInnerIter + j, j * mem_coalesce_width + i * (groupSize / mem_coalesce_width) * N, dataFormat);
                    kernelString += string("    }\n");
                    if (i != numOuterIter - 1)
                        kernelString += string("    jj += ") + num2str(groupSize / mem_coalesce_width) + string(";\n");
                }
            kernelString += string("}\n");
            kernelString += string("else {\n");
            for (i = 0; i < numOuterIter; i++)
                {
                    for (j = 0; j < numInnerIter; j++)
                        formattedStore(kernelString, i * numInnerIter + j, j * mem_coalesce_width + i * (groupSize / mem_coalesce_width) * N, dataFormat);
                }
            kernelString += string("}\n");

            lMemSize = (N + numWorkItemsPerXForm) * numXFormsPerWG;
        }
    else
        {
            kernelString += string("    lMemLoad  = sMem + mad24( jj,") + num2str(N + numWorkItemsPerXForm) + string(", ii );\n");

            kernelString += string("    ii = lId & ") + num2str(N - 1) + string(";\n");
            kernelString += string("    jj = lId >> ") + num2str((int)log2(N)) + string(";\n");
            kernelString += string("    lMemStore = sMem + mad24( jj,") + num2str(N + numWorkItemsPerXForm) + string(", ii );\n");

            for (i = 0; i < maxRadix; i++)
                {
                    j = i % numIter;
                    k = i / numIter;
                    ind = j * Nr + k;
                    kernelString += string("    lMemLoad[") + num2str(i * numWorkItemsPerXForm) + string("] = a[") + num2str(ind) + string("].x;\n");
                }
            kernelString += string("    barrier( CLK_LOCAL_MEM_FENCE );\n");

            for (i = 0; i < maxRadix; i++)
                kernelString += string("    a[") + num2str(i) + string("].x = lMemStore[") + num2str(i * (groupSize / N) * (N + numWorkItemsPerXForm)) + string("];\n");
            kernelString += string("    barrier( CLK_LOCAL_MEM_FENCE );\n");

            for (i = 0; i < maxRadix; i++)
                {
                    j = i % numIter;
                    k = i / numIter;
                    ind = j * Nr + k;
                    kernelString += string("    lMemLoad[") + num2str(i * numWorkItemsPerXForm) + string("] = a[") + num2str(ind) + string("].y;\n");
                }
            kernelString += string("    barrier( CLK_LOCAL_MEM_FENCE );\n");

            for (i = 0; i < maxRadix; i++)
                kernelString += string("    a[") + num2str(i) + string("].y = lMemStore[") + num2str(i * (groupSize / N) * (N + numWorkItemsPerXForm)) + string("];\n");
            kernelString += string("    barrier( CLK_LOCAL_MEM_FENCE );\n");

            kernelString += string("if((groupId == get_num_groups(0)-1) && s) {\n");
            for (i = 0; i < maxRadix; i++)
                {
                    kernelString += string("    if(jj < s ) {\n");
                    formattedStore(kernelString, i, i * groupSize, dataFormat);
                    kernelString += string("    }\n");
                    if (i != maxRadix - 1)
                        kernelString += string("    jj +=") + num2str(groupSize / N) + string(";\n");
                }
            kernelString += string("}\n");
            kernelString += string("else {\n");
            for (i = 0; i < maxRadix; i++)
                {
                    formattedStore(kernelString, i, i * groupSize, dataFormat);
                }
            kernelString += string("}\n");

            lMemSize = (N + numWorkItemsPerXForm) * numXFormsPerWG;
        }

    return lMemSize;
}

static void
insertfftKernel(string &kernelString, int Nr, int numIter)
{
    int i;
    for (i = 0; i < numIter; i++)
        {
            kernelString += string("    fftKernel") + num2str(Nr) + string("(a+") + num2str(i * Nr) + string(", dir);\n");
        }
}

static void
insertTwiddleKernel(string &kernelString, int Nr, int numIter, int Nprev, int len, int numWorkItemsPerXForm)
{
    int z, k;
    int logNPrev = (int)log2(Nprev);

    for (z = 0; z < numIter; z++)
        {
            if (z == 0)
                {
                    if (Nprev > 1)
                        kernelString += string("    angf = (float) (ii >> ") + num2str(logNPrev) + string(");\n");
                    else
                        kernelString += string("    angf = (float) ii;\n");
                }
            else
                {
                    if (Nprev > 1)
                        kernelString += string("    angf = (float) ((") + num2str(z * numWorkItemsPerXForm) + string(" + ii) >>") + num2str(logNPrev) + string(");\n");
                    else
                        kernelString += string("    angf = (float) (") + num2str(z * numWorkItemsPerXForm) + string(" + ii);\n");
                }

            for (k = 1; k < Nr; k++)
                {
                    int ind = z * Nr + k;
                    //float fac =  (float) (2.0 * M_PI * (double) k / (double) len);
                    kernelString += string("    ang = dir * ( 2.0f * M_PI * ") + num2str(k) + string(".0f / ") + num2str(len) + string(".0f )") + string(" * angf;\n");
                    kernelString += string("    w = (float2)(native_cos(ang), native_sin(ang));\n");
                    kernelString += string("    a[") + num2str(ind) + string("] = complexMul(a[") + num2str(ind) + string("], w);\n");
                }
        }
}

static int
getPadding(int numWorkItemsPerXForm, int Nprev, int numWorkItemsReq, int numXFormsPerWG, int Nr, int numBanks, int *offset, int *midPad)
{
    if ((numWorkItemsPerXForm <= Nprev) || (Nprev >= numBanks))
        *offset = 0;
    else
        {
            int numRowsReq = ((numWorkItemsPerXForm < numBanks) ? numWorkItemsPerXForm : numBanks) / Nprev;
            int numColsReq = 1;
            if (numRowsReq > Nr)
                numColsReq = numRowsReq / Nr;
            numColsReq = Nprev * numColsReq;
            *offset = numColsReq;
        }

    if (numWorkItemsPerXForm >= numBanks || numXFormsPerWG == 1)
        *midPad = 0;
    else
        {
            int bankNum = ((numWorkItemsReq + *offset) * Nr) & (numBanks - 1);
            if (bankNum >= numWorkItemsPerXForm)
                *midPad = 0;
            else
                *midPad = numWorkItemsPerXForm - bankNum;
        }

    int lMemSize = (numWorkItemsReq + *offset) * Nr * numXFormsPerWG + *midPad * (numXFormsPerWG - 1);
    return lMemSize;
}


static void
insertLocalStores(string &kernelString, int numIter, int Nr, int numWorkItemsPerXForm, int numWorkItemsReq, int offset, string &comp)
{
    int z, k;

    for (z = 0; z < numIter; z++)
        {
            for (k = 0; k < Nr; k++)
                {
                    int index = k * (numWorkItemsReq + offset) + z * numWorkItemsPerXForm;
                    kernelString += string("    lMemStore[") + num2str(index) + string("] = a[") + num2str(z * Nr + k) + string("].") + comp + string(";\n");
                }
        }
    kernelString += string("    barrier(CLK_LOCAL_MEM_FENCE);\n");
}

static void
insertLocalLoads(string &kernelString, int n, int Nr, int Nrn, int Nprev, int Ncurr, int numWorkItemsPerXForm, int numWorkItemsReq, int offset, string &comp)
{
    int numWorkItemsReqN = n / Nrn;
    int interBlockHNum = max(Nprev / numWorkItemsPerXForm, 1);
    int interBlockHStride = numWorkItemsPerXForm;
    int vertWidth = max(numWorkItemsPerXForm / Nprev, 1);
    vertWidth = min(vertWidth, Nr);
    int vertNum = Nr / vertWidth;
    int vertStride = (n / Nr + offset) * vertWidth;
    int iter = max(numWorkItemsReqN / numWorkItemsPerXForm, 1);
    int intraBlockHStride = (numWorkItemsPerXForm / (Nprev * Nr)) > 1 ? (numWorkItemsPerXForm / (Nprev * Nr)) : 1;
    intraBlockHStride *= Nprev;

    int stride = numWorkItemsReq / Nrn;
    int i;
    for (i = 0; i < iter; i++)
        {
            int ii = i / (interBlockHNum * vertNum);
            int zz = i % (interBlockHNum * vertNum);
            int jj = zz % interBlockHNum;
            int kk = zz / interBlockHNum;
            int z;
            for (z = 0; z < Nrn; z++)
                {
                    int st = kk * vertStride + jj * interBlockHStride + ii * intraBlockHStride + z * stride;
                    kernelString += string("    a[") + num2str(i * Nrn + z) + string("].") + comp + string(" = lMemLoad[") + num2str(st) + string("];\n");
                }
        }
    kernelString += string("    barrier(CLK_LOCAL_MEM_FENCE);\n");
}

static void
insertLocalLoadIndexArithmatic(string &kernelString, int Nprev, int Nr, int numWorkItemsReq, int numWorkItemsPerXForm, int numXFormsPerWG, int offset, int midPad)
{
    int Ncurr = Nprev * Nr;
    int logNcurr = (int)log2(Ncurr);
    int logNprev = (int)log2(Nprev);
    int incr = (numWorkItemsReq + offset) * Nr + midPad;

    if (Ncurr < numWorkItemsPerXForm)
        {
            if (Nprev == 1)
                kernelString += string("    j = ii & ") + num2str(Ncurr - 1) + string(";\n");
            else
                kernelString += string("    j = (ii & ") + num2str(Ncurr - 1) + string(") >> ") + num2str(logNprev) + string(";\n");

            if (Nprev == 1)
                kernelString += string("    i = ii >> ") + num2str(logNcurr) + string(";\n");
            else
                kernelString += string("    i = mad24(ii >> ") + num2str(logNcurr) + string(", ") + num2str(Nprev) + string(", ii & ") + num2str(Nprev - 1) + string(");\n");
        }
    else
        {
            if (Nprev == 1)
                kernelString += string("    j = ii;\n");
            else
                kernelString += string("    j = ii >> ") + num2str(logNprev) + string(";\n");
            if (Nprev == 1)
                kernelString += string("    i = 0;\n");
            else
                kernelString += string("    i = ii & ") + num2str(Nprev - 1) + string(";\n");
        }

    if (numXFormsPerWG > 1)
        kernelString += string("    i = mad24(jj, ") + num2str(incr) + string(", i);\n");

    kernelString += string("    lMemLoad = sMem + mad24(j, ") + num2str(numWorkItemsReq + offset) + string(", i);\n");
}

static void
insertLocalStoreIndexArithmatic(string &kernelString, int numWorkItemsReq, int numXFormsPerWG, int Nr, int offset, int midPad)
{
    if (numXFormsPerWG == 1)
        {
            kernelString += string("    lMemStore = sMem + ii;\n");
        }
    else
        {
            kernelString += string("    lMemStore = sMem + mad24(jj, ") + num2str((numWorkItemsReq + offset) * Nr + midPad) + string(", ii);\n");
        }
}


static void
createLocalMemfftKernelString(cl_fft_plan *plan)
{
    unsigned int radixArray[10];
    unsigned int numRadix;

    unsigned int n = plan->n.x;

    assert(n <= plan->max_work_item_per_workgroup * plan->max_radix && "signal lenght too big for local mem fft\n");

    getRadixArray(n, radixArray, &numRadix, 0);
    assert(numRadix > 0 && "no radix array supplied\n");

    if (n / radixArray[0] > plan->max_work_item_per_workgroup)
        getRadixArray(n, radixArray, &numRadix, plan->max_radix);

    assert(radixArray[0] <= plan->max_radix && "max radix choosen is greater than allowed\n");
    assert(n / radixArray[0] <= plan->max_work_item_per_workgroup && "required work items per xform greater than maximum work items allowed per work group for local mem fft\n");

    unsigned int tmpLen = 1;
    unsigned int i;
    for (i = 0; i < numRadix; i++)
        {
            assert(radixArray[i] && !((radixArray[i] - 1) & radixArray[i]));
            tmpLen *= radixArray[i];
        }
    assert(tmpLen == n && "product of radices choosen doesnt match the length of signal\n");

    int offset, midPad;
    string localString(""), kernelName("");

    clFFT_DataFormat dataFormat = plan->format;
    string *kernelString = plan->kernel_string;


    cl_fft_kernel_info **kInfo = &plan->kernel_info;
    int kCount = 0;

    while (*kInfo)
        {
            kInfo = &(*kInfo)->next;
            kCount++;
        }

    kernelName = string("fft") + num2str(kCount);

    *kInfo = (cl_fft_kernel_info *)malloc(sizeof(cl_fft_kernel_info));
    (*kInfo)->kernel = 0;
    (*kInfo)->lmem_size = 0;
    (*kInfo)->num_workgroups = 0;
    (*kInfo)->num_workitems_per_workgroup = 0;
    (*kInfo)->dir = cl_fft_kernel_x;
    (*kInfo)->in_place_possible = 1;
    (*kInfo)->next = NULL;
    (*kInfo)->kernel_name = (char *)malloc(sizeof(char) * (kernelName.size() + 1));
    strcpy((*kInfo)->kernel_name, kernelName.c_str());

    unsigned int numWorkItemsPerXForm = n / radixArray[0];
    unsigned int numWorkItemsPerWG = numWorkItemsPerXForm <= 64 ? 64 : numWorkItemsPerXForm;
    assert(numWorkItemsPerWG <= plan->max_work_item_per_workgroup);
    int numXFormsPerWG = numWorkItemsPerWG / numWorkItemsPerXForm;
    (*kInfo)->num_workgroups = 1;
    (*kInfo)->num_xforms_per_workgroup = numXFormsPerWG;
    (*kInfo)->num_workitems_per_workgroup = numWorkItemsPerWG;

    unsigned int *N = radixArray;
    unsigned int maxRadix = N[0];
    unsigned int lMemSize = 0;

    insertVariables(localString, maxRadix);

    lMemSize = insertGlobalLoadsAndTranspose(localString, n, numWorkItemsPerXForm, numXFormsPerWG, maxRadix, plan->min_mem_coalesce_width, dataFormat);
    (*kInfo)->lmem_size = (lMemSize > (*kInfo)->lmem_size) ? lMemSize : (*kInfo)->lmem_size;

    string xcomp = string("x");
    string ycomp = string("y");

    unsigned int Nprev = 1;
    unsigned int len = n;
    unsigned int r;
    for (r = 0; r < numRadix; r++)
        {
            int numIter = N[0] / N[r];
            int numWorkItemsReq = n / N[r];
            int Ncurr = Nprev * N[r];
            insertfftKernel(localString, N[r], numIter);

            if (r < (numRadix - 1))
                {
                    insertTwiddleKernel(localString, N[r], numIter, Nprev, len, numWorkItemsPerXForm);
                    lMemSize = getPadding(numWorkItemsPerXForm, Nprev, numWorkItemsReq, numXFormsPerWG, N[r], plan->num_local_mem_banks, &offset, &midPad);
                    (*kInfo)->lmem_size = (lMemSize > (*kInfo)->lmem_size) ? lMemSize : (*kInfo)->lmem_size;
                    insertLocalStoreIndexArithmatic(localString, numWorkItemsReq, numXFormsPerWG, N[r], offset, midPad);
                    insertLocalLoadIndexArithmatic(localString, Nprev, N[r], numWorkItemsReq, numWorkItemsPerXForm, numXFormsPerWG, offset, midPad);
                    insertLocalStores(localString, numIter, N[r], numWorkItemsPerXForm, numWorkItemsReq, offset, xcomp);
                    insertLocalLoads(localString, n, N[r], N[r + 1], Nprev, Ncurr, numWorkItemsPerXForm, numWorkItemsReq, offset, xcomp);
                    insertLocalStores(localString, numIter, N[r], numWorkItemsPerXForm, numWorkItemsReq, offset, ycomp);
                    insertLocalLoads(localString, n, N[r], N[r + 1], Nprev, Ncurr, numWorkItemsPerXForm, numWorkItemsReq, offset, ycomp);
                    Nprev = Ncurr;
                    len = len / N[r];
                }
        }

    lMemSize = insertGlobalStoresAndTranspose(localString, n, maxRadix, N[numRadix - 1], numWorkItemsPerXForm, numXFormsPerWG, plan->min_mem_coalesce_width, dataFormat);
    (*kInfo)->lmem_size = (lMemSize > (*kInfo)->lmem_size) ? lMemSize : (*kInfo)->lmem_size;

    insertHeader(*kernelString, kernelName, dataFormat);
    *kernelString += string("{\n");
    if ((*kInfo)->lmem_size)
        *kernelString += string("    __local float sMem[") + num2str((*kInfo)->lmem_size) + string("];\n");
    *kernelString += localString;
    *kernelString += string("}\n");
}

// For n larger than what can be computed using local memory fft, global transposes
// multiple kernel launces is needed. For these sizes, n can be decomposed using
// much larger base radices i.e. say n = 262144 = 128 x 64 x 32. Thus three kernel
// launches will be needed, first computing 64 x 32, length 128 ffts, second computing
// 128 x 32 length 64 ffts, and finally a kernel computing 128 x 64 length 32 ffts.
// Each of these base radices can futher be divided into factors so that each of these
// base ffts can be computed within one kernel launch using in-register ffts and local
// memory transposes i.e for the first kernel above which computes 64 x 32 ffts on length
// 128, 128 can be decomposed into 128 = 16 x 8 i.e. 8 work items can compute 8 length
// 16 ffts followed by transpose using local memory followed by each of these eight
// work items computing 2 length 8 ffts thus computing 16 length 8 ffts in total. This
// means only 8 work items are needed for computing one length 128 fft. If we choose
// work group size of say 64, we can compute 64/8 = 8 length 128 ffts within one
// work group. Since we need to compute 64 x 32 length 128 ffts in first kernel, this
// means we need to launch 64 x 32 / 8 = 256 work groups with 64 work items in each
// work group where each work group is computing 8 length 128 ffts where each length
// 128 fft is computed by 8 work items. Same logic can be applied to other two kernels
// in this example. Users can play with difference base radices and difference
// decompositions of base radices to generates different kernels and see which gives
// best performance. Following function is just fixed to use 128 as base radix

void getGlobalRadixInfo(int n, int *radix, int *R1, int *R2, int *numRadices)
{
    int baseRadix = min(n, 128);

    int numR = 0;
    int N = n;
    while (N > baseRadix)
        {
            N /= baseRadix;
            numR++;
        }

    for (int i = 0; i < numR; i++)
        radix[i] = baseRadix;

    radix[numR] = N;
    numR++;
    *numRadices = numR;

    for (int i = 0; i < numR; i++)
        {
            int B = radix[i];
            if (B <= 8)
                {
                    R1[i] = B;
                    R2[i] = 1;
                    continue;
                }

            int r1 = 2;
            int r2 = B / r1;
            while (r2 > r1)
                {
                    r1 *= 2;
                    r2 = B / r1;
                }
            R1[i] = r1;
            R2[i] = r2;
        }
}

static void
createGlobalFFTKernelString(cl_fft_plan *plan, int n, int BS, cl_fft_kernel_dir dir, int vertBS)
{
    int i, j, k, t;
    int radixArr[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int R1Arr[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int R2Arr[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int radix, R1, R2;
    int numRadices;

    int maxThreadsPerBlock = plan->max_work_item_per_workgroup;
    int maxArrayLen = plan->max_radix;
    int batchSize = plan->min_mem_coalesce_width;
    clFFT_DataFormat dataFormat = plan->format;
    int vertical = (dir == cl_fft_kernel_x) ? 0 : 1;

    getGlobalRadixInfo(n, radixArr, R1Arr, R2Arr, &numRadices);

    int numPasses = numRadices;

    string localString(""), kernelName("");
    string *kernelString = plan->kernel_string;
    cl_fft_kernel_info **kInfo = &plan->kernel_info;
    int kCount = 0;

    while (*kInfo)
        {
            kInfo = &(*kInfo)->next;
            kCount++;
        }

    int N = n;
    int m = (int)log2(n);
    int Rinit = vertical ? BS : 1;
    batchSize = vertical ? min(BS, batchSize) : batchSize;
    int passNum;

    for (passNum = 0; passNum < numPasses; passNum++)
        {
            localString.clear();
            kernelName.clear();

            radix = radixArr[passNum];
            R1 = R1Arr[passNum];
            R2 = R2Arr[passNum];

            int strideI = Rinit;
            for (i = 0; i < numPasses; i++)
                if (i != passNum)
                    strideI *= radixArr[i];

            int strideO = Rinit;
            for (i = 0; i < passNum; i++)
                strideO *= radixArr[i];

            int threadsPerXForm = R2;
            batchSize = R2 == 1 ? plan->max_work_item_per_workgroup : batchSize;
            batchSize = min(batchSize, strideI);
            int threadsPerBlock = batchSize * threadsPerXForm;
            threadsPerBlock = min(threadsPerBlock, maxThreadsPerBlock);
            batchSize = threadsPerBlock / threadsPerXForm;
            assert(R2 <= R1);
            assert(R1 * R2 == radix);
            assert(R1 <= maxArrayLen);
            assert(threadsPerBlock <= maxThreadsPerBlock);

            int numIter = R1 / R2;
            int gInInc = threadsPerBlock / batchSize;


            int lgStrideO = (int)log2(strideO);
            int numBlocksPerXForm = strideI / batchSize;
            int numBlocks = numBlocksPerXForm;
            if (!vertical)
                numBlocks *= BS;
            else
                numBlocks *= vertBS;

            kernelName = string("fft") + num2str(kCount);
            *kInfo = (cl_fft_kernel_info *)malloc(sizeof(cl_fft_kernel_info));
            (*kInfo)->kernel = 0;
            if (R2 == 1)
                (*kInfo)->lmem_size = 0;
            else
                {
                    if (strideO == 1)
                        (*kInfo)->lmem_size = (radix + 1) * batchSize;
                    else
                        (*kInfo)->lmem_size = threadsPerBlock * R1;
                }
            (*kInfo)->num_workgroups = numBlocks;
            (*kInfo)->num_xforms_per_workgroup = 1;
            (*kInfo)->num_workitems_per_workgroup = threadsPerBlock;
            (*kInfo)->dir = dir;
            if ((passNum == (numPasses - 1)) && (numPasses & 1))
                (*kInfo)->in_place_possible = 1;
            else
                (*kInfo)->in_place_possible = 0;
            (*kInfo)->next = NULL;
            (*kInfo)->kernel_name = (char *)malloc(sizeof(char) * (kernelName.size() + 1));
            strcpy((*kInfo)->kernel_name, kernelName.c_str());

            insertVariables(localString, R1);

            if (vertical)
                {
                    localString += string("xNum = groupId >> ") + num2str((int)log2(numBlocksPerXForm)) + string(";\n");
                    localString += string("groupId = groupId & ") + num2str(numBlocksPerXForm - 1) + string(";\n");
                    localString += string("indexIn = mad24(groupId, ") + num2str(batchSize) + string(", xNum << ") + num2str((int)log2(n * BS)) + string(");\n");
                    localString += string("tid = mul24(groupId, ") + num2str(batchSize) + string(");\n");
                    localString += string("i = tid >> ") + num2str(lgStrideO) + string(";\n");
                    localString += string("j = tid & ") + num2str(strideO - 1) + string(";\n");
                    int stride = radix * Rinit;
                    for (i = 0; i < passNum; i++)
                        stride *= radixArr[i];
                    localString += string("indexOut = mad24(i, ") + num2str(stride) + string(", j + ") + string("(xNum << ") + num2str((int)log2(n * BS)) + string("));\n");
                    localString += string("bNum = groupId;\n");
                }
            else
                {
                    int lgNumBlocksPerXForm = (int)log2(numBlocksPerXForm);
                    localString += string("bNum = groupId & ") + num2str(numBlocksPerXForm - 1) + string(";\n");
                    localString += string("xNum = groupId >> ") + num2str(lgNumBlocksPerXForm) + string(";\n");
                    localString += string("indexIn = mul24(bNum, ") + num2str(batchSize) + string(");\n");
                    localString += string("tid = indexIn;\n");
                    localString += string("i = tid >> ") + num2str(lgStrideO) + string(";\n");
                    localString += string("j = tid & ") + num2str(strideO - 1) + string(";\n");
                    int stride = radix * Rinit;
                    for (i = 0; i < passNum; i++)
                        stride *= radixArr[i];
                    localString += string("indexOut = mad24(i, ") + num2str(stride) + string(", j);\n");
                    localString += string("indexIn += (xNum << ") + num2str(m) + string(");\n");
                    localString += string("indexOut += (xNum << ") + num2str(m) + string(");\n");
                }

            // Load Data
            int lgBatchSize = (int)log2(batchSize);
            localString += string("tid = lId;\n");
            localString += string("i = tid & ") + num2str(batchSize - 1) + string(";\n");
            localString += string("j = tid >> ") + num2str(lgBatchSize) + string(";\n");
            localString += string("indexIn += mad24(j, ") + num2str(strideI) + string(", i);\n");

            if (dataFormat == clFFT_SplitComplexFormat)
                {
                    localString += string("in_real += indexIn;\n");
                    localString += string("in_imag += indexIn;\n");
                    for (j = 0; j < R1; j++)
                        localString += string("a[") + num2str(j) + string("].x = in_real[") + num2str(j * gInInc * strideI) + string("];\n");
                    for (j = 0; j < R1; j++)
                        localString += string("a[") + num2str(j) + string("].y = in_imag[") + num2str(j * gInInc * strideI) + string("];\n");
                }
            else
                {
                    localString += string("in += indexIn;\n");
                    for (j = 0; j < R1; j++)
                        localString += string("a[") + num2str(j) + string("] = in[") + num2str(j * gInInc * strideI) + string("];\n");
                }

            localString += string("fftKernel") + num2str(R1) + string("(a, dir);\n");

            if (R2 > 1)
                {
                    // twiddle
                    for (k = 1; k < R1; k++)
                        {
                            localString += string("ang = dir*(2.0f*M_PI*") + num2str(k) + string("/") + num2str(radix) + string(")*j;\n");
                            localString += string("w = (float2)(native_cos(ang), native_sin(ang));\n");
                            localString += string("a[") + num2str(k) + string("] = complexMul(a[") + num2str(k) + string("], w);\n");
                        }

                    // shuffle
                    numIter = R1 / R2;
                    localString += string("indexIn = mad24(j, ") + num2str(threadsPerBlock * numIter) + string(", i);\n");
                    localString += string("lMemStore = sMem + tid;\n");
                    localString += string("lMemLoad = sMem + indexIn;\n");
                    for (k = 0; k < R1; k++)
                        localString += string("lMemStore[") + num2str(k * threadsPerBlock) + string("] = a[") + num2str(k) + string("].x;\n");
                    localString += string("barrier(CLK_LOCAL_MEM_FENCE);\n");
                    for (k = 0; k < numIter; k++)
                        for (t = 0; t < R2; t++)
                            localString += string("a[") + num2str(k * R2 + t) + string("].x = lMemLoad[") + num2str(t * batchSize + k * threadsPerBlock) + string("];\n");
                    localString += string("barrier(CLK_LOCAL_MEM_FENCE);\n");
                    for (k = 0; k < R1; k++)
                        localString += string("lMemStore[") + num2str(k * threadsPerBlock) + string("] = a[") + num2str(k) + string("].y;\n");
                    localString += string("barrier(CLK_LOCAL_MEM_FENCE);\n");
                    for (k = 0; k < numIter; k++)
                        for (t = 0; t < R2; t++)
                            localString += string("a[") + num2str(k * R2 + t) + string("].y = lMemLoad[") + num2str(t * batchSize + k * threadsPerBlock) + string("];\n");
                    localString += string("barrier(CLK_LOCAL_MEM_FENCE);\n");

                    for (j = 0; j < numIter; j++)
                        localString += string("fftKernel") + num2str(R2) + string("(a + ") + num2str(j * R2) + string(", dir);\n");
                }

            // twiddle
            if (passNum < (numPasses - 1))
                {
                    localString += string("l = ((bNum << ") + num2str(lgBatchSize) + string(") + i) >> ") + num2str(lgStrideO) + string(";\n");
                    localString += string("k = j << ") + num2str((int)log2(R1 / R2)) + string(";\n");
                    localString += string("ang1 = dir*(2.0f*M_PI/") + num2str(N) + string(")*l;\n");
                    for (t = 0; t < R1; t++)
                        {
                            localString += string("ang = ang1*(k + ") + num2str((t % R2) * R1 + (t / R2)) + string(");\n");
                            localString += string("w = (float2)(native_cos(ang), native_sin(ang));\n");
                            localString += string("a[") + num2str(t) + string("] = complexMul(a[") + num2str(t) + string("], w);\n");
                        }
                }

            // Store Data
            if (strideO == 1)
                {
                    localString += string("lMemStore = sMem + mad24(i, ") + num2str(radix + 1) + string(", j << ") + num2str((int)log2(R1 / R2)) + string(");\n");
                    localString += string("lMemLoad = sMem + mad24(tid >> ") + num2str((int)log2(radix)) + string(", ") + num2str(radix + 1) + string(", tid & ") + num2str(radix - 1) + string(");\n");

                    for (i = 0; i < R1 / R2; i++)
                        for (j = 0; j < R2; j++)
                            localString += string("lMemStore[ ") + num2str(i + j * R1) + string("] = a[") + num2str(i * R2 + j) + string("].x;\n");
                    localString += string("barrier(CLK_LOCAL_MEM_FENCE);\n");
                    if (threadsPerBlock >= radix)
                        {
                            for (i = 0; i < R1; i++)
                                localString += string("a[") + num2str(i) + string("].x = lMemLoad[") + num2str(i * (radix + 1) * (threadsPerBlock / radix)) + string("];\n");
                        }
                    else
                        {
                            int innerIter = radix / threadsPerBlock;
                            int outerIter = R1 / innerIter;
                            for (i = 0; i < outerIter; i++)
                                for (j = 0; j < innerIter; j++)
                                    localString += string("a[") + num2str(i * innerIter + j) + string("].x = lMemLoad[") + num2str(j * threadsPerBlock + i * (radix + 1)) + string("];\n");
                        }
                    localString += string("barrier(CLK_LOCAL_MEM_FENCE);\n");

                    for (i = 0; i < R1 / R2; i++)
                        for (j = 0; j < R2; j++)
                            localString += string("lMemStore[ ") + num2str(i + j * R1) + string("] = a[") + num2str(i * R2 + j) + string("].y;\n");
                    localString += string("barrier(CLK_LOCAL_MEM_FENCE);\n");
                    if (threadsPerBlock >= radix)
                        {
                            for (i = 0; i < R1; i++)
                                localString += string("a[") + num2str(i) + string("].y = lMemLoad[") + num2str(i * (radix + 1) * (threadsPerBlock / radix)) + string("];\n");
                        }
                    else
                        {
                            int innerIter = radix / threadsPerBlock;
                            int outerIter = R1 / innerIter;
                            for (i = 0; i < outerIter; i++)
                                for (j = 0; j < innerIter; j++)
                                    localString += string("a[") + num2str(i * innerIter + j) + string("].y = lMemLoad[") + num2str(j * threadsPerBlock + i * (radix + 1)) + string("];\n");
                        }
                    localString += string("barrier(CLK_LOCAL_MEM_FENCE);\n");

                    localString += string("indexOut += tid;\n");
                    if (dataFormat == clFFT_SplitComplexFormat)
                        {
                            localString += string("out_real += indexOut;\n");
                            localString += string("out_imag += indexOut;\n");
                            for (k = 0; k < R1; k++)
                                localString += string("out_real[") + num2str(k * threadsPerBlock) + string("] = a[") + num2str(k) + string("].x;\n");
                            for (k = 0; k < R1; k++)
                                localString += string("out_imag[") + num2str(k * threadsPerBlock) + string("] = a[") + num2str(k) + string("].y;\n");
                        }
                    else
                        {
                            localString += string("out += indexOut;\n");
                            for (k = 0; k < R1; k++)
                                localString += string("out[") + num2str(k * threadsPerBlock) + string("] = a[") + num2str(k) + string("];\n");
                        }
                }
            else
                {
                    localString += string("indexOut += mad24(j, ") + num2str(numIter * strideO) + string(", i);\n");
                    if (dataFormat == clFFT_SplitComplexFormat)
                        {
                            localString += string("out_real += indexOut;\n");
                            localString += string("out_imag += indexOut;\n");
                            for (k = 0; k < R1; k++)
                                localString += string("out_real[") + num2str(((k % R2) * R1 + (k / R2)) * strideO) + string("] = a[") + num2str(k) + string("].x;\n");
                            for (k = 0; k < R1; k++)
                                localString += string("out_imag[") + num2str(((k % R2) * R1 + (k / R2)) * strideO) + string("] = a[") + num2str(k) + string("].y;\n");
                        }
                    else
                        {
                            localString += string("out += indexOut;\n");
                            for (k = 0; k < R1; k++)
                                localString += string("out[") + num2str(((k % R2) * R1 + (k / R2)) * strideO) + string("] = a[") + num2str(k) + string("];\n");
                        }
                }

            insertHeader(*kernelString, kernelName, dataFormat);
            *kernelString += string("{\n");
            if ((*kInfo)->lmem_size)
                *kernelString += string("    __local float sMem[") + num2str((*kInfo)->lmem_size) + string("];\n");
            *kernelString += localString;
            *kernelString += string("}\n");

            N /= radix;
            kInfo = &(*kInfo)->next;
            kCount++;
        }
}

void FFT1D(cl_fft_plan *plan, cl_fft_kernel_dir dir)
{
    unsigned int radixArray[10];
    unsigned int numRadix;

    switch (dir)
        {
        case cl_fft_kernel_x:
            if (plan->n.x > plan->max_localmem_fft_size)
                {
                    createGlobalFFTKernelString(plan, plan->n.x, 1, cl_fft_kernel_x, 1);
                }
            else if (plan->n.x > 1)
                {
                    getRadixArray(plan->n.x, radixArray, &numRadix, 0);
                    if (plan->n.x / radixArray[0] <= plan->max_work_item_per_workgroup)
                        {
                            createLocalMemfftKernelString(plan);
                        }
                    else
                        {
                            getRadixArray(plan->n.x, radixArray, &numRadix, plan->max_radix);
                            if (plan->n.x / radixArray[0] <= plan->max_work_item_per_workgroup)
                                createLocalMemfftKernelString(plan);
                            else
                                createGlobalFFTKernelString(plan, plan->n.x, 1, cl_fft_kernel_x, 1);
                        }
                }
            break;

        case cl_fft_kernel_y:
            if (plan->n.y > 1)
                createGlobalFFTKernelString(plan, plan->n.y, plan->n.x, cl_fft_kernel_y, 1);
            break;

        case cl_fft_kernel_z:
            if (plan->n.z > 1)
                createGlobalFFTKernelString(plan, plan->n.z, plan->n.x * plan->n.y, cl_fft_kernel_z, 1);
        default:
            return;
        }
}
