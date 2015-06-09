/* Copyright (C) 2010-2015 (see AUTHORS file for a list of contributors)
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
 */


#include <volk_gnsssdr/volk_gnsssdr.h>
#include <qa_16s_permute_and_scalar_add_aligned16.h>
#include <volk_gnsssdr/volk_gnsssdr_16s_permute_and_scalar_add_aligned16.h>
#include <cstdlib>
#include <ctime>

//test for sse2

#ifndef LV_HAVE_SSE2

void qa_16s_permute_and_scalar_add_aligned16::t1()
{
    printf("sse2 not available... no test performed\n");
}

#else

void qa_16s_permute_and_scalar_add_aligned16::t1()
{
    const int vlen = 64;

    unsigned int num_bytes = vlen << 1;

    volk_gnsssdr_environment_init();
    clock_t start, end;
    double total;

    __VOLK_ATTR_ALIGNED(16) short target[vlen];
    __VOLK_ATTR_ALIGNED(16) short target2[vlen];
    __VOLK_ATTR_ALIGNED(16) short src0[vlen];
    __VOLK_ATTR_ALIGNED(16) short permute_indexes[vlen];
    __VOLK_ATTR_ALIGNED(16) short cntl0[vlen];
    __VOLK_ATTR_ALIGNED(16) short cntl1[vlen];
    __VOLK_ATTR_ALIGNED(16) short cntl2[vlen];
    __VOLK_ATTR_ALIGNED(16) short cntl3[vlen];
    __VOLK_ATTR_ALIGNED(16) short scalars[4] = {1, 2, 3, 4};

    for(int i = 0; i < vlen; ++i)
        {
            src0[i] = i;
            permute_indexes[i] = (3 * i)%vlen;
            cntl0[i] = 0xff;
            cntl1[i] = 0xff * (i%2);
            cntl2[i] = 0xff * ((i>>1)%2);
            cntl3[i] = 0xff * ((i%4) == 3);
        }

    printf("16s_permute_and_scalar_add_aligned\n");

    start = clock();
    for(int i = 0; i < 100000; ++i)
        {
            volk_gnsssdr_16s_permute_and_scalar_add_aligned16_manual(target, src0, permute_indexes, cntl0, cntl1, cntl2, cntl3, scalars, num_bytes, "generic");
        }
    end = clock();

    total = (double)(end-start)/(double)CLOCKS_PER_SEC;

    printf("generic_time: %f\n", total);

    start = clock();
    for(int i = 0; i < 100000; ++i)
        {
            volk_gnsssdr_16s_permute_and_scalar_add_aligned16_manual(target2, src0, permute_indexes, cntl0, cntl1, cntl2, cntl3, scalars, num_bytes, "sse2");
        }
    end = clock();

    total = (double)(end-start)/(double)CLOCKS_PER_SEC;

    printf("sse2_time: %f\n", total);

    //for(int i = 0; i < vlen; ++i) {
    //printf("generic... %d, sse2... %d\n", target[i], target2[i]);
    //}

    for(int i = 0; i < vlen; ++i)
        {
            CPPUNIT_ASSERT(target[i] == target2[i]);
        }
}

#endif
