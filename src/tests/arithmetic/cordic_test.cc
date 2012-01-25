/*!
 * \file cordic_test.cc
 * \brief Test of the CORDIC (COordinate Rotation DIgital Computer) algorithm
 * \author Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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


#include <iostream>
#include <math.h>
#include <gtest/gtest.h>
#include "cordic.h"
#include <sys/time.h>
#include <algorithm>
#include <cstdlib> // for RAND_MAX


TEST(Cordic_Test, StandardCIsFasterThanCordic)
{
    int largest_k = 10;
    Cordic* cordicPtr;
    cordicPtr = new Cordic(largest_k);
    double phase = rand();
    phase = (phase/RAND_MAX) * 3.141592;
    double  cos_phase1 = 0;
    double  sin_phase1 = 0;
    double  cos_phase2 = 0;
    double  sin_phase2 = 0;

    double niter = 10000000;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin1 = tv.tv_sec * 1000000 + tv.tv_usec;

    for(int i=0; i<niter; i++)
        {
            cordicPtr->cordic_get_cos_sin(phase, cos_phase1, sin_phase1);
        }

    gettimeofday(&tv, NULL);
    long long int end1 = tv.tv_sec *1000000 + tv.tv_usec;

    long long int begin2 = tv.tv_sec * 1000000 + tv.tv_usec;

    for(int i=0; i<niter; i++)
        {
            cos_phase2 = cos(phase);
            sin_phase2 = sin(phase);
        }

    gettimeofday(&tv, NULL);
    long long int end2 = tv.tv_sec *1000000 + tv.tv_usec;

    std::cout << "CORDIC sin =" << sin_phase1 << ", cos =" << cos_phase1 << " computed " << niter << " times in " << (end1-begin1) << " microseconds" << std::endl;
    std::cout << "STD    sin =" << sin_phase2 << ", cos =" << cos_phase2 << " computed " << niter << " times in "  << (end2-begin2) << " microseconds" << std::endl;

    EXPECT_TRUE((end2-begin2) < (end1-begin1));
}
