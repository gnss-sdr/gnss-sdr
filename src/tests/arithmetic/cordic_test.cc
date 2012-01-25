
#include <iostream>
#include <math.h>
#include <gtest/gtest.h>
#include "cordic.h"
#include <sys/time.h>


TEST(Cordic_Test, StandardCIsFasterThanCordic)
{
    int largest_k = 0;
    Cordic* cordicPtr;
    cordicPtr = new Cordic(largest_k);
    double phase = 0.1;
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

    std::cout << "CORDIC sin =" << sin_phase2 << " computed in " << (end1-begin1) << " microseconds" << std::endl;
    std::cout << "STD    sin =" << sin(phase) << " computed in "  << (end2-begin2) << " microseconds" << std::endl;

    EXPECT_TRUE((end2-begin2) < (end1-begin1));


}
