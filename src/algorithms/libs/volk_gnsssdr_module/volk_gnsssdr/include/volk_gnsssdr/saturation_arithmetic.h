#ifndef SATURATION_ARITHMETIC_H_
#define SATURATION_ARITHMETIC_H_

#include <limits.h>
//#include <types.h>
static inline int16_t sat_adds16i(int16_t x, int16_t y)
{
    //	int16_t ux = x;
    //	int16_t uy = y;
    //	int16_t res = ux + uy;
    //
    //	/* Calculate overflowed result. (Don't change the sign bit of ux) */
    //	ux = (ux >> 15) + SHRT_MAX;
    //
    //	/* Force compiler to use cmovns instruction */
    //	if ((int16_t) ((ux ^ uy) | ~(uy ^ res)) >= 0)
    //	{
    //		res = ux;
    //	}
    //
    //	return res;

    int32_t res = (int32_t) x + (int32_t) y;

    if (res < SHRT_MIN) res = SHRT_MIN;
    if (res > SHRT_MAX) res = SHRT_MAX;

    return res;
}

#endif /*SATURATION_ARITHMETIC_H_*/
